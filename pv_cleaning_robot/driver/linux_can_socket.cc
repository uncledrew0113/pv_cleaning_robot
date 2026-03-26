#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>  // CAN_ERR_FLAG, CAN_ERR_BUSOFF
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/eventfd.h>  // 必须引入 eventfd 头文件
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include "pv_cleaning_robot/driver/linux_can_socket.h"

namespace robot::driver {

LinuxCanSocket::LinuxCanSocket(std::string interface) : interface_(std::move(interface)) {}

LinuxCanSocket::~LinuxCanSocket() {
    close();
}

bool LinuxCanSocket::open() {
    // 防止重入：若已打开则先安全关闭，避免旧 socket fd 被覆写泳漏
    if (is_open())
        close();

    socket_fd_ = ::socket(AF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (socket_fd_ < 0) {
        spdlog::error("[LinuxCanSocket] socket() failed: {}", strerror(errno));
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }

    int sndbuf = 1024 * 1024;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    int rcvbuf = 1024 * 1024;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    cancel_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (cancel_fd_ < 0) {
        spdlog::error("[LinuxCanSocket] eventfd() failed: {}", strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        spdlog::error(
            "[LinuxCanSocket] ioctl SIOCGIFINDEX failed for {}: {}", interface_, strerror(errno));
        close();
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        spdlog::error("[LinuxCanSocket] bind() failed: {}", strerror(errno));
        close();
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }

    can_err_mask_t err_mask =
        CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_TX_TIMEOUT | CAN_ERR_RESTARTED;
    ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    connected_.store(true);
    bus_off_.store(false);
    last_error_.store(hal::CanResult::OK);
    spdlog::info("[LinuxCanSocket] opened: {}", interface_);
    return true;
}

void LinuxCanSocket::close() {
    connected_.store(false, std::memory_order_release);

    // ─── 优化点 1：使用 eventfd 瞬间唤醒阻塞的 poll ───
    if (cancel_fd_ >= 0) {
        uint64_t val = 1;
        // 写入 8 字节数据，触发 POLLIN 事件。
        // 这会立刻打断 recv() 中正在死等的 poll()，实现真正的 0 延迟安全退出。
        ::write(cancel_fd_, &val, sizeof(val));
    }

    // atomic exchange：FD 改为 -1 并一并获取旧志。
    // 其他线程在 close() 之后拿到 fd==-1 会得到 EBADF，
    // 而不会操作被内核重新分配的新 FD（FD 窜号）。
    int old_fd = socket_fd_.exchange(-1, std::memory_order_acq_rel);
    if (old_fd >= 0) {
        // ::shutdown(old_fd, SHUT_RDWR);
        ::close(old_fd);
        spdlog::debug("[LinuxCanSocket] closed: {}", interface_);
    }

    // 释放 eventfd 资源
    if (cancel_fd_ >= 0) {
        ::close(cancel_fd_);
        cancel_fd_ = -1;
    }
}

bool LinuxCanSocket::is_open() const {
    // 通过 atomic 标志返回，避免裸 int socket_fd_ 读取引发的数据竞争
    return connected_.load();
}

bool LinuxCanSocket::send(const hal::CanFrame& frame) {
    // 将 FD 快照到栏变量：即使 close() 并发，最差收到 EBADF，而不会操作被内核重用的新 FD
    int fd = socket_fd_.load(std::memory_order_acquire);
    if (fd < 0) {
        return false;
    }

    if (bus_off_.load()) {
        // RT 热路径：不打印日志（spdlog 内存分配可导致延迟峰値）。
        // Bus-Off 状态变化时已在 recv() 中记录了一次。
        return false;
    }

    can_frame raw{};
    raw.can_id = frame.is_ext ? (frame.id | CAN_EFF_FLAG) : frame.id;
    if (frame.is_rtr)
        raw.can_id |= CAN_RTR_FLAG;
    const uint8_t safe_len = (frame.len <= 8u) ? frame.len : 8u;
    raw.can_dlc = safe_len;
    std::memcpy(raw.data, frame.data, safe_len);

    ssize_t written = ::write(fd, &raw, sizeof(raw));
    if (written != static_cast<ssize_t>(sizeof(raw))) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) {
            // RT 热路径：不打印 spdlog，用无锁原子计数器替代。
            // 上层可用 get_tx_drop_count() 周期性检查总线拥塞情况。
            tx_drop_count_.fetch_add(1u, std::memory_order_relaxed);
            return false;
        }
        if (errno == ENETDOWN) {
            // 状态转换日志：只说一次
            bool was_off = bus_off_.exchange(true, std::memory_order_relaxed);
            if (!was_off) {
                spdlog::error("[LinuxCanSocket] Bus-Off detected on {} (send ENETDOWN)",
                              interface_);
            }
        }
        return false;
    }
    return true;
}

bool LinuxCanSocket::recv(hal::CanFrame& frame, int timeout_ms) {
    int fd = socket_fd_.load(std::memory_order_acquire);
    if (fd < 0 || cancel_fd_ < 0) {
        last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
        return false;
    }

    // pollfd pfd{};
    // pfd.fd = fd;
    // pfd.events = POLLIN;

    // ─── 优化点 1：同时监听 CAN FD 和 Event FD ───
    pollfd pfds[2]{};
    pfds[0].fd = fd;
    pfds[0].events = POLLIN;
    pfds[1].fd = cancel_fd_;
    pfds[1].events = POLLIN;

    // int ret = ::poll(&pfd, 1, timeout_ms);
    int ret = ::poll(pfds, 2, timeout_ms);
    if (ret < 0) {
        last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
        return false;
    }
    if (ret == 0) {
        last_error_.store(hal::CanResult::TIMEOUT, std::memory_order_relaxed);
        return false;
    }

    // 检查是否是 cancel_fd_ 触发了退出信号
    if (pfds[1].revents & POLLIN) {
        spdlog::debug("[LinuxCanSocket] recv poll interrupted safely by close() on {}", interface_);
        // 直接返回 false 放弃本次接收，不用更新 last_error_ 避免覆盖真实状态
        return false;
    }

    if (pfds[0].revents & (POLLERR | POLLHUP)) {
        // 状态转换日志：只在首次进入 Bus-Off 时打印，防止每次 poll 都输出
        bool was_off = bus_off_.exchange(true, std::memory_order_relaxed);
        last_error_.store(hal::CanResult::BUS_OFF, std::memory_order_relaxed);
        if (!was_off) {
            spdlog::error("[LinuxCanSocket] {} Bus-Off/error (POLLERR/POLLHUP)", interface_);
        }
        return false;
    }

    if (pfds[0].revents & POLLIN) {
        can_frame raw{};
        ssize_t n = ::read(fd, &raw, sizeof(raw));
        if (n != static_cast<ssize_t>(sizeof(raw))) {
            last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
            return false;
        }

        if (raw.can_id & CAN_ERR_FLAG) {
            if (raw.can_id & CAN_ERR_BUSOFF) {
                bool was_off = bus_off_.exchange(true, std::memory_order_relaxed);
                last_error_.store(hal::CanResult::BUS_OFF, std::memory_order_relaxed);
                if (!was_off) {
                    spdlog::error("[LinuxCanSocket] Bus-Off error frame on {}", interface_);
                }
            } else if (raw.can_id & CAN_ERR_CRTL) {
                // raw.data[1] 包含了具体的控制器状态
                if (raw.data[1] & CAN_ERR_CRTL_ACTIVE) {
                    // 硬件已恢复到 Error Active 状态！
                    bool was_off = bus_off_.exchange(false, std::memory_order_relaxed);
                    last_error_.store(hal::CanResult::OK, std::memory_order_relaxed);
                    if (was_off) {
                        spdlog::info(
                            "[LinuxCanSocket] Bus-Off RECOVERED automatically by kernel on {}",
                            interface_);
                    }
                } else {
                    spdlog::warn("[LinuxCanSocket] CAN ctrl warning on {}: err_data[1]={:#04x}",
                                 interface_,
                                 raw.data[1]);
                }
                // last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
                // spdlog::warn("[LinuxCanSocket] CAN ctrl warning on {}: err_data[1]={:#04x}",
                //              interface_,
                //              raw.data[1]);
            }
            // 捕获显式的重启事件（部分驱动会发送这个标志）
            else if (raw.can_id & CAN_ERR_RESTARTED) {
                bus_off_.store(false, std::memory_order_relaxed);
                last_error_.store(hal::CanResult::OK, std::memory_order_relaxed);
                spdlog::info("[LinuxCanSocket] Controller RESTARTED by kernel on {}", interface_);
            } else {
                last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
            }
            return false;
        }

        frame.id = raw.can_id & CAN_EFF_MASK;
        frame.is_ext = (raw.can_id & CAN_EFF_FLAG) != 0;
        frame.is_rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
        frame.len = raw.can_dlc;
        std::memcpy(frame.data, raw.data, raw.can_dlc);

        last_error_.store(hal::CanResult::OK, std::memory_order_relaxed);
        return true;
    }

    last_error_.store(hal::CanResult::SYS_ERROR, std::memory_order_relaxed);
    return false;
}

bool LinuxCanSocket::set_filters(const hal::CanFilter* filters, size_t count) {
    int fd = socket_fd_.load(std::memory_order_acquire);
    if (fd < 0) {
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }
    if (!is_open()) {
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }
    // count==0 语义等同于清空过滤器（接收全部帧），委托给 clear_filter()
    if (count == 0 || filters == nullptr) {
        return clear_filter();
    }

    // 栈上数组，无堆分配；kMaxFilters 定义于类头文件（static constexpr）
    if (count > kMaxFilters) {
        spdlog::warn("[LinuxCanSocket] set_filters: {} filters requested, truncated to {}",
                     count,
                     kMaxFilters);
    }
    const size_t actual = std::min(count, kMaxFilters);
    can_filter raw_filters[kMaxFilters];
    for (size_t i = 0; i < actual; ++i) {
        raw_filters[i] = {filters[i].id, filters[i].mask};
    }

    int ret = ::setsockopt(fd,
                           SOL_CAN_RAW,
                           CAN_RAW_FILTER,
                           raw_filters,
                           static_cast<socklen_t>(actual * sizeof(can_filter)));

    if (ret == 0) {
        // 保存过滤器配置，供 recover() 重建 socket 后恢复使用
        {
            std::lock_guard<std::mutex> lk(filter_mutex_);
            saved_filter_count_ = actual;
            std::memcpy(saved_filters_, filters, actual * sizeof(hal::CanFilter));
        }
        last_error_.store(hal::CanResult::OK);
        return true;
    } else {
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }
}

bool LinuxCanSocket::clear_filter() {
    int fd = socket_fd_.load(std::memory_order_acquire);
    if (fd < 0 || !is_open()) {
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }
    int ret = ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
    if (ret == 0) {
        {
            std::lock_guard<std::mutex> lk(filter_mutex_);
            saved_filter_count_ = 0u;  // 过滤器已清空，同步清除保存的状态
        }
        last_error_.store(hal::CanResult::OK);
        return true;
    } else {
        last_error_.store(hal::CanResult::SYS_ERROR);
        return false;
    }
}

bool LinuxCanSocket::recover() {
    spdlog::info("[LinuxCanSocket] recovering Bus-Off on {}", interface_);

    // recover() 中在 close() 之前快照过滤器配置
    hal::CanFilter filters_snapshot[kMaxFilters];
    size_t count_snapshot;
    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        count_snapshot = saved_filter_count_;
        std::memcpy(filters_snapshot, saved_filters_, count_snapshot * sizeof(hal::CanFilter));
    }

    // Step 1: 关闭并释放旧 CAN socket
    close();

    // Step 2: 通过 SIOCSIFFLAGS 将 CAN 接口先 down 在 up，触发内核 CAN 控制器硬件复位。
    // 原因：用户态重新执行 socket()/bind() 只是创建了新的 socket 实例，
    // 并不会清除 CAN 硬件控制器的 Bus-Off 状态寄存器。
    // 需要 CAP_NET_ADMIN 权限（嵌入式设备通常以 root 运行）。
    //
    // 推荐配合内核 restart-ms 参数：
    //   ip link set can0 type can restart-ms 100
    // 当配置了 restart-ms 时，内核会在 Bus-Off 后自动复位，本函数的 down/up 只是备用手段。
    int ctl_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (ctl_fd >= 0) {
        ifreq ifr{};
        std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
        if (::ioctl(ctl_fd, SIOCGIFFLAGS, &ifr) == 0) {
            short saved_flags = ifr.ifr_flags;
            // bring down: 硬件控制器进入复位
            ifr.ifr_flags = static_cast<short>(saved_flags & ~IFF_UP);
            ::ioctl(ctl_fd, SIOCSIFFLAGS, &ifr);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            // bring up: 硬件控制器重新初始化
            ifr.ifr_flags = static_cast<short>(saved_flags | IFF_UP);
            ::ioctl(ctl_fd, SIOCSIFFLAGS, &ifr);
            // 等待内核重启 CAN 控制器
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            spdlog::warn("[LinuxCanSocket] recover: SIOCGIFFLAGS failed: {}", strerror(errno));
        }
        ::close(ctl_fd);
    } else {
        spdlog::warn("[LinuxCanSocket] recover: cannot open inet socket: {}", strerror(errno));
    }

    bus_off_.store(false);
    last_error_.store(hal::CanResult::OK);

    // Step 3: 重建 CAN socket 并恢复过滤器
    if (open()) {
        if (count_snapshot > 0u) {
            set_filters(filters_snapshot, count_snapshot);
        }
        spdlog::info("[LinuxCanSocket] Bus-Off recovery succeeded on {}", interface_);
        return true;
    }

    spdlog::error("[LinuxCanSocket] Bus-Off recovery failed on {}", interface_);
    last_error_.store(hal::CanResult::SYS_ERROR);
    return false;
}

}  // namespace robot::driver
