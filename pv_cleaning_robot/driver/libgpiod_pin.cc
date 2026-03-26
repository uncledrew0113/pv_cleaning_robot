#include <cstring>
#include <gpiod.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "pv_cleaning_robot/driver/libgpiod_pin.h"

namespace robot::driver {

LibGpiodPin::LibGpiodPin(std::string chip_name, unsigned int line_num, std::string consumer)
    : chip_name_(std::move(chip_name))
    , line_num_(line_num)
    , consumer_(std::move(consumer))
    , last_event_time_(std::chrono::steady_clock::now()) {}

LibGpiodPin::~LibGpiodPin() {
    close();  // close 现在安全地包含了 stop_monitoring
}

bool LibGpiodPin::request_line_as_input() {
    struct gpiod_line_request_config cfg {};
    cfg.consumer = consumer_.c_str();
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;

    if (config_.bias == hal::GpioBias::PULL_UP) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (config_.bias == hal::GpioBias::PULL_DOWN) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
        spdlog::error("[LibGpiodPin] input line request failed for {}/{}: {}",
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        return false;
    }
    return true;
}

bool LibGpiodPin::open(const hal::GpioConfig& config) {
    std::unique_lock<std::shared_mutex> lock(io_mutex_);
    // 允许以新配置重新打开：先安全释放旧资源，再重新申请
    if (chip_ && line_) {
        stop_monitoring_locked();
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
    }

    config_ = config;

    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
        spdlog::error("[LibGpiodPin] gpiod_chip_open_by_name({}) failed", chip_name_);
        return false;
    }

    line_ = gpiod_chip_get_line(chip_, line_num_);
    if (!line_) {
        spdlog::error("[LibGpiodPin] gpiod_chip_get_line({}) failed", line_num_);
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        return false;
    }

    bool success = false;
    if (config_.direction == hal::GpioDirection::OUTPUT) {
        success = (gpiod_line_request_output(line_, consumer_.c_str(), 0) == 0);
        if (!success) {
            spdlog::error("[LibGpiodPin] output line request failed for {}/{}: {}",
                          chip_name_,
                          line_num_,
                          std::strerror(errno));
        }
    } else {
        success = request_line_as_input();
    }

    if (!success) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        return false;
    }

    spdlog::info("[LibGpiodPin] opened {}/{} as {} (Bias: {}, Debounce: {}ms)",
                 chip_name_,
                 line_num_,
                 config_.direction == hal::GpioDirection::OUTPUT ? "OUTPUT" : "INPUT",
                 static_cast<int>(config_.bias),
                 config_.debounce_ms);
    return true;
}

void LibGpiodPin::stop_monitoring_locked() {
    bool was_running = running_.exchange(false);
    if (!was_running)
        return;

    // 写入 EventFD，瞬间唤醒正在死等 (-1) 的 poll()，拒绝 RT 延迟
    if (cancel_fd_ >= 0) {
        uint64_t val = 1;
        ::write(cancel_fd_, &val, sizeof(val));
    }

    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }

    if (cancel_fd_ >= 0) {
        ::close(cancel_fd_);
        cancel_fd_ = -1;
    }

    if (chip_ && line_) {
        gpiod_line_release(line_);
        request_line_as_input();
    }

    spdlog::debug("[LibGpiodPin] monitoring stopped: {}/{}", chip_name_, line_num_);
}

void LibGpiodPin::close() {
    std::unique_lock<std::shared_mutex> lock(io_mutex_);
    stop_monitoring_locked();

    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        spdlog::debug("[LibGpiodPin] closed: {}/{}", chip_name_, line_num_);
    }
}

bool LibGpiodPin::is_open() const {
    std::shared_lock<std::shared_mutex> lock(io_mutex_);
    return chip_ != nullptr && line_ != nullptr;
}

bool LibGpiodPin::read_value() {
    std::shared_lock<std::shared_mutex> lock(io_mutex_);
    if (!chip_ || !line_)
        return false;
    return gpiod_line_get_value(line_) == 1;
}

bool LibGpiodPin::write_value(bool high) {
    std::shared_lock<std::shared_mutex> lock(io_mutex_);
    if (!chip_ || !line_ || config_.direction != hal::GpioDirection::OUTPUT) {
        spdlog::warn("[LibGpiodPin] write_value invalid state on {}/{}", chip_name_, line_num_);
        return false;
    }
    if (gpiod_line_set_value(line_, high ? 1 : 0) != 0) {
        spdlog::error("[LibGpiodPin] write_value({}) failed for {}/{}: {}",
                      high,
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        return false;
    }
    return true;
}

/// @brief 设置边缘触发回调函数。
/// @warning 【死锁警告】严禁在回调函数内部调用本对象的 read_value()、
/// write_value()、close() 或 stop_monitoring()，否则将引发永久死锁。
/// 建议回调函数只做最轻量的原子变量标记，或通过 EventBus 将事件抛给外层处理。

void LibGpiodPin::set_edge_callback(hal::GpioEdge edge, std::function<void()> cb) {
    std::lock_guard<hal::PiMutex> lock(cb_mutex_);
    edge_ = edge;
    callback_ = std::move(cb);
}

void LibGpiodPin::start_monitoring() {
    std::unique_lock<std::shared_mutex> lock(io_mutex_);
    if (!chip_ || !line_)
        return;

    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true))
        return;

    if (config_.direction == hal::GpioDirection::OUTPUT) {
        spdlog::warn(
            "[LibGpiodPin] start_monitoring() on OUTPUT pin {}/{}, ignored", chip_name_, line_num_);
        running_.store(false);
        return;
    }

    gpiod_line_release(line_);

    hal::GpioEdge snapshot_edge;
    {
        std::lock_guard<hal::PiMutex> cb_lock(cb_mutex_);
        snapshot_edge = edge_;
    }

    struct gpiod_line_request_config cfg {};
    cfg.consumer = consumer_.c_str();

    switch (snapshot_edge) {
        case hal::GpioEdge::RISING:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE;
            break;
        case hal::GpioEdge::FALLING:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
            break;
        case hal::GpioEdge::BOTH:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
            break;
    }

    if (config_.bias == hal::GpioBias::PULL_UP) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (config_.bias == hal::GpioBias::PULL_DOWN) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
        spdlog::error("[LibGpiodPin] request edge events failed for {}/{}: {}",
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        request_line_as_input();
        running_.store(false);
        return;
    }

    // 创建 EventFD 以备退出打断之用
    cancel_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (cancel_fd_ < 0) {
        spdlog::error("[LibGpiodPin] Failed to create eventfd for {}/{}: {}",
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        request_line_as_input();
        running_.store(false);
        return;
    }

    last_event_time_ = std::chrono::steady_clock::now();
    monitor_thread_ = std::thread(&LibGpiodPin::monitor_loop, this);
    spdlog::debug("[LibGpiodPin] monitoring started: {}/{}", chip_name_, line_num_);
}

void LibGpiodPin::stop_monitoring() {
    std::unique_lock<std::shared_mutex> lock(io_mutex_);
    stop_monitoring_locked();
}

void LibGpiodPin::monitor_loop() {
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    // 确保后续所有代码和硬件响应均在 RT 级别
    if (config_.rt_priority > 0) {
        sched_param sp{};
        sp.sched_priority = config_.rt_priority;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[LibGpiodPin] RT priority elevation failed for {}/{}: {}",
                         chip_name_,
                         line_num_,
                         strerror(rc));
        } else {
            spdlog::info("[LibGpiodPin] {}/{} thread elevated to RT SCHED_FIFO priority {}",
                         chip_name_,
                         line_num_,
                         config_.rt_priority);
        }
    }
    if (config_.cpu_affinity != 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (int i = 0; i < 64; ++i) {
            if (config_.cpu_affinity & (1 << i))
                CPU_SET(i, &cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[LibGpiodPin] CPU affinity set failed for {}/{}: {}",
                         chip_name_, line_num_, strerror(errno));
        }
    }

    // 取到底层的 GPIO 中断描述符
    int gpio_fd = gpiod_line_event_get_fd(line_);
    if (gpio_fd < 0 || cancel_fd_ < 0) {
        spdlog::error("[LibGpiodPin] Invalid file descriptors for {}/{}", chip_name_, line_num_);
        return;
    }

    pollfd pfds[2]{};
    pfds[0].fd = gpio_fd;
    pfds[0].events = POLLIN;
    pfds[1].fd = cancel_fd_;
    pfds[1].events = POLLIN;

    while (running_.load()) {
        // -1 永久阻塞，让该核心彻底休眠。硬件中断或退出信号一到，立马唤醒。
        int ret = ::poll(pfds, 2, -1);
        if (ret <= 0)
            continue;

        // 收到来自 stop_monitoring 的打断信号，安全切出循环
        if (pfds[1].revents & POLLIN) {
            break;
        }

        if (pfds[0].revents & POLLIN) {
            gpiod_line_event event{};
            if (gpiod_line_event_read(line_, &event) != 0)
                continue;

            // 软件消抖 (完全线程隔离，无锁开销)
            if (config_.debounce_ms > 0) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_event_time_)
                        .count();
                if (elapsed_ms < config_.debounce_ms)
                    continue;
                last_event_time_ = now;
            }

            // 安全调用回调：捕获 EOWNERDEAD 导致的互斥锁异常，避免程序崩溃
            try {
                std::lock_guard<hal::PiMutex> lock(cb_mutex_);
                if (callback_) {
                    callback_();
                }
            } catch (const std::exception& e) {
                spdlog::error(
                    "[LibGpiodPin] Callback skipped due to mutex state error on {}/{}: {}",
                    chip_name_,
                    line_num_,
                    e.what());
            }
        }
    }
}

}  // namespace robot::driver
