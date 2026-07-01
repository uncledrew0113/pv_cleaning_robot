#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <cstring>
#include <spdlog/spdlog.h>
#include <chrono>

namespace robot::app {

WatchdogMgr::WatchdogMgr(std::string hw_watchdog_path)
    : hw_watchdog_path_(std::move(hw_watchdog_path))
{
}

WatchdogMgr::~WatchdogMgr()
{
    stop();
}

bool WatchdogMgr::start()
{
    // 打开硬件看门狗（可选）。软件看门狗始终可用，硬件路径为空时只做线程心跳监控。
    if (!hw_watchdog_path_.empty()) {
        hw_watchdog_fd_ = ::open(hw_watchdog_path_.c_str(), O_WRONLY);
        if (hw_watchdog_fd_ >= 0) {
            int timeout = 30;  // 硬件看门狗 30 秒超时。
            ::ioctl(hw_watchdog_fd_, WDIOC_SETTIMEOUT, &timeout);
        }
    }

    running_.store(true);
    monitor_thread_ = std::thread(&WatchdogMgr::monitor_loop, this);
    return true;
}

void WatchdogMgr::stop()
{
    running_.store(false);
    if (monitor_thread_.joinable()) monitor_thread_.join();

    if (hw_watchdog_fd_ >= 0) {
        // Linux watchdog magic close：写入 'V' 表示正常关闭，防止 stop() 后立即重启。
        ::write(hw_watchdog_fd_, "V", 1);
        ::close(hw_watchdog_fd_);
        hw_watchdog_fd_ = -1;
    }
}

int WatchdogMgr::register_thread(const std::string& name, int timeout_ms)
{
    std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
    int id = next_ticket_id_++;
    tickets_[id] = {name, timeout_ms, std::chrono::steady_clock::now(), false, false};
    return id;
}

void WatchdogMgr::heartbeat(int ticket_id)
{
    std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
    auto it = tickets_.find(ticket_id);
    if (it != tickets_.end()) {
        it->second.last_beat = std::chrono::steady_clock::now();
        it->second.expired   = false;
    }
}

bool WatchdogMgr::set_thread_paused(int ticket_id, bool paused)
{
    std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
    auto it = tickets_.find(ticket_id);
    if (it == tickets_.end()) {
        return false;
    }

    it->second.paused = paused;
    if (!paused) {
        // 恢复监控时重新计时，避免主动恢复刚结束就按旧 heartbeat 误报超时。
        it->second.last_beat = std::chrono::steady_clock::now();
        it->second.expired = false;
    }
    return true;
}

void WatchdogMgr::set_timeout_callback(TimeoutCallback cb)
{
    on_timeout_ = std::move(cb);
}

void WatchdogMgr::monitor_loop()
{
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    {
        sched_param sp{};
        sp.sched_priority = 50;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[WatchdogMgr] RT priority elevation failed: {}", strerror(rc));
        }
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(7, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[WatchdogMgr] CPU 7 affinity set failed: {}", strerror(errno));
        }
        pthread_setname_np(pthread_self(), "watchdog_mon");
    }
    while (running_.load()) {
        // 硬件看门狗心跳独立于软件 ticket；只要监控线程存活就持续喂狗。
        feed_hw_watchdog();

        // 锁内仅收集超时线程名，锁外触发回调；回调可能提交 ErrorManager 或访问其他服务，
        // 不能在 tickets_mtx_ 持锁期间执行。
        std::vector<std::string> expired_names;
        {
            std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
            auto now = std::chrono::steady_clock::now();
            for (auto& [id, ticket] : tickets_) {
                if (ticket.paused) continue;
                if (ticket.expired) continue;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - ticket.last_beat).count();
                if (elapsed > ticket.timeout_ms) {
                    ticket.expired = true;
                    expired_names.push_back(ticket.name);
                }
            }
        }
        for (auto& name : expired_names) {
            if (on_timeout_) on_timeout_(name);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void WatchdogMgr::feed_hw_watchdog()
{
    if (hw_watchdog_fd_ >= 0) {
        ::write(hw_watchdog_fd_, "1", 1);
    }
}

} // namespace robot::app
