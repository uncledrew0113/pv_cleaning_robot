#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
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
    // 打开硬件看门狗（可选）
    if (!hw_watchdog_path_.empty()) {
        hw_watchdog_fd_ = ::open(hw_watchdog_path_.c_str(), O_WRONLY);
        if (hw_watchdog_fd_ >= 0) {
            int timeout = 30;  // 30 秒超时
            ::ioctl(hw_watchdog_fd_, WDIOC_SETTIMEOUT, &timeout);
        }
    }

    running_.store(true);
    monitor_thread_ = std::thread(&WatchdogMgr::monitor_loop, this);

    // 看门狗监控线程：SCHED_FIFO 50，绑定到 CPU 7（大核，用于管理任务）
    // 需要能抢占所有 SCHED_OTHER 线程以保证超时检测及时
    {
        sched_param sp{};
        sp.sched_priority = 50;
        pthread_setschedparam(monitor_thread_.native_handle(), SCHED_FIFO, &sp);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(7, &cpuset);
        pthread_setaffinity_np(monitor_thread_.native_handle(), sizeof(cpuset), &cpuset);
        pthread_setname_np(monitor_thread_.native_handle(), "watchdog_mon");
    }
    return true;
}

void WatchdogMgr::stop()
{
    running_.store(false);
    if (monitor_thread_.joinable()) monitor_thread_.join();

    if (hw_watchdog_fd_ >= 0) {
        // 发送 'V' 表示正常关闭（防止立即重启）
        ::write(hw_watchdog_fd_, "V", 1);
        ::close(hw_watchdog_fd_);
        hw_watchdog_fd_ = -1;
    }
}

int WatchdogMgr::register_thread(const std::string& name, int timeout_ms)
{
    std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
    int id = next_ticket_id_++;
    tickets_[id] = {name, timeout_ms, std::chrono::steady_clock::now(), false};
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

void WatchdogMgr::set_timeout_callback(TimeoutCallback cb)
{
    on_timeout_ = std::move(cb);
}

void WatchdogMgr::monitor_loop()
{
    while (running_.load()) {
        // 喂硬件看门狗
        feed_hw_watchdog();

        // 检查所有票据
        {
            std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
            auto now = std::chrono::steady_clock::now();
            for (auto& [id, ticket] : tickets_) {
                if (ticket.expired) continue;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - ticket.last_beat).count();
                if (elapsed > ticket.timeout_ms) {
                    ticket.expired = true;
                    if (on_timeout_) on_timeout_(ticket.name);
                }
            }
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
