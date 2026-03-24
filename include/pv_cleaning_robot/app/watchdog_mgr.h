#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace robot::app {

/// @brief 软件看门狗管理器
///
/// 每个线程申请一个"票据"（ticket），必须在指定周期内调用 heartbeat()。
/// WatchdogMgr 主线程检测超时后调用 on_timeout 回调（触发 P0 故障或重启）。
/// 同时可选地向 /dev/watchdog 写入喂狗信号（硬件看门狗）。
class WatchdogMgr {
public:
    using TimeoutCallback = std::function<void(const std::string& thread_name)>;

    /// @param hw_watchdog_path  硬件看门狗设备路径（空=不启用）
    explicit WatchdogMgr(std::string hw_watchdog_path = "");
    ~WatchdogMgr();

    /// 启动看门狗监控线程
    bool start();
    void stop();

    /// 注册受监控线程
    /// @param name          线程标识名
    /// @param timeout_ms    最大允许心跳间隔（毫秒）
    /// @return ticket_id    调用 heartbeat() 需要传入此 ID
    int register_thread(const std::string& name, int timeout_ms);

    /// 线程汇报心跳（应在每次 update() 中调用）
    void heartbeat(int ticket_id);

    /// 设置超时回调
    void set_timeout_callback(TimeoutCallback cb);

private:
    void monitor_loop();
    void feed_hw_watchdog();

    struct Ticket {
        std::string name;
        int         timeout_ms;
        std::chrono::steady_clock::time_point last_beat;
        bool        expired{false};
    };

    std::string hw_watchdog_path_;
    int         hw_watchdog_fd_{-1};

    std::unordered_map<int, Ticket> tickets_;
    mutable std::mutex              tickets_mtx_;
    int                             next_ticket_id_{0};

    TimeoutCallback                 on_timeout_;
    std::atomic<bool>               running_{false};
    std::thread                     monitor_thread_;
};

} // namespace robot::app
