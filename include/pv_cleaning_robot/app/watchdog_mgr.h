/**
 * @file watchdog_mgr.h
 * @brief 软件线程看门狗与可选硬件看门狗接口。
 *
 * WatchdogMgr 维护各线程心跳 ticket，超时后通过回调上报线程名。错误归因和故障处理由
 * ErrorManager 完成；硬件看门狗路径可选，用于进程级失活后的系统复位保护。
 */
#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include <unordered_map>

namespace robot::app {

/// @brief 软件/硬件看门狗管理器。
///
/// 每个受监控线程注册一个 ticket，并在自己的 update() 路径中周期性调用 heartbeat()。
/// WatchdogMgr 检测到超时后只上报线程名，具体错误归因和恢复流程由 ErrorManager 统一处理。
/// 同时可选地向 /dev/watchdog 写入硬件看门狗心跳。
class WatchdogMgr {
public:
    using TimeoutCallback = std::function<void(const std::string& thread_name)>;

    /// @param hw_watchdog_path 硬件看门狗设备路径，空字符串表示不启用硬件看门狗。
    explicit WatchdogMgr(std::string hw_watchdog_path = "");
    ~WatchdogMgr();

    /// @brief 启动看门狗监控线程。
    bool start();

    /// @brief 停止监控线程，并在启用硬件看门狗时执行 magic close。
    void stop();

    /// @brief 注册受监控线程。
    /// @param name 线程标识名，ErrorManager 依赖该名称映射错误组件。
    /// @param timeout_ms 最大允许心跳间隔（毫秒）。
    /// @return ticket_id 调用 heartbeat() 和 set_thread_paused() 需要传入此 ID。
    int register_thread(const std::string& name, int timeout_ms);

    /// @brief 线程汇报心跳；通常在受监控线程每轮 update() 成功推进后调用。
    void heartbeat(int ticket_id);

    /// @brief 暂停或恢复某个 ticket 的超时检测。
    ///
    /// 恢复流程会主动停止 ThreadExecutor，此时没有 heartbeat 是预期行为；
    /// 使用该接口避免把“主动停线程”误判成线程卡死。
    bool set_thread_paused(int ticket_id, bool paused);

    /// @brief 设置超时回调；回调在监控锁外执行，允许内部提交 ErrorManager 事件。
    void set_timeout_callback(TimeoutCallback cb);

private:
    void monitor_loop();
    void feed_hw_watchdog();

    struct Ticket {
        std::string name;
        int         timeout_ms;
        std::chrono::steady_clock::time_point last_beat;
        bool        expired{false};
        bool        paused{false};
    };

    std::string hw_watchdog_path_;
    int         hw_watchdog_fd_{-1};

    std::unordered_map<int, Ticket> tickets_;
    mutable hal::PiMutex            tickets_mtx_;
    int                             next_ticket_id_{0};

    TimeoutCallback                 on_timeout_;
    std::atomic<bool>               running_{false};
    std::thread                     monitor_thread_;
};

} // namespace robot::app
