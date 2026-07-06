/**
 * @file scheduler_service.h
 * @brief 清扫任务时间窗口调度服务接口。
 *
 * SchedulerService 根据运行配置中的时间窗口触发启动事件。服务只发布“窗口命中”事实，
 * 不直接启动运动，任务接受与否由 RobotController 判定。
 */
#pragma once
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace robot::service {

/// @brief 时间窗口触发服务
///
/// 支持配置一个或多个触发窗口（HH:MM 开始时间）。
/// 到达窗口时，调用注册的命中回调；窗口持续超时后，调用注册的过期回调。
class SchedulerService {
public:
    struct TimeWindow {
        int hour{8};    ///< 24h 制
        int minute{0};
    };

    using TriggerCallback = std::function<void()>;

    void add_window(TimeWindow w);
    void clear_windows();  ///< 清除所有时间窗口（远程配置更新时使用）
    std::vector<TimeWindow> snapshot_windows() const;
    void set_on_window_hit(TriggerCallback cb);
    void set_on_window_expired(TriggerCallback cb);

    /// 检查是否进入已配置的时间窗口（由主循环定期调用，main.cc 以 100ms 间隔调用，即 ~10Hz）。
    /// 内部计时器精度取决于调用间隔；建议调用间隔 ≤ 1000ms。
    void tick();

private:
    std::vector<TimeWindow> windows_;
    TriggerCallback         on_hit_;
    TriggerCallback         on_expired_;
    bool                    in_window_{false};
    std::chrono::system_clock::time_point window_start_time_;
    static constexpr int kWindowDurationSec = 3600;  ///< 默认窗口有效期 1 小时
};

} // namespace robot::service
