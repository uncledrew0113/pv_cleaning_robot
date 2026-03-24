#pragma once
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace robot::service {

/// @brief 清扫任务调度服务（时间窗口调度）
///
/// 支持配置一个或多个"清扫窗口"（HH:MM 开始时间）。
/// 到达时间窗口时，调用注册的 on_task_start 回调触发 FSM 开始清扫。
class SchedulerService {
public:
    struct TimeWindow {
        int hour{8};    ///< 24h 制
        int minute{0};
    };

    using TaskCallback = std::function<void()>;

    void add_window(TimeWindow w);
    void set_on_task_start(TaskCallback cb);
    void set_on_task_end(TaskCallback cb);

    /// 检查是否进入调度窗口（应由 1Hz 循环调用）
    void tick();

private:
    std::vector<TimeWindow> windows_;
    TaskCallback            on_start_;
    TaskCallback            on_end_;
    bool                    in_window_{false};
    std::chrono::system_clock::time_point window_start_time_;
    static constexpr int kWindowDurationSec = 3600;  ///< 默认单次清扫窗口 1 小时
};

} // namespace robot::service
