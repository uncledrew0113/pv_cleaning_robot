#include "pv_cleaning_robot/service/scheduler_service.h"
#include <chrono>
#include <ctime>

namespace robot::service {

void SchedulerService::add_window(TimeWindow w)
{
    windows_.push_back(w);
}

void SchedulerService::clear_windows()
{
    windows_.clear();
    in_window_ = false;  // 清除后立即允许下次调度触发
}

void SchedulerService::set_on_task_start(TaskCallback cb)
{
    on_start_ = std::move(cb);
}

void SchedulerService::set_on_task_end(TaskCallback cb)
{
    on_end_ = std::move(cb);
}

void SchedulerService::tick()
{
    // 获取当前本地时间
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local;
    localtime_r(&t, &tm_local);

    bool in_window_now = false;
    for (auto& w : windows_) {
        if (tm_local.tm_hour == w.hour && tm_local.tm_min == w.minute) {
            in_window_now = true;
            break;
        }
    }

    // 窗口内且已超过最大时长则强制退出（防止任务持续到第二天）
    if (in_window_ && on_end_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - window_start_time_).count();
        if (elapsed >= kWindowDurationSec) {
            in_window_ = false;
            on_end_();
            return;
        }
    }

    if (in_window_now && !in_window_) {
        in_window_ = true;
        window_start_time_ = now;
        if (on_start_) on_start_();
    } else if (!in_window_now && in_window_) {
        // 检查是否超时退出已在上方处理，此处额外保护
        in_window_ = false;
    }
}

} // namespace robot::service
