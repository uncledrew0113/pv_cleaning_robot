/**
 * @file scheduler_service.cc
 * @brief 清扫任务时间窗口调度服务实现。
 *
 * 本文件按当前本地时间匹配运行配置中的调度窗口，并在首次命中时调用回调。是否接受启动请求
 * 由 RobotController 根据状态、电量和位置决定。
 */
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

std::vector<SchedulerService::TimeWindow> SchedulerService::snapshot_windows() const
{
    return windows_;
}

void SchedulerService::set_on_window_hit(TriggerCallback cb)
{
    on_hit_ = std::move(cb);
}

void SchedulerService::set_on_window_expired(TriggerCallback cb)
{
    on_expired_ = std::move(cb);
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

    // 窗口内且已超过最大时长则强制退出（防止窗口状态持续到第二天）
    if (in_window_ && on_expired_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - window_start_time_).count();
        if (elapsed >= kWindowDurationSec) {
            in_window_ = false;
            on_expired_();
            return;
        }
    }

    if (in_window_now && !in_window_) {
        in_window_ = true;
        window_start_time_ = now;
        if (on_hit_) on_hit_();
    } else if (!in_window_now && in_window_) {
        // 检查是否超时退出已在上方处理，此处仅清掉本轮窗口状态
        in_window_ = false;
    }
}

} // namespace robot::service
