#pragma once
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include <atomic>

namespace robot::app {

/// @brief 单次清扫任务（行程追踪 + 暂停/恢复）
///
/// 由 RobotFsm 的 CleanFwd/CleanReturn 状态持有，
/// 追踪当前趟次和里程计，判断是否到达轨道端头。
class CleanTask {
public:
    struct Config {
        float track_length_m{1000.0f};  ///< 轨道全长（米）
        int   passes{1};                ///< 清扫趟数（单程/往复）
    };

    CleanTask(std::shared_ptr<service::MotionService> motion,
              std::shared_ptr<service::NavService>    nav,
              Config                                  cfg);

    /// 开始或恢复任务
    bool start();

    /// 暂停（不停电机，仅挂起里程追踪）
    void pause();

    /// 停止并重置
    void stop();

    /// 检查当前方向是否到达端头（由 FSM 轮询调用）
    bool reached_end() const;

    /// 检查是否完成全部趟数
    bool all_passes_done() const;

    /// 切换方向（下一趟）
    void switch_direction();

    int current_pass() const { return pass_count_; }
    bool is_paused() const   { return paused_; }

private:
    std::shared_ptr<service::MotionService> motion_;
    std::shared_ptr<service::NavService>    nav_;
    Config                                  cfg_;

    std::atomic<bool> paused_{true};
    int               pass_count_{0};
    float             pass_start_distance_{0.0f};
    bool              going_forward_{true};
};

} // namespace robot::app
