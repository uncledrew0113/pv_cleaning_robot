#pragma once

#include <functional>
#include <initializer_list>
#include <string>

#include "pv_cleaning_robot/app/error_manager.h"

namespace robot::app {

struct RecoveryRequest {
    RecoveryPlanId plan{RecoveryPlanId::None};
    ComponentId component{};
};

struct RecoveryResult {
    bool ok{false};
    std::string reason;
};

/// @brief 执行 ErrorManager 选定的恢复流程。
///
/// RecoveryExecutor 属于应用层，只编排恢复流程，不决定错误优先级、不切换状态机，
/// 也不直接持有硬件对象。具体动作通过 Ports 注入，便于把“恢复策略”和“硬件操作”
/// 分开维护。
///
/// v1 的成功语义是“恢复流程已执行完成”。通信链路或设备是否真正恢复，由后续
/// DiagnosticsCollector 和 ErrorManager 周期性确认，避免在恢复动作中阻塞等待低频数据。
class RecoveryExecutor {
public:
    struct Ports {
        std::function<void()> pause_gps_stuck;
        std::function<void()> resume_gps_stuck;

        std::function<bool()> stop_walk;
        std::function<bool()> stop_walk_executor;
        std::function<bool()> restart_walk_driver;
        std::function<bool()> start_walk_executor;

        std::function<bool()> stop_brush;
        std::function<bool()> stop_brush_executor;
        std::function<bool()> restart_brush_driver;
        std::function<bool()> start_brush_executor;

        std::function<bool()> stop_bms_executor;
        std::function<bool()> restart_bms_driver;
        std::function<bool()> start_bms_executor;

        std::function<bool()> stop_gps_executor;
        std::function<bool()> restart_gps_driver;
        std::function<bool()> start_gps_executor;

        std::function<bool()> stop_imu_executor;
        std::function<bool()> restart_imu_driver;
        std::function<bool()> start_imu_executor;

        std::function<bool()> reverse_walk_motion;
        std::function<bool()> lower_attitude_center;
    };

    explicit RecoveryExecutor(Ports ports);

    RecoveryResult execute(const RecoveryRequest& request);

private:
    struct Step {
        const char* name;
        std::function<bool()>* fn;
    };

    RecoveryResult run_steps(std::initializer_list<Step> steps);
    RecoveryResult run_with_gps_stuck_paused(std::initializer_list<Step> steps);
    RecoveryResult recover_reverse_motion();
    RecoveryResult recover_attitude_center();
    RecoveryResult fail_if_missing(bool present, const char* name) const;
    RecoveryResult fail_step(const char* name) const;

    Ports ports_;
};

}  // namespace robot::app
