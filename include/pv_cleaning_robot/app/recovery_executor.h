/**
 * @file recovery_executor.h
 * @brief 应用层恢复流程执行器接口。
 *
 * 本文件定义 ErrorManager 选定恢复计划后的执行入口。恢复执行器只编排动作顺序和端口完整性，
 * 不进行错误仲裁、不读写状态机，也不直接访问硬件设备。
 */
#pragma once

#include <functional>
#include <initializer_list>
#include <string>

#include "pv_cleaning_robot/app/error_manager.h"

namespace robot::app {

/// @brief 恢复请求，包含待执行计划和根因组件。
struct RecoveryRequest {
    RecoveryPlanId plan{RecoveryPlanId::None};
    ComponentId component{};
};

/// @brief 恢复执行结果；ok 表示流程是否被执行器判定为完成。
struct RecoveryResult {
    bool ok{false};
    std::string reason;
};

/// @brief 单个恢复动作的完成状态。
enum class RecoveryStepOutcome {
    Completed,
    TimedOut,
    InterruptedBySafetyOverride,
};

/// @brief 恢复动作端口返回给执行器的结果。
struct RecoveryStepResult {
    RecoveryStepOutcome outcome{RecoveryStepOutcome::TimedOut};
};

/// @brief 执行 ErrorManager 选定的恢复流程。
///
/// RecoveryExecutor 属于应用层，只编排恢复流程，不决定错误优先级、不切换状态机，
/// 也不直接持有硬件对象。具体动作通过 Ports 注入，便于把“恢复策略”和“硬件操作”
/// 分开维护。
///
/// 成功语义是“恢复流程已执行完成”。通信链路或设备是否真正恢复，由后续
/// DiagnosticsCollector 和 ErrorManager 周期性确认，避免在恢复动作中阻塞等待低频数据。
class RecoveryExecutor {
public:
    struct Ports {
        std::function<void()> pause_gps_stuck;
        std::function<void()> resume_gps_stuck;

        std::function<bool()> stop_walk;
        std::function<bool()> reverse_walk_motion;
        std::function<RecoveryStepResult()> lower_attitude_center;
    };

    explicit RecoveryExecutor(Ports ports);

    /// @brief 按请求执行恢复计划。
    ///
    /// @param request ErrorManager 产生的恢复请求。
    /// @return 恢复流程执行结果；缺失端口或未知计划返回失败。
    RecoveryResult execute(const RecoveryRequest& request);

private:
    struct Step {
        const char* name;
        std::function<bool()>* fn;
    };

    RecoveryResult recover_attitude_center();
    RecoveryResult recover_attitude_center_then_reverse();
    RecoveryResult fail_if_missing(bool present, const char* name) const;
    RecoveryResult fail_step(const char* name) const;

    Ports ports_;
};

}  // namespace robot::app
