#pragma once

/// @file robot_fsm.h
/// @brief 机器人业务状态机的纯决策层。
///
/// FSM 不直接访问电机、配置、云端或文件系统。它只维护业务状态和
/// MissionContext，并返回 RobotAction 列表；动作由 RobotSupervisor 串行分发。
#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::app {

enum class RobotState {
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    Recovering,
    Charging,
    FaultStopped,
};

/// @brief FSM 输出动作。动作是“意图”，由 RobotSupervisor 串行调用服务层完成副作用。
enum class RobotActionKind {
    StartSegmentMotion,
    StartRecoveryMotion,
    StopMotion,
    EmergencyStopMotion,
    ClearFault,
};

struct RobotAction {
    RobotActionKind kind{RobotActionKind::StopMotion};
    std::string command_id;
    std::optional<domain::MissionSegment> segment;
};

struct FsmResult {
    bool accepted{false};
    bool safety_fault{false};
    std::string reason;
    std::vector<RobotAction> actions;

    bool has_action(RobotActionKind kind) const {
        return std::any_of(actions.begin(), actions.end(), [kind](const RobotAction& action) {
            return action.kind == kind;
        });
    }
};

enum class FaultResponse {
    Ignore,
    Recover,
    ReturnHome,
    Stop,
};

struct FaultHandling {
    uint32_t code{0};
    FaultResponse response{FaultResponse::Ignore};
    std::string reason;
    std::optional<domain::MissionContext> return_mission;
};

class RobotFsm {
public:
    RobotFsm();

    std::string current_state() const;
    RobotState robot_state() const;
    const std::optional<domain::MissionContext>& mission() const noexcept;
    uint32_t repeat_count() const;
    uint32_t completed_cycles() const;

    /// 进入启动前自检；真正的电量、姿态、配置检查由 Supervisor/Service 执行。
    FsmResult start(domain::MissionContext mission);
    /// 结束自检。通过则启动当前任务段；失败则回 Idle 或 FaultStopped。
    FsmResult complete_self_check(bool ok, bool fatal, const std::string& reason);
    /// 端点稳定确认。只在 ExecutingMission 中接受；异常端点会进入 FaultStopped。
    FsmResult settle_endpoint(domain::Endpoint endpoint, bool limit_consistent = true);
    /// 远程/人工停止。只允许 ExecutingMission 或 Recovering。
    FsmResult stop(const std::string& command_id);
    /// 应用故障策略。策略由 Supervisor/故障表决定，FSM 只执行状态变更。
    FsmResult apply_fault(const FaultHandling& fault);
    /// 结束恢复动作。成功恢复原任务段，失败进入 FaultStopped。
    FsmResult complete_recovery(bool ok, const std::string& reason);
    /// 云端故障复位。只允许 FaultStopped。
    FsmResult reset_fault(const std::string& command_id);

private:
    static const char* state_name(RobotState state) noexcept;
    FsmResult start_current_segment(const char* reason);
    FsmResult enter_fault_stopped(const char* reason,
                                  RobotActionKind action,
                                  const std::string& command_id = {},
                                  bool safety_fault = false);

    mutable hal::PiMutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
};

}  // namespace robot::app
