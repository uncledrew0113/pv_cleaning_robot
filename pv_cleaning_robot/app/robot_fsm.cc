#include "pv_cleaning_robot/app/robot_fsm.h"

#include <mutex>
#include <utility>

namespace robot::app {
namespace {

RobotAction action(RobotActionKind kind,
                   const std::string& command_id = {},
                   std::optional<domain::MissionSegment> segment = std::nullopt) {
    return RobotAction{kind, command_id, std::move(segment)};
}

FsmResult accepted(const char* reason,
                   std::vector<RobotAction> actions = {},
                   bool safety_fault = false) {
    return FsmResult{true, safety_fault, reason ? reason : "", std::move(actions)};
}

FsmResult rejected(const char* reason) {
    return FsmResult{false, false, reason ? reason : "", {}};
}

const char* fallback_reason(const std::string& reason, const char* fallback) {
    return reason.empty() ? fallback : reason.c_str();
}

}  // namespace

RobotFsm::RobotFsm() = default;

const char* RobotFsm::state_name(RobotState state) noexcept {
    switch (state) {
    case RobotState::Idle:
        return "Idle";
    case RobotState::SelfChecking:
        return "SelfChecking";
    case RobotState::ExecutingMission:
        return "ExecutingMission";
    case RobotState::SettlingEndpoint:
        return "SettlingEndpoint";
    case RobotState::Recovering:
        return "Recovering";
    case RobotState::Charging:
        return "Charging";
    case RobotState::FaultStopped:
        return "FaultStopped";
    }
    return "Unknown";
}

std::string RobotFsm::current_state() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return state_name(state_);
}

RobotState RobotFsm::robot_state() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return state_;
}

const std::optional<domain::MissionContext>& RobotFsm::mission() const noexcept {
    return mission_;
}

uint32_t RobotFsm::repeat_count() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return mission_ ? mission_->repeat_count : 0u;
}

uint32_t RobotFsm::completed_cycles() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return mission_ ? mission_->completed_cycles : 0u;
}

FsmResult RobotFsm::start(domain::MissionContext mission) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ != RobotState::Idle) {
        return rejected("start_not_allowed");
    }

    mission_ = std::move(mission);
    state_ = RobotState::SelfChecking;
    return accepted("self_checking");
}

FsmResult RobotFsm::complete_self_check(bool ok, bool fatal, const std::string& reason) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ != RobotState::SelfChecking || !mission_) {
        return rejected("self_check_not_expected");
    }

    if (!ok) {
        mission_.reset();
        if (fatal) {
            return enter_fault_stopped(
                fallback_reason(reason, "self_check_fatal"),
                RobotActionKind::EmergencyStopMotion);
        }
        state_ = RobotState::Idle;
        return accepted(fallback_reason(reason, "self_check_failed"));
    }

    return start_current_segment("executing_mission");
}

FsmResult RobotFsm::settle_endpoint(domain::Endpoint endpoint, bool limit_consistent) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ == RobotState::Recovering) {
        return enter_fault_stopped(
            "endpoint_during_recovery", RobotActionKind::EmergencyStopMotion, {}, true);
    }
    if (state_ != RobotState::ExecutingMission) {
        return rejected("endpoint_not_expected");
    }
    if (!mission_) {
        return rejected("mission_missing");
    }

    const auto* segment = mission_->current_segment();
    if (!limit_consistent || segment == nullptr || segment->target != endpoint) {
        return enter_fault_stopped(limit_consistent ? "unexpected_endpoint"
                                                   : "endpoint_inconsistent",
                                   RobotActionKind::EmergencyStopMotion,
                                   {},
                                   true);
    }

    state_ = RobotState::SettlingEndpoint;
    std::vector<RobotAction> actions{action(RobotActionKind::StopMotion, mission_->command_id)};
    ++mission_->current_segment_index;

    if (mission_->current_segment_index < mission_->segments.size()) {
        const auto& next_segment = mission_->segments[mission_->current_segment_index];
        state_ = RobotState::ExecutingMission;
        actions.push_back(
            action(RobotActionKind::StartSegmentMotion, mission_->command_id, next_segment));
        return accepted("next_segment", std::move(actions));
    }

    ++mission_->completed_cycles;
    const bool repeat =
        mission_->kind == domain::MissionKind::ConfiguredMission &&
        mission_->completed_cycles < mission_->repeat_count && !mission_->segments.empty();
    if (repeat) {
        mission_->current_segment_index = 0;
        state_ = RobotState::ExecutingMission;
        actions.push_back(
            action(RobotActionKind::StartSegmentMotion, mission_->command_id, mission_->segments[0]));
        return accepted("next_cycle", std::move(actions));
    }

    mission_.reset();
    state_ = RobotState::Idle;
    return accepted("mission_done", std::move(actions));
}

FsmResult RobotFsm::stop(const std::string& command_id) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ != RobotState::ExecutingMission && state_ != RobotState::Recovering) {
        return rejected("stop_not_allowed");
    }

    const std::string id = !command_id.empty()
                               ? command_id
                               : (mission_ ? mission_->command_id : std::string{});
    mission_.reset();
    state_ = RobotState::Idle;
    return accepted("stopped", {action(RobotActionKind::StopMotion, id)});
}

FsmResult RobotFsm::apply_fault(const FaultHandling& fault) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    switch (fault.response) {
    case FaultResponse::Ignore:
        return accepted("fault_recorded");

    case FaultResponse::Recover:
        if (state_ != RobotState::ExecutingMission) {
            return rejected("recover_not_allowed");
        }
        if (!mission_ || mission_->current_segment() == nullptr) {
            return rejected("mission_missing");
        }
        state_ = RobotState::Recovering;
        return accepted(fallback_reason(fault.reason, "recovering"),
                        {action(RobotActionKind::StopMotion, mission_->command_id),
                         action(RobotActionKind::StartRecoveryMotion, mission_->command_id)});

    case FaultResponse::ReturnHome:
        if (state_ != RobotState::ExecutingMission) {
            return rejected("return_not_allowed");
        }
        if (!fault.return_mission ||
            fault.return_mission->current_segment() == nullptr) {
            return rejected("return_mission_missing");
        }
        mission_ = *fault.return_mission;
        state_ = RobotState::ExecutingMission;
        return accepted(
            fallback_reason(fault.reason, "returning_home"),
            {action(RobotActionKind::StopMotion, mission_->command_id),
             action(RobotActionKind::StartSegmentMotion,
                    mission_->command_id,
                    *mission_->current_segment())});

    case FaultResponse::Stop:
        return enter_fault_stopped(
            fallback_reason(fault.reason, "fault_stopped"),
            RobotActionKind::EmergencyStopMotion);
    }

    return rejected("unknown_fault_response");
}

FsmResult RobotFsm::complete_recovery(bool ok, const std::string& reason) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ != RobotState::Recovering) {
        return rejected("recovery_not_expected");
    }

    if (!ok || !mission_ || mission_->current_segment() == nullptr) {
        const std::string id = mission_ ? mission_->command_id : std::string{};
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return accepted(fallback_reason(reason, "recovery_failed"),
                        {action(RobotActionKind::StopMotion, id)});
    }

    return start_current_segment("recovery_succeeded");
}

FsmResult RobotFsm::reset_fault(const std::string& command_id) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (state_ != RobotState::FaultStopped) {
        return rejected("fault_reset_not_allowed");
    }

    mission_.reset();
    state_ = RobotState::Idle;
    return accepted("fault_reset", {action(RobotActionKind::ClearFault, command_id)});
}

FsmResult RobotFsm::start_current_segment(const char* reason) {
    const auto* segment = mission_ ? mission_->current_segment() : nullptr;
    if (segment == nullptr) {
        mission_.reset();
        state_ = RobotState::Idle;
        return rejected("empty_mission");
    }

    state_ = RobotState::ExecutingMission;
    return accepted(reason, {action(RobotActionKind::StartSegmentMotion,
                                    mission_->command_id,
                                    *segment)});
}

FsmResult RobotFsm::enter_fault_stopped(const char* reason,
                                        RobotActionKind action_kind,
                                        const std::string& command_id,
                                        bool safety_fault) {
    mission_.reset();
    state_ = RobotState::FaultStopped;
    return accepted(reason, {action(action_kind, command_id)}, safety_fault);
}

}  // namespace robot::app
