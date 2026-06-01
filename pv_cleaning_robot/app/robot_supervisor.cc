#include "pv_cleaning_robot/app/robot_supervisor.h"

#include <algorithm>
#include <optional>
#include <spdlog/spdlog.h>
#include <utility>

#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/recovery_motion.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::app {
namespace {

domain::DockMode configured_dock_mode(const robot::service::ConfigService& config) {
    return config.get<std::string>("robot.dock_mode", "single_dock") == "dual_dock"
               ? domain::DockMode::DualDock
               : domain::DockMode::SingleDock;
}

domain::LimitState limit_state(bool left_limit_active, bool right_limit_active) {
    return domain::LimitState{left_limit_active, right_limit_active};
}

}  // namespace

bool RobotSupervisor::can_trigger_spin_free_fault(RobotState state) {
    return state == RobotState::ExecutingMission;
}

RobotSupervisor::RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                                 service::ConfigService& config,
                                 std::shared_ptr<service::FaultReporter> fault,
                                 std::shared_ptr<service::NavService> nav)
    : fsm_(std::move(fsm))
    , config_(config)
    , fault_(std::move(fault))
    , nav_(std::move(nav)) {}

RuntimeConfig RobotSupervisor::start_runtime_config() const {
    return config_.has_pending_runtime_config() ? *config_.pending_runtime_config()
                                                : config_.active_runtime_config();
}

domain::PositionState RobotSupervisor::current_position_state() const {
    return position_state_query_ ? position_state_query_() : domain::PositionState::Unknown;
}

domain::LaneConfig RobotSupervisor::start_lane_config() const {
    const auto cfg = start_runtime_config();
    return domain::LaneConfig{configured_dock_mode(config_), cfg.primary_dock};
}

void RobotSupervisor::set_position_state_query(std::function<domain::PositionState()> query) {
    position_state_query_ = std::move(query);
}

void RobotSupervisor::set_motion_service(std::shared_ptr<service::MotionService> motion) {
    motion_ = std::move(motion);
}

void RobotSupervisor::set_recovery_motion(std::shared_ptr<service::RecoveryMotion> recovery) {
    recovery_ = std::move(recovery);
}

void RobotSupervisor::set_battery_soc_query(std::function<float()> query) {
    battery_soc_query_ = std::move(query);
}

domain::PositionState RobotSupervisor::active_position_state(bool left_limit_active,
                                                             bool right_limit_active) const {
    return domain::estimate_position(limit_state(left_limit_active, right_limit_active));
}

domain::PositionState RobotSupervisor::start_position_state(bool left_limit_active,
                                                            bool right_limit_active) const {
    return domain::estimate_position(limit_state(left_limit_active, right_limit_active));
}

RobotSupervisor::StartupPositionAssessment RobotSupervisor::handle_startup_position(
    bool left_limit_active,
    bool right_limit_active) {
    StartupPositionAssessment result;
    result.position_state = active_position_state(left_limit_active, right_limit_active);

    if (result.position_state == domain::PositionState::Inconsistent) {
        result.status_reason = "dual_endpoint_active";
    } else if (result.position_state == domain::PositionState::OnSegment ||
               result.position_state == domain::PositionState::Unknown) {
        result.status_reason = "robot_not_at_any_endpoint";
    } else if (!domain::can_start_configured_mission(start_lane_config(), result.position_state)) {
        result.status_reason = "robot_not_at_primary_dock";
    }
    return result;
}

void RobotSupervisor::handle_limit_settled(domain::Endpoint endpoint, float /*battery_soc*/) {
    handle_limit_settled(endpoint,
                         endpoint == domain::Endpoint::A,
                         endpoint == domain::Endpoint::B,
                         0.0f);
}

void RobotSupervisor::handle_limit_settled(domain::Endpoint endpoint,
                                           bool left_limit_active,
                                           bool right_limit_active,
                                           float /*battery_soc*/) {
    if (left_limit_active && right_limit_active) {
        spdlog::error("[RobotSupervisor] 左右限位同时有效，按 P0 收口");
        fault_->report(service::FaultEvent::Level::P0,
                       service::FaultCode::kConflictingLimitSides,
                       "conflicting_limit_sides");
        return;
    }

    const auto result = fsm_->settle_endpoint(endpoint);
    if (result.safety_fault) {
        spdlog::error("[RobotSupervisor] 非预期限位触发，按 P0 收口");
        fault_->report(service::FaultEvent::Level::P0,
                       service::FaultCode::kUnexpectedLimitSide,
                       "unexpected_limit_side");
        return;
    }
    dispatch_actions(result.actions);
}

void RobotSupervisor::register_limit_settled_bridge(
    middleware::EventBus& event_bus,
    std::function<std::pair<bool, bool>()> current_limit_levels,
    std::function<float()> current_battery_soc) {
    event_bus.subscribe<middleware::SafetyMonitor::LimitSettledEvent>(
        [this,
         current_limit_levels = std::move(current_limit_levels),
         current_battery_soc = std::move(current_battery_soc)](
            const middleware::SafetyMonitor::LimitSettledEvent& evt) {
            const auto [left_active, right_active] = current_limit_levels();
            handle_limit_settled(evt.endpoint, left_active, right_active, current_battery_soc());
        });
    event_bus.subscribe<middleware::SafetyMonitor::LimitUnstableEvent>(
        [this](const middleware::SafetyMonitor::LimitUnstableEvent& evt) {
            spdlog::error("[Limit] {} 端点限位触发已急停但未稳定，按 P0 收口",
                          evt.endpoint == domain::Endpoint::A ? "A" : "B");
            fault_->report(service::FaultEvent::Level::P0,
                           service::FaultCode::kLimitUnstableAfterEmergencyStop,
                           "limit_unstable_after_emergency_stop");
        });
}

void RobotSupervisor::register_scheduler_window(
    service::SchedulerService& scheduler,
    std::function<std::pair<bool, bool>()> current_limit_levels,
    std::function<float()> current_battery_soc) {
    scheduler.set_on_window_hit(
        [this,
         current_limit_levels = std::move(current_limit_levels),
         current_battery_soc = std::move(current_battery_soc)]() {
            const auto [left_limit_active, right_limit_active] = current_limit_levels();
            if (!handle_scheduler_window_hit(
                    left_limit_active, right_limit_active, current_battery_soc())) {
                spdlog::warn("[RobotSupervisor] 调度启动被拒绝");
            }
        });
}

RobotSupervisor::CommandResult RobotSupervisor::submit_command(
    const domain::RobotCommand& command) {
    if (command.kind == domain::RobotCommandKind::Stop) {
        return handle_stop_command(command);
    }
    if (command.kind == domain::RobotCommandKind::FaultReset) {
        return handle_fault_reset_command(command);
    }

    if (command.kind != domain::RobotCommandKind::StartConfiguredMission &&
        command.kind != domain::RobotCommandKind::CleanTowardOppositeEndpoint &&
        command.kind != domain::RobotCommandKind::CleanTowardPrimaryDock) {
        return {false, "command_not_supported"};
    }

    return handle_start_command(command);
}

RobotSupervisor::CommandResult RobotSupervisor::handle_stop_command(
    const domain::RobotCommand& command) {
    nav_->clear_spin_detection();
    const auto result = fsm_->stop(command.command_id);
    dispatch_actions(result.actions);
    return {result.accepted, result.reason};
}

RobotSupervisor::CommandResult RobotSupervisor::handle_fault_reset_command(
    const domain::RobotCommand& command) {
    const auto result = fsm_->reset_fault(command.command_id);
    dispatch_actions(result.actions);
    return {result.accepted, result.reason};
}

RobotSupervisor::CommandResult RobotSupervisor::handle_start_command(
    const domain::RobotCommand& command) {
    const auto lane = start_lane_config();
    const auto position_state = current_position_state();
    const auto validation = validate_start_command(command, lane, position_state);
    if (!validation.accepted || validation.reason == "already_at_target") {
        return validation;
    }

    if (config_.has_pending_runtime_config() && !config_.promote_pending_runtime_to_active()) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kRuntimeConfigPromoteFailed,
                       "runtime_config_promote_failed");
        return {false, "runtime_config_promote_failed"};
    }

    auto mission = build_start_mission(command, position_state);
    const auto result = fsm_->start(std::move(mission));
    return {result.accepted, result.reason};
}

RobotSupervisor::CommandResult RobotSupervisor::validate_start_command(
    const domain::RobotCommand& command,
    const domain::LaneConfig& lane,
    domain::PositionState position_state) {
    if (command.kind == domain::RobotCommandKind::StartConfiguredMission &&
        !domain::can_start_configured_mission(lane, position_state)) {
        return {false, "configured_mission_requires_start_endpoint"};
    }

    if (command.kind != domain::RobotCommandKind::StartConfiguredMission) {
        if (!domain::is_trusted_position_state(position_state)) {
            return {false,
                    position_state == domain::PositionState::Inconsistent
                        ? "robot_position_inconsistent"
                        : "robot_position_unknown"};
        }
        const auto target = command.kind == domain::RobotCommandKind::CleanTowardPrimaryDock
                                ? lane.primary_dock
                                : domain::opposite_endpoint(lane.primary_dock);
        if (domain::is_at_target(position_state, target)) {
            return {true, "already_at_target"};
        }
    }

    return {true, "start_allowed"};
}

domain::MissionContext RobotSupervisor::build_start_mission(
    const domain::RobotCommand& command,
    domain::PositionState position_state) {
    if (command.kind == domain::RobotCommandKind::StartConfiguredMission) {
        return domain::build_configured_mission_context(
            start_lane_config(),
            position_state,
            command.source,
            command.command_id,
            std::max<uint32_t>(1u, config_.active_runtime_config().repeat_count));
    }

    return domain::build_directional_clean_context(
        command.kind == domain::RobotCommandKind::CleanTowardPrimaryDock
            ? domain::MissionKind::CleanTowardPrimaryDock
            : domain::MissionKind::CleanTowardOppositeEndpoint,
        start_lane_config(),
        command.source,
        command.command_id);
}

RobotSupervisor::CommandResult RobotSupervisor::handle_self_check_passed() {
    if (fsm_->robot_state() == RobotState::SelfChecking && battery_soc_query_) {
        const auto runtime_cfg = config_.active_runtime_config();
        if (battery_soc_query_() < static_cast<float>(runtime_cfg.min_battery_soc)) {
            const auto result =
                fsm_->complete_self_check(false, false, "battery_below_start_threshold");
            dispatch_actions(result.actions);
            return {false, result.reason};
        }
    }

    const auto result = fsm_->complete_self_check(true, false, {});
    dispatch_actions(result.actions);
    return {result.accepted, result.reason};
}

void RobotSupervisor::handle_fault_event(const service::FaultEvent& event) {
    using Level = service::FaultEvent::Level;
    if (event.level == Level::P0) {
        const auto result = fsm_->apply_fault(
            FaultHandling{event.code, FaultResponse::Stop, event.description, std::nullopt});
        dispatch_actions(result.actions);
        return;
    }
    if (event.code == service::FaultCode::kTransientAttitudeError) {
        const auto result = fsm_->apply_fault(
            FaultHandling{event.code, FaultResponse::Recover, event.description, std::nullopt});
        dispatch_actions(result.actions);
        return;
    }
    if (event.level == Level::P1) {
        if (event.code == service::FaultCode::kBrushFaultReturnRequired) {
            const auto return_mission = domain::build_brush_off_return_context(
                start_lane_config(), domain::CommandSource::FaultPolicy, "fault_return_home");
            const auto result = fsm_->apply_fault(FaultHandling{
                event.code, FaultResponse::ReturnHome, event.description, return_mission});
            dispatch_actions(result.actions);
            return;
        }
        const auto result = fsm_->apply_fault(
            FaultHandling{event.code, FaultResponse::Ignore, event.description, std::nullopt});
        dispatch_actions(result.actions);
    }
}

void RobotSupervisor::dispatch_actions(const std::vector<RobotAction>& actions) {
    for (const auto& action : actions) {
        switch (action.kind) {
        case RobotActionKind::StartSegmentMotion:
            if (motion_ && action.segment.has_value()) {
                motion_->start_segment(*action.segment);
            }
            break;
        case RobotActionKind::StopMotion:
            if (motion_) {
                motion_->stop_cleaning();
            }
            break;
        case RobotActionKind::EmergencyStopMotion:
            if (motion_) {
                motion_->emergency_stop();
            }
            break;
        case RobotActionKind::StartRecoveryMotion:
            if (recovery_) {
                recovery_->start();
            }
            break;
        case RobotActionKind::ClearFault:
            if (fault_) {
                fault_->clear_active_fault();
            }
            break;
        default:
            break;
        }
    }
}

bool RobotSupervisor::handle_scheduler_window_hit(bool left_limit_active,
                                                  bool right_limit_active,
                                                  float battery_soc) {
    if (battery_soc < static_cast<float>(start_runtime_config().min_battery_soc)) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedLowBattery,
                       "start_rejected_low_battery");
        return false;
    }

    const auto position_state = start_position_state(left_limit_active, right_limit_active);
    if (position_state == domain::PositionState::Inconsistent) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedInvalidPosition,
                       "start_rejected_invalid_position");
        return false;
    }

    return submit_command(domain::RobotCommand{
        domain::RobotCommandKind::StartConfiguredMission,
        domain::CommandSource::Scheduler,
        "schedule"}).accepted;
}

void RobotSupervisor::tick_safety() {
    if (!can_trigger_spin_free_fault(fsm_->robot_state())) {
        return;
    }
    if (nav_->get_pose().spin_free_detected) {
        spdlog::error("[RobotSupervisor] 悬空检测触发，按 P0 急停");
        fault_->report(service::FaultService::FaultEvent::Level::P0,
                       service::FaultCode::kWheelSpinFree,
                       "wheel spin-free detected");
        nav_->clear_spin_detection();
    }
}

void RobotSupervisor::tick_recovery() {
    if (fsm_->robot_state() != RobotState::Recovering || !recovery_) {
        return;
    }

    const auto recovery_result = recovery_->step();
    if (recovery_result == service::RecoveryMotion::Result::Running) {
        return;
    }

    const auto result =
        recovery_result == service::RecoveryMotion::Result::Done
            ? fsm_->complete_recovery(true, {})
            : fsm_->complete_recovery(false, "recovery_failed");
    dispatch_actions(result.actions);
}

std::string RobotSupervisor::current_state() const {
    return fsm_->current_state();
}

RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.state = fsm_->current_state();
    if (fault_ && fault_->has_active_fault()) {
        snap.fault = fault_->last_fault().code;
    }
    snap.repeat_count = fsm_->repeat_count();
    snap.completed_cycles = fsm_->completed_cycles();
    snap.active_config = config_.active_runtime_config();
    snap.pending_config = config_.pending_runtime_config();
    if (snap.active_config) {
        snap.cfg_ver = config_.runtime_config_version(*snap.active_config);
    }
    return snap;
}

}  // namespace robot::app
