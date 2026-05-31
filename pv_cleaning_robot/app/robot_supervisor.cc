#include "pv_cleaning_robot/app/robot_supervisor.h"

#include <functional>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>

#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::app {

bool RobotSupervisor::is_new_task_start_state(const std::string& state) {
    return state == "Idle" || state == "Charging";
}

bool RobotSupervisor::is_cleaning_state(const std::string& state) {
    return state == "ExecutingSegment";
}

bool RobotSupervisor::is_return_allowed_state(const std::string& state) {
    return is_cleaning_state(state) || state == "Idle";
}

bool RobotSupervisor::can_trigger_spin_free_fault(const std::string& state) {
    return is_cleaning_state(state);
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
    // 新任务启动前只需要一种“若现在允许启动，将采用哪份配置”的视图。
    // pending 存在时优先读取 pending，避免调用点重复写同一段选择逻辑。
    return config_.has_pending_runtime_config() ? *config_.pending_runtime_config()
                                                : config_.active_runtime_config();
}

namespace {

bool dual_dock_mode_enabled(const robot::service::ConfigService& config) {
    return config.get<bool>("robot.dual_dock_mode", false);
}

}  // namespace

ParkingSideFacts RobotSupervisor::active_parking_facts(bool left_limit_active,
                                                       bool right_limit_active) const {
    return ParkingSideRuntime::from_physical_limits(
        config_.active_runtime_config().parking_side, left_limit_active, right_limit_active);
}

ParkingSideFacts RobotSupervisor::start_parking_facts(bool left_limit_active,
                                                      bool right_limit_active) const {
    return ParkingSideRuntime::from_physical_limits(
        start_runtime_config().parking_side, left_limit_active, right_limit_active);
}

RobotSupervisor::StartupPositionAssessment RobotSupervisor::handle_startup_position(
    bool left_limit_active,
    bool right_limit_active) {
    StartupPositionAssessment result;
    result.facts = active_parking_facts(left_limit_active, right_limit_active);

    if (result.facts.dual_endpoint_active) {
        result.status_reason = "dual_endpoint_active";
        return result;
    }
    if (result.facts.no_endpoint_active) {
        result.status_reason = "robot_not_at_any_endpoint";
        result.should_request_return = fsm_->current_state() == "Idle";
        if (result.should_request_return) {
            fsm_->dispatch(EvManualReturn{});
        }
        return result;
    }
    if (!result.facts.at_parking_side) {
        if (dual_dock_mode_enabled(config_) && result.facts.at_far_end) {
            return result;
        }
        result.status_reason = "robot_not_at_parking_side";
        result.should_request_return = fsm_->current_state() == "Idle";
        if (result.should_request_return) {
            fsm_->dispatch(EvManualReturn{});
        }
        return result;
    }
    return result;
}

void RobotSupervisor::handle_limit_settled(domain::PhysicalLimitSide side, float battery_soc) {
    handle_limit_settled(side,
                         side == domain::PhysicalLimitSide::Left,
                         side == domain::PhysicalLimitSide::Right,
                         battery_soc);
}

void RobotSupervisor::handle_limit_settled(domain::PhysicalLimitSide side,
                                           bool left_limit_active,
                                           bool right_limit_active,
                                           float battery_soc) {
    const auto active_cfg = config_.active_runtime_config();
    if (left_limit_active && right_limit_active) {
        spdlog::error("[RobotSupervisor] 左右限位同时有效，按 P0 收口");
        fault_->report(service::FaultEvent::Level::P0,
                       service::FaultCode::kConflictingLimitSides,
                       "conflicting_limit_sides");
        return;
    }

    const bool parking_left = active_cfg.parking_side == service::ParkingSide::Left;
    const bool parking_side_hit = (parking_left && side == domain::PhysicalLimitSide::Left) ||
                                  (!parking_left && side == domain::PhysicalLimitSide::Right);
    const auto segment_direction = fsm_->current_segment_direction();

    if (segment_direction.has_value()) {
        const bool wrong_parking_side_hit =
            parking_side_hit && *segment_direction == SegmentDirection::ToFarEnd;
        const bool wrong_far_end_hit =
            !parking_side_hit && *segment_direction == SegmentDirection::ToParkingSide;
        if (wrong_parking_side_hit || wrong_far_end_hit) {
            spdlog::error("[RobotSupervisor] 非预期限位触发 side={} direction={}，按 P0 收口",
                          side == domain::PhysicalLimitSide::Left ? "left" : "right",
                          *segment_direction == SegmentDirection::ToFarEnd ? "ToFarEnd"
                                                                           : "ToParkingSide");
            fault_->report(service::FaultEvent::Level::P0,
                           service::FaultCode::kUnexpectedLimitSide,
                           "unexpected_limit_side");
            return;
        }
    }

    if (parking_side_hit) {
        const bool should_charge =
            battery_soc < static_cast<float>(active_cfg.min_battery_soc);
        fsm_->dispatch(EvParkingSideLimitSettled{should_charge});
        return;
    }

    const bool should_charge = dual_dock_mode_enabled(config_) &&
                               battery_soc < static_cast<float>(active_cfg.min_battery_soc);
    fsm_->dispatch(EvFarEndLimitSettled{should_charge});
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
            spdlog::info("[Limit] {} 侧防抖完成，交由 Supervisor 解释业务语义",
                         evt.side == domain::PhysicalLimitSide::Left ? "left" : "right");
            const auto [left_active, right_active] = current_limit_levels();
            handle_limit_settled(evt.side, left_active, right_active, current_battery_soc());
        });
    event_bus.subscribe<middleware::SafetyMonitor::LimitUnstableEvent>(
        [this](const middleware::SafetyMonitor::LimitUnstableEvent& evt) {
            spdlog::error("[Limit] {} 侧限位触发已急停但未稳定，按 P0 收口",
                          evt.side == domain::PhysicalLimitSide::Left ? "left" : "right");
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

bool RobotSupervisor::start_task(bool at_parking_side, bool position_valid, float battery_soc) {
    const auto state = fsm_->current_state();
    const bool dual_dock_mode = dual_dock_mode_enabled(config_);
    if (!is_new_task_start_state(state)) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedBusy,
                       "start_rejected_busy");
        return false;
    }
    if (!position_valid || (!dual_dock_mode && !at_parking_side)) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedInvalidPosition,
                       "start_rejected_invalid_position");
        return false;
    }
    const auto runtime_cfg = start_runtime_config();
    if (battery_soc < static_cast<float>(runtime_cfg.min_battery_soc)) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedLowBattery,
                       "start_rejected_low_battery");
        return false;
    }
    if (config_.has_pending_runtime_config() && !config_.promote_pending_runtime_to_active()) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kRuntimeConfigPromoteFailed,
                       "runtime_config_promote_failed");
        return false;
    }
    EvScheduleStart start_evt;
    start_evt.at_parking_side = at_parking_side;
    start_evt.at_far_end = position_valid && !at_parking_side;
    start_evt.passes = static_cast<float>(config_.active_runtime_config().passes);
    start_evt.dual_dock_mode = dual_dock_mode;
    fsm_->dispatch(start_evt);
    return fsm_->current_state() == "ExecutingSegment";
}

bool RobotSupervisor::start_task_from_current_position(bool at_parking_side,
                                                       bool at_far_end,
                                                       bool position_valid,
                                                       float battery_soc) {
    const auto state = fsm_->current_state();
    if (!is_new_task_start_state(state) || !position_valid) {
        fault_->report(service::FaultEvent::Level::P2,
                       !is_new_task_start_state(state)
                           ? service::FaultCode::kStartRejectedBusy
                           : service::FaultCode::kStartRejectedInvalidPosition,
                       !is_new_task_start_state(state) ? "start_rejected_busy"
                                                       : "start_rejected_invalid_position");
        return false;
    }
    const auto runtime_cfg = start_runtime_config();
    if (battery_soc < static_cast<float>(runtime_cfg.min_battery_soc)) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kStartRejectedLowBattery,
                       "start_rejected_low_battery");
        return false;
    }
    if (config_.has_pending_runtime_config() && !config_.promote_pending_runtime_to_active()) {
        fault_->report(service::FaultEvent::Level::P2,
                       service::FaultCode::kRuntimeConfigPromoteFailed,
                       "runtime_config_promote_failed");
        return false;
    }
    EvRpcStartTask start_evt;
    start_evt.passes = static_cast<float>(config_.active_runtime_config().passes);
    start_evt.at_parking_side = at_parking_side;
    start_evt.at_far_end = at_far_end;
    start_evt.dual_dock_mode = dual_dock_mode_enabled(config_);
    fsm_->dispatch(start_evt);
    return fsm_->current_state() == "ExecutingSegment";
}

bool RobotSupervisor::stop_task() {
    const auto state = fsm_->current_state();
    if (!is_cleaning_state(state)) {
        return false;
    }
    nav_->clear_spin_detection();
    fsm_->dispatch(EvStopTask{});
    return fsm_->current_state() == "Idle";
}

bool RobotSupervisor::return_task(bool at_parking_side) {
    const auto state = fsm_->current_state();
    if (at_parking_side || !is_return_allowed_state(state)) {
        return false;
    }
    nav_->clear_spin_detection();
    fsm_->dispatch(EvManualReturn{});
    return fsm_->current_state() == "ExecutingSegment";
}

bool RobotSupervisor::handle_scheduler_window_hit(bool left_limit_active,
                                                  bool right_limit_active,
                                                  float battery_soc) {
    const auto facts = start_parking_facts(left_limit_active, right_limit_active);
    return start_task(facts.at_parking_side,
                      facts.is_valid_start_position(dual_dock_mode_enabled(config_)),
                      battery_soc);
}

void RobotSupervisor::tick_safety() {
    const auto state = fsm_->current_state();
    if (!can_trigger_spin_free_fault(state)) {
        return;
    }
    if (nav_->get_pose().spin_free_detected) {
        spdlog::error("[RobotSupervisor] 悬空检测触发——立即停机");
        fault_->report(service::FaultService::FaultEvent::Level::P0,
                       service::FaultCode::kWheelSpinFree,
                       "wheel spin-free detected");
        nav_->clear_spin_detection();
    }
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
    snap.target_passes = fsm_->target_passes();
    snap.completed_passes = fsm_->completed_passes();
    snap.clean_count = snap.completed_passes;
    snap.active_config = config_.active_runtime_config();
    snap.pending_config = config_.pending_runtime_config();
    if (snap.active_config) {
        snap.cfg_ver = config_.runtime_config_version(*snap.active_config);
    }
    return snap;
}

domain::RobotControlPort RobotSupervisor::make_control_port(
    std::shared_ptr<RobotSupervisor> supervisor) {
    return {
        [supervisor]() { return supervisor->current_state(); },
        [supervisor](bool at_parking_side,
                     bool at_far_end,
                     bool position_valid,
                     float battery_soc) {
            return supervisor->start_task_from_current_position(
                at_parking_side, at_far_end, position_valid, battery_soc);
        },
        [supervisor]() { return supervisor->stop_task(); },
        [supervisor](bool at_parking_side) { return supervisor->return_task(at_parking_side); },
        [supervisor]() { return supervisor->snapshot(); },
    };
}

}  // namespace robot::app
