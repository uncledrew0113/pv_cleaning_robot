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
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::app {

bool RobotSupervisor::is_new_task_start_state(const std::string& state) {
    return state == "Idle" || state == "Charging" || state == "Stopped";
}

bool RobotSupervisor::is_cleaning_state(const std::string& state) {
    return state == "CleanFwd" || state == "CleanReturn";
}

bool RobotSupervisor::is_return_allowed_state(const std::string& state) {
    return is_cleaning_state(state) || state == "Stopped" || state == "Idle";
}

bool RobotSupervisor::can_trigger_spin_free_fault(const std::string& state) {
    return is_cleaning_state(state) || state == "Returning";
}

RobotSupervisor::RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                                 service::ConfigService& config,
                                 std::shared_ptr<service::CommandTracker> command_tracker,
                                 std::shared_ptr<service::FaultService> fault,
                                 std::shared_ptr<service::NavService> nav)
    : fsm_(std::move(fsm))
    , config_(config)
    , command_tracker_(std::move(command_tracker))
    , fault_(std::move(fault))
    , nav_(std::move(nav)) {}

service::RuntimeConfig RobotSupervisor::start_runtime_config() const {
    // 新任务启动前只需要一种“若现在允许启动，将采用哪份配置”的视图。
    // pending 存在时优先读取 pending，避免调用点重复写同一段选择逻辑。
    return config_.has_pending_runtime_config() ? *config_.pending_runtime_config()
                                                : config_.active_runtime_config();
}

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
        result.status_reason = "robot_not_at_parking_side";
        return result;
    }
    return result;
}

void RobotSupervisor::handle_limit_settled(device::LimitSide side, float battery_soc) {
    const auto active_cfg = config_.active_runtime_config();
    const bool parking_left = active_cfg.parking_side == service::ParkingSide::Left;
    const bool parking_side_hit =
        (parking_left && side == device::LimitSide::LEFT) ||
        (!parking_left && side == device::LimitSide::RIGHT);

    if (parking_side_hit) {
        const bool should_charge =
            battery_soc < static_cast<float>(active_cfg.charge_start_soc);
        fsm_->dispatch(EvParkingSideLimitSettled{should_charge});
        return;
    }

    fsm_->dispatch(EvFarEndLimitSettled{});
}

void RobotSupervisor::register_limit_settled_bridge(
    middleware::EventBus& event_bus,
    std::function<float()> current_battery_soc) {
    event_bus.subscribe<middleware::SafetyMonitor::LimitSettledEvent>(
        [this, current_battery_soc = std::move(current_battery_soc)](
            const middleware::SafetyMonitor::LimitSettledEvent& evt) {
            spdlog::info("[Limit] {} 侧防抖完成，交由 Supervisor 解释业务语义",
                         evt.side == device::LimitSide::LEFT ? "left" : "right");
            handle_limit_settled(evt.side, current_battery_soc());
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
    if (!is_new_task_start_state(state) || !position_valid || !at_parking_side) {
        return false;
    }
    const auto runtime_cfg = start_runtime_config();
    if (battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)) {
        return false;
    }
    if (config_.has_pending_runtime_config() && !config_.promote_pending_runtime_to_active()) {
        return false;
    }
    EvScheduleStart start_evt;
    start_evt.at_parking_side = at_parking_side;
    start_evt.passes = static_cast<float>(config_.active_runtime_config().passes);
    fsm_->dispatch(start_evt);
    return fsm_->current_state() == "CleanFwd" || fsm_->current_state() == "CleanReturn";
}

bool RobotSupervisor::start_task_from_current_position(bool position_valid, float battery_soc) {
    const auto state = fsm_->current_state();
    if (!is_new_task_start_state(state) || !position_valid) {
        return false;
    }
    const auto runtime_cfg = start_runtime_config();
    if (battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)) {
        return false;
    }
    if (config_.has_pending_runtime_config() && !config_.promote_pending_runtime_to_active()) {
        return false;
    }
    EvRpcStartTask start_evt;
    start_evt.passes = static_cast<float>(config_.active_runtime_config().passes);
    fsm_->dispatch(start_evt);
    return fsm_->current_state() == "CleanFwd" || fsm_->current_state() == "CleanReturn";
}

bool RobotSupervisor::stop_task() {
    const auto state = fsm_->current_state();
    if (!is_cleaning_state(state) && state != "Returning") {
        return false;
    }
    nav_->clear_spin_detection();
    fsm_->dispatch(EvStopTask{});
    return fsm_->current_state() == "Stopped";
}

bool RobotSupervisor::return_task(bool at_parking_side) {
    const auto state = fsm_->current_state();
    if (at_parking_side || !is_return_allowed_state(state)) {
        return false;
    }
    nav_->clear_spin_detection();
    fsm_->dispatch(EvManualReturn{});
    return fsm_->current_state() == "Returning";
}

bool RobotSupervisor::handle_scheduler_window_hit(bool left_limit_active,
                                                  bool right_limit_active,
                                                  float battery_soc) {
    const auto facts = start_parking_facts(left_limit_active, right_limit_active);
    return start_task(facts.at_parking_side, facts.is_valid_start_position(), battery_soc);
}

void RobotSupervisor::tick_safety() {
    const auto state = fsm_->current_state();
    if (!can_trigger_spin_free_fault(state)) {
        return;
    }
    if (nav_->get_pose().spin_free_detected) {
        spdlog::error("[RobotSupervisor] 悬空检测触发——立即停机");
        fault_->report(service::FaultService::FaultEvent::Level::P0,
                       0x0002,
                       "wheel spin-free detected");
        nav_->clear_spin_detection();
    }
}

std::string RobotSupervisor::current_state() const {
    return fsm_->current_state();
}

RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.device_state = fsm_->current_state();
    snap.task_state = task_state_from_device_state(snap.device_state);
    snap.target_passes = fsm_->target_passes();
    snap.completed_passes = fsm_->completed_passes();
    snap.clean_count = snap.completed_passes;
    snap.active_config = config_.active_runtime_config();
    snap.pending_config = config_.pending_runtime_config();
    if (snap.active_config) {
        snap.active_config_version = config_.runtime_config_version(*snap.active_config);
    }
    snap.active_command = command_tracker_->active();
    snap.last_command = command_tracker_->last_completed();
    return snap;
}

std::string RobotSupervisor::task_state_from_device_state(const std::string& device_state) {
    if (device_state == "CleanFwd" || device_state == "CleanReturn")
        return "RunningTask";
    if (device_state == "Returning")
        return "ReturningTask";
    if (device_state == "Charging")
        return "ChargingTask";
    if (device_state == "Fault")
        return "FaultedTask";
    if (device_state == "Stopped")
        return "StoppedTask";
    return "IdleTask";
}

}  // namespace robot::app
