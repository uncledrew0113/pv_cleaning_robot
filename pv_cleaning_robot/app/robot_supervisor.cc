#include "pv_cleaning_robot/app/robot_supervisor.h"

#include <functional>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>

#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::app {

namespace {

bool is_new_task_start_state(const std::string& state) {
    return state == "Idle" || state == "Charging";
}

bool is_cleaning_state(const std::string& state) {
    return state == "CleanFwd" || state == "CleanReturn";
}

bool can_trigger_low_battery_return(const std::string& state) {
    return state != "Idle" && state != "Paused" && state != "Returning" &&
           state != "Charging" && state != "Fault" && state != "Terminated";
}

bool can_trigger_spin_free_fault(const std::string& state) {
    return state != "Idle" && state != "Charging" && state != "Fault" &&
           state != "Paused" && state != "Terminated";
}

}  // namespace

RobotSupervisor::RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                                 std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg,
                                 std::shared_ptr<service::CommandTracker> command_tracker,
                                 std::shared_ptr<service::FaultService> fault,
                                 std::shared_ptr<service::NavService> nav)
    : fsm_(std::move(fsm))
    , tb_cfg_(std::move(tb_cfg))
    , command_tracker_(std::move(command_tracker))
    , fault_(std::move(fault))
    , nav_(std::move(nav)) {}

bool RobotSupervisor::start_task(bool at_parking_side) {
    if (!is_new_task_start_state(fsm_->current_state()) || !at_parking_side) {
        return false;
    }
    if (tb_cfg_->has_pending_config() && !tb_cfg_->promote_pending_to_active()) {
        return false;
    }
    EvScheduleStart start_evt;
    start_evt.at_parking_side = at_parking_side;
    start_evt.passes = static_cast<float>(tb_cfg_->active_config().passes);
    fsm_->dispatch(start_evt);
    return fsm_->current_state() == "CleanFwd" || fsm_->current_state() == "CleanReturn";
}

bool RobotSupervisor::resume_paused_task() {
    if (fsm_->current_state() != "Paused") {
        return false;
    }
    fsm_->dispatch(EvResumeTask{});
    return fsm_->current_state() == "CleanFwd" || fsm_->current_state() == "CleanReturn";
}

bool RobotSupervisor::pause_task() {
    if (!is_cleaning_state(fsm_->current_state())) {
        return false;
    }
    fsm_->dispatch(EvPauseTask{});
    return fsm_->current_state() == "Paused";
}

bool RobotSupervisor::return_task() {
    const auto state = fsm_->current_state();
    if (!is_cleaning_state(state) && state != "Paused") {
        return false;
    }
    fsm_->dispatch(EvManualReturn{});
    return fsm_->current_state() == "Returning";
}

bool RobotSupervisor::terminate_task() {
    const auto state = fsm_->current_state();
    if (!is_cleaning_state(state) && state != "Paused" && state != "Returning") {
        return false;
    }
    fsm_->dispatch(EvTerminateTask{});
    return fsm_->current_state() == "Terminated";
}

bool RobotSupervisor::reset_task(bool at_parking_side) {
    const auto state = fsm_->current_state();
    if (!at_parking_side || (state != "Fault" && state != "Terminated")) {
        return false;
    }
    fsm_->dispatch(EvFaultReset{});
    return fsm_->current_state() == "Idle";
}

void RobotSupervisor::tick_safety(bool low_battery) {
    const auto state = fsm_->current_state();
    if (low_battery && can_trigger_low_battery_return(state)) {
        spdlog::warn("[RobotSupervisor] 电量不足，触发返回");
        fsm_->dispatch(EvLowBattery{});
        return;
    }

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
    snap.active_config = tb_cfg_->active_config();
    snap.pending_config = tb_cfg_->pending_config();
    if (snap.active_config) {
        snap.active_config_version = runtime_config_version(*snap.active_config);
    }
    snap.active_command = command_tracker_->active();
    snap.last_command = command_tracker_->last_completed();
    return snap;
}

std::string RobotSupervisor::task_state_from_device_state(const std::string& device_state) {
    if (device_state == "CleanFwd" || device_state == "CleanReturn")
        return "RunningTask";
    if (device_state == "Paused")
        return "PausedTask";
    if (device_state == "Returning")
        return "ReturningTask";
    if (device_state == "Charging")
        return "ChargingTask";
    if (device_state == "Fault")
        return "FaultedTask";
    if (device_state == "Terminated")
        return "TerminatedTask";
    return "IdleTask";
}

uint64_t RobotSupervisor::runtime_config_version(const service::TbRuntimeConfig& config) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("passes");
    writer.Double(config.passes);
    writer.Key("clean_speed_rpm");
    writer.Double(config.clean_speed_rpm);
    writer.Key("return_speed_rpm");
    writer.Double(config.return_speed_rpm);
    writer.Key("brush_rpm");
    writer.Int(config.brush_rpm);
    writer.Key("parking_side");
    writer.String(service::parking_side_config_string(config.parking_side));
    writer.Key("schedules");
    writer.StartArray();
    for (const auto& schedule : config.schedules) {
        writer.StartObject();
        writer.Key("hour");
        writer.Int(schedule.hour);
        writer.Key("minute");
        writer.Int(schedule.minute);
        writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();
    return static_cast<uint64_t>(std::hash<std::string>{}(std::string(buffer.GetString(), buffer.GetSize())));
}

}  // namespace robot::app
