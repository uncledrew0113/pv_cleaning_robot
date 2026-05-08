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
    return state == "Idle" || state == "Charging" || state == "Stopped";
}

bool is_cleaning_state(const std::string& state) {
    return state == "CleanFwd" || state == "CleanReturn";
}

bool can_trigger_spin_free_fault(const std::string& state) {
    return state != "Idle" && state != "Charging" && state != "Fault" &&
           state != "Stopped";
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

bool RobotSupervisor::start_task(bool at_parking_side, bool position_valid, float battery_soc) {
    if (!is_new_task_start_state(fsm_->current_state()) || !position_valid || !at_parking_side) {
        return false;
    }
    const auto runtime_cfg = tb_cfg_->has_pending_config() ? *tb_cfg_->pending_config()
                                                           : tb_cfg_->active_config();
    if (battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)) {
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

bool RobotSupervisor::stop_task() {
    const auto state = fsm_->current_state();
    if (!is_cleaning_state(state) && state != "Returning") {
        return false;
    }
    fsm_->dispatch(EvStopTask{});
    return fsm_->current_state() == "Stopped";
}

bool RobotSupervisor::return_task(bool at_parking_side) {
    const auto state = fsm_->current_state();
    if (at_parking_side) {
        return false;
    }
    if (!is_cleaning_state(state) && state != "Stopped" && state != "Idle") {
        return false;
    }
    fsm_->dispatch(EvManualReturn{});
    return fsm_->current_state() == "Returning";
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
    writer.Key("start_battery_soc");
    writer.Double(config.start_battery_soc);
    writer.Key("charge_start_soc");
    writer.Double(config.charge_start_soc);
    writer.Key("charge_stop_soc");
    writer.Double(config.charge_stop_soc);
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
