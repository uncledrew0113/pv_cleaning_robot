#include "pv_cleaning_robot/app/robot_controller.h"

namespace robot::app {

const char* RobotController::state_name(RobotState state) noexcept {
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

bool RobotController::mission_active() const noexcept {
    return state_ == RobotState::SelfChecking ||
           state_ == RobotState::ExecutingMission ||
           state_ == RobotState::SettlingEndpoint ||
           state_ == RobotState::Recovering;
}

CommandResult RobotController::submit_command(const domain::RobotCommand& command) {
    std::lock_guard<std::mutex> lk(mtx_);
    return submit_command_locked(command);
}

CommandResult RobotController::submit_command_locked(const domain::RobotCommand& command) {
    switch (command.kind) {
    case domain::RobotCommandKind::StartConfiguredMission:
    case domain::RobotCommandKind::CleanTowardOppositeEndpoint:
    case domain::RobotCommandKind::CleanTowardPrimaryDock:
        return start_command_locked(command);
    case domain::RobotCommandKind::Stop:
        return stop_locked();
    case domain::RobotCommandKind::FaultReset:
        if (state_ != RobotState::FaultStopped) {
            return {false, "not_fault_stopped"};
        }
        active_fault_.reset();
        mission_.reset();
        state_ = RobotState::Idle;
        return {true, "accepted"};
    }
    return {false, "unknown_command"};
}

CommandResult RobotController::start_command_locked(const domain::RobotCommand& command) {
    if (state_ != RobotState::Idle) {
        return {false, "busy"};
    }

    domain::LaneConfig lane{};
    lane.primary_dock = domain::Endpoint::A;
    lane.dock_mode = domain::DockMode::SingleDock;

    if (command.kind == domain::RobotCommandKind::CleanTowardOppositeEndpoint) {
        mission_ = domain::build_directional_clean_context(
            domain::MissionKind::CleanTowardOppositeEndpoint,
            lane,
            command.source,
            command.command_id);
    } else if (command.kind == domain::RobotCommandKind::CleanTowardPrimaryDock) {
        mission_ = domain::build_directional_clean_context(
            domain::MissionKind::CleanTowardPrimaryDock,
            lane,
            command.source,
            command.command_id);
    } else {
        mission_ = domain::build_configured_mission_context(
            lane,
            domain::PositionState::AtA,
            command.source,
            command.command_id,
            1);
    }

    state_ = RobotState::SelfChecking;
    return {true, "accepted"};
}

CommandResult RobotController::stop_locked() {
    if (!mission_active()) {
        return {false, "not_running"};
    }
    mission_.reset();
    state_ = RobotState::Idle;
    return {true, "accepted"};
}

RobotControllerSnapshot RobotController::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    RobotControllerSnapshot snap;
    snap.state = state_name(state_);
    snap.fault = active_fault_;
    if (mission_) {
        snap.completed_cycles = static_cast<int>(mission_->completed_cycles);
    }
    return snap;
}

void RobotController::complete_self_check_for_test(bool ok) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != RobotState::SelfChecking) {
        return;
    }
    if (!ok) {
        mission_.reset();
        state_ = RobotState::Idle;
        return;
    }
    state_ = RobotState::ExecutingMission;
}

}  // namespace robot::app
