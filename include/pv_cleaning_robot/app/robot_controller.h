#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "pv_cleaning_robot/domain/robot_domain.h"

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

struct CommandResult {
    bool accepted{false};
    std::string reason;
};

struct RobotControllerSnapshot {
    std::string state{"Idle"};
    std::optional<uint32_t> fault;
    int completed_cycles{0};
};

class RobotController {
public:
    CommandResult submit_command(const domain::RobotCommand& command);
    RobotControllerSnapshot snapshot() const;

    void complete_self_check_for_test(bool ok);

private:
    static const char* state_name(RobotState state) noexcept;
    bool mission_active() const noexcept;
    CommandResult submit_command_locked(const domain::RobotCommand& command);
    CommandResult start_command_locked(const domain::RobotCommand& command);
    CommandResult stop_locked();

    mutable std::mutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
    std::optional<uint32_t> active_fault_;
};

}  // namespace robot::app
