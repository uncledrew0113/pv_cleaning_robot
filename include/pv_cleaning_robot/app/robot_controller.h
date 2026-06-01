#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

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
    ~RobotController();

    void start();
    void stop();

    CommandResult submit_command(const domain::RobotCommand& command);
    RobotControllerSnapshot snapshot() const;

    void complete_self_check_for_test(bool ok);
    void handle_limit_settled_for_test(domain::Endpoint endpoint);
    void post_for_test(std::function<void()> fn);
    void drain_for_test();

private:
    static const char* state_name(RobotState state) noexcept;
    void loop();
    void post(std::function<void()> fn);
    bool mission_active() const noexcept;
    CommandResult submit_command_locked(const domain::RobotCommand& command);
    CommandResult start_command_locked(const domain::RobotCommand& command);
    CommandResult stop_locked();
    void handle_limit_settled_locked(domain::Endpoint endpoint);

    mutable std::mutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
    std::optional<uint32_t> active_fault_;

    mutable std::mutex queue_mtx_;
    std::condition_variable queue_cv_;
    std::condition_variable idle_cv_;
    std::deque<std::function<void()>> queue_;
    bool running_{false};
    bool stop_requested_{false};
    bool handling_event_{false};
    std::thread worker_;
};

}  // namespace robot::app
