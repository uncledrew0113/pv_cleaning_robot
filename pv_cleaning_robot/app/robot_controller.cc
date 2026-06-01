#include "pv_cleaning_robot/app/robot_controller.h"

#include <chrono>
#include <utility>

namespace robot::app {

RobotController::RobotController(ActionPorts ports) : actions_(std::move(ports)) {}

RobotController::~RobotController() {
    stop();
}

void RobotController::start() {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    if (running_) {
        return;
    }
    stop_requested_ = false;
    running_ = true;
    worker_ = std::thread([this] { loop(); });
}

void RobotController::stop() {
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        if (!running_) {
            return;
        }
        stop_requested_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        running_ = false;
        stop_requested_ = false;
        handling_event_ = false;
    }
    idle_cv_.notify_all();
}

void RobotController::post(std::function<void()> fn) {
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        queue_.push_back(std::move(fn));
    }
    queue_cv_.notify_one();
}

void RobotController::post_for_test(std::function<void()> fn) {
    post(std::move(fn));
}

void RobotController::drain_for_test() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    idle_cv_.wait(lk, [this] { return queue_.empty() && !handling_event_; });
}

void RobotController::loop() {
    for (;;) {
        std::function<void()> fn;
        {
            std::unique_lock<std::mutex> lk(queue_mtx_);
            queue_cv_.wait(lk, [this] { return stop_requested_ || !queue_.empty(); });
            if (stop_requested_ && queue_.empty()) {
                return;
            }
            fn = std::move(queue_.front());
            queue_.pop_front();
            handling_event_ = true;
        }

        fn();

        {
            std::lock_guard<std::mutex> lk(queue_mtx_);
            handling_event_ = false;
        }
        idle_cv_.notify_all();
    }
}

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
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        if (!running_) {
            std::lock_guard<std::mutex> state_lk(mtx_);
            return submit_command_locked(command);
        }
    }

    auto promise = std::make_shared<std::promise<CommandResult>>();
    auto future = promise->get_future();
    post([this, command, promise] {
        std::lock_guard<std::mutex> lk(mtx_);
        promise->set_value(submit_command_locked(command));
    });

    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return {false, "controller_timeout"};
    }
    return future.get();
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
        if (actions_.clear_fault) {
            actions_.clear_fault();
        }
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
    if (actions_.stop_motion) {
        actions_.stop_motion();
    }
    mission_.reset();
    state_ = RobotState::Idle;
    return {true, "accepted"};
}

bool RobotController::start_current_segment_locked() {
    namespace FaultCode = robot::domain::FaultCode;
    if (!mission_) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        state_ = RobotState::FaultStopped;
        return false;
    }
    const auto* segment = mission_->current_segment();
    if (segment == nullptr) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return false;
    }
    if (actions_.start_segment && !actions_.start_segment(*segment)) {
        active_fault_ = FaultCode::kSegmentStartFailed;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        if (actions_.emergency_stop) {
            actions_.emergency_stop();
        }
        return false;
    }
    return true;
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
    start_current_segment_locked();
}

void RobotController::handle_limit_settled_for_test(domain::Endpoint endpoint) {
    std::lock_guard<std::mutex> lk(mtx_);
    handle_limit_settled_locked(endpoint);
}

void RobotController::handle_limit_settled_locked(domain::Endpoint endpoint) {
    namespace FaultCode = robot::domain::FaultCode;

    if (state_ != RobotState::ExecutingMission || !mission_) {
        active_fault_ = FaultCode::kUnexpectedLimitSide;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return;
    }

    const auto* segment = mission_->current_segment();
    if (segment == nullptr || segment->target != endpoint) {
        active_fault_ = FaultCode::kUnexpectedLimitSide;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return;
    }

    state_ = RobotState::SettlingEndpoint;
    ++mission_->current_segment_index;
    if (mission_->current_segment_index >= mission_->segments.size()) {
        ++mission_->completed_cycles;
    }
    if (mission_->completed_cycles >= mission_->repeat_count) {
        mission_.reset();
        state_ = RobotState::Idle;
        return;
    }
    mission_->current_segment_index = 0;
    state_ = RobotState::ExecutingMission;
    start_current_segment_locked();
}

void RobotController::handle_fault_for_test(const FaultFact& fact) {
    std::lock_guard<std::mutex> lk(mtx_);
    handle_fault_locked(fact);
}

void RobotController::handle_fault_locked(const FaultFact& fact) {
    const auto decision = fault_policy_.decide(fact);
    switch (decision.action) {
    case FaultAction::WarnOnly:
        return;
    case FaultAction::RejectStart:
        if (state_ == RobotState::SelfChecking) {
            mission_.reset();
            state_ = RobotState::Idle;
        }
        return;
    case FaultAction::StartRecovery:
        if (state_ == RobotState::ExecutingMission && mission_) {
            if (actions_.stop_motion) {
                actions_.stop_motion();
            }
            if (actions_.start_recovery) {
                actions_.start_recovery();
            }
            state_ = RobotState::Recovering;
        }
        return;
    case FaultAction::EmergencyStopAndLatch:
        active_fault_ = fact.code;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        if (actions_.emergency_stop) {
            actions_.emergency_stop();
        }
        return;
    }
}

}  // namespace robot::app
