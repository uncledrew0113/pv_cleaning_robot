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

void RobotController::set_config_ports(ConfigPorts ports) {
    std::lock_guard<std::mutex> lk(mtx_);
    config_ = std::move(ports);
}

void RobotController::set_position_state_query(std::function<domain::PositionState()> query) {
    std::lock_guard<std::mutex> lk(mtx_);
    position_state_query_ = std::move(query);
}

void RobotController::set_battery_soc_query(std::function<float()> query) {
    std::lock_guard<std::mutex> lk(mtx_);
    battery_soc_query_ = std::move(query);
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

void RobotController::post_limit_settled(domain::Endpoint endpoint) {
    post([this, endpoint] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_limit_settled_locked(endpoint);
    });
}

void RobotController::post_limit_unstable(domain::Endpoint endpoint) {
    post([this, endpoint] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_limit_unstable_locked(endpoint);
    });
}

void RobotController::post_watchdog_timeout(std::string thread_name) {
    post([this, thread_name = std::move(thread_name)] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_watchdog_timeout_locked(thread_name);
    });
}

void RobotController::post_recovery_finished(bool ok) {
    post([this, ok] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_recovery_finished_locked(ok);
    });
}

void RobotController::post_schedule_window_hit() {
    post([this] {
        std::lock_guard<std::mutex> lk(mtx_);
        (void)submit_command_locked(domain::RobotCommand{
            domain::RobotCommandKind::StartConfiguredMission,
            domain::CommandSource::Scheduler,
            "schedule"});
    });
}

void RobotController::post_fault(FaultFact fact) {
    post([this, fact = std::move(fact)] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_fault_locked(fact);
    });
}

void RobotController::post_tick() {
    post([] {});
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

    const auto active_cfg = config_.active_runtime_config ? config_.active_runtime_config()
                                                          : domain::RuntimeConfig{};
    const auto pending_cfg = config_.pending_runtime_config ? config_.pending_runtime_config()
                                                            : std::optional<domain::RuntimeConfig>{};
    const auto start_cfg = pending_cfg.value_or(active_cfg);
    if (battery_soc_query_ && battery_soc_query_() < static_cast<float>(start_cfg.min_battery_soc)) {
        return {false, "battery_below_start_threshold"};
    }

    domain::LaneConfig lane = config_.lane_config ? config_.lane_config() : domain::LaneConfig{};
    lane.primary_dock = start_cfg.primary_dock;

    const auto position_state = position_state_query_ ? position_state_query_()
                                                      : domain::PositionState::Unknown;
    const auto validation = validate_start_command_locked(command, lane, position_state);
    if (!validation.accepted || validation.reason == "already_at_target") {
        return validation;
    }

    if (pending_cfg && config_.promote_pending_runtime_config &&
        !config_.promote_pending_runtime_config()) {
        return {false, "runtime_config_promote_failed"};
    }

    mission_ = build_start_mission_locked(command, lane, position_state);
    state_ = RobotState::SelfChecking;
    return {true, "accepted"};
}

CommandResult RobotController::validate_start_command_locked(
    const domain::RobotCommand& command,
    const domain::LaneConfig& lane,
    domain::PositionState position_state) const {
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

domain::MissionContext RobotController::build_start_mission_locked(
    const domain::RobotCommand& command,
    const domain::LaneConfig& lane,
    domain::PositionState position_state) const {
    if (command.kind == domain::RobotCommandKind::StartConfiguredMission) {
        const auto active_cfg = config_.active_runtime_config ? config_.active_runtime_config()
                                                              : domain::RuntimeConfig{};
        return domain::build_configured_mission_context(
            lane,
            position_state,
            command.source,
            command.command_id,
            std::max<uint32_t>(1u, active_cfg.repeat_count));
    }

    return domain::build_directional_clean_context(
        command.kind == domain::RobotCommandKind::CleanTowardPrimaryDock
            ? domain::MissionKind::CleanTowardPrimaryDock
            : domain::MissionKind::CleanTowardOppositeEndpoint,
        lane,
        command.source,
        command.command_id);
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
        snap.repeat_count = mission_->repeat_count;
        snap.completed_cycles = static_cast<int>(mission_->completed_cycles);
    }
    if (config_.active_runtime_config) {
        snap.active_config = config_.active_runtime_config();
    }
    if (config_.pending_runtime_config) {
        snap.pending_config = config_.pending_runtime_config();
    }
    if (snap.active_config && config_.runtime_config_version) {
        snap.cfg_ver = config_.runtime_config_version(*snap.active_config);
    }
    return snap;
}

void RobotController::complete_self_check(bool ok) {
    if (ok) {
        post([this] {
            std::lock_guard<std::mutex> lk(mtx_);
            complete_self_check_locked(true);
        });
        return;
    }
    post([this] {
        std::lock_guard<std::mutex> lk(mtx_);
        complete_self_check_locked(false);
    });
}

void RobotController::complete_self_check_for_test(bool ok) {
    std::lock_guard<std::mutex> lk(mtx_);
    complete_self_check_locked(ok);
}

void RobotController::complete_self_check_locked(bool ok) {
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

    if (state_ == RobotState::Idle ||
        state_ == RobotState::SelfChecking ||
        state_ == RobotState::FaultStopped ||
        state_ == RobotState::Charging) {
        return;
    }

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

void RobotController::handle_limit_unstable_locked(domain::Endpoint) {
    handle_fault_locked(FaultFact{FaultSource::SafetyMonitor,
                                  domain::FaultCode::kLimitUnstableAfterEmergencyStop,
                                  "limit_unstable_after_hard_stop"});
}

void RobotController::handle_watchdog_timeout_locked(const std::string& thread_name) {
    handle_fault_locked(FaultFact{FaultSource::Watchdog,
                                  domain::FaultCode::kCanCommunicationLost,
                                  "watchdog_timeout:" + thread_name});
}

void RobotController::handle_recovery_finished_locked(bool ok) {
    if (state_ != RobotState::Recovering) {
        return;
    }
    if (!ok) {
        handle_fault_locked(FaultFact{FaultSource::Recovery,
                                      domain::FaultCode::kP1DuringReturnEscalatedToP0,
                                      "recovery_failed"});
        return;
    }
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
