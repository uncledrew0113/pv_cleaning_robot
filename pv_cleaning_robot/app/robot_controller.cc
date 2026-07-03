#include "pv_cleaning_robot/app/robot_controller.h"

#include <chrono>
#include <future>
#include <spdlog/spdlog.h>
#include <utility>

namespace robot::app {

namespace {

bool is_rpc_command(const domain::RobotCommand& command) noexcept {
    return command.source == domain::CommandSource::Rpc;
}

domain::PositionState position_state_from_endpoint(domain::Endpoint endpoint) noexcept {
    return endpoint == domain::Endpoint::A ? domain::PositionState::AtA
                                           : domain::PositionState::AtB;
}

const char* position_state_name(domain::PositionState state) noexcept {
    switch (state) {
    case domain::PositionState::Unknown:
        return "Unknown";
    case domain::PositionState::AtA:
        return "AtA";
    case domain::PositionState::AtB:
        return "AtB";
    case domain::PositionState::OnSegment:
        return "OnSegment";
    case domain::PositionState::Inconsistent:
        return "Inconsistent";
    }
    return "Unknown";
}

const char* dock_mode_name(domain::DockMode mode) noexcept {
    switch (mode) {
    case domain::DockMode::SingleDock:
        return "SingleDock";
    case domain::DockMode::DualDock:
        return "DualDock";
    }
    return "SingleDock";
}

uint32_t fault_code_from_error_decision(const ErrorDecision& decision) noexcept {
    switch (decision.root_error.code) {
    case ErrorCode::AttitudeLimitBoth:
        return domain::FaultCode::kAttitudeLimitBoth;
    case ErrorCode::RecoveryFailed:
        return domain::FaultCode::kRecoveryFailed;
    case ErrorCode::GpsStuck:
        return domain::FaultCode::kGpsStuck;
    case ErrorCode::DriverCommError:
        switch (decision.root_error.component.kind) {
        case ComponentKind::WalkMotorGroup:
            return domain::FaultCode::kCanCommunicationLost;
        case ComponentKind::Gps:
        case ComponentKind::GpsStuckService:
            return domain::FaultCode::kGpsCommunicationLost;
        case ComponentKind::Bms:
            return domain::FaultCode::kBmsCommunicationLost;
        case ComponentKind::BrushMotor:
            return domain::FaultCode::kBrushMotorCommunicationLost;
        case ComponentKind::Imu:
            return domain::FaultCode::kImuCommunicationLost;
        case ComponentKind::AttitudeLimitSwitch:
            return domain::FaultCode::kRecoveryFailed;
        }
        return domain::FaultCode::kRecoveryFailed;
    case ErrorCode::WalkMotorStall:
        return domain::FaultCode::kWalkMotorStall;
    case ErrorCode::BrushMotorFault:
        return domain::FaultCode::kBrushMotorFault;
    case ErrorCode::AttitudeLimit:
        return domain::FaultCode::kRepeatedAttitudeLimit;
    }
    return domain::FaultCode::kRecoveryFailed;
}

}  // namespace

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
    post([this, endpoint] { handle_limit_settled_unlocked(endpoint); });
}

void RobotController::post_recovery_finished(bool ok) {
    post([this, ok] {
        std::optional<domain::Endpoint> already_at_target;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            already_at_target = handle_recovery_finished_locked(ok);
        }
        if (already_at_target.has_value()) {
            handle_limit_settled_unlocked(*already_at_target);
        }
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

void RobotController::post_tick() {
    post([] {});
}

void RobotController::apply_error_decision(const ErrorDecision& decision) {
    bool do_stop_motion = false;
    bool do_start_recovery = false;
    bool do_emergency_stop = false;

    {
        std::lock_guard<std::mutex> lk(mtx_);

        switch (decision.action) {
        case ErrorAction::Ignore:
        case ErrorAction::WarnOnly:
            return;
        case ErrorAction::StartRecovery:
            if (!decision.requires_robot_recovering) {
                return;
            }
            if (state_ == RobotState::ExecutingMission && mission_) {
                recovery_return_state_ = RobotState::ExecutingMission;
                state_ = RobotState::Recovering;
                do_stop_motion = true;
                do_start_recovery = true;
            }
            break;
        case ErrorAction::FaultStopped:
            active_fault_ = fault_code_from_error_decision(decision);
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            do_emergency_stop = true;
            break;
        }
    }

    // 外部动作可能访问 CAN/串口/运动服务，必须在状态机锁外执行，避免阻塞 RPC、
    // snapshot() 和后续事件处理。动作只依据上面锁内计算出的标志执行，不再重新读取状态。
    if (do_stop_motion && actions_.stop_motion) {
        actions_.stop_motion();
    }
    if (do_start_recovery && actions_.start_recovery) {
        actions_.start_recovery();
    }
    if (do_emergency_stop && actions_.emergency_stop) {
        actions_.emergency_stop();
    }
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

        try {
            fn();
        } catch (const std::exception& ex) {
            spdlog::error("[RobotController] event handler threw exception: {}", ex.what());
            std::lock_guard<std::mutex> state_lk(mtx_);
            active_fault_ = domain::FaultCode::kTaskContextInconsistent;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            if (actions_.emergency_stop) {
                try {
                    actions_.emergency_stop();
                } catch (const std::exception& stop_ex) {
                    spdlog::error(
                        "[RobotController] emergency_stop threw during exception fallback: {}",
                        stop_ex.what());
                } catch (...) {
                    spdlog::error(
                        "[RobotController] emergency_stop threw unknown exception during fallback");
                }
            }
        } catch (...) {
            spdlog::error("[RobotController] event handler threw unknown exception");
            std::lock_guard<std::mutex> state_lk(mtx_);
            active_fault_ = domain::FaultCode::kTaskContextInconsistent;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            if (actions_.emergency_stop) {
                try {
                    actions_.emergency_stop();
                } catch (...) {
                    spdlog::error(
                        "[RobotController] emergency_stop threw during unknown exception fallback");
                }
            }
        }

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
        recovery_return_state_.reset();
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
    // RPC 在本项目中定义为维护类指令：人工确认后执行，不参与调度启动的电量门槛。
    if (!is_rpc_command(command) && battery_soc_query_ &&
        battery_soc_query_() < static_cast<float>(start_cfg.min_battery_soc)) {
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
    recovery_return_state_.reset();
    state_ = RobotState::SelfChecking;
    return {true, "accepted"};
}

CommandResult RobotController::validate_start_command_locked(
    const domain::RobotCommand& command,
    const domain::LaneConfig& lane,
    domain::PositionState position_state) const {
    // RPC 统一作为维护类指令处理：跳过位置合法性检查，避免维护场景被限位事实阻断。
    // 调度和本地自动任务仍保留位置门槛，防止无人值守时盲动。
    if (is_rpc_command(command)) {
        return {true, "start_allowed"};
    }

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
        // RPC 配置任务按维护约定假定机器人当前位于 primary_dock；
        // 非 RPC 任务继续使用真实位置，让调度启动保持自动安全边界。
        const auto start_state = is_rpc_command(command)
                                     ? position_state_from_endpoint(lane.primary_dock)
                                     : position_state;
        return domain::build_configured_mission_context(
            lane,
            start_state,
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
    // stop RPC 只允许任务正常执行中响应；自检、恢复、端点切段和故障态都拒绝，
    // 避免维护命令打断恢复闭环或把半完成状态静默清成 Idle。
    if (state_ != RobotState::ExecutingMission) {
        return {false, "not_running"};
    }
    if (actions_.stop_motion) {
        actions_.stop_motion();
    }
    mission_.reset();
    recovery_return_state_.reset();
    state_ = RobotState::Idle;
    return {true, "accepted"};
}

RobotController::StartSegmentResult RobotController::start_current_segment_locked() {
    namespace FaultCode = robot::domain::FaultCode;
    if (!mission_) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        recovery_return_state_.reset();
        state_ = RobotState::FaultStopped;
        return {};
    }
    const auto* segment = mission_->current_segment();
    if (segment == nullptr) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        mission_.reset();
        recovery_return_state_.reset();
        state_ = RobotState::FaultStopped;
        return {};
    }
    if (position_state_query_ &&
        domain::is_at_target(position_state_query_(), segment->target)) {
        spdlog::warn(
            "[RobotController] current segment target already active before motion start: "
            "target={}",
            domain::endpoint_config_string(segment->target));
        return StartSegmentResult{true, segment->target};
    }
    if (actions_.start_segment && !actions_.start_segment(*segment)) {
        active_fault_ = FaultCode::kSegmentStartFailed;
        mission_.reset();
        recovery_return_state_.reset();
        state_ = RobotState::FaultStopped;
        if (actions_.emergency_stop) {
            actions_.emergency_stop();
        }
        return {};
    }
    return StartSegmentResult{true, std::nullopt};
}

RobotControllerSnapshot RobotController::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    RobotControllerSnapshot snap;
    snap.state = state_name(state_);
    snap.fault = active_fault_;
    if (mission_) {
        snap.repeat_count = mission_->repeat_count;
        snap.completed_cycles = static_cast<int>(mission_->completed_cycles);
        if (const auto* segment = mission_->current_segment()) {
            snap.current_segment_target = segment->target;
            snap.current_segment_mode = segment->mode;
        }
    }
    if (config_.active_runtime_config && config_.runtime_config_version) {
        const auto active_config = config_.active_runtime_config();
        snap.cfg_ver = config_.runtime_config_version(active_config);
    }
    return snap;
}

void RobotController::complete_self_check(bool ok) {
    post([this, ok] { complete_self_check_unlocked(ok); });
}

void RobotController::complete_self_check_for_test(bool ok) {
    complete_self_check_unlocked(ok);
}

void RobotController::complete_self_check_unlocked(bool ok) {
    std::function<bool()> close_lock_motor;
    bool do_emergency_stop = false;
    std::optional<domain::Endpoint> already_at_target;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ != RobotState::SelfChecking) {
            return;
        }
        if (!ok) {
            active_fault_ = domain::FaultCode::kSelfCheckFailed;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            do_emergency_stop = static_cast<bool>(actions_.emergency_stop);
        } else {
            close_lock_motor = actions_.close_lock_motor;
        }
    }

    if (do_emergency_stop) {
        actions_.emergency_stop();
        return;
    }

    const bool lock_closed = !close_lock_motor || close_lock_motor();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ != RobotState::SelfChecking) {
            return;
        }
        if (!lock_closed) {
            // 锁止电机关动作失败时禁止启动清扫：设备没有机械反馈，GPIO 写入失败必须显式停机上报。
            active_fault_ = domain::FaultCode::kLockMotorCloseFailed;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            do_emergency_stop = static_cast<bool>(actions_.emergency_stop);
        } else {
            state_ = RobotState::ExecutingMission;
            already_at_target = start_current_segment_locked().already_at_target;
        }
    }

    if (do_emergency_stop) {
        actions_.emergency_stop();
        return;
    }
    if (already_at_target.has_value()) {
        handle_limit_settled_unlocked(*already_at_target);
    }
}

void RobotController::handle_limit_settled_for_test(domain::Endpoint endpoint) {
    handle_limit_settled_unlocked(endpoint);
}

void RobotController::handle_limit_settled_unlocked(domain::Endpoint endpoint) {
    namespace FaultCode = robot::domain::FaultCode;

    std::function<void()> stop_motion;
    std::function<bool()> open_lock_motor;
    bool mission_finished = false;
    bool do_emergency_stop = false;
    std::optional<domain::Endpoint> already_at_target;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ == RobotState::Idle ||
            state_ == RobotState::SelfChecking ||
            state_ == RobotState::Recovering ||
            state_ == RobotState::FaultStopped ||
            state_ == RobotState::Charging) {
            return;
        }

        if (state_ != RobotState::ExecutingMission || !mission_) {
            active_fault_ = FaultCode::kUnexpectedLimitSide;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            return;
        }

        const auto* segment = mission_->current_segment();
        if (segment == nullptr) {
            active_fault_ = FaultCode::kUnexpectedLimitSide;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            return;
        }
        if (segment->target != endpoint) {
            already_at_target = handle_source_limit_repeat_locked(endpoint);
        }
        if (!already_at_target.has_value() && segment->target == endpoint) {
            state_ = RobotState::SettlingEndpoint;
            ++mission_->current_segment_index;
            if (mission_->current_segment_index >= mission_->segments.size()) {
                ++mission_->completed_cycles;
                mission_->current_segment_index = 0;
            }
            if (mission_->completed_cycles < mission_->repeat_count) {
                state_ = RobotState::ExecutingMission;
                already_at_target = start_current_segment_locked().already_at_target;
            } else {
                mission_finished = true;
                stop_motion = actions_.stop_motion;
                auto lane = config_.lane_config ? config_.lane_config() : domain::LaneConfig{};
                if (config_.active_runtime_config) {
                    lane.primary_dock = config_.active_runtime_config().primary_dock;
                }
                const bool finished_at_dock =
                    lane.dock_mode == domain::DockMode::DualDock || endpoint == lane.primary_dock;
                if (finished_at_dock) {
                    const auto expected_endpoint =
                        lane.dock_mode == domain::DockMode::DualDock ? endpoint : lane.primary_dock;
                    const auto confirmed_position = position_state_query_
                                                        ? position_state_query_()
                                                        : domain::PositionState::Unknown;
                    if (domain::is_at_target(confirmed_position, expected_endpoint)) {
                        open_lock_motor = actions_.open_lock_motor;
                    } else {
                        spdlog::warn(
                            "[RobotController] skip lock open: dock position confirmation failed: "
                            "dock_mode={} finished_endpoint={} expected_endpoint={} position={}",
                            dock_mode_name(lane.dock_mode),
                            domain::endpoint_config_string(endpoint),
                            domain::endpoint_config_string(expected_endpoint),
                            position_state_name(confirmed_position));
                    }
                }
            }
        }
    }

    if (already_at_target.has_value()) {
        handle_limit_settled_unlocked(*already_at_target);
        return;
    }
    if (!mission_finished) {
        return;
    }
    if (stop_motion) {
        stop_motion();
    }
    const bool lock_opened = !open_lock_motor || open_lock_motor();

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ != RobotState::SettlingEndpoint) {
            return;
        }
        if (!lock_opened) {
            // 锁止电机只在最终停机位动作；GPIO 写入失败必须进入故障停机，避免误认为已锁车。
            active_fault_ = FaultCode::kLockMotorOpenFailed;
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::FaultStopped;
            do_emergency_stop = static_cast<bool>(actions_.emergency_stop);
        } else {
            mission_.reset();
            recovery_return_state_.reset();
            state_ = RobotState::Idle;
        }
    }

    if (do_emergency_stop) {
        actions_.emergency_stop();
    }
}

std::optional<domain::Endpoint> RobotController::handle_source_limit_repeat_locked(
    domain::Endpoint) {
    // 单侧主限位重复触发按业务事件处理：重新启动当前段，不再作为 v1 错误来源。
    state_ = RobotState::ExecutingMission;
    return start_current_segment_locked().already_at_target;
}

std::optional<domain::Endpoint> RobotController::handle_recovery_finished_locked(bool ok) {
    if (state_ != RobotState::Recovering) {
        return std::nullopt;
    }
    if (!ok) {
        // RecoveryExecutor 的连续失败升级已由 ErrorManager 仲裁；这里仅处理执行器明确返回失败的兜底闭环。
        active_fault_ = domain::FaultCode::kRecoveryFailed;
        mission_.reset();
        recovery_return_state_.reset();
        state_ = RobotState::FaultStopped;
        if (actions_.emergency_stop) {
            actions_.emergency_stop();
        }
        return std::nullopt;
    }
    const auto return_state = recovery_return_state_.value_or(RobotState::ExecutingMission);
    recovery_return_state_.reset();
    if (return_state == RobotState::SelfChecking) {
        state_ = RobotState::SelfChecking;
        return std::nullopt;
    }
    state_ = RobotState::ExecutingMission;
    return start_current_segment_locked().already_at_target;
}

}  // namespace robot::app
