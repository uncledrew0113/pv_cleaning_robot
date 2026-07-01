#include "pv_cleaning_robot/app/error_manager.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace robot::app {

namespace {
// 通信错误计数连续变化 10 个采样周期后才触发，避免偶发读数抖动误报。
constexpr uint32_t kConsecutiveErrorCounterLimit = 10;

// 数据流 3 秒没有新帧，认为对应通信链路或数据源已停滞。
constexpr uint64_t kStreamTimeoutMs = 3000;

// 行走电机堵转必须连续保持 5 秒，才进入恢复，避免瞬时负载冲击误报。
constexpr uint64_t kWalkStallDurationMs = 5000;

// 行走电机 60 秒内 3 次有效堵转，说明现场恢复无效，直接升级为故障停机。
constexpr uint64_t kWalkStallWindowMs = 60000;
constexpr size_t kWalkStallFaultLimit = 3;

// GPS 卡滞 60 秒内出现 3 次，说明恢复无效，直接升级为故障停机。
constexpr uint64_t kGpsStuckWindowMs = 60000;
constexpr size_t kGpsStuckFaultLimit = 3;

// 同一个恢复动作连续失败 3 次后停机；前两次允许自动重试。
constexpr uint32_t kRecoveryFailureLimit = 3;
}  // namespace

RecoveryPlanId ErrorManager::plan_for_component(ComponentKind kind) noexcept {
    switch (kind) {
    case ComponentKind::WalkMotorGroup:
        return RecoveryPlanId::RecoverWalkMotorGroup;
    case ComponentKind::BrushMotor:
        return RecoveryPlanId::RecoverBrushMotor;
    case ComponentKind::Bms:
        return RecoveryPlanId::RecoverBms;
    case ComponentKind::Gps:
    case ComponentKind::GpsStuckService:
        return RecoveryPlanId::RecoverGps;
    case ComponentKind::Imu:
        return RecoveryPlanId::RecoverImu;
    case ComponentKind::AttitudeLimitSwitch:
        return RecoveryPlanId::None;
    }
    return RecoveryPlanId::None;
}

bool ErrorManager::requires_robot_recovering(RecoveryPlanId plan) noexcept {
    return plan != RecoveryPlanId::None;
}

bool ErrorManager::same_component(ComponentId lhs, ComponentId rhs) noexcept {
    return lhs.kind == rhs.kind && lhs.index == rhs.index;
}

bool ErrorManager::same_error_key(const ErrorFact& lhs, const ErrorFact& rhs) noexcept {
    return lhs.code == rhs.code && same_component(lhs.component, rhs.component);
}

bool ErrorManager::is_motion_recovery_plan(RecoveryPlanId plan) noexcept {
    return plan == RecoveryPlanId::RecoverWalkStall ||
           plan == RecoveryPlanId::RecoverGpsStuckReverse ||
           plan == RecoveryPlanId::RecoverAttitudeCenter;
}

bool ErrorManager::should_suppress_locked(const ErrorFact& fact) const {
    if (!active_recovery_) {
        return false;
    }

    const auto active_plan = active_recovery_->plan;
    const auto active_component = active_recovery_->component;

    // 所有恢复动作都会暂停 GpsStuck；恢复期间出现的卡滞属于预期子错误。
    if (fact.code == ErrorCode::GpsStuck) {
        return true;
    }

    // 设备恢复期间，同组件通信/设备错误只记录为同根因持续存在，不启动第二个恢复。
    if (same_component(fact.component, active_component)) {
        if (fact.code == ErrorCode::DriverCommError ||
            fact.code == ErrorCode::BrushMotorFault) {
            return true;
        }
    }

    // GPS 恢复包含 gps_stuck_exec 停启，因此 GPS 卡滞和 GPS 通信都应被压制。
    if (active_plan == RecoveryPlanId::RecoverGps &&
        fact.component.kind == ComponentKind::Gps) {
        return true;
    }

    return false;
}

ErrorDecision ErrorManager::decide(const ErrorFact& fact) {
    ErrorDecision decision;
    decision.component = fact.component;
    decision.root_error = fact;

    switch (fact.code) {
    case ErrorCode::AttitudeLimitBoth:
    case ErrorCode::RecoveryFailed:
        decision.action = ErrorAction::FaultStopped;
        decision.latch_fault = true;
        return decision;
    case ErrorCode::DriverCommError:
    case ErrorCode::BrushMotorFault:
        decision.plan = plan_for_component(fact.component.kind);
        decision.action = decision.plan == RecoveryPlanId::None ? ErrorAction::WarnOnly
                                                                : ErrorAction::StartRecovery;
        decision.requires_robot_recovering = requires_robot_recovering(decision.plan);
        return decision;
    case ErrorCode::WalkMotorStall:
        walk_stall_event_times_.push_back(fact.timestamp_ms);
        walk_stall_event_times_.erase(
            std::remove_if(walk_stall_event_times_.begin(),
                           walk_stall_event_times_.end(),
                           [fact](uint64_t ts) {
                               return fact.timestamp_ms >= ts &&
                                      fact.timestamp_ms - ts > kWalkStallWindowMs;
                           }),
            walk_stall_event_times_.end());
        if (walk_stall_event_times_.size() >= kWalkStallFaultLimit) {
            decision.action = ErrorAction::FaultStopped;
            decision.latch_fault = true;
            return decision;
        }
        decision.action = ErrorAction::StartRecovery;
        decision.plan = RecoveryPlanId::RecoverWalkStall;
        decision.requires_robot_recovering = true;
        return decision;
    case ErrorCode::GpsStuck:
        gps_stuck_event_times_.push_back(fact.timestamp_ms);
        gps_stuck_event_times_.erase(
            std::remove_if(gps_stuck_event_times_.begin(),
                           gps_stuck_event_times_.end(),
                           [fact](uint64_t ts) {
                               return fact.timestamp_ms >= ts &&
                                      fact.timestamp_ms - ts > kGpsStuckWindowMs;
                           }),
            gps_stuck_event_times_.end());
        if (gps_stuck_event_times_.size() >= kGpsStuckFaultLimit) {
            decision.action = ErrorAction::FaultStopped;
            decision.latch_fault = true;
            return decision;
        }
        decision.action = ErrorAction::StartRecovery;
        decision.plan = RecoveryPlanId::RecoverGpsStuckReverse;
        decision.requires_robot_recovering = true;
        return decision;
    case ErrorCode::AttitudeLimit:
        decision.action = ErrorAction::StartRecovery;
        decision.plan = RecoveryPlanId::RecoverAttitudeCenter;
        decision.requires_robot_recovering = true;
        return decision;
    }

    decision.action = ErrorAction::WarnOnly;
    return decision;
}

ErrorDecision ErrorManager::submit_error_locked(const ErrorFact& fact) {
    if (should_suppress_locked(fact)) {
        ErrorDecision ignored;
        ignored.root_error = fact;
        ignored.component = fact.component;
        ignored.action = ErrorAction::Ignore;
        return ignored;
    }

    if (active_recovery_ && same_error_key(active_recovery_->root_error, fact)) {
        ErrorDecision ignored;
        ignored.root_error = fact;
        ignored.component = fact.component;
        ignored.action = ErrorAction::Ignore;
        return ignored;
    }
    for (const auto& pending : pending_decisions_) {
        if (same_error_key(pending.root_error, fact)) {
            ErrorDecision ignored;
            ignored.root_error = fact;
            ignored.component = fact.component;
            ignored.action = ErrorAction::Ignore;
            return ignored;
        }
    }

    auto decision = decide(fact);
    if (decision.action != ErrorAction::Ignore) {
        pending_decisions_.push_back(decision);
    }
    return decision;
}

ErrorDecision ErrorManager::submit_error(const ErrorFact& fact) {
    std::lock_guard<std::mutex> lk(mtx_);
    return submit_error_locked(fact);
}

ErrorDecision ErrorManager::submit_watchdog_timeout(const std::string& thread_name,
                                                    uint64_t now_ms) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (thread_name == "walk_ctrl") {
        return submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                             ComponentId{ComponentKind::WalkMotorGroup, 0},
                                             "watchdog_timeout:walk_ctrl",
                                             now_ms});
    }
    if (thread_name == "bms") {
        return submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                             ComponentId{ComponentKind::Bms, 0},
                                             "watchdog_timeout:bms",
                                             now_ms});
    }
    if (thread_name == "brush") {
        return submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                             ComponentId{ComponentKind::BrushMotor, 0},
                                             "watchdog_timeout:brush",
                                             now_ms});
    }
    if (thread_name == "gps_stuck") {
        return submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                             ComponentId{ComponentKind::Gps, 0},
                                             "watchdog_timeout:gps_stuck",
                                             now_ms});
    }
    return submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                         ComponentId{ComponentKind::AttitudeLimitSwitch, 0},
                                         "watchdog_timeout:" + thread_name,
                                         now_ms});
}

void ErrorManager::update_error_counter(ErrorCounterState& state,
                                        uint32_t count,
                                        ComponentKind component,
                                        uint64_t now_ms) {
    // 使用“是否变化”而不是“大于前值”，保证 uint32_t 回绕后仍能正确计数。
    if (!state.initialized) {
        state.initialized = true;
        state.consecutive_increments = count != state.previous ? 1 : 0;
        state.previous = count;
    } else if (count != state.previous) {
        ++state.consecutive_increments;
        state.previous = count;
    } else {
        state.consecutive_increments = 0;
    }

    if (state.consecutive_increments >= kConsecutiveErrorCounterLimit) {
        submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                      ComponentId{component, 0},
                                      "error counter increased continuously",
                                      now_ms});
        state.consecutive_increments = 0;
    }
}

void ErrorManager::update_stream_timeout(const domain::StreamHealth& health,
                                         StreamTimeoutState& state,
                                         ComponentKind component,
                                         uint64_t timeout_ms,
                                         uint64_t now_ms) {
    // last_update_ms == 0 表示还没有有效数据时间戳；启动期不在这里误报。
    if (!health.enabled || health.last_update_ms == 0 || now_ms < health.last_update_ms) {
        state = StreamTimeoutState{};
        return;
    }

    if (health.last_update_ms != state.last_seen_update_ms) {
        state.last_seen_update_ms = health.last_update_ms;
        state.timed_out = false;
    }

    if (now_ms - health.last_update_ms >= timeout_ms) {
        if (state.timed_out) {
            return;
        }
        const auto decision = submit_error_locked(ErrorFact{ErrorCode::DriverCommError,
                                                            ComponentId{component, 0},
                                                            "data stream timeout",
                                                            now_ms});
        state.timed_out = decision.action != ErrorAction::Ignore;
    }
}

void ErrorManager::update(const domain::DiagnosticsSnapshot& snapshot, uint64_t now_ms) {
    std::lock_guard<std::mutex> lk(mtx_);
    update_stream_timeout(snapshot.bms_update,
                          bms_update_timeout_state_,
                          ComponentKind::Bms,
                          kStreamTimeoutMs,
                          now_ms);

    // BrushMotor 的 comm_error_count 连续变化表示串口通信异常；fault_code != 0
    // 表示滚刷驱动自身故障，通信链路仍可能是正常的。
    update_error_counter(brush_error_state_,
                         snapshot.brush.error_count,
                         ComponentKind::BrushMotor,
                         now_ms);
    if (snapshot.brush_fault_active) {
        if (!brush_fault_state_.active) {
            brush_fault_state_.active = true;
            submit_error_locked(ErrorFact{ErrorCode::BrushMotorFault,
                                          ComponentId{ComponentKind::BrushMotor, 0},
                                          "brush fault code active",
                                          now_ms});
        }
    } else {
        brush_fault_state_ = DurationFlagState{};
    }

    update_stream_timeout(snapshot.gps,
                          gps_timeout_state_,
                          ComponentKind::Gps,
                          kStreamTimeoutMs,
                          now_ms);
    update_stream_timeout(snapshot.imu,
                          imu_timeout_state_,
                          ComponentKind::Imu,
                          kStreamTimeoutMs,
                          now_ms);

    for (auto i = 0U; i < snapshot.walk_feedback.size(); ++i) {
        auto health = snapshot.walk_feedback[i];
        health.enabled = snapshot.walk_feedback_expected && health.enabled;
        // 每个行走轮独立维护“是否已对本次停滞上报”的状态；错误事实仍合并到
        // WalkMotorGroup 级别，避免四个轮同时停滞时启动多个恢复流程。
        update_stream_timeout(health,
                              walk_feedback_timeout_states_[i],
                              ComponentKind::WalkMotorGroup,
                              kStreamTimeoutMs,
                              now_ms);
    }

    if (!snapshot.walk_stall_active) {
        walk_stall_state_ = DurationFlagState{};
    } else if (!walk_stall_state_.active) {
        walk_stall_state_.active = true;
        walk_stall_state_.active_since_ms = now_ms;
        walk_stall_state_.reported = false;
    } else if (!walk_stall_state_.reported &&
               now_ms >= walk_stall_state_.active_since_ms &&
               now_ms - walk_stall_state_.active_since_ms >= kWalkStallDurationMs) {
        submit_error_locked(ErrorFact{ErrorCode::WalkMotorStall,
                                      ComponentId{ComponentKind::WalkMotorGroup, 0},
                                      "walk motor stall stayed active",
                                      now_ms});
        walk_stall_state_.reported = true;
    }

    // GpsStuckService 的布尔快照按“上升沿”记为一次事件；
    // 持续为 true 时不能每个周期累计，否则会把同一次卡滞误判成多次失败。
    if (!snapshot.gps_stuck) {
        gps_stuck_snapshot_active_ = false;
    } else if (!gps_stuck_snapshot_active_) {
        submit_error_locked(ErrorFact{ErrorCode::GpsStuck,
                                      ComponentId{ComponentKind::GpsStuckService, 0},
                                      "gps stuck service reported stuck",
                                      now_ms});
        gps_stuck_snapshot_active_ = true;
    }
}

void ErrorManager::mark_recovery_started(const ErrorDecision& decision) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (decision.action == ErrorAction::StartRecovery) {
        active_recovery_ = decision;
    }
}

ErrorDecision ErrorManager::mark_recovery_finished(const RecoveryResultFact& result) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (is_motion_recovery_plan(result.decision.plan)) {
        active_recovery_.reset();
        return ErrorDecision{};
    }

    if (result.ok) {
        recovery_failure_state_ = RecoveryFailureState{};
        active_recovery_.reset();
        return ErrorDecision{};
    }

    const bool same_target = recovery_failure_state_.active &&
                             recovery_failure_state_.plan == result.decision.plan &&
                             same_component(recovery_failure_state_.component,
                                            result.decision.component);

    if (!same_target) {
        recovery_failure_state_.active = true;
        recovery_failure_state_.plan = result.decision.plan;
        recovery_failure_state_.component = result.decision.component;
        recovery_failure_state_.consecutive_failures = 0;
    }
    ++recovery_failure_state_.consecutive_failures;

    if (recovery_failure_state_.consecutive_failures >= kRecoveryFailureLimit) {
        ErrorFact fact{ErrorCode::RecoveryFailed,
                       result.decision.component,
                       result.reason,
                       result.timestamp_ms};
        active_recovery_.reset();
        return submit_error_locked(fact);
    }

    // 前两次失败继续执行同一个恢复计划；放入队列让主循环按统一路径重试。
    auto retry = result.decision;
    retry.action = ErrorAction::StartRecovery;
    retry.root_error.detail = result.reason;
    retry.root_error.timestamp_ms = result.timestamp_ms;
    pending_decisions_.push_back(retry);
    active_recovery_ = retry;
    return retry;
}

std::vector<ErrorDecision> ErrorManager::drain_decisions() {
    std::lock_guard<std::mutex> lk(mtx_);
    auto decisions = std::move(pending_decisions_);
    pending_decisions_.clear();
    return decisions;
}

ErrorHandlingService::ErrorHandlingService(ErrorManager& manager, Ports ports)
    : manager_(manager), ports_(std::move(ports)) {}

bool ErrorHandlingService::is_task_running_state(const std::string& state) {
    return state == "ExecutingMission";
}

bool ErrorHandlingService::is_motion_recovery_plan(RecoveryPlanId plan) {
    return plan == RecoveryPlanId::RecoverWalkStall ||
           plan == RecoveryPlanId::RecoverGpsStuckReverse ||
           plan == RecoveryPlanId::RecoverAttitudeCenter;
}

bool ErrorHandlingService::recovery_plan_allowed(const std::string& state,
                                                 RecoveryPlanId plan) {
    // SelfChecking 只做健康门槛，不自动修复；Charging 不允许任何恢复动作；
    // SettlingEndpoint 是端点切段的瞬时业务态，也不挂恢复流程。
    if (state == "SelfChecking" || state == "Charging" || state == "SettlingEndpoint" ||
        state == "FaultStopped") {
        return false;
    }
    if (is_motion_recovery_plan(plan)) {
        return is_task_running_state(state);
    }
    return plan != RecoveryPlanId::None;
}

int ErrorHandlingService::decision_priority(const ErrorDecision& decision) noexcept {
    // v1 不建立复杂因果图；同一调度周期只处理一个最高优先级根错误。
    // FaultStopped 必须压过所有恢复动作，避免先执行低优先级恢复再停机。
    if (decision.action == ErrorAction::FaultStopped || decision.latch_fault) {
        return 100;
    }
    if (decision.action != ErrorAction::StartRecovery) {
        return 0;
    }

    switch (decision.plan) {
    case RecoveryPlanId::RecoverWalkMotorGroup:
    case RecoveryPlanId::RecoverBrushMotor:
    case RecoveryPlanId::RecoverBms:
    case RecoveryPlanId::RecoverGps:
    case RecoveryPlanId::RecoverImu:
        return 80;
    case RecoveryPlanId::RecoverWalkStall:
    case RecoveryPlanId::RecoverGpsStuckReverse:
    case RecoveryPlanId::RecoverAttitudeCenter:
        return 60;
    case RecoveryPlanId::None:
        return 0;
    }
    return 0;
}

uint64_t ErrorHandlingService::now_ms() const {
    return ports_.now_ms ? ports_.now_ms() : 0;
}

void ErrorHandlingService::update() {
    const std::string state = ports_.current_state ? ports_.current_state() : std::string{};

    const bool desired_gps_stuck_monitoring = is_task_running_state(state);
    if (ports_.set_gps_stuck_monitoring &&
        gps_stuck_monitoring_enabled_ != desired_gps_stuck_monitoring) {
        ports_.set_gps_stuck_monitoring(desired_gps_stuck_monitoring);
        gps_stuck_monitoring_enabled_ = desired_gps_stuck_monitoring;
    }

    if (ports_.consume_error_event) {
        if (const auto event = ports_.consume_error_event()) {
            // 姿态单侧限位在任何状态都已由 AttitudeLimitService 立即急停；
            // v1 只在任务执行中把它升级为回中恢复，空闲/自检/充电时不启动运动恢复。
            if (event->code != ErrorCode::AttitudeLimit || is_task_running_state(state)) {
                manager_.submit_error(*event);
            }
        }
    }

    if (ports_.diagnostics_snapshot) {
        manager_.update(ports_.diagnostics_snapshot(), now_ms());
    }

    auto decisions = manager_.drain_decisions();
    if (decisions.empty()) {
        return;
    }
    const auto best = std::max_element(decisions.begin(),
                                       decisions.end(),
                                       [](const ErrorDecision& lhs,
                                          const ErrorDecision& rhs) {
                                           return decision_priority(lhs) <
                                                  decision_priority(rhs);
                                       });
    const auto decision = *best;
    if (decision.action != ErrorAction::StartRecovery) {
        if (ports_.apply_error_decision) {
            ports_.apply_error_decision(decision);
        }
        return;
    }

    const std::string decision_state = ports_.current_state ? ports_.current_state() : state;
    if (!recovery_plan_allowed(decision_state, decision.plan)) {
        // v1 只在明确允许的状态执行恢复：设备恢复允许 Idle 和 ExecutingMission；
        // 运动恢复只允许 ExecutingMission。其余状态丢弃本轮恢复，持续错误会在后续诊断中再提交。
        return;
    }
    if (ports_.apply_error_decision) {
        ports_.apply_error_decision(decision);
    }
    if (!ports_.execute_recovery) {
        manager_.mark_recovery_finished(
            RecoveryResultFact{decision, false, "missing execute_recovery port", now_ms()});
        return;
    }

    manager_.mark_recovery_started(decision);
    const auto result = ports_.execute_recovery(decision);
    manager_.mark_recovery_finished(result);
    if (decision.requires_robot_recovering && result.ok && ports_.post_recovery_finished) {
        ports_.post_recovery_finished(true);
    }
}

}  // namespace robot::app
