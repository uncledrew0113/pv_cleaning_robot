/**
 * @file error_manager.cc
 * @brief 应用层错误仲裁和恢复调度实现。
 *
 * 本文件把诊断快照、看门狗事件和设备主动事件转换为统一错误决策。硬件急停、状态切换和
 * 恢复动作均通过外部端口完成，避免错误仲裁层直接访问硬件。
 */
#include "pv_cleaning_robot/app/error_manager.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace robot::app {

ErrorManager::ErrorManager()
    : ErrorManager(Config{}) {}

ErrorManager::ErrorManager(Config config)
    : config_(config) {}

bool ErrorManager::same_component(ComponentId lhs, ComponentId rhs) noexcept {
    return lhs.kind == rhs.kind && lhs.index == rhs.index;
}

bool ErrorManager::same_error_key(const ErrorFact& lhs, const ErrorFact& rhs) noexcept {
    return lhs.code == rhs.code && same_component(lhs.component, rhs.component);
}

bool ErrorManager::is_motion_recovery_plan(RecoveryPlanId plan) noexcept {
    return plan == RecoveryPlanId::RecoverAttitudeCenter ||
           plan == RecoveryPlanId::RecoverAttitudeCenterThenReverse;
}

std::string ErrorManager::attitude_limit_key(const ErrorFact& fact) {
    if (fact.detail.empty()) {
        return "attitude_limit_key:unknown";
    }
    return fact.detail;
}

bool ErrorManager::should_suppress_locked(const ErrorFact& fact) const {
    if (!active_recovery_) {
        return false;
    }

    const auto active_component = active_recovery_->component;

    // 所有恢复动作都会暂停 GpsStuck；恢复期间出现的卡滞属于预期子错误。
    if (fact.code == ErrorCode::GpsStuck) {
        return true;
    }

    // 恢复期间同组件错误只记录为同根因持续存在，不启动第二个恢复动作。
    if (same_component(fact.component, active_component)) {
        if (fact.code == ErrorCode::DriverCommError ||
            fact.code == ErrorCode::BrushMotorFault) {
            return true;
        }
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
        decision.action = ErrorAction::FaultStopped;
        decision.latch_fault = true;
        return decision;
    case ErrorCode::WalkMotorStall:
        decision.action = ErrorAction::FaultStopped;
        decision.latch_fault = true;
        return decision;
    case ErrorCode::GpsStuck:
        decision.action = ErrorAction::FaultStopped;
        decision.latch_fault = true;
        return decision;
    case ErrorCode::AttitudeLimit:
        return decide_attitude_limit(fact);
    }

    decision.action = ErrorAction::WarnOnly;
    return decision;
}

ErrorDecision ErrorManager::decide_attitude_limit(const ErrorFact& fact) {
    ErrorDecision decision;
    decision.component = fact.component;
    decision.root_error = fact;

    const auto key = attitude_limit_key(fact);
    const bool repeated_after_recent_recovery =
        attitude_recovery_state_.has_last_key &&
        attitude_recovery_state_.last_key == key &&
        fact.timestamp_ms >= attitude_recovery_state_.last_recovery_finished_ms &&
        fact.timestamp_ms - attitude_recovery_state_.last_recovery_finished_ms <
            config_.attitude_repeat_gap_ms;

    if (!repeated_after_recent_recovery) {
        attitude_recovery_state_.has_last_key = true;
        attitude_recovery_state_.last_key = key;
        attitude_recovery_state_.repeat_count = 1;
    } else {
        ++attitude_recovery_state_.repeat_count;
    }

    if (attitude_recovery_state_.repeat_count >= config_.attitude_fault_count) {
        decision.action = ErrorAction::FaultStopped;
        decision.latch_fault = true;
        return decision;
    }

    decision.action = ErrorAction::StartRecovery;
    decision.requires_robot_recovering = true;
    decision.plan = attitude_recovery_state_.repeat_count == config_.attitude_reverse_attempt_count
                        ? RecoveryPlanId::RecoverAttitudeCenterThenReverse
                        : RecoveryPlanId::RecoverAttitudeCenter;
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

    if (state.consecutive_increments >= config_.consecutive_error_limit) {
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
                          config_.stream_timeout_ms,
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
                          config_.stream_timeout_ms,
                          now_ms);
    update_stream_timeout(snapshot.imu,
                          imu_timeout_state_,
                          ComponentKind::Imu,
                          config_.stream_timeout_ms,
                          now_ms);

    for (auto i = 0U; i < snapshot.walk_feedback.size(); ++i) {
        auto health = snapshot.walk_feedback[i];
        health.enabled = snapshot.walk_feedback_expected && health.enabled;
        // 每个行走轮独立维护“是否已对本次停滞上报”的状态；错误事实仍合并到
        // WalkMotorGroup 级别，避免四个轮同时停滞时启动多个恢复流程。
        update_stream_timeout(health,
                              walk_feedback_timeout_states_[i],
                              ComponentKind::WalkMotorGroup,
                              config_.stream_timeout_ms,
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
               now_ms - walk_stall_state_.active_since_ms >= config_.walk_stall_duration_ms) {
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
        if (result.decision.root_error.code == ErrorCode::AttitudeLimit) {
            attitude_recovery_state_.has_last_key = true;
            attitude_recovery_state_.last_key = attitude_limit_key(result.decision.root_error);
            attitude_recovery_state_.last_recovery_finished_ms = result.timestamp_ms;
        }
        active_recovery_.reset();
        return ErrorDecision{};
    }

    active_recovery_.reset();
    return ErrorDecision{};
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
    return plan == RecoveryPlanId::RecoverAttitudeCenter ||
           plan == RecoveryPlanId::RecoverAttitudeCenterThenReverse;
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
    // 不建立复杂因果图；同一调度周期只处理一个最高优先级根错误。
    // FaultStopped 必须压过所有恢复动作，避免先执行低优先级恢复再停机。
    if (decision.action == ErrorAction::FaultStopped || decision.latch_fault) {
        return 100;
    }
    if (decision.action != ErrorAction::StartRecovery) {
        return 0;
    }

    switch (decision.plan) {
    case RecoveryPlanId::RecoverAttitudeCenter:
    case RecoveryPlanId::RecoverAttitudeCenterThenReverse:
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
            // 只在任务执行中把它升级为回中恢复，空闲/自检/充电时不启动运动恢复。
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
        // 只允许任务运行态执行姿态恢复；其余状态丢弃本轮恢复，
        // 持续错误会在后续诊断中按统一策略再次提交。
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
