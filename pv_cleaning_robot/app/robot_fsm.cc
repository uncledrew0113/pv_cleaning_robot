#include <algorithm>
#include <cmath>
#include <functional>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/app/robot_fsm.h"

namespace robot::app {

// ── RobotFsm 实现 ─────────────────────────────────────────────────────

RobotFsm::RobotFsm(std::shared_ptr<domain::MotionPort> motion,
                   std::shared_ptr<service::FaultReporter> fault,
                   middleware::EventBus& bus)
    : motion_(std::move(motion))
    , fault_(std::move(fault))
    , bus_(bus)
    , sm_(std::make_unique<sml::sm<Fsm>>()) {}

std::string RobotFsm::current_state() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return state_name_;
}

std::optional<SegmentDirection> RobotFsm::current_segment_direction() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    const auto* segment = mission_ ? mission_->current_segment() : nullptr;
    if (!segment || !sm_->is(sml::state<StateExecutingSegment>)) {
        return std::nullopt;
    }
    return segment->direction;
}

int RobotFsm::target_passes() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return target_passes_;
}

int RobotFsm::completed_passes() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return completed_passes_;
}

MissionContext RobotFsm::build_round_trip_mission(int passes) {
    MissionContext mission;
    mission.type = MissionType::RoundTripClean;
    mission.segments.reserve(static_cast<std::size_t>(passes) * 2u);
    for (int i = 0; i < passes; ++i) {
        mission.segments.push_back(
            {SegmentDirection::ToFarEnd, SegmentMode::Clean, CompletionCondition::ReachFarEnd});
        mission.segments.push_back({SegmentDirection::ToParkingSide,
                                    SegmentMode::Clean,
                                    CompletionCondition::ReachParkingSide});
    }
    return mission;
}

MissionContext RobotFsm::build_single_clean_mission(SegmentDirection direction) {
    MissionContext mission;
    mission.type = MissionType::SingleLegClean;
    mission.segments.push_back(
        {direction,
         SegmentMode::Clean,
         direction == SegmentDirection::ToFarEnd ? CompletionCondition::ReachFarEnd
                                                 : CompletionCondition::ReachParkingSide});
    return mission;
}

MissionContext RobotFsm::build_return_mission() {
    MissionContext mission;
    mission.type = MissionType::SingleLegReturn;
    mission.segments.push_back({SegmentDirection::ToParkingSide,
                                SegmentMode::ReturnNoBrush,
                                CompletionCondition::ReachParkingSide});
    return mission;
}

bool RobotFsm::start_current_segment() {
    if (!mission_) {
        return false;
    }
    const SegmentSpec* segment = mission_->current_segment();
    if (!segment) {
        return false;
    }
    switch (segment->mode) {
    case SegmentMode::Clean:
        return segment->direction == SegmentDirection::ToFarEnd ? motion_->start_cleaning()
                                                                : motion_->start_returning();
    case SegmentMode::ReturnNoBrush:
        return motion_->start_returning_no_brush();
    }
    return false;
}

bool RobotFsm::advance_to_next_segment() {
    if (!mission_) {
        return false;
    }
    if (mission_->current_segment_index + 1 >= mission_->segments.size()) {
        return false;
    }
    ++mission_->current_segment_index;
    return true;
}

void RobotFsm::handle_segment_start_failure(const char* context) {
    spdlog::error("[FSM] 段启动失败：{}", context ? context : "unknown");
    fault_->report(service::FaultService::FaultEvent::Level::P0,
                   service::FaultCode::kSegmentStartFailed,
                   context ? context : "segment_start_failed");
    bool should_stop = false;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->is(sml::state<StateFaultStopped>) && sm_->process_event(EvFaultP0{})) {
            mission_.reset();
            completed_passes_ = 0;
            target_passes_ = 0;
            state_name_ = "FaultStopped";
            should_stop = true;
        }
    }
    if (should_stop) {
        motion_->emergency_stop();
    }
}

// ── 事件分发特化 ──────────────────────────────────────────────────────
// 设计说明：
//   1. 所有 I/O（CAN/Modbus）在锁外执行，防止阻塞 EventBus 调用线程
//   2. sm_->process_event() 维护 SML 内部状态与 state_name_ 一致
//   3. 内部事件（EvSelfCheckOk 等）仅在本 .cc 内使用，不对外暴露

template <>
void RobotFsm::dispatch<EvInitDone>(EvInitDone e) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (!sm_->process_event(e)) {
        spdlog::warn("[FSM] 忽略 EvInitDone (state={})", state_name_);
        return;
    }
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle");
}

template <>
void RobotFsm::dispatch<EvScheduleStart>(EvScheduleStart e) {
    bool should_start_segment = false;
    auto report_self_check_failure = [this](const char* reason) {
        fault_->report(service::FaultService::FaultEvent::Level::P2,
                       service::FaultCode::kSelfCheckFailed,
                       reason);
    };
    {
        std::unique_lock<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvScheduleStart (state={})", state_name_);
            return;
        }

        const float rounded_passes = std::round(e.passes);
        const bool passes_is_integer =
            std::fabs(e.passes - rounded_passes) < 1e-4f && rounded_passes >= 1.0f;

        if (!passes_is_integer) {
            sm_->process_event(EvSelfCheckFail{});
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（自检失败：首版仅支持整数趟，拒绝 passes={:.1f}）",
                          e.passes);
            lk.unlock();
            report_self_check_failure("self_check_failed_invalid_passes");
            return;
        }

        completed_passes_ = 0;
        mission_.reset();

        state_name_ = "SelfCheck";
        spdlog::info("[FSM] → SelfCheck（趟数={:.1f}，目标完整趟数={}）",
                     e.passes,
                     static_cast<int>(rounded_passes));

        if (e.dual_dock_mode) {
            if (!e.at_parking_side && !e.at_far_end) {
                sm_->process_event(EvSelfCheckFail{});
                state_name_ = "Idle";
                spdlog::error("[FSM] → Idle（自检失败：双停机位模式要求当前位于任一端点）");
                lk.unlock();
                report_self_check_failure("self_check_failed_dual_dock_position");
                return;
            }
            target_passes_ = 1;
            mission_ = build_single_clean_mission(
                e.at_parking_side ? SegmentDirection::ToFarEnd : SegmentDirection::ToParkingSide);
        } else if (e.at_parking_side) {
            target_passes_ = static_cast<int>(rounded_passes);
            mission_ = build_round_trip_mission(target_passes_);
        } else {
            sm_->process_event(EvSelfCheckFail{});
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（自检失败：首版仅支持停机位启动，拒绝启动清扫）");
            lk.unlock();
            report_self_check_failure("self_check_failed_not_at_parking_side");
            return;
        }

        if (!sm_->process_event(EvSelfCheckOk{})) {
            spdlog::warn("[FSM] 忽略 EvSelfCheckOk (state={})", state_name_);
            sm_->process_event(EvSelfCheckFail{});
            mission_.reset();
            state_name_ = "Idle";
            lk.unlock();
            report_self_check_failure("self_check_transition_failed");
            return;
        }
        state_name_ = "ExecutingSegment";
        spdlog::info("[FSM] → ExecutingSegment（自检通过，开始执行任务段）");
        should_start_segment = true;
    }
    if (should_start_segment && !start_current_segment()) {
        handle_segment_start_failure("start_schedule_segment_failed");
    }
}

template <>
void RobotFsm::dispatch<EvRpcStartTask>(EvRpcStartTask e) {
    bool should_start_segment = false;
    auto report_self_check_failure = [this](const char* reason) {
        fault_->report(service::FaultService::FaultEvent::Level::P2,
                       service::FaultCode::kSelfCheckFailed,
                       reason);
    };
    {
        std::unique_lock<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvRpcStartTask (state={})", state_name_);
            return;
        }

        const float rounded_passes = std::round(e.passes);
        const bool passes_is_integer =
            std::fabs(e.passes - rounded_passes) < 1e-4f && rounded_passes >= 1.0f;

        if (!passes_is_integer) {
            sm_->process_event(EvSelfCheckFail{});
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（RPC start 失败：首版仅支持整数趟，拒绝 passes={:.1f}）",
                          e.passes);
            lk.unlock();
            report_self_check_failure("self_check_failed_invalid_passes");
            return;
        }

        completed_passes_ = 0;
        mission_.reset();

        state_name_ = "SelfCheck";
        spdlog::info("[FSM] → SelfCheck（RPC start，趟数={:.1f}，目标完整趟数={}）",
                     e.passes,
                     static_cast<int>(rounded_passes));

        if (e.dual_dock_mode) {
            target_passes_ = 1;
            mission_ = build_single_clean_mission(
                e.at_parking_side ? SegmentDirection::ToFarEnd : SegmentDirection::ToParkingSide);
        } else if (e.at_parking_side) {
            target_passes_ = static_cast<int>(rounded_passes);
            mission_ = build_round_trip_mission(target_passes_);
        } else {
            if (e.at_far_end) {
                target_passes_ = 1;
                mission_ = build_single_clean_mission(SegmentDirection::ToParkingSide);
            } else {
                target_passes_ = static_cast<int>(rounded_passes);
                mission_ = build_round_trip_mission(target_passes_);
            }
        }

        if (!sm_->process_event(EvSelfCheckOk{})) {
            spdlog::warn("[FSM] 忽略 RPC EvSelfCheckOk (state={})", state_name_);
            sm_->process_event(EvSelfCheckFail{});
            mission_.reset();
            state_name_ = "Idle";
            lk.unlock();
            report_self_check_failure("self_check_transition_failed");
            return;
        }
        state_name_ = "ExecutingSegment";
        spdlog::info("[FSM] → ExecutingSegment（RPC 特权启动，从当前位置开始清扫）");
        should_start_segment = true;
    }
    if (should_start_segment && !start_current_segment()) {
        handle_segment_start_failure("start_rpc_segment_failed");
    }
}

template <>
void RobotFsm::dispatch<EvFarEndLimitSettled>(EvFarEndLimitSettled e) {
    enum class PostAction { None, StartSegment, StopMotion };
    PostAction post_action{PostAction::None};
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        const auto* segment = mission_ ? mission_->current_segment() : nullptr;
        if (!segment || segment->completion != CompletionCondition::ReachFarEnd ||
            !sm_->is(sml::state<StateExecutingSegment>)) {
            spdlog::warn("[FSM] 忽略 EvFarEndLimitSettled (state={})", state_name_);
            return;
        }

        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvFarEndLimitSettled transition (state={})", state_name_);
            return;
        }
        state_name_ = "SegmentBoundary";
        if (mission_->type == MissionType::RoundTripClean) {
            if (!advance_to_next_segment()) {
                spdlog::warn("[FSM] 对侧端点后没有下一段");
                if (!sm_->process_event(EvTaskCompleteIdle{})) {
                    return;
                }
                state_name_ = "Idle";
                mission_.reset();
                post_action = PostAction::StopMotion;
            } else {
                if (!sm_->process_event(EvSegmentContinue{})) {
                    spdlog::warn("[FSM] 忽略 EvSegmentContinue (state={})", state_name_);
                    return;
                }
                state_name_ = "ExecutingSegment";
                spdlog::info("[FSM] → ExecutingSegment（到达对侧，切换到下一清扫段）");
                post_action = PostAction::StartSegment;
            }
        } else {
            completed_passes_ = 1;
            const bool ok = e.should_charge ? sm_->process_event(EvTaskCompleteCharge{})
                                            : sm_->process_event(EvTaskCompleteIdle{});
            if (!ok) {
                spdlog::warn("[FSM] 忽略对侧完成事件 (state={})", state_name_);
                return;
            }
            state_name_ = e.should_charge ? "Charging" : "Idle";
            mission_.reset();
            post_action = PostAction::StopMotion;
            spdlog::info("[FSM] → {}（到达对侧停机位，任务完成）", state_name_);
        }
    }
    if (post_action == PostAction::StartSegment) {
        if (!start_current_segment()) {
            handle_segment_start_failure("start_next_segment_after_far_end_failed");
        }
    } else if (post_action == PostAction::StopMotion) {
        motion_->stop_cleaning();
    }
}

template <>
void RobotFsm::dispatch<EvParkingSideLimitSettled>(EvParkingSideLimitSettled e) {
    enum class PostAction { None, StartSegment, StopMotion };
    PostAction post_action{PostAction::None};
    bool should_clear_fault{false};
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        const auto* segment = mission_ ? mission_->current_segment() : nullptr;
        if (!segment || segment->completion != CompletionCondition::ReachParkingSide ||
            !sm_->is(sml::state<StateExecutingSegment>)) {
            return;
        }

        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvParkingSideLimitSettled transition (state={})", state_name_);
            return;
        }
        state_name_ = "SegmentBoundary";
        if (mission_->type == MissionType::RoundTripClean &&
            segment->mode == SegmentMode::Clean) {
            completed_passes_++;
            spdlog::info("[FSM] 停机侧限位已稳定（已完成趟数 {}/{}）",
                         completed_passes_, target_passes_);

            if (completed_passes_ >= target_passes_) {
                const bool ok = e.should_charge ? sm_->process_event(EvTaskCompleteCharge{})
                                                : sm_->process_event(EvTaskCompleteIdle{});
                if (!ok) {
                    spdlog::warn("[FSM] 忽略 task complete after parking-side limit (state={})", state_name_);
                    return;
                }
                state_name_ = e.should_charge ? "Charging" : "Idle";
                spdlog::info("[FSM] → {}（全部趟数完成，回到停机位）", state_name_);
                mission_.reset();
                post_action = PostAction::StopMotion;
            } else {
                if (!advance_to_next_segment() || !sm_->process_event(EvSegmentContinue{})) {
                    spdlog::warn("[FSM] 忽略下一段切换 (state={})", state_name_);
                    return;
                }
                state_name_ = "ExecutingSegment";
                spdlog::info("[FSM] → ExecutingSegment（回到停机位，继续下一清扫段）");
                post_action = PostAction::StartSegment;
            }
        } else {
            if (mission_->type == MissionType::SingleLegClean) {
                completed_passes_ = 1;
            }
            const bool ok = (mission_->type == MissionType::SingleLegReturn)
                                ? (e.should_charge ? sm_->process_event(EvReturnCompleteCharge{})
                                                   : sm_->process_event(EvReturnCompleteIdle{}))
                                : (e.should_charge ? sm_->process_event(EvTaskCompleteCharge{})
                                                   : sm_->process_event(EvTaskCompleteIdle{}));
            if (!ok) {
                spdlog::warn("[FSM] 忽略停机侧完成事件 (state={})", state_name_);
                return;
            }
            state_name_ = e.should_charge ? "Charging" : "Idle";
            should_clear_fault = mission_->type == MissionType::SingleLegReturn;
            mission_.reset();
            spdlog::info("[FSM] → {}（返回停机位完成）", state_name_);
            post_action = PostAction::StopMotion;
        }
    }
    if (should_clear_fault) {
        fault_->clear_active_fault();
    }
    if (post_action == PostAction::StartSegment) {
        if (!start_current_segment()) {
            handle_segment_start_failure("start_next_segment_after_parking_side_failed");
        }
    } else if (post_action == PostAction::StopMotion) {
        motion_->stop_cleaning();
    }
}

template <>
void RobotFsm::dispatch<EvFaultP0>(EvFaultP0 e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvFaultP0 (state={})", state_name_);
            return;
        }
        mission_.reset();
        completed_passes_ = 0;
        target_passes_ = 0;
        state_name_ = "FaultStopped";
        spdlog::error("[FSM] → FaultStopped (P0 严重故障)");
        action = [this]() { motion_->emergency_stop(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvFaultP1>(EvFaultP1 e) {
    bool should_start_segment = false;
    bool should_report_escalated_p0 = false;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvFaultP1 (state={})", state_name_);
            return;
        }
        const auto* current_segment = mission_ ? mission_->current_segment() : nullptr;
        if (current_segment && current_segment->mode == SegmentMode::ReturnNoBrush) {
            spdlog::error("[FSM] 返航中再次出现 P1，升级为 P0");
            should_report_escalated_p0 = true;
        } else {
            mission_ = build_return_mission();
            target_passes_ = 0;
            completed_passes_ = 0;
            state_name_ = "ExecutingSegment";
            spdlog::warn("[FSM] → ExecutingSegment (P1 故障，切换到无刷返航段)");
            should_start_segment = true;
        }
    }
    if (should_report_escalated_p0) {
        // 锁外上报，统一走 FaultService → FaultHandler → EvFaultP0 的闭环，避免同步 publish
        // 在 FSM 锁内重入 dispatch。
        fault_->report(service::FaultService::FaultEvent::Level::P0,
                       service::FaultCode::kP1DuringReturnEscalatedToP0,
                       "p1_during_return_escalated_to_p0");
        return;
    }
    if (should_start_segment && !start_current_segment()) {
        handle_segment_start_failure("start_p1_return_segment_failed");
    }
}

template <>
void RobotFsm::dispatch<EvFaultP2>(EvFaultP2) {
    // P2 故障：不转换状态，仅记录告警
    // fault_->report() 已由 FaultHandler 在 EventBus 回调中调用
    spdlog::warn("[FSM] P2 故障告警，继续执行 (state={})", current_state());
}

template <>
void RobotFsm::dispatch<EvFaultReset>(EvFaultReset e) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (!sm_->process_event(e)) {
        spdlog::warn("[FSM] 忽略 EvFaultReset (state={})", state_name_);
        return;
    }
    mission_.reset();
    completed_passes_ = 0;
    target_passes_ = 0;
    state_name_ = "Idle";
    fault_->clear_active_fault();
    spdlog::info("[FSM] → Idle (故障复位)");
}

template <>
void RobotFsm::dispatch<EvChargeDone>(EvChargeDone e) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (!sm_->process_event(e)) {
        spdlog::warn("[FSM] 忽略 EvChargeDone (state={})", state_name_);
        return;
    }
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (充电完成)");
}

template <>
void RobotFsm::dispatch<EvManualReturn>(EvManualReturn e) {
    bool should_start_segment = false;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvManualReturn (state={})", state_name_);
            return;
        }
        mission_ = build_return_mission();
        target_passes_ = 0;
        completed_passes_ = 0;
        state_name_ = "ExecutingSegment";
        spdlog::info("[FSM] → ExecutingSegment（任务级返回停机位）");
        should_start_segment = true;
    }
    if (should_start_segment && !start_current_segment()) {
        handle_segment_start_failure("start_manual_return_segment_failed");
    }
}

template <>
void RobotFsm::dispatch<EvStopTask>(EvStopTask e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvStopTask (state={})", state_name_);
            return;
        }
        mission_.reset();
        state_name_ = "Idle";
        completed_passes_ = 0;
        target_passes_ = 0;
        spdlog::warn("[FSM] → Idle（任务停止，机器停住）");
        action = [this]() { motion_->stop_cleaning(); };
    }
    if (action) action();
}

} // namespace robot::app
