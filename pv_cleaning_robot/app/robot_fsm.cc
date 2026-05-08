#include <algorithm>
#include <cmath>
#include <functional>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/app/robot_fsm.h"

namespace robot::app {

// ── RobotFsm 实现 ─────────────────────────────────────────────────────

RobotFsm::RobotFsm(std::shared_ptr<service::MotionService> motion,
                   std::shared_ptr<service::NavService> nav,
                   std::shared_ptr<service::FaultService> fault,
                   middleware::EventBus& bus)
    : motion_(std::move(motion))
    , nav_(std::move(nav))
    , fault_(std::move(fault))
    , bus_(bus)
    , sm_(std::make_unique<sml::sm<Fsm>>()) {}

std::string RobotFsm::current_state() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return state_name_;
}

int RobotFsm::target_passes() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return target_passes_;
}

int RobotFsm::completed_passes() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return completed_passes_;
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
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
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
            return;
        }

        target_passes_ = static_cast<int>(rounded_passes);
        completed_passes_ = 0;

        state_name_ = "SelfCheck";
        spdlog::info("[FSM] → SelfCheck（趟数={:.1f}，目标完整趟数={}）",
                     e.passes, target_passes_);

        if (e.at_parking_side) {
            // 正常情况：从停机位正向清扫
            going_forward_ = true;
            if (!sm_->process_event(EvSelfCheckOk{})) {
                spdlog::warn("[FSM] 忽略 EvSelfCheckOk (state={})", state_name_);
                sm_->process_event(EvSelfCheckFail{});
                state_name_ = "Idle";
                return;
            }
            state_name_ = "CleanFwd";
            spdlog::info("[FSM] → CleanFwd（自检通过，从停机位出发）");
            action = [this]() { motion_->start_cleaning(); };
        } else {
            // 首版仅允许从停机位启动
            sm_->process_event(EvSelfCheckFail{});
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（自检失败：首版仅支持停机位启动，拒绝启动清扫）");
        }
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvFarEndLimitSettled>(EvFarEndLimitSettled e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->is(sml::state<StateCleanFwd>)) {
            spdlog::warn("[FSM] 忽略 EvFarEndLimitSettled (state={})", state_name_);
            return;
        }

        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvFarEndLimitSettled transition (state={})", state_name_);
            return;
        }
        going_forward_ = false;
        state_name_ = "CleanReturn";
        spdlog::info("[FSM] → CleanReturn（到达对侧，刷反向返回）");
        action = [this]() { motion_->start_returning(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvParkingSideLimitSettled>(EvParkingSideLimitSettled e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);

        if (sm_->is(sml::state<StateCleanReturn>)) {
            completed_passes_++;
            spdlog::info("[FSM] 停机侧限位已稳定（已完成趟数 {}/{}）",
                         completed_passes_, target_passes_);

            if (completed_passes_ >= target_passes_) {
                if (!sm_->process_event(EvTaskComplete{})) {
                    spdlog::warn("[FSM] 忽略 EvTaskComplete after parking-side limit (state={})", state_name_);
                    return;
                }
                state_name_ = "Charging";
                spdlog::info("[FSM] → Charging（全部趟数完成，回到停机位）");
                action = [this]() { motion_->stop_cleaning(); };
            } else {
                if (!sm_->process_event(e)) {
                    spdlog::warn("[FSM] 忽略 EvParkingSideLimitSettled transition (state={})", state_name_);
                    return;
                }
                going_forward_ = true;
                state_name_ = "CleanFwd";
                spdlog::info("[FSM] → CleanFwd（回到停机位，继续正向清扫）");
                action = [this]() { motion_->start_cleaning(); };
            }
        } else if (sm_->is(sml::state<StateReturning>)) {
            if (!sm_->process_event(e)) {
                spdlog::warn("[FSM] 忽略 Returning->EvParkingSideLimitSettled (state={})", state_name_);
                return;
            }
            state_name_ = "Charging";
            spdlog::info("[FSM] → Charging（故障/低电返回停机位完成）");
            action = [this]() { motion_->stop_cleaning(); };
        }
        // 其他状态收到尾端信号：忽略
    }
    if (action) action();
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
        state_name_ = "Fault";
        spdlog::error("[FSM] → Fault (P0 严重故障)");
        action = [this]() { motion_->emergency_stop(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvFaultP1>(EvFaultP1 e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvFaultP1 (state={})", state_name_);
            return;
        }
        state_name_ = "Returning";
        spdlog::warn("[FSM] → Returning (P1 故障，停刷安全返回)");
        action = [this]() { motion_->start_returning_no_brush(); };
    }
    if (action) action();
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
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (故障复位)");
}

template <>
void RobotFsm::dispatch<EvLowBattery>(EvLowBattery e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvLowBattery (state={})", state_name_);
            return;
        }
        state_name_ = "Returning";
        spdlog::warn("[FSM] → Returning (低电量，带刷返回)");
        going_forward_ = false;
        // 低电量属于计划内返回，刷保持运行
        action = [this]() { motion_->start_returning(); };
    }
    if (action) action();
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
void RobotFsm::dispatch<EvPauseTask>(EvPauseTask e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvPauseTask (state={})", state_name_);
            return;
        }
        state_name_ = "Paused";
        spdlog::info("[FSM] → Paused (任务暂停)");
        action = [this]() { motion_->pause_task(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvResumeTask>(EvResumeTask) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->is(sml::state<StatePaused>)) {
            spdlog::warn("[FSM] 忽略 EvResumeTask (state={})", state_name_);
            return;
        }

        const bool resume_forward = going_forward_;
        const bool ok = resume_forward ? sm_->process_event(EvResumeForward{})
                                       : sm_->process_event(EvResumeReturn{});
        if (!ok) {
            spdlog::warn("[FSM] 忽略 EvResumeTask transition (state={})", state_name_);
            return;
        }

        if (resume_forward) {
            state_name_ = "CleanFwd";
            spdlog::info("[FSM] → CleanFwd（恢复暂停任务）");
            action = [this]() { motion_->start_cleaning(); };
        } else {
            state_name_ = "CleanReturn";
            spdlog::info("[FSM] → CleanReturn（恢复暂停任务）");
            action = [this]() { motion_->start_returning(); };
        }
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvManualReturn>(EvManualReturn e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvManualReturn (state={})", state_name_);
            return;
        }
        state_name_ = "Returning";
        going_forward_ = false;
        completed_passes_ = target_passes_;
        spdlog::info("[FSM] → Returning（任务级返回停机位）");
        action = [this]() { motion_->start_returning(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvTerminateTask>(EvTerminateTask e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvTerminateTask (state={})", state_name_);
            return;
        }
        state_name_ = "Terminated";
        completed_passes_ = 0;
        target_passes_ = 0;
        spdlog::warn("[FSM] → Terminated（人工终止任务）");
        action = [this]() { motion_->emergency_stop(); };
    }
    if (action) action();
}

} // namespace robot::app
