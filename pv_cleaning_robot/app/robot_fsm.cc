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

// ── 事件分发特化 ──────────────────────────────────────────────────────
// 设计说明：
//   1. 所有 I/O（CAN/Modbus）在锁外执行，防止阻塞 EventBus 调用线程
//   2. sm_->process_event() 维护 SML 内部状态与 state_name_ 一致
//   3. 内部事件（EvSelfCheckOk 等）仅在本 .cc 内使用，不对外暴露

template <>
void RobotFsm::dispatch<EvInitDone>(EvInitDone e) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle");
}

template <>
void RobotFsm::dispatch<EvScheduleStart>(EvScheduleStart e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        // 计算目标半趟数（passes * 2，最小 1）
        target_half_passes_ = std::max(1, static_cast<int>(std::round(e.passes * 2.0f)));
        completed_half_passes_ = 0;

        // → SelfCheck
        sm_->process_event(e);
        state_name_ = "SelfCheck";
        spdlog::info("[FSM] → SelfCheck（趟数={:.1f}，目标半趟={}）",
                     e.passes, target_half_passes_);

        if (e.at_home) {
            // 正常情况：从停机位正向清扫
            going_forward_ = true;
            sm_->process_event(EvSelfCheckOk{});
            state_name_ = "CleanFwd";
            spdlog::info("[FSM] → CleanFwd（自检通过，从停机位出发）");
            action = [this]() { motion_->start_cleaning(); };
        } else if (e.at_front) {
            // N=0.5 恢复：上次停在前端，本次从前端反向返回
            going_forward_ = false;
            sm_->process_event(EvSelfCheckOkReturn{});
            state_name_ = "CleanReturn";
            spdlog::info("[FSM] → CleanReturn（自检：机器在前端，开始反向清扫返回）");
            action = [this]() { motion_->start_returning(); };
        } else {
            // 既不在停机位也不在前端 → 拒绝
            sm_->process_event(EvSelfCheckFail{});
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（自检失败：设备不在已知端点，拒绝启动清扫）");
        }
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvFrontLimitSettled>(EvFrontLimitSettled e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        completed_half_passes_++;
        spdlog::info("[FSM] 前端限位已稳定（已完成半趟 {}/{}）",
                     completed_half_passes_, target_half_passes_);

        if (completed_half_passes_ >= target_half_passes_) {
            // 任务完成（N=0.5 单程结束）
            sm_->process_event(EvTaskComplete{});
            state_name_ = "Charging";
            spdlog::info("[FSM] → Charging（任务完成，停在前端）");
            action = [this]() { motion_->stop_cleaning(); };
        } else {
            // 继续：前端到头，反向返回
            sm_->process_event(e);
            state_name_ = "CleanReturn";
            spdlog::info("[FSM] → CleanReturn（前端到达，刷反向返回）");
            action = [this]() { motion_->start_returning(); };
        }
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvRearLimitSettled>(EvRearLimitSettled e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);

        if (state_name_ == "CleanReturn") {
            completed_half_passes_++;
            spdlog::info("[FSM] 尾端限位已稳定（已完成半趟 {}/{}）",
                         completed_half_passes_, target_half_passes_);

            if (completed_half_passes_ >= target_half_passes_) {
                sm_->process_event(EvTaskComplete{});
                state_name_ = "Charging";
                spdlog::info("[FSM] → Charging（全部趟数完成，回到停机位）");
                action = [this]() { motion_->stop_cleaning(); };
            } else {
                sm_->process_event(e);
                state_name_ = "CleanFwd";
                spdlog::info("[FSM] → CleanFwd（回到停机位，继续正向清扫）");
                action = [this]() { motion_->start_cleaning(); };
            }
        } else if (state_name_ == "Returning") {
            sm_->process_event(e);
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
        sm_->process_event(e);
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
        sm_->process_event(e);
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
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (故障复位)");
}

template <>
void RobotFsm::dispatch<EvLowBattery>(EvLowBattery e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        sm_->process_event(e);
        state_name_ = "Returning";
        spdlog::warn("[FSM] → Returning (低电量，带刷返回)");
        // 低电量属于计划内返回，刷保持运行
        action = [this]() { motion_->start_returning(); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvChargeDone>(EvChargeDone e) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (充电完成)");
}

} // namespace robot::app
