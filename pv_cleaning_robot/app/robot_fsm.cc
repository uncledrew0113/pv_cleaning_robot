#include "pv_cleaning_robot/app/robot_fsm.h"
#include <spdlog/spdlog.h>

namespace robot::app {

// ── RobotFsm 实现 ─────────────────────────────────────────────────────

RobotFsm::RobotFsm(std::shared_ptr<service::MotionService> motion,
                   std::shared_ptr<service::NavService>    nav,
                   std::shared_ptr<service::FaultService>  fault,
                   middleware::EventBus&                   bus)
    : motion_(std::move(motion))
    , nav_(std::move(nav))
    , fault_(std::move(fault))
    , bus_(bus)
    , sm_(std::make_unique<sml::sm<Fsm>>())
{
}

std::string RobotFsm::current_state() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return state_name_;
}

// 显式实例化需要在 .cc 中定义，以下是事件分发的外部显式特化：
// （由于 SML 在 .cc 中，模板定义必须在此处提供）

template <>
void RobotFsm::dispatch<EvStart>(EvStart e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Homing";
    spdlog::info("[FSM] → Homing");
    // 无实体 homing，直接发 HomeDone
    sm_->process_event(EvHomeDone{});
    state_name_ = "CleanFwd";
    spdlog::info("[FSM] → CleanFwd");
    motion_->start_cleaning();
}

template <>
void RobotFsm::dispatch<EvStop>(EvStop e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Returning";
    spdlog::info("[FSM] → Returning");
    motion_->start_returning();
}

template <>
void RobotFsm::dispatch<EvReachEnd>(EvReachEnd e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "CleanReturn";
    spdlog::info("[FSM] → CleanReturn");
}

template <>
void RobotFsm::dispatch<EvReachHome>(EvReachHome e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    spdlog::info("[FSM] EvReachHome processed");
}

template <>
void RobotFsm::dispatch<EvFaultP0>(EvFaultP0 e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Fault";
    spdlog::error("[FSM] → Fault (P0)");
    motion_->emergency_stop();
}

template <>
void RobotFsm::dispatch<EvFaultP1>(EvFaultP1 e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Returning";
    spdlog::warn("[FSM] → Returning (P1 fault)");
    motion_->start_returning();
}

template <>
void RobotFsm::dispatch<EvFaultReset>(EvFaultReset e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (fault reset)");
}

template <>
void RobotFsm::dispatch<EvLowBattery>(EvLowBattery e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Returning";
    spdlog::warn("[FSM] → Returning (low battery)");
    motion_->start_returning();
}

template <>
void RobotFsm::dispatch<EvChargeDone>(EvChargeDone e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (charge done)");
}

template <>
void RobotFsm::dispatch<EvInitDone>(EvInitDone e)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sm_->process_event(e);
    state_name_ = "Idle";
    spdlog::info("[FSM] → Idle (init done)");
}

} // namespace robot::app
