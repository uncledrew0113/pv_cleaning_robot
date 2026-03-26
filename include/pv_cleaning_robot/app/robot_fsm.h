#pragma once

// Boost.SML（需 C++17，header-only）
#include <boost/sml.hpp>
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include <memory>
#include <mutex>
#include <string>
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace sml = boost::sml;

namespace robot::app {

// ── 状态 ──────────────────────────────────────────────────────────────
struct StateInit        {};
struct StateIdle        {};
struct StateHoming      {};
struct StateCleanFwd    {};
struct StateCleanReturn {};
struct StateReturning   {};
struct StateCharging    {};
struct StateFault       {};

// ── 事件 ──────────────────────────────────────────────────────────────
struct EvStart          {};
struct EvStop           {};
struct EvHomeDone       {};
struct EvReachEnd       {};
struct EvReachHome      {};
struct EvChargeDone     {};
struct EvFaultP0        {};
struct EvFaultP1        {};
struct EvFaultReset     {};
struct EvLowBattery     {};
struct EvInitDone       {};

/// @brief 机器人有限状态机（Boost.SML）
///
/// 状态转换矩阵（简化描述）：
///   Init      --EvInitDone-->  Idle
///   Idle      --EvStart-->     Homing
///   Homing    --EvHomeDone-->  CleanFwd
///   CleanFwd  --EvReachEnd-->  CleanReturn
///   CleanReturn --EvReachHome--> CleanFwd    (N 趟往复)
///   CleanFwd  --EvStop-->      Returning
///   Returning --EvReachHome--> Charging
///   Charging  --EvChargeDone--> Idle
///   Any       --EvFaultP0-->   Fault
///   Any       --EvFaultP1-->   Returning
///   Fault     --EvFaultReset--> Idle
///   Any       --EvLowBattery--> Returning
class RobotFsm {
public:
    RobotFsm(std::shared_ptr<service::MotionService> motion,
             std::shared_ptr<service::NavService>    nav,
             std::shared_ptr<service::FaultService>  fault,
             middleware::EventBus&                   bus);

    /// 向状态机发送事件（线程安全）
    template <typename Event>
    void dispatch(Event e);

    std::string current_state() const;

    // SML 状态机定义（必须在头文件中完整定义，sml::sm<> 模板需要完整类型）
    struct Fsm {
        auto operator()() const noexcept {
            using namespace sml;
            return make_transition_table(
                // Init
                *state<StateInit>      + event<EvInitDone>    = state<StateIdle>,
                // Idle → Homing
                state<StateIdle>       + event<EvStart>        = state<StateHoming>,
                state<StateHoming>     + event<EvHomeDone>     = state<StateCleanFwd>,
                // 清扫往复
                state<StateCleanFwd>   + event<EvReachEnd>     = state<StateCleanReturn>,
                state<StateCleanReturn>+ event<EvReachHome>    = state<StateCleanFwd>,
                // 停止 → 返回停机位
                state<StateCleanFwd>   + event<EvStop>         = state<StateReturning>,
                state<StateCleanReturn>+ event<EvStop>         = state<StateReturning>,
                state<StateReturning>  + event<EvReachHome>    = state<StateCharging>,
                state<StateCharging>   + event<EvChargeDone>   = state<StateIdle>,
                // 低电量 → 返回
                state<StateCleanFwd>   + event<EvLowBattery>   = state<StateReturning>,
                state<StateCleanReturn>+ event<EvLowBattery>   = state<StateReturning>,
                // 故障
                state<StateCleanFwd>   + event<EvFaultP0>      = state<StateFault>,
                state<StateCleanReturn>+ event<EvFaultP0>      = state<StateFault>,
                state<StateReturning>  + event<EvFaultP0>      = state<StateFault>,
                state<StateCleanFwd>   + event<EvFaultP1>      = state<StateReturning>,
                state<StateCleanReturn>+ event<EvFaultP1>      = state<StateReturning>,
                state<StateFault>      + event<EvFaultReset>   = state<StateIdle>
            );
        }
    };

private:
    std::shared_ptr<service::MotionService> motion_;
    std::shared_ptr<service::NavService>    nav_;
    std::shared_ptr<service::FaultService>  fault_;
    middleware::EventBus&                   bus_;

    mutable hal::PiMutex          mtx_;
    std::string                   state_name_{"Init"};
    std::unique_ptr<sml::sm<Fsm>> sm_;
};

} // namespace robot::app
