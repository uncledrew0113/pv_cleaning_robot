#pragma once

// Boost.SML（需 C++17，header-only）
#include <boost/sml.hpp>
#include <boost/lockfree/spsc_queue.hpp>
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
struct StateSelfCheck   {};  ///< 调度触发后自检（确认停机位 + 初始化计数器）
struct StateCleanFwd    {};  ///< 正向清扫（从停机位向前）
struct StateCleanReturn {};  ///< 返程清扫（从前端返回停机位）
struct StateReturning   {};  ///< 故障/低电主动返回停机位
struct StateCharging    {};  ///< 在停机位（充电或待机）
struct StateFault       {};  ///< 严重故障，等待人工复位

// ── 公开事件 ──────────────────────────────────────────────────────────
/// 调度器触发（SchedulerService 或 RPC "start"）
struct EvScheduleStart {
    bool  at_home{false};  ///< 尾端限位是否触发（设备在停机位）
    bool  at_front{false}; ///< 前端限位是否触发（设备在前端，N=0.5 恢复场景）
    float passes{1.0f};    ///< 本次任务趟数（0.5=单程，1=一来回，2=两来回…）
};
struct EvFrontLimitSettled {};  ///< 前端限位防抖完成（SafetyMonitor 延迟后发布）
struct EvRearLimitSettled  {};  ///< 尾端限位防抖完成
struct EvFaultP2           {};  ///< P2 故障（不转换状态，仅记录告警）
struct EvFaultP0           {};  ///< P0 严重故障 → Fault
struct EvFaultP1           {};  ///< P1 故障 → 停刷安全返回
struct EvFaultReset        {};  ///< 故障复位（人工确认后）
struct EvLowBattery        {};  ///< 低电量 → 返回
struct EvChargeDone        {};  ///< 充电完成（BMS-less 电池兼容预留）
struct EvInitDone          {};  ///< 系统初始化完成

// ── 内部事件（仅在 dispatch<> 中使用，不对外派发）──────────────────
struct EvSelfCheckOk       {};  ///< 自检通过，从停机位正向出发
struct EvSelfCheckOkReturn {};  ///< 自检通过，从前端反向出发（N=0.5 恢复）
struct EvSelfCheckFail     {};  ///< 自检失败，留在 Idle
struct EvTaskComplete      {};  ///< 所有指定趟数完成 → Charging

/// @brief 机器人有限状态机（Boost.SML）
///
/// 转换逻辑概述：
///   Idle/Charging   --EvScheduleStart→ SelfCheck（自检）
///   SelfCheck       --[ok,正向]→       CleanFwd
///   SelfCheck       --[ok,反向]→       CleanReturn（N=0.5 恢复）
///   SelfCheck       --[fail]→          Idle（拒绝）
///   CleanFwd        --EvFrontLimitSettled→ CleanReturn  或  Charging（N=0.5 单程结束）
///   CleanReturn     --EvRearLimitSettled→  CleanFwd     或  Charging（趟数完成）
///   CleanFwd/Return --EvFaultP0→        Fault
///   CleanFwd/Return --EvFaultP1→        Returning（停刷）
///   CleanFwd/Return --EvFaultP2→        (不变，仅告警)
///   CleanFwd/Return --EvLowBattery→     Returning
///   Returning       --EvRearLimitSettled→ Charging
///   Fault           --EvFaultReset→     Idle
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
                // 初始化
                *state<StateInit>           + event<EvInitDone>              = state<StateIdle>,
                // 调度触发 → 自检
                state<StateIdle>            + event<EvScheduleStart>         = state<StateSelfCheck>,
                state<StateCharging>        + event<EvScheduleStart>         = state<StateSelfCheck>,
                // 自检结论
                state<StateSelfCheck>       + event<EvSelfCheckOk>           = state<StateCleanFwd>,
                state<StateSelfCheck>       + event<EvSelfCheckOkReturn>     = state<StateCleanReturn>,
                state<StateSelfCheck>       + event<EvSelfCheckFail>         = state<StateIdle>,
                // 清扫往复
                state<StateCleanFwd>        + event<EvFrontLimitSettled>     = state<StateCleanReturn>,
                state<StateCleanFwd>        + event<EvTaskComplete>          = state<StateCharging>,
                state<StateCleanReturn>     + event<EvRearLimitSettled>      = state<StateCleanFwd>,
                state<StateCleanReturn>     + event<EvTaskComplete>          = state<StateCharging>,
                // 故障/低电返回
                state<StateCleanFwd>        + event<EvLowBattery>            = state<StateReturning>,
                state<StateCleanReturn>     + event<EvLowBattery>            = state<StateReturning>,
                state<StateCleanFwd>        + event<EvFaultP1>               = state<StateReturning>,
                state<StateCleanReturn>     + event<EvFaultP1>               = state<StateReturning>,
                state<StateReturning>       + event<EvRearLimitSettled>      = state<StateCharging>,
                state<StateCharging>        + event<EvChargeDone>            = state<StateIdle>,
                // P0 故障
                state<StateCleanFwd>        + event<EvFaultP0>               = state<StateFault>,
                state<StateCleanReturn>     + event<EvFaultP0>               = state<StateFault>,
                state<StateReturning>       + event<EvFaultP0>               = state<StateFault>,
                // 故障复位
                state<StateFault>           + event<EvFaultReset>            = state<StateIdle>
            );
        }
    };

private:
    std::shared_ptr<service::MotionService> motion_;
    std::shared_ptr<service::NavService>    nav_;
    std::shared_ptr<service::FaultService>  fault_;
    middleware::EventBus&                   bus_;

    mutable hal::PiMutex          mtx_;
    /// 仅用于日志输出，不参与任何业务判断。
    /// 状态判断请使用 sm_->is(sml::state<StateXxx>)。
    std::string                   state_name_{"Init"};
    std::unique_ptr<sml::sm<Fsm>> sm_;

    // N 趟计数
    int  target_half_passes_{2};     ///< passes * 2（0.5→1, 1→2, 2→4）
    int  completed_half_passes_{0};  ///< 已完成半趟数
    bool going_forward_{true};       ///< 当前方向（N=0.5 跨任务翻转）
};

} // namespace robot::app
