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
struct StatePaused      {};  ///< 任务暂停，保留后续恢复所需上下文
struct StateCharging    {};  ///< 在停机位（充电或待机）
struct StateFault       {};  ///< 严重故障，等待人工复位
struct StateTerminated  {};  ///< 人工终止，等待复位重新投入服务

// ── 公开事件 ──────────────────────────────────────────────────────────
/// 调度器触发（SchedulerService 或 RPC "start"）
struct EvScheduleStart {
    bool  at_home{false};  ///< 尾端限位是否触发（设备在停机位）
    bool  at_front{false}; ///< 前端限位是否触发（设备在前端）
    float passes{1.0f};    ///< 本次任务趟数（首版仅支持整数趟）
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
struct EvPauseTask         {};  ///< 任务暂停（停刷、零速、可恢复）
struct EvResumeTask        {};  ///< 从 Paused 恢复当前任务
struct EvManualReturn      {};  ///< 任务级返回停机位，到位后本任务结束
struct EvTerminateTask     {};  ///< 人工终止任务，失能后等待复位

// ── 内部事件（仅在 dispatch<> 中使用，不对外派发）──────────────────
struct EvSelfCheckOk       {};  ///< 自检通过，从停机位正向出发
struct EvSelfCheckFail     {};  ///< 自检失败，留在 Idle
struct EvTaskComplete      {};  ///< 所有指定趟数完成 → Charging
struct EvResumeForward     {};  ///< 从暂停态恢复正向清扫
struct EvResumeReturn      {};  ///< 从暂停态恢复返程清扫

/// @brief 机器人有限状态机（Boost.SML）
///
/// 转换逻辑概述：
///   Idle/Charging   --EvScheduleStart→ SelfCheck（自检）
///   SelfCheck       --[ok,正向]→       CleanFwd
///   SelfCheck       --[fail]→          Idle（拒绝）
///   CleanFwd        --EvFrontLimitSettled→ CleanReturn  或  Charging（任务完成）
///   CleanReturn     --EvRearLimitSettled→  CleanFwd     或  Charging（趟数完成）
///   CleanFwd/Return --EvFaultP0→        Fault
///   CleanFwd/Return --EvFaultP1→        Returning（停刷）
///   CleanFwd/Return --EvFaultP2→        (不变，仅告警)
///   CleanFwd/Return --EvLowBattery→     Returning
///   CleanFwd/Return --EvPauseTask→      Paused
///   Paused         --EvResumeTask→      CleanFwd/CleanReturn（按暂停前方向恢复）
///   CleanFwd/Return/Paused --EvManualReturn→ Returning
///   CleanFwd/Return/Paused/Returning --EvTerminateTask→ Terminated
///   Returning       --EvRearLimitSettled→ Charging
///   Fault/Terminated --EvFaultReset→    Idle
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
    int target_half_passes() const;
    int completed_half_passes() const;

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
                state<StateSelfCheck>       + event<EvSelfCheckFail>         = state<StateIdle>,
                // 清扫往复
                state<StateCleanFwd>        + event<EvFrontLimitSettled>     = state<StateCleanReturn>,
                state<StateCleanFwd>        + event<EvTaskComplete>          = state<StateCharging>,
                state<StateCleanReturn>     + event<EvRearLimitSettled>      = state<StateCleanFwd>,
                state<StateCleanReturn>     + event<EvTaskComplete>          = state<StateCharging>,
                // 任务控制
                state<StateCleanFwd>        + event<EvPauseTask>             = state<StatePaused>,
                state<StateCleanReturn>     + event<EvPauseTask>             = state<StatePaused>,
                state<StatePaused>          + event<EvResumeForward>         = state<StateCleanFwd>,
                state<StatePaused>          + event<EvResumeReturn>          = state<StateCleanReturn>,
                state<StateCleanFwd>        + event<EvManualReturn>          = state<StateReturning>,
                state<StateCleanReturn>     + event<EvManualReturn>          = state<StateReturning>,
                state<StatePaused>          + event<EvManualReturn>          = state<StateReturning>,
                state<StateCleanFwd>        + event<EvTerminateTask>         = state<StateTerminated>,
                state<StateCleanReturn>     + event<EvTerminateTask>         = state<StateTerminated>,
                state<StatePaused>          + event<EvTerminateTask>         = state<StateTerminated>,
                state<StateReturning>       + event<EvTerminateTask>         = state<StateTerminated>,
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
                state<StateFault>           + event<EvFaultReset>            = state<StateIdle>,
                state<StateTerminated>      + event<EvFaultReset>            = state<StateIdle>
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
    int  target_half_passes_{2};     ///< passes * 2（1→2, 2→4, 3→6）
    int  completed_half_passes_{0};  ///< 已完成半趟数
    bool going_forward_{true};       ///< 当前方向（用于暂停后恢复）
};

} // namespace robot::app
