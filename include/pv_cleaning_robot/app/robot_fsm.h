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
struct StateSelfCheck   {};  ///< 调度触发后自检（确认当前在停机位 + 初始化计数器）
struct StateCleanFwd    {};  ///< 正向清扫（从停机位驶向对侧端点）
struct StateCleanReturn {};  ///< 返程清扫（从对侧端点返回停机位）
struct StateReturning   {};  ///< 主动返回停机位
struct StateStopped     {};  ///< 人工 stop 后原地停住，等待 start 或 return
struct StateCharging    {};  ///< 在停机位（充电或待机）
struct StateFault       {};  ///< 严重故障，等待人工复位

// ── 公开事件 ──────────────────────────────────────────────────────────
/// 调度器触发（SchedulerService 或 RPC "start"）
struct EvScheduleStart {
    bool  at_parking_side{false};  ///< 当前是否位于停机位一侧
    bool  at_far_end{false};       ///< 当前是否位于对侧端点
    float passes{1.0f};            ///< 本次任务趟数（首版仅支持整数趟）
};
/// 云端 RPC 特权启动：允许从非停机位直接开始完整清扫任务。
struct EvRpcStartTask {
    float passes{1.0f};  ///< 本次任务趟数（首版仅支持整数趟）
};
struct EvFarEndLimitSettled {};  ///< 对侧端点限位防抖完成（SafetyMonitor 延迟后发布）
struct EvParkingSideLimitSettled  { bool should_charge{true}; };  ///< 停机位一侧限位防抖完成
struct EvFaultP2           {};  ///< P2 故障（不转换状态，仅记录告警）
struct EvFaultP0           {};  ///< P0 严重故障 → Fault
struct EvFaultP1           {};  ///< P1 故障 → Fault
struct EvFaultReset        {};  ///< 故障复位（人工确认后）
struct EvChargeDone        {};  ///< 充电完成（BMS-less 电池兼容预留）
struct EvInitDone          {};  ///< 系统初始化完成
struct EvStopTask          {};  ///< 终止当前任务并原地停住
struct EvManualReturn      {};  ///< 任务级返回停机位，到位后本任务结束

// ── 内部事件（仅在 dispatch<> 中使用，不对外派发）──────────────────
struct EvSelfCheckOk       {};  ///< 自检通过，从停机位正向出发
struct EvSelfCheckFail     {};  ///< 自检失败，留在 Idle
struct EvTaskCompleteCharge {};  ///< 所有指定趟数完成且需充电
struct EvTaskCompleteIdle   {};  ///< 所有指定趟数完成且无需充电
struct EvReturnCompleteCharge {};  ///< 返回停机位后进入充电
struct EvReturnCompleteIdle   {};  ///< 返回停机位后进入空闲

/// @brief 机器人有限状态机（Boost.SML）
///
/// 转换逻辑概述：
///   Idle/Charging   --EvScheduleStart→ SelfCheck（自检）
///   SelfCheck       --[ok]→            CleanFwd
///   SelfCheck       --[fail]→          Idle（拒绝）
///   CleanFwd        --EvFarEndLimitSettled→ CleanReturn
///   CleanReturn     --EvParkingSideLimitSettled→  CleanFwd     或  Charging（整数趟完成）
///   CleanFwd/Return --EvFaultP0→        Fault
///   CleanFwd/Return --EvFaultP1→        Fault
///   CleanFwd/Return --EvFaultP2→        (不变，仅告警)
///   CleanFwd/Return/Idle/Stopped --EvManualReturn→ Returning
///   CleanFwd/Return/Returning --EvStopTask→ Stopped
///   Returning       --EvParkingSideLimitSettled→ Charging/Idle（回到停机位）
///   Fault --EvFaultReset→    Idle
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
    int target_passes() const;
    int completed_passes() const;

    // SML 状态机定义（必须在头文件中完整定义，sml::sm<> 模板需要完整类型）
    struct Fsm {
        auto operator()() const noexcept {
            using namespace sml;
            return make_transition_table(
                // 初始化
                *state<StateInit>           + event<EvInitDone>              = state<StateIdle>,
                // 调度触发 → 自检
                state<StateIdle>            + event<EvScheduleStart>         = state<StateSelfCheck>,
                state<StateStopped>         + event<EvScheduleStart>         = state<StateSelfCheck>,
                state<StateCharging>        + event<EvScheduleStart>         = state<StateSelfCheck>,
                // RPC 特权启动 → 自检（不要求当前位于停机位）
                state<StateIdle>            + event<EvRpcStartTask>          = state<StateSelfCheck>,
                state<StateStopped>         + event<EvRpcStartTask>          = state<StateSelfCheck>,
                state<StateCharging>        + event<EvRpcStartTask>          = state<StateSelfCheck>,
                // 自检结论
                state<StateSelfCheck>       + event<EvSelfCheckOk>           = state<StateCleanFwd>,
                state<StateSelfCheck>       + event<EvSelfCheckFail>         = state<StateIdle>,
                // 清扫往复
                state<StateCleanFwd>        + event<EvFarEndLimitSettled>     = state<StateCleanReturn>,
                state<StateCleanReturn>     + event<EvParkingSideLimitSettled>      = state<StateCleanFwd>,
                state<StateCleanReturn>     + event<EvTaskCompleteCharge>    = state<StateCharging>,
                state<StateCleanReturn>     + event<EvTaskCompleteIdle>      = state<StateIdle>,
                // 任务控制
                state<StateCleanFwd>        + event<EvManualReturn>          = state<StateReturning>,
                state<StateCleanReturn>     + event<EvManualReturn>          = state<StateReturning>,
                state<StateIdle>            + event<EvManualReturn>          = state<StateReturning>,
                state<StateStopped>         + event<EvManualReturn>          = state<StateReturning>,
                state<StateCleanFwd>        + event<EvStopTask>              = state<StateStopped>,
                state<StateCleanReturn>     + event<EvStopTask>              = state<StateStopped>,
                state<StateReturning>       + event<EvStopTask>              = state<StateStopped>,
                // 故障/返回
                state<StateCleanFwd>        + event<EvFaultP1>               = state<StateFault>,
                state<StateCleanReturn>     + event<EvFaultP1>               = state<StateFault>,
                state<StateReturning>       + event<EvReturnCompleteCharge>  = state<StateCharging>,
                state<StateReturning>       + event<EvReturnCompleteIdle>    = state<StateIdle>,
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
    int  target_passes_{1};          ///< 目标完整趟数（1=一去一回）
    int  completed_passes_{0};       ///< 已完成完整趟数
    bool going_forward_{true};       ///< 当前方向（用于任务中换向）
};

} // namespace robot::app
