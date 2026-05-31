#pragma once

// Boost.SML（需 C++17，header-only）
#include <boost/sml.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace sml = boost::sml;

namespace robot::app {

// ── 状态 ──────────────────────────────────────────────────────────────
struct StateInit {};
struct StateIdle {};
struct StateSelfCheck {};        ///< 启动前检查
struct StateExecutingSegment {}; ///< 执行当前任务段
struct StateSegmentBoundary {};  ///< 段完成后的瞬时决策态
struct StateCharging {};         ///< 在停机位（充电或待机）
struct StateFaultStopped {};     ///< 故障停机，等待人工复位

using robot::domain::CompletionCondition;
using robot::domain::MissionContext;
using robot::domain::MissionType;
using robot::domain::SegmentDirection;
using robot::domain::SegmentMode;
using robot::domain::SegmentSpec;

// ── 公开事件 ──────────────────────────────────────────────────────────
/// 调度器触发（SchedulerService 或 RPC "start"）
struct EvScheduleStart {
    bool  at_parking_side{false};  ///< 当前是否位于停机位一侧
    bool  at_far_end{false};       ///< 当前是否位于对侧端点
    float passes{1.0f};            ///< 本次任务趟数（首版仅支持整数趟）
    bool dual_dock_mode{false};    ///< 双停机位模式：任一端点都允许启动
};
/// 云端 RPC 特权启动：允许从非停机位直接开始完整清扫任务。
struct EvRpcStartTask {
    float passes{1.0f};          ///< 本次任务趟数（首版仅支持整数趟）
    bool at_parking_side{false}; ///< 当前是否位于配置停机位一侧
    bool at_far_end{false};      ///< 当前是否位于对侧端点
    bool dual_dock_mode{false};  ///< 双停机位模式：任一端点都允许启动
};
struct EvFarEndLimitSettled { bool should_charge{true}; };  ///< 对侧端点限位防抖完成（SafetyMonitor 延迟后发布）
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
struct EvSegmentContinue      {};  ///< 切换到下一段并继续执行

/// @brief 机器人有限状态机（Boost.SML）
///
/// 转换逻辑概述：
///   Idle/Charging   --EvScheduleStart→ SelfCheck（自检）
///   SelfCheck       --[ok]→            ExecutingSegment
///   SelfCheck       --[fail]→          Idle（拒绝）
///   ExecutingSegment --EvFarEndLimitSettled/EvParkingSideLimitSettled→ SegmentBoundary
///   SegmentBoundary --EvSegmentContinue→ ExecutingSegment
///   SegmentBoundary --EvTaskComplete*/EvReturnComplete*→ Idle/Charging
///   ExecutingSegment --EvFaultP0/EvFaultP1→ FaultStopped
///   ExecutingSegment --EvFaultP2→        (不变，仅告警)
///   ExecutingSegment/Idle --EvManualReturn→ ExecutingSegment
///   ExecutingSegment --EvStopTask→ Idle
///   FaultStopped --EvFaultReset→ Idle
class RobotFsm {
public:
    RobotFsm(std::shared_ptr<domain::MotionPort> motion,
             std::shared_ptr<service::FaultReporter> fault,
             middleware::EventBus&                   bus);

    /// 向状态机发送事件（线程安全）
    template <typename Event>
    void dispatch(Event e);

    std::string current_state() const;
    std::optional<SegmentDirection> current_segment_direction() const;
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
                state<StateCharging>        + event<EvScheduleStart>         = state<StateSelfCheck>,
                // RPC 特权启动 → 自检（不要求当前位于停机位）
                state<StateIdle>            + event<EvRpcStartTask>          = state<StateSelfCheck>,
                state<StateCharging>        + event<EvRpcStartTask>          = state<StateSelfCheck>,
                // 自检结论
                state<StateSelfCheck>       + event<EvSelfCheckOk>           = state<StateExecutingSegment>,
                state<StateSelfCheck>       + event<EvSelfCheckFail>         = state<StateIdle>,
                // 任务段执行/换段
                state<StateExecutingSegment> + event<EvFarEndLimitSettled>   = state<StateSegmentBoundary>,
                state<StateExecutingSegment> + event<EvParkingSideLimitSettled> = state<StateSegmentBoundary>,
                state<StateSegmentBoundary> + event<EvSegmentContinue>       = state<StateExecutingSegment>,
                state<StateSegmentBoundary> + event<EvTaskCompleteCharge>    = state<StateCharging>,
                state<StateSegmentBoundary> + event<EvTaskCompleteIdle>      = state<StateIdle>,
                state<StateSegmentBoundary> + event<EvReturnCompleteCharge>  = state<StateCharging>,
                state<StateSegmentBoundary> + event<EvReturnCompleteIdle>    = state<StateIdle>,
                // 任务控制
                state<StateExecutingSegment> + event<EvManualReturn>         = state<StateExecutingSegment>,
                state<StateIdle>            + event<EvManualReturn>          = state<StateExecutingSegment>,
                state<StateExecutingSegment> + event<EvStopTask>             = state<StateIdle>,
                // 故障/返回
                state<StateCharging>        + event<EvChargeDone>            = state<StateIdle>,
                // P1/P0 故障
                state<StateExecutingSegment> + event<EvFaultP1>              = state<StateExecutingSegment>,
                state<StateExecutingSegment> + event<EvFaultP0>              = state<StateFaultStopped>,
                // 故障复位
                state<StateFaultStopped>    + event<EvFaultReset>            = state<StateIdle>
            );
        }
    };

private:
    std::shared_ptr<domain::MotionPort>     motion_;
    std::shared_ptr<service::FaultReporter> fault_;
    middleware::EventBus&                   bus_;

    mutable hal::PiMutex          mtx_;
    /// 仅用于日志输出，不参与任何业务判断。
    /// 状态判断请使用 sm_->is(sml::state<StateXxx>)。
    std::string                   state_name_{"Init"};
    std::unique_ptr<sml::sm<Fsm>> sm_;
    std::optional<MissionContext> mission_;

    // N 趟计数
    int  target_passes_{1};          ///< 目标完整趟数（1=一去一回）
    int  completed_passes_{0};       ///< 已完成完整趟数

    static MissionContext build_round_trip_mission(int passes);
    static MissionContext build_single_clean_mission(SegmentDirection direction);
    static MissionContext build_return_mission();
    bool start_current_segment();
    bool advance_to_next_segment();
    void handle_segment_start_failure(const char* context);
};

} // namespace robot::app
