#pragma once

/// @file robot_domain.h
/// @brief 无人值守光伏清扫机器人的纯业务类型和纯函数。
///
/// Domain 层只表达“机器人在业务上是什么、任务如何被描述、状态机需要哪些事实”。
/// 本文件禁止引入硬件驱动、线程、文件、网络和云端 SDK 依赖；所有函数都应保持
/// 无副作用，便于单元测试和审查业务语义。
///
/// 核心术语：
/// - Endpoint::A/B 表示机器人物理行走轴上的两个固定端点。
/// - device/middleware 层可以继续使用左/右接近传感器，进入 domain 前必须转换为 A/B。
/// - primary_dock 表示配置中的主停机端：单停机位时是唯一停机/充电端，双停机位时是默认参考端。
/// - PositionState 是 app 层应使用的位置语义，避免到处重复解释两端限位。
/// - repeat_count 表示完整任务次数，不再使用历史的“趟数 passes”语义。

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robot::domain {

/// @brief 物理行走轴上的两个端点。
///
/// A/B 是安装标定坐标，不代表绝对南北、东西或机器人左右。当前装配约定是：
/// 左接近传感器触发 -> Endpoint::A，右接近传感器触发 -> Endpoint::B。
enum class Endpoint {
    A,
    B,
};

/// @brief 停机位模式，决定配置任务何时算完成。
enum class DockMode {
    /// 单停机位：从停机位出发，到返机位，再回到停机位，算一次完整任务。
    SingleDock,
    /// 双停机位：从任一停机位到另一停机位，算一次完整任务。
    DualDock,
};

/// @brief 命令来源，用于审计、上报和故障策略区分。
enum class CommandSource {
    /// 远程云端 RPC。
    Rpc,
    /// 本地调度窗口触发。
    Scheduler,
    /// 本地按钮、维护工具或近端控制。
    Local,
    /// 故障处理表触发的保护动作。
    FaultPolicy,
    /// 系统启动、恢复上下文等内部流程。
    System,
};

/// @brief 两端接近传感器的原始快照。
struct LimitState {
    bool a_active{false};
    bool b_active{false};
};

/// @brief 业务层统一使用的位置状态。
enum class PositionState {
    /// 无可靠位置事实。商业启动必须拒绝，避免盲动。
    Unknown,
    /// 已确认在物理 A 端。
    AtA,
    /// 已确认在物理 B 端。
    AtB,
    /// 在两端之间的板面区间。允许定向单段清扫，不允许配置完整任务启动。
    OnSegment,
    /// 两端同时触发或事实矛盾。必须禁止启动并上报故障。
    Inconsistent,
};

/// @brief 通道配置：停机端和任务完成模式。
struct LaneConfig {
    /// 单停机位或双停机位模式。
    DockMode dock_mode{DockMode::SingleDock};
    /// 主停机端。单停机位时是唯一停机/充电端；双停机位时作为默认参考端。
    Endpoint primary_dock{Endpoint::A};
};

/// @brief 转换为配置/云端使用的稳定字符串。
inline const char* endpoint_config_string(Endpoint value) noexcept {
    switch (value) {
    case Endpoint::A:
        return "A";
    case Endpoint::B:
        return "B";
    }
    return "A";
}

/// @brief 单个日内调度时间点。
///
/// 调度语义只表达“何时触发启动请求”，不在 domain 中绑定时区、RTC 或线程。
struct RuntimeScheduleEntry {
    /// 调度小时，24 小时制。
    int hour{0};
    /// 调度分钟。
    int minute{0};

    bool operator==(const RuntimeScheduleEntry& other) const {
        return hour == other.hour && minute == other.minute;
    }
};

/// @brief 运行期业务配置。
///
/// ConfigService 负责 active/pending 生效规则：
/// - schedules 修改立即作用于调度器；
/// - 其他运行参数进入 pending，在下一次任务启动前 promote 为 active；
/// - domain 只表达字段含义，不负责持久化、解析或云端协议。
struct RuntimeConfig {
    /// 完整任务次数：单停机位=去返一轮，双停机位=从一端到另一端。
    uint32_t repeat_count{1};
    /// 清扫段行走速度，业务配置使用绝对值，具体方向由 MotionService 根据任务段换算。
    double clean_speed_rpm{300.0};
    /// 向主停机端运行时的速度，业务配置使用绝对值。
    double return_speed_rpm{300.0};
    /// 滚刷目标转速绝对值；正反方向由任务段和停机端点决定。
    int brush_rpm{1000};
    /// 主停机端配置。云端修改后通常进入 pending，下次任务启动前生效。
    Endpoint primary_dock{Endpoint::A};
    /// 低电量启动门槛。当前策略是低于该值禁止启动，而不是运行中主动返航。
    double min_battery_soc{30.0};
    /// 充电停止阈值，用于业务状态和充电策略判断。
    double charge_stop_soc{95.0};
    /// 调度时间表。和其他运行参数不同，云端修改后应立即重装调度器。
    std::vector<RuntimeScheduleEntry> schedules;

    bool operator==(const RuntimeConfig& other) const {
        return repeat_count == other.repeat_count &&
               clean_speed_rpm == other.clean_speed_rpm &&
               return_speed_rpm == other.return_speed_rpm &&
               brush_rpm == other.brush_rpm &&
               primary_dock == other.primary_dock &&
               min_battery_soc == other.min_battery_soc &&
               charge_stop_soc == other.charge_stop_soc &&
               schedules == other.schedules;
    }
};

/// @brief 返回物理相对端。
inline Endpoint opposite_endpoint(Endpoint endpoint) noexcept {
    return endpoint == Endpoint::A ? Endpoint::B : Endpoint::A;
}

/// @brief 判断某个物理端是否为配置主停机端。
inline bool is_primary_dock(const LaneConfig& lane, Endpoint endpoint) noexcept {
    return lane.primary_dock == endpoint;
}

/// @brief 将位置状态转换为物理端点；不在端点时返回 nullopt。
inline std::optional<Endpoint> endpoint_from_position(PositionState state) noexcept {
    switch (state) {
    case PositionState::AtA:
        return Endpoint::A;
    case PositionState::AtB:
        return Endpoint::B;
    case PositionState::Unknown:
    case PositionState::OnSegment:
    case PositionState::Inconsistent:
        return std::nullopt;
    }
    return std::nullopt;
}

/// @brief 将原始限位快照转换成业务位置状态。
inline PositionState estimate_position(LimitState limits) noexcept {
    if (limits.a_active && limits.b_active) {
        return PositionState::Inconsistent;
    }
    if (!limits.a_active && !limits.b_active) {
        return PositionState::OnSegment;
    }
    return limits.a_active ? PositionState::AtA : PositionState::AtB;
}

/// @brief 判断是否是可用于定向 RPC 的可信位置。
inline bool is_trusted_position_state(PositionState state) noexcept {
    return state == PositionState::AtA || state == PositionState::AtB ||
           state == PositionState::OnSegment;
}

/// @brief 判断配置完整任务是否允许从当前位置启动。
inline bool can_start_configured_mission(const LaneConfig& lane, PositionState state) noexcept {
    const auto endpoint = endpoint_from_position(state);
    if (!endpoint) {
        return false;
    }
    return lane.dock_mode == DockMode::DualDock || *endpoint == lane.primary_dock;
}

/// @brief 当前运动沿物理轴的方向。
enum class TravelDirection {
    AToB,
    BToA,
};

inline TravelDirection travel_direction_to(Endpoint target) noexcept {
    return target == Endpoint::A ? TravelDirection::BToA : TravelDirection::AToB;
}

/// @brief 任务段动作模式；是否清扫由业务段显式决定，不由物理方向隐式推导。
enum class SegmentMode {
    /// 行走并开启滚刷，属于清扫段。
    Cleaning,
};

/// @brief 一个可执行任务段：物理目标端点 + 动作模式。
///
/// 该结构不包含速度、PID、方向符号等低层细节；这些由 MotionService 根据
/// RuntimeConfig、LaneConfig 和 Endpoint 统一换算。
struct SegmentSpec {
    /// 本段结束时应稳定确认的物理端点。
    Endpoint target{Endpoint::B};
    /// 本段的运动模式。
    SegmentMode mode{SegmentMode::Cleaning};
};

using MissionSegment = SegmentSpec;

enum class MissionKind {
    /// 按配置执行完整任务。
    ConfiguredMission,
    /// RPC: 无论当前位置，向主停机端的相对端清扫，到达后停止。
    CleanTowardOppositeEndpoint,
    /// RPC: 无论当前位置，向主停机端清扫，到达后停止。
    CleanTowardPrimaryDock,
};

/// @brief 当前任务的最小业务事实；控制器只推进段序号和完成次数，不直接控制硬件。
struct MissionContext {
    /// 任务来源语义，决定段生成规则和完成条件。
    MissionKind kind{MissionKind::ConfiguredMission};
    /// 命令来源，用于上报和审计。
    CommandSource source{CommandSource::System};
    /// 远程命令或调度命令的关联 ID；本地系统命令可为空。
    std::string command_id;
    /// 当前任务的段模板。配置任务在单停机位下包含“去返机端、回停机端”两段。
    std::vector<SegmentSpec> segments;
    /// 当前正在执行或等待端点确认的段序号。
    std::size_t current_segment_index{0};
    /// 目标完整任务次数。
    uint32_t repeat_count{1};
    /// 已完成完整任务次数；只有完成任务定义中的最后一段后才递增。
    uint32_t completed_cycles{0};

    /// @brief 返回当前段；任务完成、被停止或上下文异常时返回 nullptr。
    const SegmentSpec* current_segment() const noexcept {
        return current_segment_index < segments.size() ? &segments[current_segment_index] : nullptr;
    }
};

enum class RobotCommandKind {
    /// 按配置从停机位启动完整任务。
    StartConfiguredMission,
    /// 远程单段清扫到主停机端的相对端。
    CleanTowardOppositeEndpoint,
    /// 远程单段清扫到主停机端。
    CleanTowardPrimaryDock,
    /// 停止当前任务或当前运动。
    Stop,
    /// 云端故障复位：清除锁存故障并回到 Idle。
    FaultReset,
};

struct RobotCommand {
    /// 请求的业务动作。
    RobotCommandKind kind{RobotCommandKind::StartConfiguredMission};
    /// 请求来源。
    CommandSource source{CommandSource::System};
    /// 外部命令 ID，用于 RPC 应答和 command tracking；本地命令可为空。
    std::string command_id;
};

/// @brief 判断当前位置是否已经在指定任务目标端。
inline bool is_at_target(PositionState state, Endpoint target) noexcept {
    const auto endpoint = endpoint_from_position(state);
    return endpoint && *endpoint == target;
}

/// @brief 构造远程定向清扫任务。
///
/// 该任务始终只有一个清扫段，允许从端点或板面区间启动：
/// - CleanTowardOppositeEndpoint: 到主停机端的相对端停止；
/// - CleanTowardPrimaryDock: 到主停机端停止。
/// kind 必须是上述两类之一；调用方负责不要传入 ConfiguredMission。
inline MissionContext build_directional_clean_context(MissionKind kind,
                                                       const LaneConfig& lane,
                                                       CommandSource source,
                                                       std::string command_id) {
    MissionContext ctx;
    ctx.kind = kind;
    ctx.source = source;
    ctx.command_id = std::move(command_id);
    ctx.repeat_count = 1;
    ctx.completed_cycles = 0;
    ctx.segments.push_back(MissionSegment{
        kind == MissionKind::CleanTowardPrimaryDock ? lane.primary_dock
                                                    : opposite_endpoint(lane.primary_dock),
        SegmentMode::Cleaning});
    return ctx;
}

/// @brief 构造按配置执行的完整任务。
///
/// 单停机位：
/// - 固定生成两段：清扫到返机端，再清扫回停机端；
/// - 这两段全部完成后 completed_cycles 才递增。
///
/// 双停机位：
/// - 从当前端点清扫到另一端点；
/// - 单段完成即代表一次完整任务。
///
/// start_pose 只用于决定双停机位的首段方向；启动合法性由 RobotController 在调用前校验。
inline MissionContext build_configured_mission_context(const LaneConfig& lane,
                                                       PositionState start_state,
                                                       CommandSource source,
                                                       std::string command_id,
                                                       uint32_t repeat_count) {
    MissionContext ctx;
    ctx.kind = MissionKind::ConfiguredMission;
    ctx.source = source;
    ctx.command_id = std::move(command_id);
    ctx.repeat_count = repeat_count == 0 ? 1 : repeat_count;
    ctx.completed_cycles = 0;

    if (lane.dock_mode == DockMode::DualDock) {
        const auto start_endpoint = endpoint_from_position(start_state);
        ctx.segments.push_back(MissionSegment{
            start_endpoint ? opposite_endpoint(*start_endpoint) : opposite_endpoint(lane.primary_dock),
            SegmentMode::Cleaning});
        return ctx;
    }

    ctx.segments.push_back(
        MissionSegment{opposite_endpoint(lane.primary_dock), SegmentMode::Cleaning});
    ctx.segments.push_back(MissionSegment{lane.primary_dock, SegmentMode::Cleaning});
    return ctx;
}

namespace FaultCode {
// 常用关键故障源表。
//
// 编码规则：
// - 0x0xxx: 本地设备/安全监控原始故障，可由 app 层故障策略提升等级。
// - 0x1xxx: P0 类故障，立即急停并进入 FaultStopped。
// - 0x2xxx: P1 类故障，当前主流程只锁存上报，动作由策略表显式定义。
// - 0x3xxx: P2 类告警或业务拒绝，通常只上报，不强制改变运行状态。
//
// 这里定义的是跨层共享的稳定故障码，不在 domain 中放故障处理逻辑；
// 具体动作由 app::FaultPolicy 和 RobotController 统一执行。
static constexpr uint32_t kRobotStuck = 0x0002u;
static constexpr uint32_t kCanCommunicationLost = 0x1001u;
static constexpr uint32_t kSegmentStartFailed = 0x1101u;
static constexpr uint32_t kRecoveryFailed = 0x1102u;
static constexpr uint32_t kTaskContextInconsistent = 0x1103u;
static constexpr uint32_t kUnexpectedLimitSide = 0x1003u;
static constexpr uint32_t kConflictingLimitSides = 0x1004u;
static constexpr uint32_t kLimitUnstableAfterEmergencyStop = 0x1005u;
static constexpr uint32_t kSelfCheckFailed = 0x3001u;
static constexpr uint32_t kTransientAttitudeError = 0x3007u;
static constexpr uint32_t kStartRejectedBusy = 0x3003u;
static constexpr uint32_t kStartRejectedInvalidPosition = 0x3004u;
static constexpr uint32_t kStartRejectedLowBattery = 0x3005u;
static constexpr uint32_t kRuntimeConfigPromoteFailed = 0x3006u;
}  // namespace FaultCode

/// @brief 对云端和本地诊断暴露的业务运行快照。
///
/// Snapshot 是“只读事实投影”，不应被反向写回控制器。ThingsBoardControlPlane、
/// HealthService 或本地诊断工具可以使用它生成 telemetry/status payload。
struct RobotRuntimeSnapshot {
    /// 当前业务状态，使用 app::RobotState 对应的稳定字符串。
    std::string state;
    /// 最近的有效故障码；0 表示当前没有需要上报的故障。
    uint32_t fault{0};
    /// 当前任务计划执行的完整清扫次数。
    uint32_t repeat_count{0};
    /// 当前任务已经完成的完整清扫次数。
    uint32_t completed_cycles{0};
    /// 当前激活运行配置的版本号。
    uint64_t cfg_ver{0};
    /// 当前正在使用的运行时配置；无配置服务时可为空。
    std::optional<RuntimeConfig> active_config;
    /// 当前待下次任务生效的运行时配置；没有 pending 修改时为空。
    std::optional<RuntimeConfig> pending_config;
};

/// @brief 最小急停端口。
///
/// Domain 只定义“需要急停能力”这一边界，不关心急停由电机组、继电器还是安全
/// 控制器实现。这样 SafetyMonitor 或其他安全组件可依赖稳定接口，而不依赖具体服务。
class EmergencyStopPort {
   public:
    virtual ~EmergencyStopPort() = default;
    virtual void emergency_stop() = 0;
};

}  // namespace robot::domain
