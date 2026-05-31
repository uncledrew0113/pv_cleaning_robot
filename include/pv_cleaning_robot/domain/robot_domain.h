#pragma once

/// @file robot_domain.h
/// @brief 无人值守光伏清扫机器人的纯业务类型和纯函数。

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace robot::domain {

enum class ParkingSide {
    Left,
    Right,
};

enum class PhysicalLimitSide {
    Left,
    Right,
};

inline const char* parking_side_config_string(ParkingSide value) noexcept {
    switch (value) {
    case ParkingSide::Left:
        return "left";
    case ParkingSide::Right:
        return "right";
    }
    return "left";
}

struct RuntimeScheduleEntry {
    int hour{0};
    int minute{0};

    bool operator==(const RuntimeScheduleEntry& other) const {
        return hour == other.hour && minute == other.minute;
    }
};

struct RuntimeConfig {
    double passes{1.0};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    ParkingSide parking_side{ParkingSide::Left};
    double min_battery_soc{30.0};
    double charge_stop_soc{95.0};
    std::vector<RuntimeScheduleEntry> schedules;

    bool operator==(const RuntimeConfig& other) const {
        return passes == other.passes &&
               clean_speed_rpm == other.clean_speed_rpm &&
               return_speed_rpm == other.return_speed_rpm &&
               brush_rpm == other.brush_rpm &&
               parking_side == other.parking_side &&
               min_battery_soc == other.min_battery_soc &&
               charge_stop_soc == other.charge_stop_soc &&
               schedules == other.schedules;
    }
};

struct ParkingSideFacts {
    /// 左限位开关是否触发
    bool left_limit_active{false};
    /// 右限位开关是否触发
    bool right_limit_active{false};
    /// 是否处于停机侧位置
    bool at_parking_side{false};
    /// 是否处于远端侧位置
    bool at_far_end{false};
    /// 两端限位同时触发表示异常双端点
    bool dual_endpoint_active{false};
    /// 无限位触发表示位置不可判定
    bool no_endpoint_active{false};

    /// @brief 返回是否为合法的任务启动位置
    bool is_valid_start_position(bool dual_dock_mode = false) const noexcept {
        if (dual_endpoint_active || no_endpoint_active) {
            return false;
        }
        return at_parking_side || (dual_dock_mode && at_far_end);
    }

    /// @brief 返回是否存在异常位置状态
    bool is_invalid_position() const noexcept {
        return dual_endpoint_active || no_endpoint_active;
    }
};

struct ParkingSideRuntime {
    /// @brief 从实际限位开关状态生成停车侧运行事实。
    static ParkingSideFacts from_physical_limits(ParkingSide parking_side,
                                                 bool left_limit_active,
                                                 bool right_limit_active) noexcept {
        ParkingSideFacts facts;
        facts.left_limit_active = left_limit_active;
        facts.right_limit_active = right_limit_active;
        facts.dual_endpoint_active = left_limit_active && right_limit_active;
        facts.no_endpoint_active = !left_limit_active && !right_limit_active;
        if (parking_side == ParkingSide::Left) {
            facts.at_parking_side = left_limit_active;
            facts.at_far_end = right_limit_active;
            return facts;
        }
        facts.at_parking_side = right_limit_active;
        facts.at_far_end = left_limit_active;
        return facts;
    }
};

enum class MissionType {
    RoundTripClean,
    SingleLegClean,
    SingleLegReturn,
};

enum class SegmentDirection {
    ToFarEnd,
    ToParkingSide,
};

enum class SegmentMode {
    Clean,
    ReturnNoBrush,
};

enum class CompletionCondition {
    ReachFarEnd,
    ReachParkingSide,
};

struct SegmentSpec {
    SegmentDirection direction{SegmentDirection::ToFarEnd};
    SegmentMode mode{SegmentMode::Clean};
    CompletionCondition completion{CompletionCondition::ReachFarEnd};
};

struct MissionContext {
    MissionType type{MissionType::RoundTripClean};
    std::vector<SegmentSpec> segments;
    std::size_t current_segment_index{0};

    const SegmentSpec* current_segment() const noexcept {
        return current_segment_index < segments.size() ? &segments[current_segment_index] : nullptr;
    }
};

namespace FaultCode {
// 常用关键故障源表：
// - 0x0xxx: device/safety local faults
// - 0x1xxx: P0, immediate emergency stop and FaultStopped
// - 0x2xxx: P1, stop brush and return to parking side
// - 0x3xxx: P2, warning/status event; selected codes may be promoted by FaultHandler
static constexpr uint32_t kWheelSpinFree = 0x0002u;
static constexpr uint32_t kCanCommunicationLost = 0x1001u;
static constexpr uint32_t kSegmentStartFailed = 0x1101u;
static constexpr uint32_t kP1DuringReturnEscalatedToP0 = 0x1102u;
static constexpr uint32_t kUnexpectedLimitSide = 0x1003u;
static constexpr uint32_t kConflictingLimitSides = 0x1004u;
static constexpr uint32_t kLimitUnstableAfterEmergencyStop = 0x1005u;
static constexpr uint32_t kSelfCheckFailed = 0x3001u;
static constexpr uint32_t kStartRejectedBusy = 0x3003u;
static constexpr uint32_t kStartRejectedInvalidPosition = 0x3004u;
static constexpr uint32_t kStartRejectedLowBattery = 0x3005u;
static constexpr uint32_t kRuntimeConfigPromoteFailed = 0x3006u;
static constexpr uint32_t kBrushFaultReturnRequired = 0x2001u;
static constexpr uint32_t kReturnPathBlocked = 0x2002u;
static constexpr uint32_t kGpsLostRequiresReturn = 0x3002u;
}  // namespace FaultCode

struct RobotRuntimeSnapshot {
    /// 周期业务真相：当前业务状态。
    std::string state;
    /// 最近的有效故障码；0 表示当前没有需要上报的故障。
    uint32_t fault{0};
    /// 期望通过次数
    int target_passes{0};
    /// 已完成的通过次数
    int completed_passes{0};
    /// 累计清扫次数
    int clean_count{0};
    /// 当前激活运行配置的版本号
    uint64_t cfg_ver{0};
    /// 当前正在使用的云端运行时配置
    std::optional<RuntimeConfig> active_config;
    /// 当前待生效的云端运行时配置
    std::optional<RuntimeConfig> pending_config;
};

class EmergencyStopPort {
   public:
    virtual ~EmergencyStopPort() = default;
    virtual void emergency_stop() = 0;
};

class MotionPort : public EmergencyStopPort {
   public:
    ~MotionPort() override = default;
    virtual bool start_cleaning() = 0;
    virtual void stop_cleaning() = 0;
    virtual bool start_returning() = 0;
    virtual bool start_returning_no_brush() = 0;
};

struct RobotControlPort {
    std::function<std::string()> current_state;
    std::function<bool(bool at_parking_side,
                       bool at_far_end,
                       bool position_valid,
                       float battery_soc)>
        start_task_from_current_position;
    std::function<bool()> stop_task;
    std::function<bool(bool at_parking_side)> return_task;
    std::function<RobotRuntimeSnapshot()> snapshot;
};

}  // namespace robot::domain
