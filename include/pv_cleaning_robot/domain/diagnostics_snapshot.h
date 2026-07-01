#pragma once

#include <array>
#include <cstdint>

namespace robot::domain {

/// @brief 数据流健康指标，表示最近一次收到有效数据的时间。
///
/// 采集层在发现设备原始计数变化时更新 last_update_ms。消费者只判断数据年龄，
/// 不依赖自身调用周期推断 GPS、IMU、行走反馈等设备的真实更新频率。
struct StreamHealth {
    bool enabled{false};
    uint64_t last_update_ms{0};
};

/// @brief 错误累计计数类健康指标，例如滚刷通信错误计数。
struct ErrorCounterHealth {
    bool enabled{false};
    uint32_t error_count{0};
};

/// @brief 系统诊断快照。
///
/// DiagnosticsCollector 负责生产该快照；HealthService 和 ErrorManager 消费同一份事实，
/// 避免重复读取设备诊断或重复处理计数变化。
struct DiagnosticsSnapshot {
    // BMS 通信用“成功 update_count 是否持续变化”判断，比 error_count 更能表达数据流是否恢复。
    StreamHealth bms_update;
    ErrorCounterHealth brush;
    StreamHealth gps;
    StreamHealth imu;
    std::array<StreamHealth, 4> walk_feedback{};
    bool walk_feedback_expected{false};
    bool walk_stall_active{false};
    bool brush_fault_active{false};
    bool gps_stuck{false};
};

}  // namespace robot::domain
