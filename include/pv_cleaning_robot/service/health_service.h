/**
 * @file health_service.h
 * @brief 健康状态和诊断上报服务接口。
 *
 * HealthService 基于 DiagnosticsCollector 快照生成云端遥测或本地 JSONL 记录。上报模式决定字段
 * 粒度，不改变底层诊断采集频率。
 */
#pragma once
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/diagnostics_collector.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace spdlog {
class logger;
}

namespace robot::service {

// 健康 payload 序列化只由 HealthService 在运行时消费。格式契约保留在服务边界内，
// 避免为了单一调用点再拆出额外 helper 文件，增加维护跳转成本。
class HealthPayloadBuilder {
public:
    struct HealthView {
        uint64_t ts_ms{0};
        device::WalkMotorGroup::GroupStatus walk{};
        device::BrushMotor::Status brush{};
        device::BMS::BatteryData bms{};
        device::ImuDevice::ImuData imu{};
        device::GpsDevice::GpsData gps{};
    };

    struct DiagnosticsView {
        uint64_t ts_ms{0};
        device::WalkMotorGroup::GroupDiagnostics walk{};
        device::BrushMotor::Diagnostics brush{};
        device::BMS::Diagnostics bms{};
        device::ImuDevice::Diagnostics imu{};
        device::GpsDevice::Diagnostics gps{};
    };

    static size_t build_health(const HealthView& view, char* out, size_t cap) noexcept;
    static size_t build_health(const DiagnosticsCollector::Snapshot& snapshot,
                               char* out,
                               size_t cap) noexcept;
    static size_t build_diagnostics(const DiagnosticsView& view,
                                    char* out,
                                    size_t cap) noexcept;
    static size_t build_diagnostics(const DiagnosticsCollector::Snapshot& snapshot,
                                    char* out,
                                    size_t cap) noexcept;
};

/// @brief 遥测上报服务（HEALTH / DIAGNOSTICS 双模式）
///
/// HEALTH 模式（生产）：精简 Status 字段，适合 4G/LoRaWAN 低带宽场景。
/// DIAGNOSTICS 模式（开发）：完整 Diagnostics 字段，便于快速定位问题。
/// 通过 fixed config 的 diagnostics.mode 字段在启动时选择，无需更换类。
///
/// 实际上报频率不在本类内部固定，而是由外部 ThreadExecutor 调度决定。
/// 当前主程序会根据 RobotController 的运行态在 active / idle 两档周期之间切换，
/// 因此 Health telemetry 和 business telemetry 会一起跟随该周期变化。
///
/// 当 local_log_path 非空时，update() 在每次上报时额外把 JSON payload 以
/// JSONL 格式追加写入本地文件，完全独立于 MQTT/LoRaWAN，离线测试阶段也可用。
class HealthService : public middleware::IRunnable {
public:
    enum class Mode { HEALTH, DIAGNOSTICS };

    HealthService(std::shared_ptr<DiagnosticsCollector> diagnostics,
                  std::shared_ptr<CloudService> cloud,
                  Mode mode = Mode::HEALTH,
                  std::string local_log_path = "",
                  size_t local_log_max_bytes = 10u * 1024u * 1024u,
                  size_t local_log_max_files = 3u);

    HealthService(std::shared_ptr<device::WalkMotorGroup> walk,
                  std::shared_ptr<device::BrushMotor>     brush,
                  std::shared_ptr<device::BMS>            bms,
                  std::shared_ptr<device::ImuDevice>      imu,
                  std::shared_ptr<device::GpsDevice>      gps,
                  std::shared_ptr<CloudService>           cloud,
                  Mode                                    mode = Mode::HEALTH,
                  std::string                             local_log_path = "",
                  size_t                                  local_log_max_bytes = 10u * 1024u * 1024u,
                  size_t                                  local_log_max_files = 3u);

    /// 由 ThreadExecutor 调用。优先使用 DiagnosticsCollector 快照；兼容构造路径只读取设备缓存 getter。
    void update() override;

private:
    static constexpr size_t kPayloadBufferBytes = 8192;
    size_t build_payload(char* out, size_t cap) const;

    std::shared_ptr<device::WalkMotorGroup> walk_;
    std::shared_ptr<device::BrushMotor>     brush_;
    std::shared_ptr<device::BMS>            bms_;
    std::shared_ptr<device::ImuDevice>      imu_;
    std::shared_ptr<device::GpsDevice>      gps_;
    std::shared_ptr<CloudService>           cloud_;
    std::shared_ptr<DiagnosticsCollector> diagnostics_;
    Mode                                mode_;
    mutable std::array<char, kPayloadBufferBytes> payload_buf_{};
    mutable std::string                 payload_cache_;
    std::shared_ptr<spdlog::logger>     local_log_; ///< 本地 JSONL 轮转日志器，local_log_path 非空时创建。
};

}  // namespace robot::service
