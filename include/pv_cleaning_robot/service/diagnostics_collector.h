/**
 * @file diagnostics_collector.h
 * @brief 设备诊断快照采集接口。
 *
 * DiagnosticsCollector 从各设备和服务读取缓存状态，生成统一诊断快照。采集过程不主动控制硬件，
 * 供 HealthService、ErrorManager 和本地日志复用同一份诊断事实。
 */
#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/domain/diagnostics_snapshot.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/gps_stuck_service.h"

namespace robot::service {

/// @brief 诊断快照采集器。
///
/// 本类负责周期性采集设备原始诊断数据并更新统一 Snapshot：
/// - 原始数据计数变化时，更新对应数据流的 last_update_ms；
/// - 完整 diagnostics 透传给 HealthService；
/// - error 子快照提供给 ErrorManager；
/// - 不上报云端、不执行恢复决策。
///
/// 设计边界：DiagnosticsCollector 是健康上报和错误管理的唯一诊断数据出口，避免
/// HealthService 与 ErrorManager 分别读取设备导致频率不一致或重复 I/O。
class DiagnosticsCollector : public middleware::IRunnable {
public:
    DiagnosticsCollector() = default;
    DiagnosticsCollector(std::shared_ptr<device::WalkMotorGroup> walk,
                         std::shared_ptr<device::BrushMotor> brush,
                         std::shared_ptr<device::BMS> bms,
                         std::shared_ptr<device::ImuDevice> imu,
                         std::shared_ptr<device::GpsDevice> gps,
                         std::shared_ptr<GpsStuckService> gps_stuck);

    struct Input {
        device::WalkMotorGroup::GroupStatus walk_status{};
        device::WalkMotorGroup::GroupDiagnostics walk_diagnostics{};
        device::BrushMotor::Status brush_status{};
        device::BrushMotor::Diagnostics brush_diagnostics{};
        device::BMS::BatteryData bms_data{};
        device::BMS::Diagnostics bms_diagnostics{};
        device::ImuDevice::ImuData imu_data{};
        device::ImuDevice::Diagnostics imu_diagnostics{};
        device::GpsDevice::GpsData gps_data{};
        device::GpsDevice::Diagnostics gps_diagnostics{};

        uint32_t bms_update_count{0};
        uint32_t brush_comm_error_count{0};

        bool gps_enabled{false};
        uint32_t gps_sentence_count{0};

        bool imu_enabled{false};
        uint32_t imu_frame_count{0};

        bool walk_feedback_expected{false};
        std::array<bool, 4> walk_feedback_enabled{};
        std::array<uint32_t, 4> walk_feedback_frame_count{};

        bool walk_stall_active{false};
        bool gps_stuck{false};
    };

    struct Snapshot {
        uint64_t ts_ms{0};     ///< 单调时钟毫秒，用于错误检测、超时和数据流新鲜度判断。
        uint64_t epoch_ms{0};  ///< Unix epoch 毫秒，用于 ThingsBoard / 云端 telemetry 时间戳。
        device::WalkMotorGroup::GroupStatus walk_status{};
        device::WalkMotorGroup::GroupDiagnostics walk_diagnostics{};
        device::BrushMotor::Status brush_status{};
        device::BrushMotor::Diagnostics brush_diagnostics{};
        device::BMS::BatteryData bms_data{};
        device::BMS::Diagnostics bms_diagnostics{};
        device::ImuDevice::ImuData imu_data{};
        device::ImuDevice::Diagnostics imu_diagnostics{};
        device::GpsDevice::GpsData gps_data{};
        device::GpsDevice::Diagnostics gps_diagnostics{};
        domain::DiagnosticsSnapshot error{};
    };

    /// @brief 由 ThreadExecutor 调度，读取绑定设备并更新内部快照缓存。
    ///
    /// update() 只做快照读取和计数判断，不执行恢复流程；错误处理由 ErrorHandlingService 消费
    /// error_snapshot() 后统一仲裁。
    void update() override;

    /// @brief 测试和复用入口：用外部已组装的输入更新快照。
    Snapshot update_from_input(const Input& input, uint64_t now_ms, uint64_t epoch_ms = 0);

    /// @brief 返回最近一次完整诊断快照副本，供 HealthService 使用。
    Snapshot snapshot() const;

    /// @brief 返回最近一次错误诊断快照副本，供 ErrorManager 使用。
    domain::DiagnosticsSnapshot error_snapshot() const;

private:
    struct StreamState {
        bool initialized{false};
        uint32_t previous_count{0};
        uint64_t last_update_ms{0};
    };

    static domain::StreamHealth update_stream(StreamState& state,
                                              bool enabled,
                                              uint32_t count,
                                              uint64_t now_ms);
    static uint64_t steady_now_ms();
    static uint64_t system_epoch_ms();
    Input read_input_from_devices() const;

    std::shared_ptr<device::WalkMotorGroup> walk_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::BMS> bms_;
    std::shared_ptr<device::ImuDevice> imu_;
    std::shared_ptr<device::GpsDevice> gps_;
    std::shared_ptr<GpsStuckService> gps_stuck_;

    StreamState gps_state_;
    StreamState imu_state_;
    StreamState bms_update_state_;
    std::array<StreamState, 4> walk_feedback_states_{};

    mutable std::mutex mtx_;
    Snapshot latest_;
};

}  // namespace robot::service
