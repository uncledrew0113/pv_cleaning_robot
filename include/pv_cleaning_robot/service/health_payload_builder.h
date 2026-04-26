#pragma once

#include <cstddef>

#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/distance_sensor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

namespace robot::service {

class HealthPayloadBuilder {
public:
    struct HealthView {
        const char* ts_iso8601{""};
        device::WalkMotorGroup::GroupStatus walk{};
        device::BrushMotor::Status brush{};
        device::BMS::BatteryData bms{};
        device::ImuDevice::ImuData imu{};
        device::GpsDevice::GpsData gps{};
        const device::DistSensorData* dist{nullptr};
    };

    struct DiagnosticsView {
        const char* ts_iso8601{""};
        device::WalkMotorGroup::GroupDiagnostics walk{};
        device::BrushMotor::Diagnostics brush{};
        device::BMS::Diagnostics bms{};
        device::ImuDevice::Diagnostics imu{};
        device::GpsDevice::Diagnostics gps{};
        const device::DistSensorData* dist{nullptr};
    };

    static size_t build_health(const HealthView& view, char* out, size_t cap) noexcept;
    static size_t build_diagnostics(const DiagnosticsView& view, char* out, size_t cap) noexcept;
};

}  // namespace robot::service
