#pragma once
#include "pv_cleaning_robot/device/gps_device.h"

/// @brief GpsDevice 手工 mock
struct MockGpsDevice {
    // ── 可配置返回值 ──────────────────────────────────────────
    bool open_result{true};
    robot::device::GpsDevice::GpsData data_result{};
    robot::device::GpsDevice::Diagnostics diag_result{};

    // ── 调用记录 ──────────────────────────────────────────────
    bool opened{false};
    bool closed{false};

    bool open() {
        opened = open_result;
        return open_result;
    }
    void close() {
        opened = false;
        closed = true;
    }

    robot::device::GpsDevice::GpsData get_latest() const {
        return data_result;
    }
    robot::device::GpsDevice::Diagnostics get_diagnostics() const {
        return diag_result;
    }
};
