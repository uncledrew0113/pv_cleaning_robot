#pragma once
#include "pv_cleaning_robot/device/imu_device.h"

/// @brief ImuDevice 手工 mock
struct MockImuDevice {
    // ── 可配置返回值 ──────────────────────────────────────────
    bool open_result{true};
    robot::device::ImuDevice::ImuData data_result{};
    robot::device::ImuDevice::Diagnostics diag_result{};

    // ── 调用记录 ──────────────────────────────────────────────
    bool opened{false};
    bool closed{false};
    int get_latest_count{0};

    bool open() {
        opened = open_result;
        return open_result;
    }
    void close() {
        opened = false;
        closed = true;
    }

    robot::device::ImuDevice::ImuData get_latest() const {
        ++const_cast<MockImuDevice*>(this)->get_latest_count;
        return data_result;
    }
    robot::device::ImuDevice::Diagnostics get_diagnostics() const {
        return diag_result;
    }
};
