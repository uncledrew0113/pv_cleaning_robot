#pragma once
#include "pv_cleaning_robot/device/brush_motor.h"

/// @brief BrushMotor 手工 mock
struct MockBrushMotor {
    // ── 可配置返回值 ──────────────────────────────────────────
    bool open_result{true};
    robot::device::DeviceError start_result{robot::device::DeviceError::OK};
    robot::device::DeviceError stop_result{robot::device::DeviceError::OK};
    robot::device::DeviceError set_rpm_result{robot::device::DeviceError::OK};
    robot::device::DeviceError clear_fault_result{robot::device::DeviceError::OK};
    robot::device::BrushMotor::Status status_result{};
    robot::device::BrushMotor::Diagnostics diag_result{};

    // ── 调用记录 ──────────────────────────────────────────────
    bool opened{false};
    int start_count{0};
    int stop_count{0};
    int last_rpm{0};
    int set_rpm_count{0};
    int clear_fault_count{0};
    int update_count{0};

    bool open() {
        opened = open_result;
        return open_result;
    }

    robot::device::DeviceError start() {
        ++start_count;
        return start_result;
    }
    robot::device::DeviceError stop() {
        ++stop_count;
        return stop_result;
    }
    robot::device::DeviceError set_rpm(int rpm) {
        last_rpm = rpm;
        ++set_rpm_count;
        return set_rpm_result;
    }
    robot::device::DeviceError clear_fault() {
        ++clear_fault_count;
        return clear_fault_result;
    }

    robot::device::BrushMotor::Status get_status() const {
        return status_result;
    }
    robot::device::BrushMotor::Diagnostics get_diagnostics() const {
        return diag_result;
    }

    void update() {
        ++update_count;
    }
};
