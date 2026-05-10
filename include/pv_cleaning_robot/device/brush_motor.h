#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::device {

class BrushMotor {
   public:
    enum class ControlMode : uint8_t { SPEED = 0, TORQUE = 1 };

    struct Status {
        int actual_rpm{0};
        float current_a{0.0f};
        bool running{false};
        bool fault{false};
        uint32_t fault_code{0};
    };

    struct Diagnostics : Status {
        float temperature_c{0.0f};
        float bus_voltage_v{0.0f};
        int target_rpm{0};
        float target_torque_nm{0.0f};
        uint32_t stall_count{0};
        uint32_t comm_error_count{0};
    };

    BrushMotor(std::shared_ptr<hal::ISerialPort> serial,
               uint8_t axis,
               float counts_per_rev,
               bool watchdog_enabled,
               float watchdog_timeout_s);
    ~BrushMotor() noexcept;

    bool open();
    void close();

    DeviceError set_mode_speed();
    DeviceError set_mode_torque();
    DeviceError set_rpm(int rpm);
    DeviceError set_torque(float torque_nm);
    DeviceError stop();
    DeviceError enter_idle();
    DeviceError clear_fault();

    Status get_status() const;
    Diagnostics get_diagnostics() const;
    void update();

   private:
    DeviceError write_ascii_locked(const char* line, size_t len);
    DeviceError request_ascii_locked(const char* line,
                                     size_t len,
                                     char* response,
                                     size_t response_cap,
                                     int timeout_ms = 200);
    DeviceError read_line_locked(char* response, size_t response_cap, int timeout_ms = 200);
    void mark_comm_error_locked();
    void update_running_locked();

    std::shared_ptr<hal::ISerialPort> serial_;
    uint8_t axis_;
    float counts_per_rev_;
    bool watchdog_enabled_;
    std::chrono::milliseconds watchdog_timeout_{500};
    mutable hal::PiMutex mtx_;
    Diagnostics diag_{};
    ControlMode control_mode_{ControlMode::SPEED};
    bool active_control_{false};
    bool keepalive_required_{false};
    int target_rpm_{0};
    float target_torque_nm_{0.0f};
    std::chrono::steady_clock::time_point last_feed_time_{};
    uint32_t update_seq_{0};
};

}  // namespace robot::device
