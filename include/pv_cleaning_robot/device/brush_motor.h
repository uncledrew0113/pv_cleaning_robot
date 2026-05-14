#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::device {

class BrushMotor {
   public:
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
        uint32_t stall_count{0};
        uint32_t comm_error_count{0};
    };

    BrushMotor(std::shared_ptr<hal::ISerialPort> serial, uint8_t axis);
    ~BrushMotor() noexcept;

    bool open();
    void close();

    DeviceError set_rpm(int rpm);
    DeviceError stop();
    DeviceError clear_fault();
    DeviceError restart();

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
    mutable hal::PiMutex mtx_;
    Diagnostics diag_{};
};

}  // namespace robot::device
