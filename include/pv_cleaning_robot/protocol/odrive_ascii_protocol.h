#pragma once

#include <cstddef>
#include <cstdint>

namespace robot::protocol {

enum class OdriveControlMode : uint8_t {
    TORQUE = 1,
    VELOCITY = 2,
};

enum class OdriveReadProperty : uint8_t {
    VBUS_VOLTAGE = 0,
    IQ_MEASURED = 1,
    FET_TEMPERATURE = 2,
    MOTOR_ERROR = 3,
    AXIS_ERROR = 4,
    CONTROLLER_ERROR = 5,
};

static constexpr uint32_t kOdriveAxisStateIdle = 1u;
static constexpr uint32_t kOdriveAxisStateClosedLoopControl = 8u;

size_t encode_set_velocity(uint8_t axis, float counts_per_sec, char* out, size_t cap) noexcept;
size_t encode_set_torque(uint8_t axis, float torque_nm, char* out, size_t cap) noexcept;
size_t encode_watchdog_feed(uint8_t axis, char* out, size_t cap) noexcept;
size_t encode_feedback_request(uint8_t axis, char* out, size_t cap) noexcept;
size_t encode_clear_errors(char* out, size_t cap) noexcept;
size_t encode_set_control_mode(uint8_t axis,
                               OdriveControlMode mode,
                               char* out,
                               size_t cap) noexcept;
size_t encode_set_requested_state(uint8_t axis,
                                  uint32_t state,
                                  char* out,
                                  size_t cap) noexcept;
size_t encode_set_watchdog_enabled(uint8_t axis,
                                   bool enabled,
                                   char* out,
                                   size_t cap) noexcept;
size_t encode_set_watchdog_timeout(uint8_t axis,
                                   float timeout_s,
                                   char* out,
                                   size_t cap) noexcept;
size_t encode_read_property(OdriveReadProperty property,
                            uint8_t axis,
                            char* out,
                            size_t cap) noexcept;

bool parse_feedback_response(const char* line, float* position, float* velocity) noexcept;
bool parse_float_response(const char* line, float* value) noexcept;
bool parse_u32_response(const char* line, uint32_t* value) noexcept;

}  // namespace robot::protocol
