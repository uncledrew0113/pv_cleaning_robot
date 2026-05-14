#pragma once

#include <cstddef>
#include <cstdint>

namespace robot::protocol {

enum class OdriveReadProperty : uint8_t {
    VBUS_VOLTAGE = 0,
    IQ_MEASURED = 1,
    FET_TEMPERATURE = 2,
    MOTOR_ERROR = 3,
    AXIS_ERROR = 4,
    CONTROLLER_ERROR = 5,
};

size_t encode_set_velocity(uint8_t axis, float counts_per_sec, char* out, size_t cap) noexcept;
size_t encode_feedback_request(uint8_t axis, char* out, size_t cap) noexcept;
size_t encode_clear_errors(char* out, size_t cap) noexcept;
size_t encode_restart(char* out, size_t cap) noexcept;
size_t encode_read_property(OdriveReadProperty property,
                            uint8_t axis,
                            char* out,
                            size_t cap) noexcept;

bool parse_feedback_response(const char* line, float* position, float* velocity) noexcept;
bool parse_float_response(const char* line, float* value) noexcept;
bool parse_u32_response(const char* line, uint32_t* value) noexcept;

}  // namespace robot::protocol
