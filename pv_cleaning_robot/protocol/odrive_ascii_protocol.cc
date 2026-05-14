#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "pv_cleaning_robot/protocol/odrive_ascii_protocol.h"

namespace robot::protocol {

namespace {

size_t encode_line(char* out, size_t cap, const char* fmt, ...) noexcept {
    if (!out || cap == 0 || !fmt) {
        return 0;
    }
    va_list args;
    va_start(args, fmt);
    const int written = std::vsnprintf(out, cap, fmt, args);
    va_end(args);
    if (written < 0 || static_cast<size_t>(written) >= cap) {
        out[0] = '\0';
        return 0;
    }
    return static_cast<size_t>(written);
}

const char* property_path(OdriveReadProperty property, uint8_t axis) noexcept {
    switch (property) {
        case OdriveReadProperty::VBUS_VOLTAGE:
            return "vbus_voltage";
        case OdriveReadProperty::IQ_MEASURED:
            return axis == 0 ? "axis0.motor.current_control.Iq_measured"
                             : "axis1.motor.current_control.Iq_measured";
        case OdriveReadProperty::FET_TEMPERATURE:
            return axis == 0 ? "axis0.fet_thermistor.temperature"
                             : "axis1.fet_thermistor.temperature";
        case OdriveReadProperty::MOTOR_ERROR:
            return axis == 0 ? "axis0.motor.error" : "axis1.motor.error";
        case OdriveReadProperty::AXIS_ERROR:
            return axis == 0 ? "axis0.error" : "axis1.error";
        case OdriveReadProperty::CONTROLLER_ERROR:
            return axis == 0 ? "axis0.controller.error" : "axis1.controller.error";
        default:
            return "";
    }
}

bool parse_float_token(const char* line, char** end_out, float* value) noexcept {
    if (!line || !value) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const float parsed = std::strtof(line, &end);
    if (end == line || errno != 0) {
        return false;
    }
    *value = parsed;
    if (end_out) {
        *end_out = end;
    }
    return true;
}

}  // namespace

size_t encode_set_velocity(uint8_t axis, float counts_per_sec, char* out, size_t cap) noexcept {
    return encode_line(out, cap, "v %u %.3f 0\n", static_cast<unsigned>(axis), counts_per_sec);
}

size_t encode_feedback_request(uint8_t axis, char* out, size_t cap) noexcept {
    return encode_line(out, cap, "f %u\n", static_cast<unsigned>(axis));
}

size_t encode_clear_errors(char* out, size_t cap) noexcept {
    return encode_line(out, cap, "sc\n");
}

size_t encode_restart(char* out, size_t cap) noexcept {
    return encode_line(out, cap, "sr\n");
}

size_t encode_read_property(OdriveReadProperty property,
                            uint8_t axis,
                            char* out,
                            size_t cap) noexcept {
    return encode_line(out, cap, "r %s\n", property_path(property, axis));
}

bool parse_feedback_response(const char* line, float* position, float* velocity) noexcept {
    if (!line || !position || !velocity) {
        return false;
    }
    char* end = nullptr;
    float pos = 0.0f;
    if (!parse_float_token(line, &end, &pos)) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    float vel = 0.0f;
    if (!parse_float_token(end, nullptr, &vel)) {
        return false;
    }
    *position = pos;
    *velocity = vel;
    return true;
}

bool parse_float_response(const char* line, float* value) noexcept {
    return parse_float_token(line, nullptr, value);
}

bool parse_u32_response(const char* line, uint32_t* value) noexcept {
    if (!line || !value) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(line, &end, 0);
    if (end == line || errno != 0) {
        return false;
    }
    *value = static_cast<uint32_t>(parsed);
    return true;
}

}  // namespace robot::protocol
