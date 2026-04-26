#include "pv_cleaning_robot/device/brush_motor.h"

#include <cmath>
#include <mutex>
#include <utility>

#include "pv_cleaning_robot/protocol/odrive_ascii_protocol.h"

namespace robot::device {

namespace {

constexpr int kIoTimeoutMs = 200;
constexpr size_t kCmdCap = 128;
constexpr size_t kRespCap = 128;

DeviceError uart_error_to_device_error(hal::UartResult err) {
    switch (err) {
        case hal::UartResult::TIMEOUT:
            return DeviceError::COMM_TIMEOUT;
        case hal::UartResult::OK:
            return DeviceError::OK;
        default:
            return DeviceError::COMM_NO_RESP;
    }
}

int counts_per_sec_to_rpm(float counts_per_sec, float counts_per_rev) {
    if (counts_per_rev <= 0.0f) {
        return 0;
    }
    return static_cast<int>(std::lround((counts_per_sec * 60.0f) / counts_per_rev));
}

float rpm_to_counts_per_sec(int rpm, float counts_per_rev) {
    return (static_cast<float>(rpm) * counts_per_rev) / 60.0f;
}

}  // namespace

BrushMotor::BrushMotor(std::shared_ptr<hal::ISerialPort> serial,
                       uint8_t axis,
                       float counts_per_rev,
                       bool watchdog_enabled,
                       float watchdog_timeout_s)
    : serial_(std::move(serial))
    , axis_(axis)
    , counts_per_rev_(counts_per_rev)
    , watchdog_enabled_(watchdog_enabled)
    , watchdog_timeout_(static_cast<int>(watchdog_timeout_s * 1000.0f))
    , last_feed_time_(std::chrono::steady_clock::now()) {}

bool BrushMotor::open() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_) {
        return false;
    }
    if (!serial_->is_open() && !serial_->open()) {
        return false;
    }
    serial_->flush_input();
    serial_->flush_output();

    char cmd[kCmdCap];
    if (watchdog_enabled_) {
        size_t len = protocol::encode_set_watchdog_timeout(
            axis_, watchdog_timeout_.count() / 1000.0f, cmd, sizeof(cmd));
        if (len == 0 || write_ascii_locked(cmd, len) != DeviceError::OK) {
            return false;
        }
    }
    const size_t len =
        protocol::encode_set_watchdog_enabled(axis_, watchdog_enabled_, cmd, sizeof(cmd));
    return len > 0 && write_ascii_locked(cmd, len) == DeviceError::OK;
}

DeviceError BrushMotor::set_mode_speed() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    size_t len = protocol::encode_set_control_mode(
        axis_, protocol::OdriveControlMode::VELOCITY, cmd, sizeof(cmd));
    DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    len = protocol::encode_set_requested_state(
        axis_, protocol::kOdriveAxisStateClosedLoopControl, cmd, sizeof(cmd));
    err = write_ascii_locked(cmd, len);
    if (err == DeviceError::OK) {
        control_mode_ = ControlMode::SPEED;
    }
    return err;
}

DeviceError BrushMotor::set_mode_torque() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    size_t len = protocol::encode_set_control_mode(
        axis_, protocol::OdriveControlMode::TORQUE, cmd, sizeof(cmd));
    DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    len = protocol::encode_set_requested_state(
        axis_, protocol::kOdriveAxisStateClosedLoopControl, cmd, sizeof(cmd));
    err = write_ascii_locked(cmd, len);
    if (err == DeviceError::OK) {
        control_mode_ = ControlMode::TORQUE;
    }
    return err;
}

DeviceError BrushMotor::set_rpm(int rpm) {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    if (counts_per_rev_ <= 0.0f) {
        return DeviceError::INVALID_PARAM;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_velocity(
        axis_, rpm_to_counts_per_sec(rpm, counts_per_rev_), cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    control_mode_ = ControlMode::SPEED;
    target_rpm_ = rpm;
    target_torque_nm_ = 0.0f;
    diag_.target_rpm = rpm;
    diag_.target_torque_nm = 0.0f;
    active_control_ = (rpm != 0);
    keepalive_required_ = (rpm != 0);
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::set_torque(float torque_nm) {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_torque(axis_, torque_nm, cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    control_mode_ = ControlMode::TORQUE;
    target_rpm_ = 0;
    target_torque_nm_ = torque_nm;
    diag_.target_rpm = 0;
    diag_.target_torque_nm = torque_nm;
    active_control_ = (std::fabs(torque_nm) > 0.001f);
    keepalive_required_ = active_control_;
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::stop() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    size_t len = 0;
    if (control_mode_ == ControlMode::TORQUE) {
        len = protocol::encode_set_torque(axis_, 0.0f, cmd, sizeof(cmd));
    } else {
        len = protocol::encode_set_velocity(axis_, 0.0f, cmd, sizeof(cmd));
    }
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    target_rpm_ = 0;
    target_torque_nm_ = 0.0f;
    diag_.target_rpm = 0;
    diag_.target_torque_nm = 0.0f;
    active_control_ = false;
    keepalive_required_ = false;
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::enter_idle() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_requested_state(
        axis_, protocol::kOdriveAxisStateIdle, cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    target_rpm_ = 0;
    target_torque_nm_ = 0.0f;
    diag_.target_rpm = 0;
    diag_.target_torque_nm = 0.0f;
    active_control_ = false;
    keepalive_required_ = false;
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::clear_fault() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_clear_errors(cmd, sizeof(cmd));
    return write_ascii_locked(cmd, len);
}

BrushMotor::Status BrushMotor::get_status() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return Status{diag_.actual_rpm, diag_.current_a, diag_.running, diag_.fault, diag_.fault_code};
}

BrushMotor::Diagnostics BrushMotor::get_diagnostics() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return diag_;
}

void BrushMotor::update() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return;
    }
    char cmd[kCmdCap];
    char resp[kRespCap];
    float pos = 0.0f;
    float vel = 0.0f;

    size_t len = protocol::encode_feedback_request(axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_feedback_response(resp, &pos, &vel)) {
        mark_comm_error_locked();
        return;
    }
    diag_.actual_rpm = counts_per_sec_to_rpm(vel, counts_per_rev_);

    float value = 0.0f;
    len = protocol::encode_read_property(protocol::OdriveReadProperty::VBUS_VOLTAGE,
                                         axis_,
                                         cmd,
                                         sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_float_response(resp, &value)) {
        mark_comm_error_locked();
        return;
    }
    diag_.bus_voltage_v = value;

    len = protocol::encode_read_property(protocol::OdriveReadProperty::IQ_MEASURED,
                                         axis_,
                                         cmd,
                                         sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_float_response(resp, &value)) {
        mark_comm_error_locked();
        return;
    }
    diag_.current_a = value;

    len = protocol::encode_read_property(protocol::OdriveReadProperty::FET_TEMPERATURE,
                                         axis_,
                                         cmd,
                                         sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_float_response(resp, &value)) {
        mark_comm_error_locked();
        return;
    }
    diag_.temperature_c = value;

    uint32_t motor_err = 0;
    uint32_t axis_err = 0;
    uint32_t ctrl_err = 0;

    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::MOTOR_ERROR, axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_u32_response(resp, &motor_err)) {
        mark_comm_error_locked();
        return;
    }

    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::AXIS_ERROR, axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_u32_response(resp, &axis_err)) {
        mark_comm_error_locked();
        return;
    }

    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::CONTROLLER_ERROR, axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_u32_response(resp, &ctrl_err)) {
        mark_comm_error_locked();
        return;
    }

    diag_.fault_code = motor_err | axis_err | ctrl_err;
    diag_.fault = (diag_.fault_code != 0);

    const auto now = std::chrono::steady_clock::now();
    if (watchdog_enabled_ && keepalive_required_ &&
        (now - last_feed_time_) >= (watchdog_timeout_ / 2)) {
        len = protocol::encode_watchdog_feed(axis_, cmd, sizeof(cmd));
        if (write_ascii_locked(cmd, len) != DeviceError::OK) {
            mark_comm_error_locked();
            return;
        }
    }

    ++update_seq_;
    update_running_locked();
}

DeviceError BrushMotor::write_ascii_locked(const char* line, size_t len) {
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    if (!line || len == 0) {
        return DeviceError::INVALID_PARAM;
    }
    const int written = serial_->write(reinterpret_cast<const uint8_t*>(line), len, kIoTimeoutMs);
    if (written != static_cast<int>(len)) {
        ++diag_.comm_error_count;
        return uart_error_to_device_error(serial_->get_last_error());
    }
    last_feed_time_ = std::chrono::steady_clock::now();
    return DeviceError::OK;
}

DeviceError BrushMotor::request_ascii_locked(const char* line,
                                             size_t len,
                                             char* response,
                                             size_t response_cap,
                                             int timeout_ms) {
    serial_->flush_input();
    const DeviceError err = write_ascii_locked(line, len);
    if (err != DeviceError::OK) {
        return err;
    }
    return read_line_locked(response, response_cap, timeout_ms);
}

DeviceError BrushMotor::read_line_locked(char* response, size_t response_cap, int timeout_ms) {
    if (!response || response_cap < 2) {
        return DeviceError::INVALID_PARAM;
    }
    size_t used = 0;
    while (used + 1 < response_cap) {
        uint8_t ch = 0;
        const int n = serial_->read(&ch, 1, timeout_ms);
        if (n < 0) {
            ++diag_.comm_error_count;
            response[0] = '\0';
            return uart_error_to_device_error(serial_->get_last_error());
        }
        if (n == 0) {
            ++diag_.comm_error_count;
            response[0] = '\0';
            return DeviceError::COMM_TIMEOUT;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            break;
        }
        response[used++] = static_cast<char>(ch);
    }
    response[used] = '\0';
    return DeviceError::OK;
}

void BrushMotor::mark_comm_error_locked() {
    ++diag_.comm_error_count;
}

void BrushMotor::update_running_locked() {
    const bool target_active =
        (std::abs(target_rpm_) > 0) || (std::fabs(target_torque_nm_) > 0.001f);
    diag_.running = !diag_.fault && active_control_ && target_active;
}

}  // namespace robot::device
