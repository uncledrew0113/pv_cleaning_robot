/*
 * 滚刷电机 ODrive ASCII 串口驱动实现。
 *
 * 维护边界：
 * - 本类只封装 ODrive ASCII 命令、状态读取和通信错误计数；
 * - 任务何时开停滚刷由 MotionService 决定，故障恢复流程由 RecoveryExecutor 编排；
 * - close()/restart() 前应先停止 brush_exec，避免周期 update() 与串口重建并发。
 */

#include <cmath>
#include <mutex>
#include <utility>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/protocol/odrive_ascii_protocol.h"

namespace robot::device {

namespace {

// 串口读写超时。滚刷状态采集不在实时运动线程中执行，200ms 可兼顾现场响应和误超时抑制。
constexpr int kIoTimeoutMs = 200;

// ODrive ASCII 命令上限较短，固定栈缓冲避免周期 update() 堆分配。
constexpr size_t kCmdCap = 128;

// ODrive ASCII 响应同样使用固定栈缓冲，超出时由解析失败路径累计通信错误。
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

int turns_per_sec_to_rpm(float turns_per_sec) {
    return static_cast<int>(std::lround(turns_per_sec * 60.0f));
}

float rpm_to_turns_per_sec(int rpm) {
    return static_cast<float>(rpm) / 60.0f;
}

}  // namespace

BrushMotor::BrushMotor(std::shared_ptr<hal::ISerialPort> serial, uint8_t axis)
    : serial_(std::move(serial))
    , axis_(axis) {}

BrushMotor::~BrushMotor() noexcept {
    try {
        close();
    } catch (...) {
    }
}

bool BrushMotor::open() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    clear_stop_request();
    if (!serial_) {
        return false;
    }
    if (!serial_->is_open() && !serial_->open()) {
        return false;
    }
    serial_->flush_input();
    serial_->flush_output();
    return true;
}

void BrushMotor::close() {
    request_stop();
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        diag_ = Diagnostics{};
        update_running_locked();
        return;
    }

    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_velocity(axis_, 0.0f, cmd, sizeof(cmd));
    if (len > 0) {
        static_cast<void>(write_ascii_locked(cmd, len));
    }

    diag_ = Diagnostics{};
    update_running_locked();
    serial_->close();
}

void BrushMotor::request_stop() {
    stop_requested_.store(true, std::memory_order_release);
}

void BrushMotor::clear_stop_request() {
    stop_requested_.store(false, std::memory_order_release);
}

DeviceError BrushMotor::set_rpm(int rpm) {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (stop_requested()) {
        return DeviceError::EXEC_FAILED;
    }
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_velocity(
        axis_, rpm_to_turns_per_sec(rpm), cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    diag_.target_rpm = rpm;
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::stop() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_set_velocity(axis_, 0.0f, cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err != DeviceError::OK) {
        return err;
    }
    diag_.target_rpm = 0;
    update_running_locked();
    return DeviceError::OK;
}

DeviceError BrushMotor::clear_fault() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (stop_requested()) {
        return DeviceError::EXEC_FAILED;
    }
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_clear_errors(cmd, sizeof(cmd));
    return write_ascii_locked(cmd, len);
}

DeviceError BrushMotor::restart() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (stop_requested()) {
        return DeviceError::EXEC_FAILED;
    }
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_restart(cmd, sizeof(cmd));
    const DeviceError err = write_ascii_locked(cmd, len);
    if (err == DeviceError::OK) {
        diag_ = Diagnostics{};
        update_running_locked();
    }
    return err;
}

BrushMotor::Status BrushMotor::get_status() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return Status{diag_.actual_rpm, diag_.current_a, diag_.running, diag_.fault, diag_.fault_code};
}

BrushMotor::Diagnostics BrushMotor::get_diagnostics() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return diag_;
}

// 周期状态采集：连续通信失败通过 comm_error_count 暴露给 DiagnosticsCollector，
// 不在设备层直接决定恢复或停机。
void BrushMotor::update() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (stop_requested()) {
        return;
    }
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
    diag_.actual_rpm = turns_per_sec_to_rpm(vel);

    float value = 0.0f;
    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::VBUS_VOLTAGE, axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_float_response(resp, &value)) {
        mark_comm_error_locked();
        return;
    }
    diag_.bus_voltage_v = value;

    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::IQ_MEASURED, axis_, cmd, sizeof(cmd));
    if (request_ascii_locked(cmd, len, resp, sizeof(resp), kIoTimeoutMs) != DeviceError::OK ||
        !protocol::parse_float_response(resp, &value)) {
        mark_comm_error_locked();
        return;
    }
    diag_.current_a = value;

    len = protocol::encode_read_property(
        protocol::OdriveReadProperty::FET_TEMPERATURE, axis_, cmd, sizeof(cmd));
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
    return DeviceError::OK;
}

// 完整 ASCII 请求事务。调用方已持有 mtx_，保证同一串口不会交错发送多条命令。
DeviceError BrushMotor::request_ascii_locked(const char* line,
                                             size_t len,
                                             char* response,
                                             size_t response_cap,
                                             int timeout_ms) {
    if (stop_requested()) {
        return DeviceError::EXEC_FAILED;
    }
    serial_->flush_input();
    const DeviceError err = write_ascii_locked(line, len);
    if (err != DeviceError::OK) {
        return err;
    }
    return read_line_locked(response, response_cap, timeout_ms);
}

// 读取一行 ODrive ASCII 响应；超时或系统错误会累计通信错误计数。
DeviceError BrushMotor::read_line_locked(char* response, size_t response_cap, int timeout_ms) {
    if (!response || response_cap < 2) {
        return DeviceError::INVALID_PARAM;
    }
    size_t used = 0;
    while (used + 1 < response_cap) {
        if (stop_requested()) {
            response[0] = '\0';
            return DeviceError::EXEC_FAILED;
        }
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

bool BrushMotor::stop_requested() const {
    return stop_requested_.load(std::memory_order_acquire);
}

void BrushMotor::mark_comm_error_locked() {
    ++diag_.comm_error_count;
}

// running 是上报语义：目标转速非 0 且当前无锁存故障，不表示已达到目标转速。
void BrushMotor::update_running_locked() {
    diag_.running = !diag_.fault && diag_.target_rpm != 0;
}

}  // namespace robot::device
