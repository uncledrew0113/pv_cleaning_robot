/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-30 16:02:45
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/device/brush_motor.cc
 * @Description: 滚刷电机驱动类 - 基于ODrive控制器的串口通信实现
 *
 * 该文件实现了BrushMotor类，用于控制光伏清扫机器人的滚刷电机。
 * 通过串口与ODrive控制器通信，支持速度和力矩控制模式，
 * 包含看门狗机制、故障检测和状态监控功能。
 *
 * 主要特性：
 * - 支持速度和力矩两种控制模式
 * - 内置通信超时和看门狗保护机制
 * - 实时状态反馈和故障诊断
 * - 线程安全的操作接口
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */

#include <cmath>
#include <mutex>
#include <utility>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/protocol/odrive_ascii_protocol.h"

namespace robot::device {

namespace {

// 通信超时常量：200ms，用于串口读写操作的超时时间
// 这个值需要在保证响应速度的同时，避免因网络延迟导致的频繁超时
constexpr int kIoTimeoutMs = 200;

// 命令缓冲区容量：128字节，足够存放ODrive ASCII协议的命令字符串
constexpr size_t kCmdCap = 128;

// 响应缓冲区容量：128字节，足够存放ODrive的响应数据
constexpr size_t kRespCap = 128;

/**
 * @brief 将串口错误转换为设备错误枚举
 *
 * @param err 串口操作结果
 * @return 对应的设备错误类型
 */
DeviceError uart_error_to_device_error(hal::UartResult err) {
    switch (err) {
        case hal::UartResult::TIMEOUT:
            return DeviceError::COMM_TIMEOUT;  // 通信超时
        case hal::UartResult::OK:
            return DeviceError::OK;  // 操作成功
        default:
            return DeviceError::COMM_NO_RESP;  // 无响应或其他通信错误
    }
}

/**
 * @brief 将编码器计数每秒转换为RPM（转每分钟）
 *
 * @param counts_per_sec 每秒编码器计数
 * @param counts_per_rev 每转编码器计数
 * @return 对应的RPM值
 */
int turns_per_sec_to_rpm(float turns_per_sec) {
    return static_cast<int>(std::lround(turns_per_sec * 60.0f));
}

/**
 * @brief 将RPM转换为编码器计数每秒
 *
 * @param rpm 转速（RPM）
 * @param counts_per_rev 每转编码器计数
 * @return 对应的每秒编码器计数
 */
float rpm_to_turns_per_sec(int rpm) {
    return static_cast<float>(rpm) / 60.0f;
}

}  // namespace

/**
 * @brief BrushMotor构造函数
 *
 * 初始化滚刷电机控制器，配置串口通信参数和看门狗设置
 *
 * @param serial 串口接口的共享指针
 * @param axis ODrive控制器中的轴编号（通常为0）
 * @param counts_per_rev 历史保留参数；ASCII 协议的速度命令和反馈使用 turns/s
 * @param watchdog_enabled 是否启用看门狗机制
 * @param watchdog_timeout_s 看门狗超时时间（秒）
 */
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

/**
 * @brief 打开并初始化电机控制器
 *
 * 执行以下初始化步骤：
 * 1. 打开串口连接
 * 2. 清空串口缓冲区
 * 3. 配置看门狗超时时间（如果启用）
 * 4. 启用或禁用看门狗机制
 *
 * @return 初始化是否成功
 */
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

/**
 * @brief 设置电机为速度控制模式
 *
 * 将电机控制器设置为速度控制模式，并进入闭环控制状态。
 * 在速度模式下，可以通过set_rpm()设置目标转速。
 *
 * @return 操作结果
 */
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

/**
 * @brief 设置电机为力矩控制模式
 *
 * 将电机控制器设置为力矩控制模式，并进入闭环控制状态。
 * 在力矩模式下，可以通过set_torque()设置目标力矩。
 *
 * @return 操作结果
 */
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

/**
 * @brief 设置电机目标转速
 *
 * 在速度控制模式下设置电机的目标转速（RPM）。
 * 该方法会自动切换到速度控制模式。
 *
 * @param rpm 目标转速（RPM），正值表示正转，负值表示反转
 * @return 操作结果
 */
DeviceError BrushMotor::set_rpm(int rpm) {
    std::lock_guard<hal::PiMutex> guard(mtx_);
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

/**
 * @brief 设置电机目标力矩
 *
 * 在力矩控制模式下设置电机的目标力矩（牛米）。
 * 该方法会自动切换到力矩控制模式。
 *
 * @param torque_nm 目标力矩（牛米），正值表示正转力矩，负值表示反转力矩
 * @return 操作结果
 */
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

/**
 * @brief 停止电机运行
 *
 * 将电机转速或力矩设置为0，实现平滑停止。
 * 根据当前控制模式发送相应的停止命令。
 *
 * @return 操作结果
 */
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

/**
 * @brief 使电机进入空闲状态
 *
 * 将电机控制器设置为IDLE状态，此时电机将停止运行并释放控制。
 * 这是一种比stop()更彻底的停止方式。
 *
 * @return 操作结果
 */
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

/**
 * @brief 清除电机故障
 *
 * 发送清除错误命令到ODrive控制器，尝试恢复电机正常工作状态。
 *
 * @return 操作结果
 */
DeviceError BrushMotor::clear_fault() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        return DeviceError::NOT_OPEN;
    }
    char cmd[kCmdCap];
    const size_t len = protocol::encode_clear_errors(cmd, sizeof(cmd));
    return write_ascii_locked(cmd, len);
}

/**
 * @brief 获取电机当前状态
 *
 * 返回电机当前的运行状态，包括实际转速、电流、运行状态和故障信息。
 *
 * @return 电机状态结构体
 */
BrushMotor::Status BrushMotor::get_status() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return Status{diag_.actual_rpm, diag_.current_a, diag_.running, diag_.fault, diag_.fault_code};
}

/**
 * @brief 获取电机诊断信息
 *
 * 返回详细的电机诊断数据，包括所有状态信息和额外的诊断参数。
 *
 * @return 电机诊断结构体
 */
BrushMotor::Diagnostics BrushMotor::get_diagnostics() const {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    return diag_;
}

/**
 * @brief 更新电机状态（周期性调用）
 *
 * 该方法需要定期调用（通常在单独线程中），用于：
 * 1. 读取电机反馈数据（位置、速度、电压、电流、温度）
 * 2. 检查电机和控制器错误状态
 * 3. 执行看门狗喂狗操作（如果需要）
 * 4. 更新运行状态
 *
 * 这个方法是线程安全的，可以在后台线程中调用。
 */
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

/**
 * @brief 线程安全的ASCII命令写入
 *
 * 将ASCII命令写入串口，包含错误处理和超时控制。
 * 该方法会更新最后通信时间，用于看门狗喂狗判断。
 *
 * @param line 要写入的命令字符串
 * @param len 命令字符串长度
 * @return 操作结果
 */
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

/**
 * @brief 线程安全的ASCII命令请求
 *
 * 发送命令并读取响应，实现完整的命令-响应交互。
 *
 * @param line 要发送的命令
 * @param len 命令长度
 * @param response 响应缓冲区
 * @param response_cap 响应缓冲区容量
 * @param timeout_ms 超时时间（毫秒）
 * @return 操作结果
 */
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

/**
 * @brief 线程安全的行读取
 *
 * 从串口读取一行ASCII响应，以换行符结束。
 * 支持超时控制和错误处理。
 *
 * @param response 响应缓冲区
 * @param response_cap 缓冲区容量
 * @param timeout_ms 读取超时时间
 * @return 操作结果
 */
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

/**
 * @brief 标记通信错误
 *
 * 增加通信错误计数器，用于诊断通信质量问题。
 */
void BrushMotor::mark_comm_error_locked() {
    ++diag_.comm_error_count;
}

/**
 * @brief 更新运行状态（内部方法）
 *
 * 根据目标值和故障状态更新电机运行状态。
 * 运行状态用于指示电机是否正在按照目标参数工作。
 */
void BrushMotor::update_running_locked() {
    const bool target_active =
        (std::abs(target_rpm_) > 0) || (std::fabs(target_torque_nm_) > 0.001f);
    diag_.running = !diag_.fault && active_control_ && target_active;
}

}  // namespace robot::device
