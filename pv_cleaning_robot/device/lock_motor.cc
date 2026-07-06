/**
 * @file lock_motor.cc
 * @brief 锁止电机 GPIO 输出控制实现。
 *
 * 本文件通过两个 DO 脉冲控制开锁和关锁。由于设备没有机械反馈，本实现只把 GPIO 申请或
 * 写入失败作为失败返回，不判断锁止机构是否真正到位。
 */
#include "pv_cleaning_robot/device/lock_motor.h"

#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

namespace robot::device {

LockMotor::LockMotor(std::shared_ptr<hal::IGpioPin> open_pin,
                     std::shared_ptr<hal::IGpioPin> close_pin,
                     Config config)
    : open_pin_(std::move(open_pin)), close_pin_(std::move(close_pin)), config_(config) {}

LockMotor::~LockMotor() {
    shutdown();
}

bool LockMotor::initialize() {
    if (!open_pin_ || !close_pin_) {
        spdlog::error("[LockMotor] GPIO pin is null");
        return false;
    }

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::OUTPUT;
    cfg.bias = hal::GpioBias::DISABLE;
    cfg.use_irq = false;

    if (!open_pin_->open(cfg)) {
        spdlog::error("[LockMotor] Failed to open lock-open GPIO");
        return false;
    }
    if (!close_pin_->open(cfg)) {
        spdlog::error("[LockMotor] Failed to open lock-close GPIO");
        open_pin_->close();
        return false;
    }

    // 两路 DO 默认拉低，确保控制器不会因为软件启动残留高电平而误动作。
    const bool open_low_ok = open_pin_->write_value(false);
    const bool close_low_ok = close_pin_->write_value(false);
    if (!open_low_ok || !close_low_ok) {
        spdlog::error("[LockMotor] Failed to set lock motor GPIO low during initialize");
        shutdown();
        return false;
    }
    return true;
}

void LockMotor::shutdown() {
    if (open_pin_ && open_pin_->is_open()) {
        open_pin_->write_value(false);
        open_pin_->close();
    }
    if (close_pin_ && close_pin_->is_open()) {
        close_pin_->write_value(false);
        close_pin_->close();
    }
}

bool LockMotor::open_lock() {
    return pulse_pin(open_pin_, "open");
}

bool LockMotor::close_lock() {
    return pulse_pin(close_pin_, "close");
}

bool LockMotor::pulse_pin(const std::shared_ptr<hal::IGpioPin>& pin, const char* action_name) {
    if (!pin || !pin->is_open()) {
        spdlog::error("[LockMotor] {} action requested before GPIO initialize", action_name);
        return false;
    }

    if (!pin->write_value(true)) {
        spdlog::error("[LockMotor] Failed to drive {} GPIO high", action_name);
        return false;
    }
    sleep_ms(config_.pulse_ms);

    if (!pin->write_value(false)) {
        spdlog::error("[LockMotor] Failed to drive {} GPIO low", action_name);
        return false;
    }
    sleep_ms(config_.settle_ms);
    return true;
}

void LockMotor::sleep_ms(uint32_t ms) {
    if (ms == 0) {
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace robot::device
