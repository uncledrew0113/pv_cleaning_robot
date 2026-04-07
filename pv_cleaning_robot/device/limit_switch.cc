/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:02:59
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 16:25:26
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/device/limit_switch.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

namespace robot::device {

LimitSwitch::LimitSwitch(std::shared_ptr<hal::IGpioPin> pin, LimitSide side)
    : pin_(std::move(pin)), side_(side) {}

LimitSwitch::~LimitSwitch() {
    close();
}

bool LimitSwitch::open(int rt_priority, int debounce_ms, int cpu_affinity, bool use_irq) {
    // 感应式限位开关：输入模式，上拉偏置（低有效），软件消抖
    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    cfg.debounce_ms = debounce_ms;
    // 注入 RT 优先级 + CPU 亲和性，激活底层 SCHED_FIFO 提权
    cfg.rt_priority = rt_priority;
    cfg.cpu_affinity = cpu_affinity;
    cfg.use_irq = use_irq;
    if (!pin_->open(cfg)) {
        spdlog::error("[LimitSwitch] Failed to open GPIO pin for {} side",
                      side_ == LimitSide::FRONT ? "FRONT" : "REAR");
        return false;
    }
    return true;
}

void LimitSwitch::close() {
    if (pin_->is_open()) {
        pin_->stop_monitoring();
        // 【核心修复】：生命周期解绑！彻底切断底层驱动对当前实例 this 指针的引用
        pin_->set_edge_callback(hal::GpioEdge::BOTH, nullptr);
        pin_->close();
    }
}

void LimitSwitch::start_monitoring() {
    // 感应式限位开关：物体接近时输出下降沿（低有效）
    pin_->set_edge_callback(hal::GpioEdge::FALLING, [this]() { on_edge(); });
    pin_->start_monitoring();
}

void LimitSwitch::stop_monitoring() {
    pin_->stop_monitoring();
}

void LimitSwitch::set_trigger_callback(TriggerCallback cb) {
    // 加锁防止在监控线程运行期间替换回调产生 Data Race
    std::lock_guard<hal::PiMutex> lock(cb_mtx_);
    callback_ = std::move(cb);
}

bool LimitSwitch::is_triggered() const {
    return triggered_.load(std::memory_order_acquire);
}

void LimitSwitch::clear_trigger() {
    triggered_.store(false, std::memory_order_release);
}

bool LimitSwitch::read_current_level() const {
    return pin_->read_value();
}

void LimitSwitch::on_edge() {
    // 原子置位（极短路径，监控线程直接调用）
    triggered_.store(true, std::memory_order_release);

    // 通知 SafetyMonitor（回调不得阻塞，用于触发急停序列）
    std::lock_guard<hal::PiMutex> lock(cb_mtx_);
    if (callback_) {
        callback_(side_);
    }
}

}  // namespace robot::device
