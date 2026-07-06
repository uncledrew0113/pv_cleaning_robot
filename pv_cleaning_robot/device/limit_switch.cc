/**
 * @file limit_switch.cc
 * @brief 主限位接近传感器设备实现。
 *
 * device 层只负责 GPIO 触发事实，不直接处理状态机或恢复流程。
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
    // 接近传感器输入：上拉偏置、低有效触发，软件去抖由底层 GPIO 驱动执行。
    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    cfg.debounce_ms = debounce_ms;
    // 将实时优先级和 CPU 亲和性下传给 GPIO 监控线程，保证安全触发路径可预期。
    cfg.rt_priority = rt_priority;
    cfg.cpu_affinity = cpu_affinity;
    cfg.use_irq = use_irq;
    if (!pin_->open(cfg)) {
        spdlog::error("[LimitSwitch] Failed to open GPIO pin for {} side",
                      side_ == LimitSide::LEFT ? "LEFT" : "RIGHT");
        return false;
    }
    return true;
}

void LimitSwitch::close() {
    if (pin_->is_open()) {
        pin_->stop_monitoring();
        // 关闭前先解绑回调，避免底层监控线程在对象析构后继续访问 this。
        pin_->set_edge_callback(hal::GpioEdge::BOTH, nullptr);
        pin_->close();
    }
}

bool LimitSwitch::is_open() const {
    return pin_ && pin_->is_open();
}

void LimitSwitch::start_monitoring() {
    // 接近传感器低有效：下降沿表示进入触发态。
    pin_->set_edge_callback(hal::GpioEdge::FALLING, [this]() { on_edge(); });
    pin_->start_monitoring();
}

void LimitSwitch::stop_monitoring() {
    pin_->stop_monitoring();
}

void LimitSwitch::set_trigger_callback(TriggerCallback cb) {
    // 加锁防止在监控线程运行期间替换回调产生数据竞争。
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
    // 原子置位，供 SafetyMonitor 的备用轮询路径观察。
    triggered_.store(true, std::memory_order_release);

    // 通知 SafetyMonitor；回调运行在 GPIO 监控线程，必须保持极短且不可阻塞。
    TriggerCallback cb;
    {
        std::lock_guard<hal::PiMutex> lock(cb_mtx_);
        cb = callback_;
    }
    if (cb) {
        cb(side_);
    }
}

}  // namespace robot::device
