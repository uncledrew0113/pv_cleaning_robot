#include "pv_cleaning_robot/device/attitude_limit_switch.h"

#include <mutex>

namespace robot::device {

AttitudeLimitSwitch::AttitudeLimitSwitch(std::shared_ptr<hal::IGpioPin> pin,
                                         AttitudeLimitSide side)
    : pin_(std::move(pin)), side_(side) {}

AttitudeLimitSwitch::~AttitudeLimitSwitch() {
    close();
}

bool AttitudeLimitSwitch::open(int rt_priority,
                               int debounce_ms,
                               int cpu_affinity,
                               bool use_irq) {
    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    cfg.debounce_ms = debounce_ms;
    cfg.rt_priority = rt_priority;
    cfg.cpu_affinity = cpu_affinity;
    cfg.use_irq = use_irq;
    return pin_->open(cfg);
}

void AttitudeLimitSwitch::close() {
    if (pin_->is_open()) {
        pin_->stop_monitoring();
        pin_->set_edge_callback(hal::GpioEdge::BOTH, nullptr);
        pin_->close();
    }
}

void AttitudeLimitSwitch::start_monitoring() {
    pin_->set_edge_callback(hal::GpioEdge::FALLING, [this]() { on_edge(); });
    pin_->start_monitoring();
}

void AttitudeLimitSwitch::stop_monitoring() {
    pin_->stop_monitoring();
}

void AttitudeLimitSwitch::set_trigger_callback(TriggerCallback cb) {
    std::lock_guard<hal::PiMutex> lock(cb_mtx_);
    callback_ = std::move(cb);
}

bool AttitudeLimitSwitch::is_triggered() const {
    return triggered_.load(std::memory_order_acquire);
}

void AttitudeLimitSwitch::clear_trigger() {
    triggered_.store(false, std::memory_order_release);
}

bool AttitudeLimitSwitch::read_current_level() const {
    return pin_->read_value();
}

AttitudeLimitSwitch::Status AttitudeLimitSwitch::read_status() const {
    const bool level_high = read_current_level();
    return Status{side_, level_high, !level_high, is_triggered()};
}

void AttitudeLimitSwitch::on_edge() {
    triggered_.store(true, std::memory_order_release);
    std::lock_guard<hal::PiMutex> lock(cb_mtx_);
    if (callback_) {
        callback_(side_);
    }
}

}  // namespace robot::device
