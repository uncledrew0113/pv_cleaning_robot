#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

namespace robot::device {

LimitSwitch::LimitSwitch(std::shared_ptr<hal::IGpioPin> pin, LimitSide side)
    : pin_(std::move(pin)), side_(side)
{
}

LimitSwitch::~LimitSwitch()
{
    close();
}

bool LimitSwitch::open()
{
    // 感应式限位开关：输入模式，上拉偏置（低有效），10ms 软件消抖
    hal::GpioConfig cfg;
    cfg.direction    = hal::GpioDirection::INPUT;
    cfg.bias         = hal::GpioBias::PULL_UP;
    cfg.debounce_ms  = 10;
    return pin_->open(cfg);
}

void LimitSwitch::close()
{
    pin_->stop_monitoring();
    pin_->close();
}

void LimitSwitch::start_monitoring()
{
    // 感应式限位开关：物体接近时输出下降沿（低有效）
    pin_->set_edge_callback(hal::GpioEdge::FALLING,
                            [this]() { on_edge(); });
    pin_->start_monitoring();
}

void LimitSwitch::stop_monitoring()
{
    pin_->stop_monitoring();
}

void LimitSwitch::set_trigger_callback(TriggerCallback cb)
{
    callback_ = std::move(cb);
}

bool LimitSwitch::is_triggered() const
{
    return triggered_.load(std::memory_order_acquire);
}

void LimitSwitch::clear_trigger()
{
    triggered_.store(false, std::memory_order_release);
}

void LimitSwitch::on_edge()
{
    // 原子置位（极短路径，监控线程直接调用）
    triggered_.store(true, std::memory_order_release);

    // 通知 SafetyMonitor（回调不得阻塞，用于触发急停序列）
    if (callback_) {
        callback_(side_);
    }
}

} // namespace robot::device
