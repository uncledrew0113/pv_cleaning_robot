/**
 * @file attitude_limit_switch.h
 * @brief 姿态限位接近传感器设备接口。
 *
 * 本类封装左右下轮姿态限位的 GPIO 电平读取、边沿监控和触发标志。急停、错误提交和回中策略
 * 均由 AttitudeLimitService 负责，避免设备层依赖业务状态。
 */
#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "pv_cleaning_robot/hal/i_gpio_pin.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::device {

/// @brief 姿态限位安装侧。
enum class AttitudeLimitSide {
    LEFT_LOWER,
    RIGHT_LOWER,
};

/// @brief 姿态偏移极限接近传感器。
///
/// 安装在下轮附近，用于暴露机器人姿态偏移到达极限的设备层事实。
/// 本类只负责 GPIO 输入、触发标志和当前状态读取，不关联急停、业务故障或纠偏策略。
class AttitudeLimitSwitch {
public:
    using TriggerCallback = std::function<void(AttitudeLimitSide)>;

    struct Status {
        AttitudeLimitSide side{AttitudeLimitSide::LEFT_LOWER};
        bool level_high{true};
        bool active_low_asserted{false};
        bool triggered{false};
    };

    explicit AttitudeLimitSwitch(std::shared_ptr<hal::IGpioPin> pin, AttitudeLimitSide side);
    ~AttitudeLimitSwitch();

    bool open(int rt_priority = 95, int debounce_ms = 2, int cpu_affinity = 0,
              bool use_irq = true);
    void close();

    void start_monitoring();
    void stop_monitoring();
    void set_trigger_callback(TriggerCallback cb);

    bool is_triggered() const;
    void clear_trigger();
    bool read_current_level() const;
    Status read_status() const;

    AttitudeLimitSide side() const {
        return side_;
    }

private:
    void on_edge();

    std::shared_ptr<hal::IGpioPin> pin_;
    AttitudeLimitSide side_;
    hal::PiMutex cb_mtx_;
    TriggerCallback callback_;
    std::atomic<bool> triggered_{false};
};

}  // namespace robot::device
