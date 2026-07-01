#pragma once

#include <cstdint>
#include <memory>

#include "pv_cleaning_robot/hal/i_gpio_pin.h"

namespace robot::device {

/// @brief 锁止电机 DO 控制设备。
///
/// 锁止电机没有到位反馈，本类只能确认 GPIO 申请和写入是否成功，不能判断机械锁
/// 是否真的打开或关闭。这里的 open_lock()/close_lock() 是业务定义的“开/关动作”，
/// 不等同于 GPIO 生命周期 open()/close()。
class LockMotor {
   public:
    struct Config {
        uint32_t pulse_ms{200};   ///< DO 高电平保持时间。
        uint32_t settle_ms{8000}; ///< 动作后机械等待时间。
    };

    LockMotor(std::shared_ptr<hal::IGpioPin> open_pin,
              std::shared_ptr<hal::IGpioPin> close_pin,
              Config config);
    ~LockMotor();

    /// @brief 初始化两个 DO 输出，并先拉低，避免上电后残留高电平。
    bool initialize();
    void shutdown();

    /// @brief 执行“开”动作：open GPIO 高电平 pulse_ms 后拉低，再等待 settle_ms。
    bool open_lock();

    /// @brief 执行“关”动作：close GPIO 高电平 pulse_ms 后拉低，再等待 settle_ms。
    bool close_lock();

   private:
    bool pulse_pin(const std::shared_ptr<hal::IGpioPin>& pin, const char* action_name);
    static void sleep_ms(uint32_t ms);

    std::shared_ptr<hal::IGpioPin> open_pin_;
    std::shared_ptr<hal::IGpioPin> close_pin_;
    Config config_;
};

}  // namespace robot::device
