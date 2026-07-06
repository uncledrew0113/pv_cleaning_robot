/**
 * @file limit_switch.h
 * @brief 主限位接近传感器设备接口。
 *
 * 本类只封装 GPIO 电平、边沿监控和软件去抖；触发后的安全停机、切段和故障处理
 * 由 SafetyMonitor / RobotController 负责。
 */
#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

namespace robot::device {

/// @brief 主限位安装侧。
enum class LimitSide { LEFT, RIGHT };

/// @brief 主限位接近传感器，低有效 GPIO 触发。
///
/// 触发后通过回调通知 SafetyMonitor 的安全优先路径，同时置位原子标志供备用轮询查询。
/// 本类不直接发送急停指令，避免 device 层绑定具体运动控制对象。
class LimitSwitch {
public:
    using TriggerCallback = std::function<void(LimitSide)>;

    explicit LimitSwitch(std::shared_ptr<hal::IGpioPin> pin, LimitSide side);
    ~LimitSwitch();

    /// @param rt_priority GPIO 监控线程实时优先级，默认高于 SafetyMonitor 主循环。
    /// @param debounce_ms 软件去抖时间。
    /// @param cpu_affinity CPU 亲和性掩码（0=不绑定，1<<N=绑定到核心N）
    bool open(int rt_priority = 95, int debounce_ms = 2, int cpu_affinity = 0,
              bool use_irq = true);
    void close();
    bool is_open() const;

    /// 启动边沿监控线程；FALLING 表示接近传感器进入低有效触发态。
    void start_monitoring();
    void stop_monitoring();

    /// @warning 请在 start_monitoring() 前注册，避免运行期替换回调带来额外同步开销。
    void set_trigger_callback(TriggerCallback cb);

    bool is_triggered() const;
    void clear_trigger();

    /// @brief 直接读取 GPIO 当前电平（true=高/1，false=低/0）
    /// @note 用于上电自检：左右两侧的业务语义由 domain::estimate_endpoint() 决定。
    /// @pre  必须在 open() 之后调用
    bool read_current_level() const;

    LimitSide side() const {
        return side_;
    }

private:
    void on_edge();

    std::shared_ptr<hal::IGpioPin> pin_;
    LimitSide side_;

    // 保护 callback_，避免 GPIO 监控线程读回调时与控制线程替换回调产生数据竞争。
    hal::PiMutex cb_mtx_;
    TriggerCallback callback_;

    std::atomic<bool> triggered_{false};
};

}  // namespace robot::device
