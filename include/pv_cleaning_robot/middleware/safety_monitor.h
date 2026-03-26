#pragma once
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include <atomic>
#include <memory>
#include <thread>

namespace robot::middleware {

/// @brief 安全监控器
///
/// - SCHED_FIFO 优先级 99，专用线程
/// - 监听前/后限位开关触发回调（由 LimitSwitch 直接调用）
/// - 端到端响应路径（触发 → 急停指令发送）目标 ≤ 50 ms：
///     LimitSwitch::on_edge [GPIO 监控线程, SCHED_FIFO 99]
///         → on_limit_trigger() [在 GPIO 监控线程栈上同步调用]
///             → WalkMotor::emergency_stop() [直写 CAN 帧, <1ms]
///
/// @note WalkMotorGroup 的急停路径不经任何队列，直接调用 ICanBus::send()
/// emergency_override(0.0f) 同时停全部4轮并锁定心跳，防止50ms周期帧重新驱动电机
class SafetyMonitor {
public:
    /// @brief 发布到 EventBus 的限位触发事件
    struct LimitTriggerEvent {
        device::LimitSide side;
    };

    SafetyMonitor(std::shared_ptr<device::WalkMotorGroup> walk_group,
                  std::shared_ptr<device::LimitSwitch>    front_switch,
                  std::shared_ptr<device::LimitSwitch>    rear_switch,
                  EventBus&                               event_bus);
    ~SafetyMonitor();

    /// 启动安全监控（启动 GPIO 监控线程，设置 SCHED_FIFO 99）
    bool start();

    /// 停止安全监控
    void stop();

    /// 是否处于急停锁定状态（需上层手动 reset 才能解除）
    bool is_estop_active() const;

    /// 手动复位急停（上层确认安全后调用）
    void reset_estop();

private:
    /// LimitSwitch 触发回调（在 GPIO 监控线程中被调用，必须极短）
    void on_limit_trigger(device::LimitSide side);

    /// 安全监视主循环（SCHED_FIFO 99）
    void monitor_loop();

    std::shared_ptr<device::WalkMotorGroup> walk_group_;
    std::shared_ptr<device::LimitSwitch>    front_switch_;
    std::shared_ptr<device::LimitSwitch>    rear_switch_;
    EventBus&                               event_bus_;

    std::atomic<bool> running_{false};
    std::atomic<bool> estop_active_{false};
    std::thread       monitor_thread_;
};

} // namespace robot::middleware
