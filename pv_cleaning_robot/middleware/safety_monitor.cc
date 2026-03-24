#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>

namespace robot::middleware {

SafetyMonitor::SafetyMonitor(
    std::shared_ptr<device::WalkMotor>   walk_motor,
    std::shared_ptr<device::LimitSwitch> front_switch,
    std::shared_ptr<device::LimitSwitch> rear_switch,
    EventBus&                            event_bus)
    : walk_motor_(std::move(walk_motor))
    , front_switch_(std::move(front_switch))
    , rear_switch_(std::move(rear_switch))
    , event_bus_(event_bus)
{
}

SafetyMonitor::~SafetyMonitor()
{
    stop();
}

bool SafetyMonitor::start()
{
    // 注册限位触发回调（在各自 GPIO 监控线程内同步调用）
    front_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(side); });
    rear_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(side); });

    // 启动 GPIO 边沿监控
    front_switch_->start_monitoring();
    rear_switch_->start_monitoring();

    // 启动监控主循环线程（SCHED_FIFO 99）
    running_.store(true);
    monitor_thread_ = std::thread(&SafetyMonitor::monitor_loop, this);

    // 设置实时调度策略
    sched_param sp{};
    sp.sched_priority = 99;
    pthread_setschedparam(monitor_thread_.native_handle(), SCHED_FIFO, &sp);

    return true;
}

void SafetyMonitor::stop()
{
    running_.store(false);
    front_switch_->stop_monitoring();
    rear_switch_->stop_monitoring();
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

bool SafetyMonitor::is_estop_active() const
{
    return estop_active_.load(std::memory_order_acquire);
}

void SafetyMonitor::reset_estop()
{
    estop_active_.store(false, std::memory_order_release);
    // 清除限位标志，允许恢复运动
    front_switch_->clear_trigger();
    rear_switch_->clear_trigger();
}

void SafetyMonitor::on_limit_trigger(device::LimitSide side)
{
    // ============================================================
    // 安全优先关键路径：此函数在 GPIO 监控线程中被调用
    // 目标：从触发到 CAN 急停帧发出 ≤ 50 ms
    // ============================================================

    // 1. 置位急停标志（原子）
    estop_active_.store(true, std::memory_order_release);

    // 2. 立即发送急停指令（直写 CAN 帧，无队列，< 1 ms）
    walk_motor_->disable();

    // 3. 发布事件到 EventBus（供 FSM/FaultService 响应，非阻塞）
    event_bus_.publish(LimitTriggerEvent{side});
}

void SafetyMonitor::monitor_loop()
{
    // 作为看门线程：持续轮询以防回调丢失（双保险）
    while (running_.load()) {
        if (!estop_active_.load(std::memory_order_acquire)) {
            if (front_switch_->is_triggered() || rear_switch_->is_triggered()) {
                // 通过轮询检测（回调备用路径）
                device::LimitSide side = front_switch_->is_triggered()
                    ? device::LimitSide::FRONT
                    : device::LimitSide::REAR;
                on_limit_trigger(side);
            }
        }
        // 5 ms 轮询（SCHED_FIFO 99，会让出给更低优先级任务）
        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = 5 * 1000 * 1000;
        nanosleep(&ts, nullptr);
    }
}

} // namespace robot::middleware
