#include "pv_cleaning_robot/middleware/safety_monitor.h"
// WalkMotorGroup 已通过 safety_monitor.h 间接包含
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>

namespace robot::middleware {

SafetyMonitor::SafetyMonitor(
    std::shared_ptr<device::WalkMotorGroup> walk_group,
    std::shared_ptr<device::LimitSwitch>    front_switch,
    std::shared_ptr<device::LimitSwitch>    rear_switch,
    EventBus&                               event_bus)
    : walk_group_(std::move(walk_group))
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

    // 启动监控主循环线程（SCHED_FIFO 94，绑定安全专用 CPU 4）
    // 设计为低于 GPIO 线程(95)：保证边沿回调可优先执行，轮询仅作兜底。
    running_.store(true);
    monitor_thread_ = std::thread(&SafetyMonitor::monitor_loop, this);

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
    // 目标：从触发到指令发出 ≤ 50 ms
    //
    // 前端下降沿（FRONT）：到达光伏板前端 → 立即反向，不急停。
    //   此为正常行程事件，不置 estop_active_，清除触发标志防止
    //   monitor_loop 备用路径重复触发。
    //
    // 尾端下降沿（REAR）：到达停机位 → 立即禁止行走，置急停标志。
    // ============================================================

    if (side == device::LimitSide::FRONT) {
        // 清除触发标志（防止 monitor_loop 每 5 ms 重复派发）
        front_switch_->clear_trigger();
        // 发布事件，由 EventBus 订阅者驱动 FSM 发出 EvReachEnd → 反向
        event_bus_.publish(LimitTriggerEvent{side});
    } else {
        // 1. 置位急停标志（原子，防止 monitor_loop 重复触发）
        estop_active_.store(true, std::memory_order_release);
        rear_switch_->clear_trigger();
        // 2. 立即停全部4轮：emergency_override(0) 同时:
        //    a) 发1帧 4轮停止 CAN 帧（< 1 ms）
        //    b) 设置 override_active_=true，屏蔽 update() 50ms 心跳重发
        //    防止控制线程在急停后5~50ms内重新驱动电机
        walk_group_->emergency_override(0.0f);
        // 3. 发布事件，由 EventBus 订阅者驱动 FSM 发出 EvReachHome
        event_bus_.publish(LimitTriggerEvent{side});
    }
}

void SafetyMonitor::monitor_loop()
{
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    // 必须在线程内设置：安全关键路径，不容许启动竞争窗口（尤其是 SCHED_FIFO 94）。
    {
        sched_param sp{};
        sp.sched_priority = 94;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[SafetyMonitor] RT priority elevation failed: {}", strerror(rc));
        }
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(4, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[SafetyMonitor] CPU 4 affinity set failed: {}", strerror(errno));
        }
        pthread_setname_np(pthread_self(), "safety_mon");
    }

    // 备用轮询路径：以防 GPIO 边沿回调丢失（双保险）。
    // 注意：on_limit_trigger() 内部已调用 clear_trigger()，
    // 因此重复触发被抑制，无需额外去重计数器。
    while (running_.load()) {
        // 前端备用：FRONT 触发不置 estop，独立检查
        if (front_switch_->is_triggered()) {
            on_limit_trigger(device::LimitSide::FRONT);
        }
        // 尾端备用：estop 已激活则跳过（主路径已处理）
        if (rear_switch_->is_triggered() &&
            !estop_active_.load(std::memory_order_acquire)) {
            on_limit_trigger(device::LimitSide::REAR);
        }
        // 5 ms 轮询（SCHED_FIFO 94，低于 GPIO 95，避免兜底轮询反向压制边沿线程）
        struct timespec ts{0, 5 * 1000 * 1000};
        nanosleep(&ts, nullptr);
    }
}

} // namespace robot::middleware
