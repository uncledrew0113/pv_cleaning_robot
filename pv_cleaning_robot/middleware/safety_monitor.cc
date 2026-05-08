#include "pv_cleaning_robot/middleware/safety_monitor.h"
// WalkMotorGroup 已通过 safety_monitor.h 间接包含
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>

namespace {
/// 返回单调时钟毫秒时间戳（用于防抖计时）
inline uint64_t now_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
        .count());
}
}  // namespace

namespace robot::middleware {

SafetyMonitor::SafetyMonitor(
    std::shared_ptr<device::WalkMotorGroup> walk_group,
    std::shared_ptr<device::LimitSwitch>    left_switch,
    std::shared_ptr<device::LimitSwitch>    right_switch,
    EventBus&                               event_bus)
    : walk_group_(std::move(walk_group))
    , left_switch_(std::move(left_switch))
    , right_switch_(std::move(right_switch))
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
    left_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(side); });
    right_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(side); });

    // 启动 GPIO 边沿监控
    left_switch_->start_monitoring();
    right_switch_->start_monitoring();

    // 启动监控主循环线程（SCHED_FIFO 94，绑定安全专用 CPU 4）
    // 设计为低于 GPIO 线程(95)：保证边沿回调可优先执行，轮询仅作兜底。
    running_.store(true);
    monitor_thread_ = std::thread(&SafetyMonitor::monitor_loop, this);

    return true;
}

void SafetyMonitor::stop()
{
    running_.store(false);
    left_switch_->stop_monitoring();
    right_switch_->stop_monitoring();
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

void SafetyMonitor::on_limit_trigger(device::LimitSide side)
{
    // ============================================================
    // 安全优先关键路径：此函数在 GPIO 监控线程中被调用（SCHED_FIFO 95）
    // 目标：从触发到急停指令发出 ≤ 50 ms
    //
    // 两端完全对称：立即停车 + 清除触发标志 + 置 pending 时间戳。
    // 防抖延迟（180ms）和 FSM 通知由 monitor_loop（SCHED_FIFO 94）负责。
    // pending 时间戳本身就是去重门闩：同一侧在 settled 前不接受重复触发。
    // ============================================================
    std::atomic<uint64_t>& pending_ts =
        side == device::LimitSide::LEFT ? pending_left_ts_ : pending_right_ts_;
    auto& limit_switch = side == device::LimitSide::LEFT ? left_switch_ : right_switch_;

    if (pending_ts.load(std::memory_order_acquire) != 0) {
        limit_switch->clear_trigger();
        return;
    }

    limit_switch->clear_trigger();
    walk_group_->emergency_override(0.0f);
    pending_ts.store(now_ms(), std::memory_order_release);
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

    // 非阻塞并行防抖：前后端各自独立计时，互不阻塞。
    // check_settled 每 5ms 被调用一次，距触发 ≥180ms 后发布事件。
    // 前后端同时触发时，均在 ~185ms（180ms+5ms误差）内响应，消除串行 nanosleep 的 ~360ms 延迟。
    auto check_settled = [&](std::atomic<uint64_t>& ts_atom, device::LimitSide side) {
        uint64_t t = ts_atom.load(std::memory_order_acquire);
        if (t == 0) return;
        if (now_ms() - t >= 180u) {
            ts_atom.store(0, std::memory_order_release);  // 清除，防止重复触发
            event_bus_.publish(LimitSettledEvent{side});
        }
    };

    while (running_.load()) {
        // ── 非阻塞并行防抖检查（前后端独立计时）──────────────────────
        check_settled(pending_left_ts_, device::LimitSide::LEFT);
        check_settled(pending_right_ts_, device::LimitSide::RIGHT);

        // ── 备用轮询路径：以防 GPIO 边沿回调丢失（双保险）────────────
        // 注意：on_limit_trigger() 内部已调用 clear_trigger()，
        // 因此重复触发被抑制，无需额外去重计数器。
        if (left_switch_->is_triggered() &&
            pending_left_ts_.load(std::memory_order_acquire) == 0) {
            on_limit_trigger(device::LimitSide::LEFT);
        }
        if (right_switch_->is_triggered() &&
            pending_right_ts_.load(std::memory_order_acquire) == 0) {
            on_limit_trigger(device::LimitSide::RIGHT);
        }
        // 5 ms 轮询（SCHED_FIFO 94，低于 GPIO 95，避免兜底轮询反向压制边沿线程）
        struct timespec poll_ts{0, 5 * 1000 * 1000};
        nanosleep(&poll_ts, nullptr);
    }
}

} // namespace robot::middleware
