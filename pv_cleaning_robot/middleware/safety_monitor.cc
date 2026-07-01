#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

namespace {
robot::domain::Endpoint to_endpoint(robot::device::LimitSide side) {
    return side == robot::device::LimitSide::LEFT ? robot::domain::Endpoint::A
                                                  : robot::domain::Endpoint::B;
}

/// 返回单调时钟毫秒时间戳，用于限位去抖和 release re-arm 计时。
inline uint64_t now_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

constexpr uint64_t kLimitSettleStableMs = 30u;
constexpr uint64_t kReleaseStableMs = 30u;
}  // namespace

namespace robot::middleware {

SafetyMonitor::SafetyMonitor(std::function<void()> emergency_stop,
                             std::shared_ptr<device::LimitSwitch> left_switch,
                             std::shared_ptr<device::LimitSwitch> right_switch,
                             EventBus& event_bus)
    : emergency_stop_(std::move(emergency_stop))
    , left_switch_(std::move(left_switch))
    , right_switch_(std::move(right_switch))
    , event_bus_(event_bus) {}

SafetyMonitor::~SafetyMonitor() {
    stop();
}

bool SafetyMonitor::start() {
    if (!left_switch_ || !right_switch_) {
        spdlog::error("[SafetyMonitor] start failed: missing limit switch");
        return false;
    }
    if (!left_switch_->is_open() || !right_switch_->is_open()) {
        spdlog::error("[SafetyMonitor] start failed: limit switch GPIO is not open");
        return false;
    }

    // 注册限位开关触发回调；回调在各自 GPIO 监控线程内同步调用。
    left_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(to_endpoint(side)); });
    right_switch_->set_trigger_callback(
        [this](device::LimitSide side) { on_limit_trigger(to_endpoint(side)); });

    // 启动 GPIO 边沿监控。若底层配置为轮询模式，驱动层仍以相同回调语义上报触发。
    left_switch_->start_monitoring();
    right_switch_->start_monitoring();

    // 启动监控主循环线程。优先级低于 GPIO 线程，保证首响急停路径优先执行；
    // 本线程只做去抖确认、release re-arm 和备用轮询兜底。
    running_.store(true);
    monitor_thread_ = std::thread(&SafetyMonitor::monitor_loop, this);

    return true;
}

void SafetyMonitor::stop() {
    running_.store(false);
    left_switch_->stop_monitoring();
    right_switch_->stop_monitoring();
    if (monitor_thread_.joinable())
        monitor_thread_.join();
}

void SafetyMonitor::set_limit_settled_callback(std::function<void(domain::Endpoint)> cb) {
    limit_settled_cb_ = std::move(cb);
}

void SafetyMonitor::on_limit_trigger(domain::Endpoint endpoint) {
    // ============================================================
    // 安全优先关键路径：此函数在 GPIO 监控线程中被调用。
    // 目标：从触发到急停指令发出不超过 50 ms。
    //
    // 两端完全对称：立即安全停机 + 清除触发标志 + 置 pending 时间戳。
    // 稳定到位确认和状态机通知由 monitor_loop 负责。
    // pending 时间戳本身就是去重门闩：同一侧在 settled 前抑制重复触发。
    // ============================================================
    std::atomic<uint64_t>& pending_ts =
        endpoint == domain::Endpoint::A ? pending_left_ts_ : pending_right_ts_;
    std::atomic<bool>& wait_release =
        endpoint == domain::Endpoint::A ? left_wait_release_ : right_wait_release_;
    std::atomic<uint64_t>& release_ts =
        endpoint == domain::Endpoint::A ? left_release_ts_ : right_release_ts_;
    auto& limit_switch = endpoint == domain::Endpoint::A ? left_switch_ : right_switch_;

    if (wait_release.load(std::memory_order_acquire) ||
        pending_ts.load(std::memory_order_acquire) != 0) {
        limit_switch->clear_trigger();
        return;
    }

    limit_switch->clear_trigger();
    emergency_stop_();
    pending_ts.store(now_ms(), std::memory_order_release);
    wait_release.store(true, std::memory_order_release);
    release_ts.store(0, std::memory_order_release);
}

void SafetyMonitor::monitor_loop() {
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    // 线程创建后立即设置调度参数，避免安全监控循环长期运行在普通调度策略下。
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

    // 非阻塞并行去抖：前后端各自独立计时，互不阻塞。
    // check_settled 每 5ms 被调用一次，持续低有效稳定后发布事件。
    auto check_settled = [&](std::atomic<uint64_t>& ts_atom,
                             domain::Endpoint endpoint,
                             const std::shared_ptr<device::LimitSwitch>& sw) {
        uint64_t t = ts_atom.load(std::memory_order_acquire);
        if (t == 0)
            return;
        if (sw->read_current_level()) {
            ts_atom.store(0, std::memory_order_release);
            return;
        }
        if (now_ms() - t >= kLimitSettleStableMs) {
            ts_atom.store(0, std::memory_order_release);  // 清除 pending，防止同一次触发重复发布。
            event_bus_.publish(LimitSettledEvent{endpoint});
            if (limit_settled_cb_) {
                limit_settled_cb_(endpoint);
            }
        }
    };

    auto check_release = [&](std::atomic<bool>& wait_release,
                             std::atomic<uint64_t>& release_ts,
                             std::shared_ptr<device::LimitSwitch>& sw) {
        if (!wait_release.load(std::memory_order_acquire)) {
            return;
        }
        // 感应式限位低有效：电平回高且稳定后才允许同侧重新触发。
        if (!sw->read_current_level()) {
            release_ts.store(0, std::memory_order_release);
            return;
        }

        const uint64_t now = now_ms();
        uint64_t candidate_ts = release_ts.load(std::memory_order_acquire);
        if (candidate_ts == 0) {
            release_ts.store(now, std::memory_order_release);
            return;
        }
        if (now - candidate_ts >= kReleaseStableMs) {
            sw->clear_trigger();
            wait_release.store(false, std::memory_order_release);
            release_ts.store(0, std::memory_order_release);
        }
    };

    while (running_.load()) {
        check_release(left_wait_release_, left_release_ts_, left_switch_);
        check_release(right_wait_release_, right_release_ts_, right_switch_);

        // ── 非阻塞并行去抖检查（前后端独立计时）──────────────────────
        check_settled(pending_left_ts_, domain::Endpoint::A, left_switch_);
        check_settled(pending_right_ts_, domain::Endpoint::B, right_switch_);

        // ── 备用轮询路径：防止 GPIO 边沿回调丢失 ────────────────────
        // 注意：on_limit_trigger() 内部已调用 clear_trigger()，
        // 因此重复触发被抑制，无需额外去重计数器。
        if (left_switch_->is_triggered() && pending_left_ts_.load(std::memory_order_acquire) == 0) {
            on_limit_trigger(domain::Endpoint::A);
        }
        if (right_switch_->is_triggered() &&
            pending_right_ts_.load(std::memory_order_acquire) == 0) {
            on_limit_trigger(domain::Endpoint::B);
        }
        // 5 ms 轮询；低于 GPIO 监控线程优先级，避免兜底路径反向压制首响急停。
        struct timespec poll_ts {
            0, 5 * 1000 * 1000
        };
        nanosleep(&poll_ts, nullptr);
    }
}

}  // namespace robot::middleware
