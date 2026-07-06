/**
 * @file thread_executor.h
 * @brief 周期任务线程执行器接口。
 *
 * ThreadExecutor 负责按固定周期调用 IRunnable::update()，并可设置调度策略、优先级和 CPU 亲和性。
 * 线程本身不处理业务错误，调用方通过看门狗和 ErrorManager 观察执行停滞。
 */
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot::middleware {

/// @brief 可调度任务接口。
///
/// 使用方式（推荐用 RunnableAdapter 包装 lambda）：
/// @code
/// // 方式1：实现接口
/// struct MyTask : IRunnable {
///     void update() override { /* 业务逻辑 */ }
/// };
///
/// // 方式2：lambda 包装（推荐）
/// auto exec = ThreadExecutor(ThreadExecutor::Config{
///     .name = "my_worker",
///     .period_ms = 50,
///     .sched_policy = SCHED_FIFO,
///     .sched_priority = 80,
/// });
/// exec.add_runnable(std::make_shared<RunnableAdapter>([]{ /* ... */ }));
/// exec.start();
/// @endcode
struct IRunnable {
    virtual ~IRunnable() = default;
    virtual void update() = 0;
};

/// @brief 线程执行器：以固定周期调用 IRunnable::update()。
///
/// 支持可选 POSIX 调度策略配置。恢复流程可能会主动 stop/restart 执行器，
/// 因此停止接口必须以“可超时、可报告失败”为边界，不能在调用方静默阻塞。
class ThreadExecutor {
public:
    struct Config {
        std::string name;            ///< 线程名（最多15字符）
        int         period_ms{100};  ///< 调用 update() 的周期（毫秒）
        int         sched_policy{0}; ///< SCHED_OTHER=0, SCHED_FIFO=1, SCHED_RR=2
        int         sched_priority{0}; ///< RT 优先级（SCHED_FIFO/RR 时有效）
        int         cpu_affinity{0}; ///< CPU 亲和性掩码（0=不绑定；1<<N=绑定到核心N）
    };

    explicit ThreadExecutor(Config cfg);
    ~ThreadExecutor();

    /// @brief 添加周期任务；只能在线程未启动且已停止时调用。
    bool add_runnable(std::shared_ptr<IRunnable> runnable);
    bool start();
    void stop();

    /// @brief 请求线程退出，不等待当前 update() 返回。
    void request_stop();

    /// @brief 等待线程退出；超时返回 false，保留 joinable 线程给调用方处理。
    bool wait_stopped(std::chrono::milliseconds timeout);

    /// @brief 协作式停止；不会 detach 被阻塞的线程。
    ///
    /// 该接口用于恢复流程停驱动前的线程收敛。返回 false 时，调用方必须把恢复视为失败，
    /// 避免 close/open 与仍在运行的 update() 并发访问同一硬件句柄。
    bool stop_with_timeout(std::chrono::milliseconds timeout);

    /// @brief 仅在线程完全停止后重新启动。
    bool restart();
    bool is_running() const { return running_.load(); }
    void set_period_ms(int period_ms);
    int period_ms() const { return period_ms_.load(); }

private:
    void loop();

    Config config_;
    std::vector<std::shared_ptr<IRunnable>> runnables_;
    mutable std::mutex runnables_mtx_;
    std::atomic<bool> running_{false};
    std::atomic<int> period_ms_;
    std::thread thread_;

    // sleep_mtx_ 只保护唤醒标志，lifecycle_mtx_ 只保护停止状态；
    // 两者分离，避免 stop 请求因为等待 join 或生命周期锁而无法及时唤醒线程。
    mutable std::mutex sleep_mtx_;
    std::condition_variable sleep_cv_;
    bool wake_requested_{false};
    mutable std::mutex lifecycle_mtx_;
    std::condition_variable stopped_cv_;
    bool stopped_{true};
};

/// @brief Lambda 适配器，将 std::function<void()> 包装为 IRunnable。
class RunnableAdapter : public IRunnable {
public:
    using Fn = std::function<void()>;
    explicit RunnableAdapter(Fn fn) : fn_(std::move(fn)) {}
    void update() override { fn_(); }
private:
    Fn fn_;
};

} // namespace robot::middleware
