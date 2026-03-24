#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace robot::middleware {

/// @brief 可调度任务接口
struct IRunnable {
    virtual ~IRunnable() = default;
    virtual void update() = 0;
};

/// @brief 线程执行器——以固定周期调用 IRunnable::update()
///
/// 支持可选 POSIX 调度策略配置（SCHED_FIFO / SCHED_RR / SCHED_OTHER）
class ThreadExecutor {
public:
    struct Config {
        std::string name;            ///< 线程名（最多15字符）
        int         period_ms{100};  ///< 调用 update() 的周期（毫秒）
        int         sched_policy{0}; ///< SCHED_OTHER=0, SCHED_FIFO=1, SCHED_RR=2
        int         sched_priority{0}; ///< RT 优先级（SCHED_FIFO/RR 时有效）
    };

    explicit ThreadExecutor(Config cfg);
    ~ThreadExecutor();

    void add_runnable(std::shared_ptr<IRunnable> runnable);
    bool start();
    void stop();
    bool is_running() const { return running_.load(); }

private:
    void loop();

    Config config_;
    std::vector<std::shared_ptr<IRunnable>> runnables_;
    std::atomic<bool> running_{false};
    std::thread       thread_;
};

/// @brief Lambda 适配器（将 std::function<void()> 包装为 IRunnable）
class RunnableAdapter : public IRunnable {
public:
    using Fn = std::function<void()>;
    explicit RunnableAdapter(Fn fn) : fn_(std::move(fn)) {}
    void update() override { fn_(); }
private:
    Fn fn_;
};

} // namespace robot::middleware
