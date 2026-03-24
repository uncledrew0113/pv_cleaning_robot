#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <pthread.h>
#include <sched.h>
#include <chrono>

namespace robot::middleware {

ThreadExecutor::ThreadExecutor(Config cfg)
    : config_(std::move(cfg))
{
}

ThreadExecutor::~ThreadExecutor()
{
    stop();
}

void ThreadExecutor::add_runnable(std::shared_ptr<IRunnable> runnable)
{
    runnables_.push_back(std::move(runnable));
}

bool ThreadExecutor::start()
{
    running_.store(true);
    thread_ = std::thread(&ThreadExecutor::loop, this);

    // 配置调度策略
    if (config_.sched_policy == SCHED_FIFO || config_.sched_policy == SCHED_RR) {
        sched_param sp{};
        sp.sched_priority = config_.sched_priority;
        pthread_setschedparam(thread_.native_handle(), config_.sched_policy, &sp);
    }

    // 设置线程名（最多15字符）
    if (!config_.name.empty()) {
        pthread_setname_np(thread_.native_handle(),
                           config_.name.substr(0, 15).c_str());
    }

    return true;
}

void ThreadExecutor::stop()
{
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void ThreadExecutor::loop()
{
    using Clock = std::chrono::steady_clock;
    auto next_wake = Clock::now();

    while (running_.load()) {
        auto now = Clock::now();

        for (auto& r : runnables_) {
            r->update();
        }

        next_wake += std::chrono::milliseconds(config_.period_ms);
        // 若执行超时则跳到下一个周期（避免积压）
        if (next_wake < Clock::now()) {
            next_wake = Clock::now() + std::chrono::milliseconds(config_.period_ms);
        }
        std::this_thread::sleep_until(next_wake);
    }
}

} // namespace robot::middleware
