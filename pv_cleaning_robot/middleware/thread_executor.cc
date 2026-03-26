#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <spdlog/spdlog.h>
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
    return true;
}

void ThreadExecutor::stop()
{
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void ThreadExecutor::loop()
{
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    // 必须在线程内部设置：从外部设置存在启动竞争窗口，
    // 高负载时新线程可能已运行多次迭代才被提权。
    if (config_.sched_policy == SCHED_FIFO || config_.sched_policy == SCHED_RR) {
        sched_param sp{};
        sp.sched_priority = config_.sched_priority;
        int rc = pthread_setschedparam(pthread_self(), config_.sched_policy, &sp);
        if (rc != 0) {
            spdlog::warn("[ThreadExecutor] '{}' RT priority elevation failed: {}",
                         config_.name, strerror(rc));
        }
    }
    if (config_.cpu_affinity != 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (int i = 0; i < 64; ++i) {
            if (config_.cpu_affinity & (1 << i))
                CPU_SET(i, &cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[ThreadExecutor] '{}' CPU affinity set failed: {}",
                         config_.name, strerror(errno));
        }
    }
    if (!config_.name.empty()) {
        pthread_setname_np(pthread_self(), config_.name.substr(0, 15).c_str());
    }

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
