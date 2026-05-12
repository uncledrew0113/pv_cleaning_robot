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
    , period_ms_(config_.period_ms)
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
    {
        std::lock_guard<std::mutex> lk(sleep_mtx_);
        wake_requested_ = false;
    }
    running_.store(true);
    thread_ = std::thread(&ThreadExecutor::loop, this);
    return true;
}

void ThreadExecutor::stop()
{
    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(sleep_mtx_);
        wake_requested_ = true;
    }
    sleep_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void ThreadExecutor::set_period_ms(int period_ms)
{
    if (period_ms <= 0) return;
    period_ms_.store(period_ms);
    {
        std::lock_guard<std::mutex> lk(sleep_mtx_);
        wake_requested_ = true;
    }
    sleep_cv_.notify_all();
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
        for (auto& r : runnables_) {
            r->update();
        }

        const int period_ms = period_ms_.load();
        next_wake += std::chrono::milliseconds(period_ms);
        // 若执行超时则跳到下一个周期（避免积压）
        if (next_wake < Clock::now()) {
            next_wake = Clock::now() + std::chrono::milliseconds(period_ms);
        }
        std::unique_lock<std::mutex> lk(sleep_mtx_);
        const bool interrupted =
            sleep_cv_.wait_until(lk, next_wake, [this]() {
                return !running_.load() || wake_requested_;
            });
        if (!running_.load()) {
            break;
        }
        if (interrupted) {
            wake_requested_ = false;
            next_wake = Clock::now();
        }
    }
}

} // namespace robot::middleware
