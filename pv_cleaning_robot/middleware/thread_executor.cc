#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <spdlog/spdlog.h>
#include <chrono>
#include <exception>

namespace robot::middleware {

namespace {

bool valid_sched_policy(int policy) {
    return policy == SCHED_OTHER || policy == SCHED_FIFO || policy == SCHED_RR;
}

bool valid_sched_priority(int policy, int priority) {
    // SCHED_OTHER 在 Linux 上不使用实时优先级；要求 priority 为 0，
    // 让配置错误尽早暴露，而不是被系统静默忽略。
    if (policy == SCHED_OTHER) {
        return priority == 0;
    }
    const int min_priority = sched_get_priority_min(policy);
    const int max_priority = sched_get_priority_max(policy);
    if (min_priority < 0 || max_priority < 0) {
        return false;
    }
    return priority >= min_priority && priority <= max_priority;
}

}  // namespace

ThreadExecutor::ThreadExecutor(Config cfg)
    : config_(std::move(cfg))
    , period_ms_(config_.period_ms)
{
}

ThreadExecutor::~ThreadExecutor()
{
    stop();
}

bool ThreadExecutor::add_runnable(std::shared_ptr<IRunnable> runnable)
{
    if (!runnable) {
        spdlog::error("[ThreadExecutor] '{}' rejected null runnable", config_.name);
        return false;
    }
    std::lock_guard<std::mutex> lk(runnables_mtx_);
    bool stopped = false;
    {
        std::lock_guard<std::mutex> lifecycle_lk(lifecycle_mtx_);
        stopped = stopped_;
    }
    if (running_.load() || !stopped) {
        spdlog::error("[ThreadExecutor] '{}' rejected add_runnable while running",
                      config_.name);
        return false;
    }
    runnables_.push_back(std::move(runnable));
    return true;
}

bool ThreadExecutor::start()
{
    if (config_.period_ms <= 0) {
        spdlog::error("[ThreadExecutor] '{}' invalid period_ms={}",
                      config_.name,
                      config_.period_ms);
        return false;
    }
    if (!valid_sched_policy(config_.sched_policy)) {
        spdlog::error("[ThreadExecutor] '{}' invalid sched_policy={}",
                      config_.name,
                      config_.sched_policy);
        return false;
    }
    if (!valid_sched_priority(config_.sched_policy, config_.sched_priority)) {
        spdlog::error("[ThreadExecutor] '{}' invalid sched_priority={} for policy={}",
                      config_.name,
                      config_.sched_priority,
                      config_.sched_policy);
        return false;
    }
    if (running_.load()) {
        return true;
    }

    // 工作线程运行期间冻结 runnable 列表，避免 loop() 遍历时与 add_runnable()
    // 并发修改容器；这样也不需要每个周期复制任务列表。
    std::lock_guard<std::mutex> runnables_lk(runnables_mtx_);
    {
        std::lock_guard<std::mutex> lk(lifecycle_mtx_);
        if (thread_.joinable() && stopped_) {
            thread_.join();
        }
        if (thread_.joinable()) {
            return false;
        }
        stopped_ = false;
    }
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
    request_stop();
    if (thread_.joinable()) {
        // 线程 join 自己会死锁；这里明确报错，由外部 owner 在线程外完成停止。
        if (thread_.get_id() == std::this_thread::get_id()) {
            spdlog::error("[ThreadExecutor] '{}' stop() called from worker thread",
                          config_.name);
            return;
        }
        thread_.join();
    }
    {
        std::lock_guard<std::mutex> lk(lifecycle_mtx_);
        stopped_ = true;
    }
    stopped_cv_.notify_all();
}

void ThreadExecutor::request_stop()
{
    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(sleep_mtx_);
        wake_requested_ = true;
    }
    sleep_cv_.notify_all();
}

bool ThreadExecutor::wait_stopped(std::chrono::milliseconds timeout)
{
    if (thread_.joinable() && thread_.get_id() == std::this_thread::get_id()) {
        spdlog::error("[ThreadExecutor] '{}' wait_stopped() called from worker thread",
                      config_.name);
        return false;
    }
    bool stopped = false;
    {
        std::unique_lock<std::mutex> lk(lifecycle_mtx_);
        stopped = stopped_cv_.wait_for(lk, timeout, [this] { return stopped_; });
    }
    if (!stopped) {
        return false;
    }

    // 只有 loop() 发布 stopped_ 后才 join；若超时，调用方仍持有未退出线程，
    // 上层恢复流程应将其视为停止失败并升级处理。
    if (thread_.joinable()) {
        thread_.join();
    }
    return true;
}

bool ThreadExecutor::stop_with_timeout(std::chrono::milliseconds timeout)
{
    if (thread_.joinable() && thread_.get_id() == std::this_thread::get_id()) {
        spdlog::error("[ThreadExecutor] '{}' stop_with_timeout() called from worker thread",
                      config_.name);
        request_stop();
        return false;
    }
    request_stop();
    return wait_stopped(timeout);
}

bool ThreadExecutor::restart()
{
    if (running_.load()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(lifecycle_mtx_);
        if (!stopped_) {
            return false;
        }
    }
    return start();
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
    {
        std::lock_guard<std::mutex> lk(lifecycle_mtx_);
        stopped_ = false;
    }

    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    // 必须在线程内部设置：从外部设置存在启动竞争窗口，高负载时新线程可能已运行多次
    // update() 才被提权。
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
        for (int i = 0; i < static_cast<int>(sizeof(config_.cpu_affinity) * 8); ++i) {
            if (config_.cpu_affinity & (1u << i))
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
            try {
                r->update();
            } catch (const std::exception& ex) {
                // 单个 runnable 抛异常不能静默杀死工作线程；记录错误后继续运行，
                // 是否恢复或停机交由 watchdog / ErrorManager 决策。
                spdlog::error("[ThreadExecutor] '{}' runnable threw exception: {}",
                              config_.name,
                              ex.what());
            } catch (...) {
                spdlog::error("[ThreadExecutor] '{}' runnable threw unknown exception",
                              config_.name);
            }
        }

        const int period_ms = period_ms_.load();
        next_wake += std::chrono::milliseconds(period_ms);
        // 若本轮 update() 已超过周期预算，直接跳到下一个周期，避免把旧周期积压执行。
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

    {
        std::lock_guard<std::mutex> lk(lifecycle_mtx_);
        stopped_ = true;
    }
    stopped_cv_.notify_all();
}

} // namespace robot::middleware
