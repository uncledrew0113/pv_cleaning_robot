#include <catch2/catch.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/middleware/thread_executor.h"

using robot::middleware::RunnableAdapter;
using robot::middleware::ThreadExecutor;

namespace {

bool wait_until_true(const std::function<bool()>& pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

}  // namespace

TEST_CASE("ThreadExecutor stop interrupts long sleep promptly", "[middleware][thread_executor]") {
    std::atomic<int> ticks{0};
    ThreadExecutor exec({"test_exec", 10000, SCHED_OTHER, 0, 0});
    exec.add_runnable(std::make_shared<RunnableAdapter>([&ticks]() { ++ticks; }));

    REQUIRE(exec.start());
    REQUIRE(wait_until_true([&ticks]() { return ticks.load() >= 1; }, std::chrono::milliseconds(200)));

    const auto stop_begin = std::chrono::steady_clock::now();
    exec.stop();
    const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - stop_begin)
                             .count();

    REQUIRE(stop_ms < 500);
}

TEST_CASE("ThreadExecutor period change wakes sleeper immediately",
          "[middleware][thread_executor]") {
    std::atomic<int> ticks{0};
    ThreadExecutor exec({"test_exec", 10000, SCHED_OTHER, 0, 0});
    exec.add_runnable(std::make_shared<RunnableAdapter>([&ticks]() { ++ticks; }));

    REQUIRE(exec.start());
    REQUIRE(wait_until_true([&ticks]() { return ticks.load() >= 1; }, std::chrono::milliseconds(200)));

    exec.set_period_ms(10);

    REQUIRE(wait_until_true([&ticks]() { return ticks.load() >= 2; }, std::chrono::milliseconds(300)));
    exec.stop();
}

TEST_CASE("ThreadExecutor stop_with_timeout returns false when runnable blocks",
          "[middleware][thread_executor]") {
    using namespace std::chrono_literals;

    std::mutex m;
    std::condition_variable cv;
    bool entered = false;
    bool release = false;

    ThreadExecutor exec({"blocking_exec", 10, SCHED_OTHER, 0, 0});
    exec.add_runnable(std::make_shared<RunnableAdapter>([&] {
        std::unique_lock<std::mutex> lk(m);
        entered = true;
        cv.notify_all();
        cv.wait(lk, [&] { return release; });
    }));

    REQUIRE(exec.start());
    {
        std::unique_lock<std::mutex> lk(m);
        REQUIRE(cv.wait_for(lk, 500ms, [&] { return entered; }));
    }

    CHECK_FALSE(exec.stop_with_timeout(50ms));

    {
        std::lock_guard<std::mutex> lk(m);
        release = true;
    }
    cv.notify_all();
    CHECK(exec.wait_stopped(500ms));
}

TEST_CASE("ThreadExecutor restart starts after clean timeout stop",
          "[middleware][thread_executor]") {
    using namespace std::chrono_literals;

    std::atomic<int> count{0};
    ThreadExecutor exec({"restart_exec", 10, SCHED_OTHER, 0, 0});
    exec.add_runnable(std::make_shared<RunnableAdapter>([&] { ++count; }));

    REQUIRE(exec.start());
    std::this_thread::sleep_for(30ms);
    REQUIRE(exec.stop_with_timeout(500ms));
    const int first = count.load();

    REQUIRE(exec.restart());
    std::this_thread::sleep_for(30ms);
    REQUIRE(exec.stop_with_timeout(500ms));
    CHECK(count.load() > first);
}
