#include <catch2/catch.hpp>

#include <atomic>
#include <chrono>
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
