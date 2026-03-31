/**
 * SchedulerService 单元测试
 * [service][scheduler]
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <ctime>

#include "pv_cleaning_robot/service/scheduler_service.h"

using robot::service::SchedulerService;

// ────────────────────────────────────────────────────────────────
// 辅助：获取当前本地时间（hour, minute）
// ────────────────────────────────────────────────────────────────
static std::pair<int, int> local_now() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* lm = std::localtime(&t);
    return {lm->tm_hour, lm->tm_min};
}

// ────────────────────────────────────────────────────────────────
// add_window() / clear_windows()
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: add_window + clear_windows 不崩溃", "[service][scheduler]") {
    SchedulerService svc;
    svc.add_window({8, 0});
    svc.add_window({14, 30});
    REQUIRE_NOTHROW(svc.clear_windows());
}

// ────────────────────────────────────────────────────────────────
// 窗口时间命中：tick() 触发 on_task_start
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: 当前时间匹配窗口时 tick() 触发 on_task_start",
          "[service][scheduler]") {
    SchedulerService svc;
    auto [h, m] = local_now();
    svc.add_window({h, m});  // 加入"现在"这一分钟

    bool started = false;
    svc.set_on_task_start([&] { started = true; });

    svc.tick();
    REQUIRE(started);
}

TEST_CASE("SchedulerService: 时间不匹配时 tick() 不触发 on_task_start", "[service][scheduler]") {
    SchedulerService svc;
    // 加入"明天此刻"的小时 + 1（不可能匹配）
    auto [h, m] = local_now();
    int future_h = (h + 1) % 24;
    svc.add_window({future_h, m});

    bool started = false;
    svc.set_on_task_start([&] { started = true; });

    svc.tick();
    REQUIRE_FALSE(started);
}

// ────────────────────────────────────────────────────────────────
// 窗口期内 tick() 不重复触发
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: 窗口期内连续 tick() 只触发一次 on_task_start",
          "[service][scheduler]") {
    SchedulerService svc;
    auto [h, m] = local_now();
    svc.add_window({h, m});

    int count = 0;
    svc.set_on_task_start([&] { ++count; });

    for (int i = 0; i < 10; ++i)
        svc.tick();
    REQUIRE(count == 1);
}

// ────────────────────────────────────────────────────────────────
// clear_windows() 后不再触发
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: clear_windows() 后 tick() 不触发", "[service][scheduler]") {
    SchedulerService svc;
    auto [h, m] = local_now();
    svc.add_window({h, m});
    svc.clear_windows();

    bool started = false;
    svc.set_on_task_start([&] { started = true; });

    svc.tick();
    REQUIRE_FALSE(started);
}

// ────────────────────────────────────────────────────────────────
// 无回调时 tick() 不崩溃
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: 无回调时 tick() 不崩溃", "[service][scheduler]") {
    SchedulerService svc;
    auto [h, m] = local_now();
    svc.add_window({h, m});
    REQUIRE_NOTHROW(svc.tick());
}

// ────────────────────────────────────────────────────────────────
// 多个窗口：任一命中均触发
// ────────────────────────────────────────────────────────────────
TEST_CASE("SchedulerService: 多个窗口中有一个匹配即触发", "[service][scheduler]") {
    SchedulerService svc;
    auto [h, m] = local_now();
    svc.add_window({(h + 1) % 24, m});  // 不匹配
    svc.add_window({h, m});             // 匹配

    bool started = false;
    svc.set_on_task_start([&] { started = true; });

    svc.tick();
    REQUIRE(started);
}
