/**
 * WatchdogMgr 单元测试
 * [app][watchdog]
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <string>
#include <thread>

#include "pv_cleaning_robot/app/watchdog_mgr.h"

using robot::app::WatchdogMgr;

// ────────────────────────────────────────────────────────────────
// start() / stop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("WatchdogMgr: start() 和 stop() 不崩溃", "[app][watchdog]") {
    WatchdogMgr wdg;
    REQUIRE(wdg.start());
    wdg.stop();
}

TEST_CASE("WatchdogMgr: 重复 stop() 不崩溃", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();
    wdg.stop();
    REQUIRE_NOTHROW(wdg.stop());
}

// ────────────────────────────────────────────────────────────────
// register_thread() + heartbeat()
// ────────────────────────────────────────────────────────────────
TEST_CASE("WatchdogMgr: register_thread() 返回非负 ticket_id", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();
    int id = wdg.register_thread("test_thread", 1000);
    REQUIRE(id >= 0);
    wdg.stop();
}

TEST_CASE("WatchdogMgr: 多线程注册 ticket_id 唯一", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();
    int id1 = wdg.register_thread("t1", 1000);
    int id2 = wdg.register_thread("t2", 1000);
    REQUIRE(id1 != id2);
    wdg.stop();
}

TEST_CASE("WatchdogMgr: heartbeat() 在超时内不触发回调", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();
    std::atomic<bool> timed_out{false};
    wdg.set_timeout_callback([&](const std::string&) { timed_out = true; });

    int id = wdg.register_thread("keep_alive", 500);  // 500ms 超时

    // 在 200ms 内持续喂狗
    for (int i = 0; i < 4; ++i) {
        wdg.heartbeat(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    REQUIRE_FALSE(timed_out.load());
    wdg.stop();
}

// ────────────────────────────────────────────────────────────────
// 超时检测
// ────────────────────────────────────────────────────────────────
TEST_CASE("WatchdogMgr: 超时后触发 on_timeout 回调", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();

    std::string timed_name;
    wdg.set_timeout_callback([&](const std::string& name) { timed_name = name; });

    wdg.register_thread("slow_thread", 100);  // 100ms 超时

    // 等待超时（不喂狗）
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    REQUIRE(timed_name == "slow_thread");
    wdg.stop();
}

TEST_CASE("WatchdogMgr: 喂狗后重置超时计时器", "[app][watchdog]") {
    WatchdogMgr wdg;
    wdg.start();

    int timeout_count = 0;
    wdg.set_timeout_callback([&](const std::string&) { ++timeout_count; });

    int id = wdg.register_thread("keep_alive", 200);  // 200ms 超时

    // 在 150ms 时喂狗，复位计时器
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    wdg.heartbeat(id);
    // 继续等 150ms（距上次喂狗未超时）
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    REQUIRE(timeout_count == 0);
    wdg.stop();
}

// ────────────────────────────────────────────────────────────────
// 硬件看门狗路径（空路径 = 不启用）
// ────────────────────────────────────────────────────────────────
TEST_CASE("WatchdogMgr: 空 hw_watchdog_path 不崩溃", "[app][watchdog]") {
    WatchdogMgr wdg("");  // 空路径 = 不启用硬件看门狗
    REQUIRE(wdg.start());
    wdg.stop();
}

// ────────────────────────────────────────────────────────────────
// 析构时自动 stop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("WatchdogMgr: 析构时自动停止监控线程不崩溃", "[app][watchdog]") {
    REQUIRE_NOTHROW([]() {
        WatchdogMgr wdg;
        wdg.start();
        wdg.register_thread("auto_stop", 1000);
        // 不调用 stop()，期望析构函数正常处理
    }());
}
