/**
 * EventBus 中间件单元测试
 * [middleware][event_bus]
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/middleware/event_bus.h"

using robot::middleware::EventBus;

// ────────────────────────────────────────────────────────────────
// 测试用事件类型
// ────────────────────────────────────────────────────────────────
struct EvIntValue {
    int val;
};
struct EvStringMsg {
    std::string msg;
};
struct EvEmpty {};

// ────────────────────────────────────────────────────────────────
// 基础 pub/sub
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: 订阅后 publish 触发回调", "[middleware][event_bus]") {
    EventBus bus;
    int received = -1;
    bus.subscribe<EvIntValue>([&](const EvIntValue& e) { received = e.val; });
    bus.publish(EvIntValue{42});
    REQUIRE(received == 42);
}

TEST_CASE("EventBus: 多个订阅者都能收到同一事件", "[middleware][event_bus]") {
    EventBus bus;
    int a = 0, b = 0;
    bus.subscribe<EvIntValue>([&](const EvIntValue& e) { a = e.val; });
    bus.subscribe<EvIntValue>([&](const EvIntValue& e) { b = e.val * 2; });
    bus.publish(EvIntValue{5});
    REQUIRE(a == 5);
    REQUIRE(b == 10);
}

TEST_CASE("EventBus: 不同类型事件互不干扰", "[middleware][event_bus]") {
    EventBus bus;
    int int_recv = 0;
    std::string str_recv;

    bus.subscribe<EvIntValue>([&](const EvIntValue& e) { int_recv = e.val; });
    bus.subscribe<EvStringMsg>([&](const EvStringMsg& e) { str_recv = e.msg; });

    bus.publish(EvIntValue{99});
    bus.publish(EvStringMsg{"hello"});

    REQUIRE(int_recv == 99);
    REQUIRE(str_recv == "hello");
}

TEST_CASE("EventBus: 无订阅者时 publish 不崩溃", "[middleware][event_bus]") {
    EventBus bus;
    REQUIRE_NOTHROW(bus.publish(EvEmpty{}));
    REQUIRE_NOTHROW(bus.publish(EvIntValue{0}));
}

// ────────────────────────────────────────────────────────────────
// unsubscribe()
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: unsubscribe() 后回调不再触发", "[middleware][event_bus]") {
    EventBus bus;
    int count = 0;
    int id = bus.subscribe<EvIntValue>([&](const EvIntValue&) { ++count; });

    bus.publish(EvIntValue{1});  // 触发一次
    bus.unsubscribe(id);
    bus.publish(EvIntValue{2});  // 不应触发

    REQUIRE(count == 1);
}

TEST_CASE("EventBus: 无效 unsubscribe id 不崩溃", "[middleware][event_bus]") {
    EventBus bus;
    REQUIRE_NOTHROW(bus.unsubscribe(9999));
}

TEST_CASE("EventBus: 取消一个订阅不影响其他订阅", "[middleware][event_bus]") {
    EventBus bus;
    int a = 0, b = 0;
    int id_a = bus.subscribe<EvIntValue>([&](const EvIntValue& e) { a = e.val; });
    bus.subscribe<EvIntValue>([&](const EvIntValue& e) { b = e.val; });

    bus.unsubscribe(id_a);
    bus.publish(EvIntValue{7});

    REQUIRE(a == 0);  // 已取消
    REQUIRE(b == 7);  // 仍有效
}

// ────────────────────────────────────────────────────────────────
// 事件值传递正确性
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: const 引用传递不拷贝", "[middleware][event_bus]") {
    EventBus bus;
    std::string received;
    bus.subscribe<EvStringMsg>([&](const EvStringMsg& e) { received = e.msg; });
    bus.publish(EvStringMsg{"test_string"});
    REQUIRE(received == "test_string");
}

// ────────────────────────────────────────────────────────────────
// 线程安全：多线程并发 publish
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: 多线程并发 publish 不崩溃", "[middleware][event_bus]") {
    EventBus bus;
    std::atomic<int> total{0};
    bus.subscribe<EvIntValue>(
        [&](const EvIntValue& e) { total.fetch_add(e.val, std::memory_order_relaxed); });

    constexpr int kThreads = 4;
    constexpr int kEventsEach = 25;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < kEventsEach; ++i) {
                bus.publish(EvIntValue{1});
            }
        });
    }
    for (auto& th : threads)
        th.join();

    REQUIRE(total.load() == kThreads * kEventsEach);
}

// ────────────────────────────────────────────────────────────────
// 订阅后立刻取消再 publish 不崩溃
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: 订阅后立即取消再 publish 无崩溃", "[middleware][event_bus]") {
    EventBus bus;
    int id = bus.subscribe<EvIntValue>([](const EvIntValue&) {});
    bus.unsubscribe(id);
    REQUIRE_NOTHROW(bus.publish(EvIntValue{1}));
}

// ────────────────────────────────────────────────────────────────
// kMaxHandlers 容量不超出
// ────────────────────────────────────────────────────────────────
TEST_CASE("EventBus: 可注册 kMaxHandlers 个订阅者", "[middleware][event_bus]") {
    EventBus bus;
    std::atomic<int> count{0};
    // 注册 32 个（kMaxHandlers=32）
    for (int i = 0; i < 32; ++i) {
        bus.subscribe<EvEmpty>([&](const EvEmpty&) { ++count; });
    }
    bus.publish(EvEmpty{});
    REQUIRE(count.load() == 32);
}
