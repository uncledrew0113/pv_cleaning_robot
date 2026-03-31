/**
 * FaultService 单元测试
 * [service][fault]
 */
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/fault_service.h"

using robot::service::FaultService;
using FaultEvent = robot::service::FaultService::FaultEvent;
using FaultLevel = robot::service::FaultService::FaultEvent::Level;
using robot::middleware::EventBus;

TEST_CASE("FaultService: 初始状态无活跃故障", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);
    REQUIRE_FALSE(fs.has_active_fault());
    REQUIRE(fs.last_fault().code == 0);
}

// ────────────────────────────────────────────────────────────────
// report() → 发布到 EventBus
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultService: report() 发布 FaultEvent 到 EventBus", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);

    FaultEvent received{};
    bus.subscribe<FaultEvent>([&](const FaultEvent& e) { received = e; });

    fs.report(FaultLevel::P0, 0x1001, "CAN lost");

    REQUIRE(received.level == FaultLevel::P0);
    REQUIRE(received.code == 0x1001);
    REQUIRE(received.description == "CAN lost");
}

// ────────────────────────────────────────────────────────────────
// has_active_fault()
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultService: P0 故障后 has_active_fault(P2) == true", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);
    fs.report(FaultLevel::P0, 1, "P0 test");
    REQUIRE(fs.has_active_fault(FaultLevel::P2));
    REQUIRE(fs.has_active_fault(FaultLevel::P0));
}

TEST_CASE("FaultService: P3 故障后 has_active_fault(P2) == false", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);
    fs.report(FaultLevel::P3, 2, "P3 minor");
    REQUIRE_FALSE(fs.has_active_fault(FaultLevel::P2));
    REQUIRE(fs.has_active_fault(FaultLevel::P3));
}

TEST_CASE("FaultService: P2 故障 has_active_fault(P2) == true", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);
    fs.report(FaultLevel::P2, 3, "P2 warn");
    REQUIRE(fs.has_active_fault(FaultLevel::P2));
    REQUIRE_FALSE(fs.has_active_fault(FaultLevel::P1));
    REQUIRE_FALSE(fs.has_active_fault(FaultLevel::P0));
}

// ────────────────────────────────────────────────────────────────
// last_fault() 缓存最新故障
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultService: last_fault() 返回最近一次故障", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);

    fs.report(FaultLevel::P1, 0x0001, "first");
    fs.report(FaultLevel::P0, 0x0002, "second");

    REQUIRE(fs.last_fault().code == 0x0002);
    REQUIRE(fs.last_fault().level == FaultLevel::P0);
    REQUIRE(fs.last_fault().description == "second");
}

// ────────────────────────────────────────────────────────────────
// timestamp 字段非零
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultService: report() 打上非零时间戳", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);
    fs.report(FaultLevel::P0, 1, "ts test");
    REQUIRE(fs.last_fault().timestamp_ms > 0);
}

// ────────────────────────────────────────────────────────────────
// 多个 EventBus 订阅者都收到
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultService: 多个订阅者都能收到 FaultEvent", "[service][fault]") {
    EventBus bus;
    FaultService fs(bus);

    int count1 = 0, count2 = 0;
    bus.subscribe<FaultEvent>([&](const FaultEvent&) { ++count1; });
    bus.subscribe<FaultEvent>([&](const FaultEvent&) { ++count2; });

    fs.report(FaultLevel::P2, 5, "multi");
    REQUIRE(count1 == 1);
    REQUIRE(count2 == 1);
}
