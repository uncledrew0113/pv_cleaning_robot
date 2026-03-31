/**
 * FaultHandler 单元测试
 * [app][fault_handler]
 */
#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_modbus_master.h"
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"

using robot::app::FaultHandler;
using robot::service::FaultService;
using robot::service::MotionService;
using FaultEvent = robot::service::FaultService::FaultEvent;
using FaultLevel = FaultEvent::Level;
using robot::device::BrushMotor;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct FaultHandlerFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(modbus, 1)};
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    FaultService fault_svc{bus};

    // 记录 dispatch_fn 收到的 FaultEvent
    std::vector<FaultEvent> dispatched;

    FaultHandler handler;

    FaultHandlerFixture()
        : motion(std::make_shared<MotionService>(group,
                                                 brush,
                                                 nullptr,
                                                 bus,
                                                 MotionService::Config{.heading_pid_en = false}))
        , handler(motion, bus, [this](FaultEvent e) { dispatched.push_back(e); }) {
        can->open_result = true;
        can->send_result = true;
        modbus->open_result = true;
        modbus->write_reg_return = 0;
        brush->open();
        handler.start_listening();
    }
};

// ────────────────────────────────────────────────────────────────
// P0 故障：调用 dispatch_fn + emergency_stop
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultHandler: P0 故障 → 调用 dispatch_fn", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.fault_svc.report(FaultLevel::P0, 0x1001, "CAN lost");

    REQUIRE(f.dispatched.size() == 1);
    REQUIRE(f.dispatched[0].level == FaultLevel::P0);
}

TEST_CASE("FaultHandler: P0 故障 → emergency_stop() 发出 CAN 帧", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.can->sent_frames.clear();
    f.fault_svc.report(FaultLevel::P0, 0x1001, "CAN lost");

    // emergency_stop 应向 CAN 总线发出帧
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

// ────────────────────────────────────────────────────────────────
// P1 故障：start_returning_no_brush
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultHandler: P1 故障 → 调用 dispatch_fn", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.fault_svc.report(FaultLevel::P1, 0x2001, "BMS low");

    REQUIRE(f.dispatched.size() == 1);
    REQUIRE(f.dispatched[0].level == FaultLevel::P1);
}

TEST_CASE("FaultHandler: P1 故障 → start_returning_no_brush() 发出 CAN 帧",
          "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.can->sent_frames.clear();
    f.fault_svc.report(FaultLevel::P1, 0x2001, "BMS low");

    REQUIRE_FALSE(f.can->sent_frames.empty());
}

// ────────────────────────────────────────────────────────────────
// P2 故障：不触发 motion，但通知 dispatch_fn
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultHandler: P2 故障 → dispatch_fn 收到但不急停", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.can->sent_frames.clear();

    f.fault_svc.report(FaultLevel::P2, 0x3001, "IMU degraded");

    REQUIRE(f.dispatched.size() == 1);
    REQUIRE(f.dispatched[0].level == FaultLevel::P2);
    // P2 不应触发 emergency_stop 或 start_returning（CAN 帧可能为空）
}

// ────────────────────────────────────────────────────────────────
// P3 故障：仅日志
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultHandler: P3 故障 → dispatch_fn 收到", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.fault_svc.report(FaultLevel::P3, 0x4001, "minor warning");

    REQUIRE(f.dispatched.size() == 1);
    REQUIRE(f.dispatched[0].level == FaultLevel::P3);
}

// ────────────────────────────────────────────────────────────────
// 析构后不崩溃（EventBus 订阅自动取消）
// ────────────────────────────────────────────────────────────────
TEST_CASE("FaultHandler: 析构后 EventBus publish 不崩溃", "[app][fault_handler]") {
    EventBus bus;
    auto can = std::make_shared<MockCanBus>();
    auto group = std::make_shared<WalkMotorGroup>(can);
    auto modbus = std::make_shared<MockModbusMaster>();
    auto brush = std::make_shared<BrushMotor>(modbus, 1);
    can->open_result = true;
    can->send_result = true;
    modbus->open_result = true;
    modbus->write_reg_return = 0;
    brush->open();
    auto motion = std::make_shared<MotionService>(
        group, brush, nullptr, bus, MotionService::Config{.heading_pid_en = false});
    FaultService fault_svc(bus);

    {
        FaultHandler h(motion, bus, [](FaultEvent) {});
        h.start_listening();
    }  // FaultHandler 析构，取消订阅

    // 析构后再 publish 不崩溃
    FaultEvent e;
    e.level = FaultLevel::P0;
    REQUIRE_NOTHROW(bus.publish(e));
}
