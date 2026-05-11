#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

using namespace robot::app;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;

namespace {

MotionService::Config make_motion_config() {
    MotionService::Config cfg;
    cfg.heading_pid_en = false;
    return cfg;
}

EvScheduleStart start_from_parking_side(float passes = 1.0f) {
    EvScheduleStart evt;
    evt.at_parking_side = true;
    evt.passes = passes;
    return evt;
}

struct FsmFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{
        std::make_shared<BrushMotor>(brush_serial, 0, 8192.0f, true, 0.5f)};
    std::shared_ptr<MockSerialPort> imu_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<ImuDevice> imu{std::make_shared<ImuDevice>(imu_serial)};
    std::shared_ptr<MockSerialPort> gps_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<GpsDevice> gps{std::make_shared<GpsDevice>(gps_serial)};
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault{std::make_shared<FaultService>(bus)};
    RobotFsm fsm;

    FsmFixture()
        : motion(std::make_shared<MotionService>(group, brush, nullptr, bus, make_motion_config()))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , fsm(motion, nav, fault, bus) {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();
        fsm.dispatch(EvInitDone{});
    }
};

}  // namespace

TEST_CASE("FSM initializes to Idle", "[app][fsm]") {
    FsmFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM starts new task only from parking side with integer passes", "[app][fsm]") {
    FsmFixture f;

    SECTION("valid start") {
        f.fsm.dispatch(start_from_parking_side(1.0f));
        REQUIRE(f.fsm.current_state() == "CleanFwd");
    }

    SECTION("reject non-integer passes") {
        f.fsm.dispatch(start_from_parking_side(0.5f));
        REQUIRE(f.fsm.current_state() == "Idle");
    }

    SECTION("reject non-parking-side start") {
        EvScheduleStart evt;
        evt.at_parking_side = false;
        evt.passes = 1.0f;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "Idle");
    }
}

TEST_CASE("FSM RPC override start can begin cleaning away from parking side", "[app][fsm]") {
    FsmFixture f;
    f.brush_serial->clear_tx();

    EvRpcStartTask evt;
    evt.passes = 1.0f;
    f.fsm.dispatch(evt);
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.brush_serial->clear_tx();
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    REQUIRE(f.brush_serial->take_tx_text().find("v 0 -20.000 0\n") != std::string::npos);

    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "Charging");
    REQUIRE(f.fsm.completed_passes() == 1);
}

TEST_CASE("FSM round trip transitions follow charge decision", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    SECTION("task completes into Charging") {
        f.fsm.dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm.current_state() == "Charging");
    }

    SECTION("task completes into Idle") {
        f.fsm.dispatch(EvParkingSideLimitSettled{false});
        REQUIRE(f.fsm.current_state() == "Idle");
    }
}

TEST_CASE("FSM multi-pass task loops back to CleanFwd before final completion", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE(f.fsm.completed_passes() == 1);
}

TEST_CASE("FSM manual return finishes at parking side without resuming cleaning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fsm.dispatch(EvManualReturn{});
    REQUIRE(f.fsm.current_state() == "Returning");

    f.fsm.dispatch(EvParkingSideLimitSettled{false});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM stop terminates active task into Stopped", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvStopTask{});
    REQUIRE(f.fsm.current_state() == "Stopped");
    REQUIRE(f.fsm.target_passes() == 0);
    REQUIRE(f.fsm.completed_passes() == 0);
}

TEST_CASE("FSM can return from Idle or Stopped to parking side", "[app][fsm]") {
    SECTION("from Idle") {
        FsmFixture f;
        f.fsm.dispatch(EvManualReturn{});
        REQUIRE(f.fsm.current_state() == "Returning");
    }

    SECTION("from Stopped") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvStopTask{});
        REQUIRE(f.fsm.current_state() == "Stopped");
        f.fsm.dispatch(EvManualReturn{});
        REQUIRE(f.fsm.current_state() == "Returning");
    }
}

TEST_CASE("FSM charge done returns Charging to Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "Charging");

    f.fsm.dispatch(EvChargeDone{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM fault handling uses Fault state", "[app][fsm]") {
    SECTION("P0 transitions to Fault") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP0{});
        REQUIRE(f.fsm.current_state() == "Fault");
    }

    SECTION("P1 transitions to Fault") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP1{});
        REQUIRE(f.fsm.current_state() == "Fault");
    }

    SECTION("P2 keeps current state") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP2{});
        REQUIRE(f.fsm.current_state() == "CleanFwd");
    }

    SECTION("fault reset returns to Idle") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP0{});
        REQUIRE(f.fsm.current_state() == "Fault");
        f.fsm.dispatch(EvFaultReset{});
        REQUIRE(f.fsm.current_state() == "Idle");
    }
}
