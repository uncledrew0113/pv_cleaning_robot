#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/fault_handler.h"
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
        std::make_shared<BrushMotor>(brush_serial, 0)};
    std::shared_ptr<MockSerialPort> imu_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<ImuDevice> imu{std::make_shared<ImuDevice>(imu_serial)};
    std::shared_ptr<MockSerialPort> gps_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<GpsDevice> gps{std::make_shared<GpsDevice>(gps_serial)};
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault{std::make_shared<FaultService>(bus)};
    std::shared_ptr<FaultHandler> fault_handler;
    RobotFsm fsm;

    FsmFixture()
        : motion(std::make_shared<MotionService>(group, brush, nullptr, bus, make_motion_config()))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , fsm(motion, fault, bus) {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();
        fault_handler = std::make_shared<FaultHandler>(
            motion,
            fault,
            bus,
            [this](const FaultService::FaultEvent& evt) {
                if (evt.level == FaultService::FaultEvent::Level::P0)
                    fsm.dispatch(EvFaultP0{});
                else if (evt.level == FaultService::FaultEvent::Level::P1)
                    fsm.dispatch(EvFaultP1{});
                else if (evt.level == FaultService::FaultEvent::Level::P2)
                    fsm.dispatch(EvFaultP2{});
            });
        fault_handler->start_listening();
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
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
    }

    SECTION("reject non-integer passes") {
        f.fsm.dispatch(start_from_parking_side(0.5f));
        REQUIRE(f.fsm.current_state() == "Idle");
        REQUIRE(f.fault->has_active_fault());
        REQUIRE(f.fault->last_fault().code == robot::service::FaultCode::kSelfCheckFailed);
    }

    SECTION("reject non-parking-side start") {
        EvScheduleStart evt;
        evt.at_parking_side = false;
        evt.passes = 1.0f;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "Idle");
        REQUIRE(f.fault->has_active_fault());
        REQUIRE(f.fault->last_fault().code == robot::service::FaultCode::kSelfCheckFailed);
    }
}

TEST_CASE("FSM RPC override start uses return clean only when starting from far end", "[app][fsm]") {
    FsmFixture f;
    f.brush_serial->clear_tx();

    SECTION("far end start returns via single clean segment") {
        EvRpcStartTask evt;
        evt.passes = 1.0f;
        evt.at_parking_side = false;
        evt.at_far_end = true;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
        REQUIRE(f.fsm.current_segment_direction().has_value());
        REQUIRE(*f.fsm.current_segment_direction() == SegmentDirection::ToParkingSide);

        f.fsm.dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm.current_state() == "Charging");
        REQUIRE(f.fsm.completed_passes() == 1);
    }

    SECTION("no-endpoint start falls back to formal round-trip task") {
        EvRpcStartTask evt;
        evt.passes = 1.0f;
        evt.at_parking_side = false;
        evt.at_far_end = false;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
        REQUIRE(f.fsm.current_segment_direction().has_value());
        REQUIRE(*f.fsm.current_segment_direction() == SegmentDirection::ToFarEnd);
    }
}

TEST_CASE("FSM dual-dock start from either dock runs a single clean segment", "[app][fsm]") {
    SECTION("from configured parking side to far dock") {
        FsmFixture f;
        EvRpcStartTask evt;
        evt.passes = 1.0f;
        evt.at_parking_side = true;
        evt.at_far_end = false;
        evt.dual_dock_mode = true;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
        REQUIRE(f.fsm.current_segment_direction().has_value());
        REQUIRE(*f.fsm.current_segment_direction() == SegmentDirection::ToFarEnd);

        f.fsm.dispatch(EvFarEndLimitSettled{true});
        REQUIRE(f.fsm.current_state() == "Charging");
        REQUIRE(f.fsm.completed_passes() == 1);
    }

    SECTION("from opposite dock back to configured parking side") {
        FsmFixture f;
        EvRpcStartTask evt;
        evt.passes = 1.0f;
        evt.at_parking_side = false;
        evt.at_far_end = true;
        evt.dual_dock_mode = true;
        f.fsm.dispatch(evt);
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
        REQUIRE(f.fsm.current_segment_direction().has_value());
        REQUIRE(*f.fsm.current_segment_direction() == SegmentDirection::ToParkingSide);

        f.fsm.dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm.current_state() == "Charging");
        REQUIRE(f.fsm.completed_passes() == 1);
    }
}

TEST_CASE("FSM segment start failure transitions to FaultStopped and records active fault",
          "[app][fsm]") {
    FsmFixture f;
    f.can->send_result = false;

    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "FaultStopped");
    REQUIRE(f.fault->has_active_fault());
    REQUIRE(f.fault->last_fault().code != 0u);
}

TEST_CASE("FSM fault reset clears active fault", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFaultP0{});
    REQUIRE(f.fsm.current_state() == "FaultStopped");
    REQUIRE(f.fault->has_active_fault());

    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");
    REQUIRE_FALSE(f.fault->has_active_fault());
}

TEST_CASE("FSM successful P1 return clears active fault at dock completion", "[app][fsm]") {
    FsmFixture f;
    f.fault->report(FaultService::FaultEvent::Level::P1, 0x2001, "brush_fault");
    f.fsm.dispatch(EvManualReturn{});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");
    REQUIRE(f.fault->has_active_fault());

    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "Charging");
    REQUIRE_FALSE(f.fault->has_active_fault());
}

TEST_CASE("FSM round trip transitions follow charge decision", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");

    SECTION("task completes into Charging") {
        f.fsm.dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm.current_state() == "Charging");
    }

    SECTION("task completes into Idle") {
        f.fsm.dispatch(EvParkingSideLimitSettled{false});
        REQUIRE(f.fsm.current_state() == "Idle");
    }
}

TEST_CASE("FSM multi-pass task loops back to ExecutingSegment before final completion", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");
    REQUIRE(f.fsm.completed_passes() == 1);
}

TEST_CASE("FSM manual return finishes at parking side without resuming cleaning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");

    f.fsm.dispatch(EvManualReturn{});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");

    f.fsm.dispatch(EvParkingSideLimitSettled{false});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM stop terminates active task into Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvStopTask{});
    REQUIRE(f.fsm.current_state() == "Idle");
    REQUIRE(f.fsm.target_passes() == 0);
    REQUIRE(f.fsm.completed_passes() == 0);
}

TEST_CASE("FSM can return from Idle to parking side", "[app][fsm]") {
    SECTION("from Idle") {
        FsmFixture f;
        f.fsm.dispatch(EvManualReturn{});
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
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

TEST_CASE("FSM fault handling follows new fault-state semantics", "[app][fsm]") {
    SECTION("P0 transitions to FaultStopped") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP0{});
        REQUIRE(f.fsm.current_state() == "FaultStopped");
    }

    SECTION("P1 switches to return segment") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP1{});
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
        REQUIRE(f.fsm.current_segment_direction().has_value());
        REQUIRE(*f.fsm.current_segment_direction() == SegmentDirection::ToParkingSide);
    }

    SECTION("P1 during return escalates to FaultStopped") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP1{});
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");

        f.fsm.dispatch(EvFaultP1{});
        REQUIRE(f.fsm.current_state() == "FaultStopped");
        REQUIRE(f.fault->has_active_fault());
        REQUIRE(f.fault->last_fault().level == FaultService::FaultEvent::Level::P0);
    }

    SECTION("P2 keeps current state") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP2{});
        REQUIRE(f.fsm.current_state() == "ExecutingSegment");
    }

    SECTION("fault reset returns to Idle") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvFaultP0{});
        REQUIRE(f.fsm.current_state() == "FaultStopped");
        f.fsm.dispatch(EvFaultReset{});
        REQUIRE(f.fsm.current_state() == "Idle");
    }
}
