#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>
#include <cstring>
#include <rapidjson/document.h>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

using namespace robot::app;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::CommandPhase;
using robot::service::CommandTracker;
using robot::service::ConfigService;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;
using robot::service::SchedulerService;
namespace fs = std::filesystem;

namespace {

struct SupervisorFixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_robot_supervisor")};

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
    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    std::vector<FaultService::FaultEvent> fault_events;
    std::shared_ptr<CommandTracker> command_tracker{std::make_shared<CommandTracker>()};
    std::shared_ptr<RobotFsm> fsm;
    std::shared_ptr<RobotSupervisor> supervisor;
    std::shared_ptr<FaultHandler> fault_handler;

    SupervisorFixture()
        : motion(nullptr)
        , nav(std::make_shared<NavService>(group, imu, gps)) {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();

        MotionService::Config motion_cfg;
        motion_cfg.heading_pid_en = false;
        motion = std::make_shared<MotionService>(group, brush, nullptr, bus, motion_cfg);

        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "parking_side": "left",
    "min_battery_soc": 30.0,
    "charge_stop_soc": 95.0
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})",
                                            R"({})");

        REQUIRE(cfg.load());
        bus.subscribe<FaultService::FaultEvent>(
            [this](const FaultService::FaultEvent& evt) { fault_events.push_back(evt); });
        cfg.apply_active_runtime_schedules(scheduler);
        motion->set_parking_side_query(
            [this]() { return cfg.active_runtime_config().parking_side; });
        motion->set_runtime_config_query(
            [this]() { return cfg.active_runtime_config(); });
        fsm = std::make_shared<RobotFsm>(motion, fault, bus);
        fsm->dispatch(EvInitDone{});
        fault_handler = std::make_shared<FaultHandler>(
            motion,
            fault,
            bus,
            [this](const FaultService::FaultEvent& evt) {
                if (evt.level == FaultService::FaultEvent::Level::P0)
                    fsm->dispatch(EvFaultP0{});
                else if (evt.level == FaultService::FaultEvent::Level::P1)
                    fsm->dispatch(EvFaultP1{});
                else if (evt.level == FaultService::FaultEvent::Level::P2)
                    fsm->dispatch(EvFaultP2{});
            });
        fault_handler->start_listening();
        supervisor =
            std::make_shared<RobotSupervisor>(fsm, cfg, fault, nav);
    }

    ~SupervisorFixture() {
        tb_test_support::cleanup_split_config_paths(paths);
    }
};

rapidjson::Document parse_json(const char* text) {
    rapidjson::Document doc;
    doc.Parse(text);
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

void induce_spin_free(const SupervisorFixture& f) {
    REQUIRE(f.group->set_speed_uniform(50.0f) == robot::device::DeviceError::OK);
    f.group->update();
    for (int i = 0; i < 50; ++i) {
        f.nav->update();
    }
    REQUIRE(f.nav->get_pose().spin_free_detected);
}

}  // namespace

TEST_CASE("RobotSupervisor start requires parking side, valid position and enough battery",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_task(false, true, 60.0f));
    REQUIRE_FALSE(f.supervisor->start_task(true, false, 60.0f));
    REQUIRE_FALSE(f.supervisor->start_task(true, true, 20.0f));

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
}

TEST_CASE("RobotSupervisor rpc start can begin task away from parking side",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task_from_current_position(false, true, true, 60.0f));
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
    REQUIRE(f.fsm->current_segment_direction().has_value());
    REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToParkingSide);
}

TEST_CASE("RobotSupervisor dual-dock mode allows starting from either dock",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.cfg.set("robot.dual_dock_mode", true);

    SECTION("from configured parking side") {
        REQUIRE(f.supervisor->start_task_from_current_position(true, false, true, 60.0f));
        REQUIRE(f.fsm->current_state() == "ExecutingSegment");
        REQUIRE(f.fsm->current_segment_direction().has_value());
        REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToFarEnd);
    }

    SECTION("from opposite dock") {
        REQUIRE(f.supervisor->start_task_from_current_position(false, true, true, 60.0f));
        REQUIRE(f.fsm->current_state() == "ExecutingSegment");
        REQUIRE(f.fsm->current_segment_direction().has_value());
        REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToParkingSide);
    }
}

TEST_CASE("RobotSupervisor startup assessment requests return when robot is at no endpoint",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    const auto result = f.supervisor->handle_startup_position(false, false);
    REQUIRE(result.facts.no_endpoint_active);
    REQUIRE(result.should_request_return);
    REQUIRE(std::string(result.status_reason) == "robot_not_at_any_endpoint");
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
}

TEST_CASE("RobotSupervisor startup assessment requests return at far end in single-dock mode",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    const auto result = f.supervisor->handle_startup_position(false, true);
    REQUIRE(result.facts.at_far_end);
    REQUIRE(result.should_request_return);
    REQUIRE(std::string(result.status_reason) == "robot_not_at_parking_side");
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
}

TEST_CASE("RobotSupervisor rpc start from no-endpoint uses formal round-trip task",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task_from_current_position(false, false, true, 60.0f));
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
    REQUIRE(f.fsm->current_segment_direction().has_value());
    REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToFarEnd);
}

TEST_CASE("RobotSupervisor scheduler entry uses start parking side facts", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->handle_scheduler_window_hit(true, false, 60.0f));
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
}

TEST_CASE("RobotSupervisor scheduler rejection reports configured fault code",
          "[app][robot_supervisor]") {
    SupervisorFixture f;

    REQUIRE_FALSE(f.supervisor->handle_scheduler_window_hit(true, false, 20.0f));

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P2);
    REQUIRE(f.fault_events[0].code == robot::service::FaultCode::kStartRejectedLowBattery);
    REQUIRE(f.supervisor->snapshot().fault == robot::service::FaultCode::kStartRejectedLowBattery);
}

TEST_CASE("RobotSupervisor scheduler entry supports dual-dock start from opposite dock",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.cfg.set("robot.dual_dock_mode", true);

    REQUIRE(f.supervisor->handle_scheduler_window_hit(false, true, 60.0f));
    REQUIRE(f.fsm->current_state() == "ExecutingSegment");
    REQUIRE(f.fsm->current_segment_direction().has_value());
    REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToParkingSide);
}

TEST_CASE("RobotSupervisor reports P0 when settled limit is opposite to current segment",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.fsm->current_segment_direction().has_value());
    REQUIRE(*f.fsm->current_segment_direction() == SegmentDirection::ToFarEnd);

    f.supervisor->handle_limit_settled(robot::domain::PhysicalLimitSide::Left, 60.0f);

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P0);
    REQUIRE(f.fault_events[0].code == robot::service::FaultCode::kUnexpectedLimitSide);
    REQUIRE(f.fsm->current_state() == "FaultStopped");
}

TEST_CASE("RobotSupervisor reports P0 when both settled limits are active",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task(true, true, 60.0f));

    f.supervisor->handle_limit_settled(
        robot::domain::PhysicalLimitSide::Right, true, true, 60.0f);

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P0);
    REQUIRE(f.fault_events[0].code == robot::service::FaultCode::kConflictingLimitSides);
    REQUIRE(f.fsm->current_state() == "FaultStopped");
}

TEST_CASE("RobotSupervisor reports P0 when limit emergency stop is not followed by stable settle",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->register_limit_settled_bridge(
        f.bus,
        []() { return std::pair<bool, bool>{false, false}; },
        []() { return 60.0f; });

    f.bus.publish(robot::middleware::SafetyMonitor::LimitUnstableEvent{
        robot::domain::PhysicalLimitSide::Left});

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P0);
    REQUIRE(f.fault_events[0].code ==
            robot::service::FaultCode::kLimitUnstableAfterEmergencyStop);
    REQUIRE(f.fsm->current_state() == "FaultStopped");
}

TEST_CASE("RobotSupervisor promotes pending config before task start", "[app][robot_supervisor]") {
    SupervisorFixture f;
    auto attrs = parse_json(R"({"passes":3.0})");
    REQUIRE(f.cfg.apply_runtime_patch(attrs).accepted);
    REQUIRE(f.cfg.has_pending_runtime_config());

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.cfg.active_runtime_config().passes == Approx(3.0));
}

TEST_CASE("RobotSupervisor start uses promoted runtime speed config", "[app][robot_supervisor]") {
    SupervisorFixture f;
    auto attrs = parse_json(R"({"clean_speed_rpm":360.0,"brush_rpm":1500})");
    REQUIRE(f.cfg.apply_runtime_patch(attrs).accepted);
    REQUIRE(f.cfg.has_pending_runtime_config());

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.cfg.active_runtime_config().clean_speed_rpm == Approx(360.0));
    f.motion->update();
    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(210.0f));
}

TEST_CASE("RobotSupervisor can start from Charging and Stopped when battery is high",
          "[app][robot_supervisor]") {
    SECTION("from Charging") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvFarEndLimitSettled{});
        f.fsm->dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm->current_state() == "Charging");

        REQUIRE(f.supervisor->start_task(true, true, 60.0f));
        REQUIRE(f.fsm->current_state() == "ExecutingSegment");
    }
}

TEST_CASE("RobotSupervisor stop only works from executing state", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->stop_task());

    f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.supervisor->stop_task());
    REQUIRE(f.fsm->current_state() == "Idle");
}

TEST_CASE("RobotSupervisor stop clears stale spin-free detection", "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
    induce_spin_free(f);

    REQUIRE(f.supervisor->stop_task());
    REQUIRE(f.fsm->current_state() == "Idle");
    REQUIRE_FALSE(f.nav->get_pose().spin_free_detected);
}

TEST_CASE("RobotSupervisor return works from idle and executing states when away from parking side",
          "[app][robot_supervisor]") {
    SECTION("from Idle") {
        SupervisorFixture f;
        REQUIRE(f.supervisor->return_task(false));
        REQUIRE(f.fsm->current_state() == "ExecutingSegment");
    }

    SECTION("reject when already at parking side") {
        SupervisorFixture f;
        REQUIRE_FALSE(f.supervisor->return_task(true));
        REQUIRE(f.fsm->current_state() == "Idle");
    }
}

TEST_CASE("RobotSupervisor tick_safety reports P0 on spin-free detection", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.group->set_speed_uniform(50.0f) == robot::device::DeviceError::OK);
    f.group->update();
    for (int i = 0; i < 50; ++i) {
        f.nav->update();
    }
    REQUIRE(f.nav->get_pose().spin_free_detected);

    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.supervisor->tick_safety();

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].code == 0x0002);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P0);
    REQUIRE_FALSE(f.nav->get_pose().spin_free_detected);
}

TEST_CASE("RobotSupervisor tick_safety ignores spin-free detection outside moving states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.group->set_speed_uniform(50.0f) == robot::device::DeviceError::OK);
    f.group->update();
    for (int i = 0; i < 50; ++i) {
        f.nav->update();
    }
    REQUIRE(f.nav->get_pose().spin_free_detected);

    SECTION("Idle") {
        REQUIRE(f.fsm->current_state() == "Idle");
        f.supervisor->tick_safety();
        REQUIRE(f.fault_events.empty());
        REQUIRE(f.nav->get_pose().spin_free_detected);
    }

    SECTION("Charging") {
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvFarEndLimitSettled{});
        f.fsm->dispatch(EvParkingSideLimitSettled{true});
        REQUIRE(f.fsm->current_state() == "Charging");
        f.supervisor->tick_safety();
        REQUIRE(f.fault_events.empty());
        REQUIRE(f.nav->get_pose().spin_free_detected);
    }
}

TEST_CASE("RobotSupervisor snapshot reflects runtime state and command visibility",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(EvFarEndLimitSettled{});

    f.fault->report(FaultService::FaultEvent::Level::P1, 0x2001, "brush_fault");

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.state == "ExecutingSegment");
    REQUIRE(snap.fault == 0x2001);
    REQUIRE(snap.target_passes == 2);
    REQUIRE(snap.completed_passes == 0);
    REQUIRE(snap.active_config.has_value());
    REQUIRE(snap.cfg_ver != 0);
}

TEST_CASE("RobotSupervisor snapshot exposes promoted P2 as active fault",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fault->report(FaultService::FaultEvent::Level::P2, 0x3002, "gps_lost_requires_return");

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.fault == 0x3002u);
}
