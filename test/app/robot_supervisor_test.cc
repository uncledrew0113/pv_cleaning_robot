#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>
#include <cstring>
#include <rapidjson/document.h>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
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
using robot::service::ThingsBoardConfigManager;
namespace fs = std::filesystem;

namespace {

struct SupervisorFixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_robot_supervisor")};

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
    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    std::vector<FaultService::FaultEvent> fault_events;
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<CommandTracker> command_tracker{std::make_shared<CommandTracker>()};
    std::shared_ptr<RobotFsm> fsm;
    std::shared_ptr<RobotSupervisor> supervisor;

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
    "start_battery_soc": 30.0,
    "charge_start_soc": 15.0,
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
        tb_cfg = std::make_shared<ThingsBoardConfigManager>(cfg, scheduler);
        motion->set_parking_side_provider(
            [this]() { return tb_cfg->active_config().parking_side; });
        motion->set_runtime_config_provider(
            [this]() { return tb_cfg->active_config(); });
        fsm = std::make_shared<RobotFsm>(motion, nav, fault, bus);
        fsm->dispatch(EvInitDone{});
        supervisor =
            std::make_shared<RobotSupervisor>(fsm, tb_cfg, command_tracker, fault, nav);
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

}  // namespace

TEST_CASE("RobotSupervisor start requires parking side, valid position and enough battery",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_task(false, true, 60.0f));
    REQUIRE_FALSE(f.supervisor->start_task(true, false, 60.0f));
    REQUIRE_FALSE(f.supervisor->start_task(true, true, 20.0f));

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.fsm->current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor rpc start can begin task away from parking side",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task_from_current_position(true, 60.0f));
    REQUIRE(f.fsm->current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor promotes pending config before task start", "[app][robot_supervisor]") {
    SupervisorFixture f;
    auto attrs = parse_json(R"({"passes":3.0})");
    REQUIRE(f.tb_cfg->apply_shared_attributes(attrs).accepted);
    REQUIRE(f.tb_cfg->has_pending_config());

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.tb_cfg->active_config().passes == Approx(3.0));
}

TEST_CASE("RobotSupervisor start uses promoted runtime speed config", "[app][robot_supervisor]") {
    SupervisorFixture f;
    auto attrs = parse_json(
        R"({"clean_speed_rpm":360.0,"brush_rpm":1500,"return_brush_rpm":900})");
    REQUIRE(f.tb_cfg->apply_shared_attributes(attrs).accepted);
    REQUIRE(f.tb_cfg->has_pending_config());

    REQUIRE(f.supervisor->start_task(true, true, 60.0f));
    REQUIRE(f.tb_cfg->active_config().clean_speed_rpm == Approx(360.0));
    REQUIRE(f.tb_cfg->active_config().return_brush_rpm == 900);
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
        REQUIRE(f.fsm->current_state() == "CleanFwd");
    }

    SECTION("from Stopped") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvStopTask{});
        REQUIRE(f.fsm->current_state() == "Stopped");

        REQUIRE(f.supervisor->start_task(true, true, 60.0f));
        REQUIRE(f.fsm->current_state() == "CleanFwd");
    }
}

TEST_CASE("RobotSupervisor stop only works from running states", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->stop_task());

    f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.supervisor->stop_task());
    REQUIRE(f.fsm->current_state() == "Stopped");
}

TEST_CASE("RobotSupervisor return works from idle, stopped, and cleaning when away from parking side",
          "[app][robot_supervisor]") {
    SECTION("from Idle") {
        SupervisorFixture f;
        REQUIRE(f.supervisor->return_task(false));
        REQUIRE(f.fsm->current_state() == "Returning");
    }

    SECTION("from Stopped") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvStopTask{});
        REQUIRE(f.supervisor->return_task(false));
        REQUIRE(f.fsm->current_state() == "Returning");
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

TEST_CASE("RobotSupervisor snapshot reflects runtime state and command visibility",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(EvFarEndLimitSettled{});

    auto accepted_id = f.command_tracker->accept("start", "req-1");
    f.command_tracker->mark_running(accepted_id);
    f.command_tracker->finish_success(accepted_id, "started_new_task");

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.device_state == "CleanReturn");
    REQUIRE(snap.task_state == "RunningTask");
    REQUIRE(snap.target_passes == 2);
    REQUIRE(snap.completed_passes == 0);
    REQUIRE(snap.active_config.has_value());
    REQUIRE(snap.active_config_version != 0);
    REQUIRE(snap.last_command.has_value());
    REQUIRE(snap.last_command->phase == CommandPhase::Succeeded);
}
