#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <memory>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

using namespace robot::app;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::ConfigService;
using robot::service::CommandPhase;
using robot::service::CommandTracker;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;
using robot::service::SchedulerService;
using robot::service::ThingsBoardConfigManager;
namespace fs = std::filesystem;

namespace {

struct SupervisorFixture {
    std::string config_path{"/tmp/test_robot_supervisor.json"};
    std::string pending_path{"/tmp/test_robot_supervisor.pending.json"};
    std::string backup_path{"/tmp/test_robot_supervisor.backup.json"};

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
    ConfigService cfg{config_path};
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
        brush_serial->open_result = true;
        brush->open();
        MotionService::Config motion_cfg;
        motion_cfg.heading_pid_en = false;
        motion = std::make_shared<MotionService>(group, brush, nullptr, bus, motion_cfg);

        std::ofstream f(config_path);
        f << R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})";
        f.close();

        REQUIRE(cfg.load());
        scheduler.clear_windows();
        scheduler.add_window({8, 0});
        bus.subscribe<FaultService::FaultEvent>(
            [this](const FaultService::FaultEvent& evt) { fault_events.push_back(evt); });
        tb_cfg = std::make_shared<ThingsBoardConfigManager>(cfg, scheduler);
        fsm = std::make_shared<RobotFsm>(motion, nav, fault, bus);
        fsm->dispatch(EvInitDone{});
        supervisor =
            std::make_shared<RobotSupervisor>(fsm, tb_cfg, command_tracker, fault, nav);
    }

    ~SupervisorFixture() {
        fs::remove(config_path);
        fs::remove(pending_path);
        fs::remove(backup_path);
    }

    void apply_pending_config_with_passes(double passes) {
        REQUIRE(tb_cfg->apply_shared_attributes(nlohmann::json{{"passes", passes}}).accepted);
        REQUIRE(tb_cfg->has_pending_config());
    }
};

}  // namespace

TEST_CASE("RobotSupervisor rejects schedule start when robot is not at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_scheduled_task(false, false));
    REQUIRE(f.fsm->current_state() == "Idle");
}

TEST_CASE("RobotSupervisor promotes pending config before scheduled task start",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.apply_pending_config_with_passes(3);

    REQUIRE(f.supervisor->start_scheduled_task(true, false));
    REQUIRE(f.fsm->current_state() == "CleanFwd");
    REQUIRE(f.tb_cfg->active_config().passes == Approx(3));
}

TEST_CASE("RobotSupervisor rejects manual start from idle when robot is not at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_manual_task(false, false));
    REQUIRE(f.fsm->current_state() == "Idle");
}

TEST_CASE("RobotSupervisor starts manual task from charging when robot is at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 0.5f});
    f.fsm->dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm->current_state() == "Charging");

    REQUIRE(f.supervisor->start_manual_task(true, false));
    REQUIRE(f.fsm->current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor resumes paused task without requiring home position",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(EvPauseTask{});

    REQUIRE(f.supervisor->resume_paused_task());
    REQUIRE(f.fsm->current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor reports active cadence for running states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.supervisor->desired_cloud_period_ms(1000, 300000) == 1000);
}

TEST_CASE("RobotSupervisor pauses only from cleaning states", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->pause_task());

    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.supervisor->pause_task());
    REQUIRE(f.fsm->current_state() == "Paused");
}

TEST_CASE("RobotSupervisor returns from paused or cleaning states",
          "[app][robot_supervisor]") {
    SECTION("return from cleaning state") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
        REQUIRE(f.supervisor->return_task());
        REQUIRE(f.fsm->current_state() == "Returning");
    }

    SECTION("return from paused state") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
        f.fsm->dispatch(EvPauseTask{});
        REQUIRE(f.supervisor->return_task());
        REQUIRE(f.fsm->current_state() == "Returning");
    }
}

TEST_CASE("RobotSupervisor terminates active tasks and returning state",
          "[app][robot_supervisor]") {
    SECTION("terminate from cleaning state") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
        REQUIRE(f.supervisor->terminate_task());
        REQUIRE(f.fsm->current_state() == "Terminated");
    }

    SECTION("terminate from returning state") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
        f.fsm->dispatch(EvManualReturn{});
        REQUIRE(f.supervisor->terminate_task());
        REQUIRE(f.fsm->current_state() == "Terminated");
    }
}

TEST_CASE("RobotSupervisor resets only from fault or terminated at home",
          "[app][robot_supervisor]") {
    SECTION("reject reset outside faulted states") {
        SupervisorFixture f;
        REQUIRE_FALSE(f.supervisor->reset_task(true));
        REQUIRE(f.fsm->current_state() == "Idle");
    }

    SECTION("reset fault state at home") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvFaultP0{});
        REQUIRE(f.fsm->current_state() == "Fault");

        REQUIRE(f.supervisor->reset_task(true));
        REQUIRE(f.fsm->current_state() == "Idle");
    }

    SECTION("reject reset when not at home") {
        SupervisorFixture f;
        f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
        f.fsm->dispatch(EvTerminateTask{});
        REQUIRE(f.fsm->current_state() == "Terminated");

        REQUIRE_FALSE(f.supervisor->reset_task(false));
        REQUIRE(f.fsm->current_state() == "Terminated");
    }
}

TEST_CASE("RobotSupervisor low battery triggers returning from active task",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});

    f.supervisor->tick_safety(true);
    REQUIRE(f.fsm->current_state() == "Returning");
}

TEST_CASE("RobotSupervisor spin-free reports P0 fault outside idle states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.can->opened = true;
    REQUIRE(f.group->set_speed_uniform(50.0f) == robot::device::DeviceError::OK);
    f.group->update();
    for (int i = 0; i < 50; ++i) {
        f.nav->update();
    }
    REQUIRE(f.nav->get_pose().spin_free_detected);

    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.supervisor->tick_safety(false);

    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].code == 0x0002);
    REQUIRE(f.fault_events[0].level == FaultService::FaultEvent::Level::P0);
    REQUIRE_FALSE(f.nav->get_pose().spin_free_detected);
}

TEST_CASE("RobotSupervisor snapshot reflects active task progress",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(EvFrontLimitSettled{});

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.device_state == "CleanReturn");
    REQUIRE(snap.task_state == "RunningTask");
    REQUIRE(snap.target_half_passes == 4);
    REQUIRE(snap.completed_half_passes == 1);
    REQUIRE(snap.clean_count == 0);
}

TEST_CASE("RobotSupervisor snapshot includes config and command visibility",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.apply_pending_config_with_passes(3);
    const auto accepted_id = f.command_tracker->accept("start", "req-1");
    f.command_tracker->mark_running(accepted_id);
    f.command_tracker->finish_success(accepted_id, "started_new_task");

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.active_config.has_value());
    REQUIRE(snap.pending_config.has_value());
    REQUIRE(snap.active_config_version != 0);
    REQUIRE_FALSE(snap.active_command.has_value());
    REQUIRE(snap.last_command.has_value());
    REQUIRE(snap.last_command->phase == CommandPhase::Succeeded);
    REQUIRE(snap.last_command->reason == "started_new_task");
}
