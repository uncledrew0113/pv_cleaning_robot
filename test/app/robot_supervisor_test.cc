#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>

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
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

namespace {

struct SupervisorFixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_robot_supervisor")};

    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<robot::device::WalkMotorGroup> group{
        std::make_shared<robot::device::WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::BrushMotor> brush{
        std::make_shared<robot::device::BrushMotor>(brush_serial, 0)};
    std::shared_ptr<MockSerialPort> imu_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::ImuDevice> imu{
        std::make_shared<robot::device::ImuDevice>(imu_serial)};
    std::shared_ptr<MockSerialPort> gps_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::GpsDevice> gps{
        std::make_shared<robot::device::GpsDevice>(gps_serial)};

    robot::middleware::EventBus bus;
    robot::service::ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    std::shared_ptr<robot::service::FaultService> fault{
        std::make_shared<robot::service::FaultService>(bus)};
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::service::NavService> nav;
    std::shared_ptr<robot::app::RobotFsm> fsm{std::make_shared<robot::app::RobotFsm>()};
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;

    SupervisorFixture()
        : motion([this] {
              robot::service::MotionService::Config motion_cfg;
              motion_cfg.heading_pid_en = false;
              return std::make_shared<robot::service::MotionService>(
                  group, brush, nullptr, bus, motion_cfg);
          }())
        , nav(std::make_shared<robot::service::NavService>(group, imu, gps)) {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();

        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "repeat_count": 2,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "primary_dock": "A",
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
        motion->set_runtime_config_query([this]() { return cfg.active_runtime_config(); });
        motion->set_primary_dock_query([this]() { return cfg.active_runtime_config().primary_dock; });
        supervisor = std::make_shared<robot::app::RobotSupervisor>(fsm, cfg, fault, nav);
        supervisor->set_motion_service(motion);
        supervisor->set_battery_soc_query([] { return 80.0f; });
    }

    ~SupervisorFixture() {
        tb_test_support::cleanup_split_config_paths(paths);
    }
};

}  // namespace

TEST_CASE("RobotSupervisor accepts directional clean from trusted on-segment pose",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });

    const auto result = f.supervisor->submit_command(robot::domain::RobotCommand{
        robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
        robot::domain::CommandSource::Rpc,
        "rpc-101"});

    REQUIRE(result.accepted);
    CHECK(f.fsm->robot_state() == robot::app::RobotState::SelfChecking);
}

TEST_CASE("RobotSupervisor rejects configured mission away from endpoint",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });

    const auto result = f.supervisor->submit_command(robot::domain::RobotCommand{
        robot::domain::RobotCommandKind::StartConfiguredMission,
        robot::domain::CommandSource::Rpc,
        "rpc-102"});

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "configured_mission_requires_start_endpoint");
}

TEST_CASE("RobotSupervisor dispatches motion after self-check succeeds",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });
    f.can->sent_frames.clear();

    REQUIRE(f.supervisor
                ->submit_command(robot::domain::RobotCommand{
                    robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                    robot::domain::CommandSource::Rpc,
                    "rpc-103"})
                .accepted);
    const auto self_check = f.supervisor->handle_self_check_passed();

    REQUIRE(self_check.accepted);
    CHECK(f.fsm->robot_state() == robot::app::RobotState::ExecutingMission);
    CHECK_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("RobotSupervisor self-check rejects low battery before motion",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_battery_soc_query([] { return 20.0f; });
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });
    f.can->sent_frames.clear();

    REQUIRE(f.supervisor
                ->submit_command(robot::domain::RobotCommand{
                    robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                    robot::domain::CommandSource::Rpc,
                    "rpc-104"})
                .accepted);
    const auto result = f.supervisor->handle_self_check_passed();

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "battery_below_start_threshold");
    CHECK(f.fsm->robot_state() == robot::app::RobotState::Idle);
    CHECK(f.can->sent_frames.empty());
}

TEST_CASE("RobotSupervisor completes mission on expected settled endpoint",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });
    REQUIRE(f.supervisor
                ->submit_command(robot::domain::RobotCommand{
                    robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                    robot::domain::CommandSource::Rpc,
                    "rpc-105"})
                .accepted);
    REQUIRE(f.supervisor->handle_self_check_passed().accepted);

    f.supervisor->handle_limit_settled(
        robot::domain::Endpoint::B, false, true, 80.0f);

    CHECK(f.fsm->robot_state() == robot::app::RobotState::Idle);
}

TEST_CASE("RobotSupervisor treats settled endpoint during recovery as P0 fault",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::OnSegment; });
    REQUIRE(f.supervisor
                ->submit_command(robot::domain::RobotCommand{
                    robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                    robot::domain::CommandSource::Rpc,
                    "rpc-106"})
                .accepted);
    REQUIRE(f.supervisor->handle_self_check_passed().accepted);

    f.supervisor->handle_fault_event(robot::service::FaultEvent{
        robot::service::FaultEvent::Level::P2,
        robot::service::FaultCode::kTransientAttitudeError,
        "pose",
        0});
    REQUIRE(f.fsm->robot_state() == robot::app::RobotState::Recovering);

    f.supervisor->handle_limit_settled(robot::domain::Endpoint::B, false, true, 80.0f);

    CHECK(f.fsm->robot_state() == robot::app::RobotState::FaultStopped);
    REQUIRE(f.fault->has_active_fault());
    CHECK(f.fault->last_fault().level == robot::service::FaultEvent::Level::P0);
    CHECK(f.fault->last_fault().code == robot::service::FaultCode::kUnexpectedLimitSide);
}

TEST_CASE("RobotSupervisor snapshot exposes repeat progress", "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.supervisor->set_position_state_query(
        [] { return robot::domain::PositionState::AtA; });
    REQUIRE(f.supervisor
                ->submit_command(robot::domain::RobotCommand{
                    robot::domain::RobotCommandKind::StartConfiguredMission,
                    robot::domain::CommandSource::Scheduler,
                    "schedule"})
                .accepted);

    const auto snap = f.supervisor->snapshot();
    CHECK(snap.state == "SelfChecking");
    CHECK(snap.repeat_count == 2u);
    CHECK(snap.completed_cycles == 0u);
    REQUIRE(snap.active_config.has_value());
    CHECK(snap.cfg_ver != 0u);
}
