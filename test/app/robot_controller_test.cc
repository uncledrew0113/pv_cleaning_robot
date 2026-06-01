#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/app/fault_policy.h"
#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::RobotController;
using robot::domain::CommandSource;
using robot::domain::RobotCommand;
using robot::domain::RobotCommandKind;

struct RecordingRobotActions {
    int start_segment_count{0};
    int stop_count{0};
    int emergency_stop_count{0};
    int start_recovery_count{0};
    int clear_fault_count{0};

    robot::app::RobotController::ActionPorts ports() {
        robot::app::RobotController::ActionPorts result;
        result.start_segment = [this](const robot::domain::MissionSegment&) {
            ++start_segment_count;
            return true;
        };
        result.stop_motion = [this] { ++stop_count; };
        result.emergency_stop = [this] { ++emergency_stop_count; };
        result.start_recovery = [this] { ++start_recovery_count; };
        result.clear_fault = [this] { ++clear_fault_count; };
        return result;
    }
};

TEST_CASE("RobotController starts configured mission through SelfChecking",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController rejects start while busy", "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-2"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "busy");
}

TEST_CASE("RobotController stop is only accepted while mission is active",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    auto result =
        controller.submit_command(RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-idle"});
    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "not_running");

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-running"});
    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
}

TEST_CASE("RobotController serializes posted callbacks on controller thread",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.start();

    std::atomic<int> count{0};
    std::vector<std::thread> workers;
    for (int i = 0; i < 8; ++i) {
        workers.emplace_back([&controller, &count] {
            for (int j = 0; j < 25; ++j) {
                controller.post_for_test([&count] { count.fetch_add(1); });
            }
        });
    }
    for (auto& worker : workers) {
        worker.join();
    }

    controller.drain_for_test();
    controller.stop();

    REQUIRE(count.load() == 200);
}

TEST_CASE("RobotController submit_command works through running queue",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.start();

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-queued"});

    controller.stop();

    REQUIRE(result.accepted);
    REQUIRE(result.reason == "accepted");
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController completes configured single-dock mission through two endpoints",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    controller.complete_self_check_for_test(true);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    REQUIRE(controller.snapshot().state == "Idle");
}

TEST_CASE("RobotController unexpected endpoint enters FaultStopped",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController P0 fault enters FaultStopped and reset returns Idle",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.handle_fault_for_test(robot::app::FaultFact{
        robot::app::FaultSource::Watchdog,
        robot::domain::FaultCode::kCanCommunicationLost,
        "can_lost"});

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kCanCommunicationLost);

    const auto reset = controller.submit_command(
        RobotCommand{RobotCommandKind::FaultReset, CommandSource::Rpc, "reset-1"});
    REQUIRE(reset.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController recoverable fault enters Recovering from ExecutingMission",
          "[app][robot_controller]") {
    RobotController controller;
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_fault_for_test(robot::app::FaultFact{
        robot::app::FaultSource::FaultDetector,
        robot::domain::FaultCode::kTransientAttitudeError,
        "tilt"});

    REQUIRE(controller.snapshot().state == "Recovering");
}

TEST_CASE("RobotController starts motion after successful self check",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(actions.start_segment_count == 1);
}

TEST_CASE("RobotController converts motion start failure into FaultStopped",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    auto ports = actions.ports();
    ports.start_segment = [](const robot::domain::MissionSegment&) { return false; };
    RobotController controller(ports);
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kSegmentStartFailed);
    REQUIRE(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController posted watchdog timeout enters FaultStopped",
          "[app][robot_controller]") {
    RobotController controller;
    controller.start();

    controller.post_watchdog_timeout("walk_ctrl");
    controller.drain_for_test();
    controller.stop();

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kCanCommunicationLost);
}

TEST_CASE("RobotController posted limit unstable enters FaultStopped",
          "[app][robot_controller]") {
    RobotController controller;
    controller.start();

    controller.post_limit_unstable(robot::domain::Endpoint::A);
    controller.drain_for_test();
    controller.stop();

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault ==
            robot::domain::FaultCode::kLimitUnstableAfterEmergencyStop);
}

TEST_CASE("RobotController rejects configured mission away from dock", "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "configured_mission_requires_start_endpoint");
}

TEST_CASE("RobotController rejects low battery before start", "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.set_battery_soc_query([] { return 10.0f; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "battery_below_start_threshold");
}
