#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/robot_controller.h"

using robot::app::RobotController;
using robot::domain::CommandSource;
using robot::domain::RobotCommand;
using robot::domain::RobotCommandKind;

TEST_CASE("RobotController starts configured mission through SelfChecking",
          "[app][robot_controller]") {
    RobotController controller;

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController rejects start while busy", "[app][robot_controller]") {
    RobotController controller;
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
