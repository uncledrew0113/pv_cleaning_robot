#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

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

TEST_CASE("RobotController serializes posted callbacks on controller thread",
          "[app][robot_controller]") {
    RobotController controller;
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
    controller.start();

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-queued"});

    controller.stop();

    REQUIRE(result.accepted);
    REQUIRE(result.reason == "accepted");
    REQUIRE(controller.snapshot().state == "SelfChecking");
}
