#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/business_telemetry_snapshot.h"

TEST_CASE("BusinessTelemetrySnapshot serializes config and command state",
          "[service][business_telemetry]") {
    robot::service::BusinessTelemetrySnapshot snap;
    snap.device_state = "Paused";
    snap.task_state = "PausedTask";
    snap.target_half_passes = 4;
    snap.completed_half_passes = 1;
    snap.clean_count = 7;
    snap.active_config_version = 42;
    robot::service::TbRuntimeConfig active_config;
    active_config.passes = 2.0;
    active_config.clean_speed_rpm = 180.0;
    active_config.return_speed_rpm = 120.0;
    active_config.brush_rpm = 900;
    active_config.schedules = {{8, 30}, {16, 45}};
    snap.active_config = active_config;

    robot::service::TbRuntimeConfig pending_config;
    pending_config.passes = 3.0;
    pending_config.clean_speed_rpm = 200.0;
    pending_config.return_speed_rpm = 130.0;
    pending_config.brush_rpm = 950;
    pending_config.schedules = {{9, 0}};
    snap.pending_config = pending_config;

    robot::service::CommandSnapshot active_command;
    active_command.id = "cmd-2";
    active_command.name = "stop";
    active_command.request_id = "req-2";
    active_command.phase = robot::service::CommandPhase::Running;
    active_command.accepted_at_ms = 100;
    snap.active_command = active_command;

    robot::service::CommandSnapshot last_command;
    last_command.id = "cmd-1";
    last_command.name = "start";
    last_command.request_id = "req-1";
    last_command.phase = robot::service::CommandPhase::Succeeded;
    last_command.reason = "started_new_task";
    last_command.accepted_at_ms = 50;
    last_command.finished_at_ms = 70;
    snap.last_command = last_command;

    const auto j = snap.to_json();

    REQUIRE(j.at("device_state").get<std::string>() == "Paused");
    REQUIRE(j.at("task_state").get<std::string>() == "PausedTask");
    REQUIRE(j.at("target_half_passes").get<int>() == 4);
    REQUIRE(j.at("completed_half_passes").get<int>() == 1);
    REQUIRE(j.at("clean_count").get<int>() == 7);
    REQUIRE(j.at("active_config_version").get<uint64_t>() == 42);
    REQUIRE(j.at("active_config").at("passes").get<double>() == Approx(2.0));
    REQUIRE(j.at("active_config").at("schedules").size() == 2);
    REQUIRE(j.at("pending_config").at("brush_rpm").get<int>() == 950);
    REQUIRE(j.at("active_command").at("phase").get<std::string>() == "running");
    REQUIRE(j.at("last_command").at("name").get<std::string>() == "start");
    REQUIRE(j.at("last_command").at("reason").get<std::string>() == "started_new_task");
}

TEST_CASE("BusinessTelemetrySnapshot omits optional sections when absent",
          "[service][business_telemetry]") {
    robot::service::BusinessTelemetrySnapshot snap;
    snap.device_state = "Idle";
    snap.task_state = "IdleTask";

    const auto j = snap.to_json();

    REQUIRE(j.at("device_state").get<std::string>() == "Idle");
    REQUIRE(j.at("task_state").get<std::string>() == "IdleTask");
    REQUIRE(j.at("active_config_version").get<uint64_t>() == 0);
    REQUIRE(j.find("active_config") == j.end());
    REQUIRE(j.find("pending_config") == j.end());
    REQUIRE(j.find("active_command") == j.end());
    REQUIRE(j.find("last_command") == j.end());
}
