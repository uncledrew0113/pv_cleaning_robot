#include <catch2/catch.hpp>

#include <string_view>

#include "pv_cleaning_robot/service/business_payload_builder.h"

TEST_CASE("BusinessPayloadBuilder emits periodic business telemetry into caller buffer",
          "[service][business_payload]") {
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
    active_config.schedules = {{8, 30}};
    snap.active_config = active_config;

    robot::service::CommandSnapshot active_command;
    active_command.id = "cmd-2";
    active_command.name = "stop";
    active_command.phase = robot::service::CommandPhase::Running;
    snap.active_command = active_command;

    char out[2048];
    const size_t len = robot::service::BusinessPayloadBuilder::build(snap, out, sizeof(out));

    REQUIRE(len > 0);
    const std::string_view payload(out, len);
    CHECK(payload.find("\"device_state\":\"Paused\"") != std::string_view::npos);
    CHECK(payload.find("\"active_config_version\":42") != std::string_view::npos);
    CHECK(payload.find("\"active_config\":{\"passes\":2.000000") != std::string_view::npos);
    CHECK(payload.find("\"active_command\":{\"id\":\"cmd-2\"") != std::string_view::npos);
}
