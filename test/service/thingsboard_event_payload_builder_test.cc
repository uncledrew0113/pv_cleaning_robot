#include <catch2/catch.hpp>

#include <string_view>

#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"

TEST_CASE("ThingsBoardEventPayloadBuilder emits shared-attribute style status event",
          "[service][tb_event_payload]") {
    char out[512];
    robot::service::ThingsBoardEventPayloadBuilder::StatusEventView view{
        "shared_attr_update", true, "ok"};

    const size_t len =
        robot::service::ThingsBoardEventPayloadBuilder::build_status_event(view, out, sizeof(out));

    REQUIRE(len > 0);
    const std::string_view payload(out, len);
    CHECK(payload.find("\"event\":\"shared_attr_update\"") != std::string_view::npos);
    CHECK(payload.find("\"accepted\":true") != std::string_view::npos);
    CHECK(payload.find("\"reason\":\"ok\"") != std::string_view::npos);
}

TEST_CASE("ThingsBoardEventPayloadBuilder emits command event payload",
          "[service][tb_event_payload]") {
    char out[1024];
    robot::service::CommandSnapshot command;
    command.id = "cmd-1";
    command.name = "start";
    command.request_id = "42";
    command.phase = robot::service::CommandPhase::Succeeded;
    command.reason = "started_new_task";
    command.accepted_at_ms = 10;
    command.finished_at_ms = 20;

    robot::service::ThingsBoardEventPayloadBuilder::CommandEventView view{
        "command_completed", &command};

    const size_t len =
        robot::service::ThingsBoardEventPayloadBuilder::build_command_event(view, out, sizeof(out));

    REQUIRE(len > 0);
    const std::string_view payload(out, len);
    CHECK(payload.find("\"event\":\"command_completed\"") != std::string_view::npos);
    CHECK(payload.find("\"command_id\":\"cmd-1\"") != std::string_view::npos);
    CHECK(payload.find("\"phase\":\"succeeded\"") != std::string_view::npos);
    CHECK(payload.find("\"reason\":\"started_new_task\"") != std::string_view::npos);
}
