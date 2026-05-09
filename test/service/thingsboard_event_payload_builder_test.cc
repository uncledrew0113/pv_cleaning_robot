#include <catch2/catch.hpp>

#include <rapidjson/document.h>

#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace {

rapidjson::Document parse_json(const char* data, size_t len)
{
    rapidjson::Document doc;
    doc.Parse(data, len);
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

}  // namespace

TEST_CASE("ThingsBoardJsonCodec emits startup attributes payload", "[service][tb_json_codec]") {
    char out[512];
    robot::service::ThingsBoardJsonCodec::StartupAttributesView view{
        "2.0.0", "A1", "pv_cleaning_robot_test", "pv_robot_test_001"};

    const size_t len = robot::service::ThingsBoardJsonCodec::build_startup_attributes(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["software_version"].GetString()) == "2.0.0");
    CHECK(std::string(payload["hardware_version"].GetString()) == "A1");
    CHECK(std::string(payload["device_model"].GetString()) == "pv_cleaning_robot_test");
    CHECK(std::string(payload["device_id"].GetString()) == "pv_robot_test_001");
    REQUIRE(payload["supported_rpc_methods"].IsArray());
    CHECK(std::string(payload["config_schema_version"].GetString()) == "thingsboard-v1");
}

TEST_CASE("ThingsBoardJsonCodec emits shared-attribute style status event",
          "[service][tb_json_codec]") {
    char out[512];
    robot::service::ThingsBoardJsonCodec::StatusEventView view{
        "shared_attr_update", true, "ok"};

    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_status_event(view, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["event"].GetString()) == "shared_attr_update");
    CHECK(payload["accepted"].GetBool() == true);
    CHECK(std::string(payload["reason"].GetString()) == "ok");
}

TEST_CASE("ThingsBoardJsonCodec emits command event payload", "[service][tb_json_codec]") {
    char out[1024];
    robot::service::CommandSnapshot command;
    command.id = "cmd-1";
    command.name = "start";
    command.request_id = "42";
    command.phase = robot::service::CommandPhase::Succeeded;
    command.reason = "started_new_task";
    command.accepted_at_ms = 10;
    command.finished_at_ms = 20;

    robot::service::ThingsBoardJsonCodec::CommandEventView view{
        "command_completed", &command};

    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_command_event(view, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["event"].GetString()) == "command_completed");
    CHECK(std::string(payload["command_id"].GetString()) == "cmd-1");
    CHECK(std::string(payload["phase"].GetString()) == "succeeded");
    CHECK(std::string(payload["reason"].GetString()) == "started_new_task");
}
