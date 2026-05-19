#include <catch2/catch.hpp>

#include <rapidjson/document.h>

#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"
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

TEST_CASE("ThingsBoardJsonCodec emits periodic business telemetry into caller buffer",
          "[service][tb_json_codec]") {
    robot::app::RobotRuntimeSnapshot snap;
    snap.device_state = "Stopped";
    snap.task_state = "StoppedTask";
    snap.target_passes = 2;
    snap.completed_passes = 0;
    snap.clean_count = 7;
    snap.active_config_version = 42;

    robot::service::RuntimeConfig active_config;
    active_config.passes = 2.0;
    active_config.clean_speed_rpm = 180.0;
    active_config.return_speed_rpm = 120.0;
    active_config.brush_rpm = 900;
    active_config.parking_side = robot::service::ParkingSide::Right;
    active_config.schedules = {{8, 30}};
    snap.active_config = active_config;

    robot::service::RuntimeConfig pending_config = active_config;
    pending_config.parking_side = robot::service::ParkingSide::Left;
    snap.pending_config = pending_config;

    robot::service::CommandSnapshot active_command;
    active_command.id = "cmd-2";
    active_command.name = "stop";
    active_command.phase = robot::service::CommandPhase::Running;
    snap.active_command = active_command;

    char out[2048];
    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_business_telemetry(snap, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["device_state"].GetString()) == "Stopped");
    CHECK(std::string(payload["task_state"].GetString()) == "StoppedTask");
    CHECK(payload["active_config_version"].GetUint64() == 42);
    CHECK(payload.MemberCount() == 3);
}
