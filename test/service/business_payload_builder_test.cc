#include <catch2/catch.hpp>

#include <rapidjson/document.h>

#include "pv_cleaning_robot/domain/robot_domain.h"
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
    robot::domain::RobotRuntimeSnapshot snap;
    snap.state = "FaultStopped";
    snap.fault = 0x2001;
    snap.repeat_count = 2;
    snap.completed_cycles = 0;
    snap.cfg_ver = 42;

    robot::service::RuntimeConfig active_config;
    active_config.repeat_count = 2;
    active_config.clean_speed_rpm = 180.0;
    active_config.return_speed_rpm = 120.0;
    active_config.brush_rpm = 900;
    active_config.min_battery_soc = 30.0;
    active_config.primary_dock = robot::domain::Endpoint::B;
    active_config.schedules = {{8, 30}};
    snap.active_config = active_config;

    robot::service::RuntimeConfig pending_config = active_config;
    pending_config.primary_dock = robot::domain::Endpoint::A;
    snap.pending_config = pending_config;

    char out[2048];
    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_business_telemetry(snap, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["state"].GetString()) == "FaultStopped");
    CHECK(payload["fault"].GetUint() == 0x2001);
    CHECK(payload["cfg_ver"].GetUint64() == 42);
    CHECK(payload["repeat_count"].GetUint() == 2);
    CHECK(payload["completed_cycles"].GetUint() == 0);
    CHECK(payload.MemberCount() == 5);
}
