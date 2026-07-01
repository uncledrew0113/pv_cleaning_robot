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
