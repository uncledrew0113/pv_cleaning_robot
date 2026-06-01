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
    CHECK(payload.MemberCount() == 4);
}
