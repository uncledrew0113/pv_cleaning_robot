#include <catch2/catch.hpp>

#include "pv_cleaning_robot/protocol/gpsd_json_parser.h"

TEST_CASE("GpsdJsonParser parses TPV without device-layer socket state", "[protocol][gpsd]") {
    robot::protocol::GpsData data{};
    robot::protocol::GpsdJsonParseResult result{};

    const bool ok = robot::protocol::GpsdJsonParser::parse_line(
        R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":3,"time":"2026-04-25T12:34:56.789Z"})",
        data,
        result);

    REQUIRE(ok);
    REQUIRE(result.message_class == robot::protocol::GpsdMessageClass::TPV);
    REQUIRE(result.updated_fix);
    REQUIRE(data.valid);
    REQUIRE(data.fix_quality == 2);
    REQUIRE(data.utc_timestamp_ms == 1777120496789ULL);
}
