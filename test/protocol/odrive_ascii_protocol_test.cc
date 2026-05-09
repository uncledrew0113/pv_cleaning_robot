#include <catch2/catch.hpp>

#include <string>

#include "pv_cleaning_robot/protocol/odrive_ascii_protocol.h"

using robot::protocol::OdriveReadProperty;

TEST_CASE("OdriveAsciiProtocol encodes velocity command", "[protocol][odrive]") {
    char line[64];
    const size_t len = robot::protocol::encode_set_velocity(1, 1024.5f, line, sizeof(line));
    REQUIRE(len > 0);
    REQUIRE(std::string(line, len) == "v 1 1024.500 0\n");
}

TEST_CASE("OdriveAsciiProtocol encodes property reads", "[protocol][odrive]") {
    char line[96];
    const size_t len = robot::protocol::encode_read_property(
        OdriveReadProperty::IQ_MEASURED, 0, line, sizeof(line));
    REQUIRE(len > 0);
    REQUIRE(std::string(line, len) == "r axis0.motor.current_control.Iq_measured\n");

    const size_t temp_len = robot::protocol::encode_read_property(
        OdriveReadProperty::FET_TEMPERATURE, 0, line, sizeof(line));
    REQUIRE(temp_len > 0);
    REQUIRE(std::string(line, temp_len) == "r axis0.fet_thermistor.temperature\n");
}

TEST_CASE("OdriveAsciiProtocol parses feedback response", "[protocol][odrive]") {
    float pos = 0.0f;
    float vel = 0.0f;
    REQUIRE(robot::protocol::parse_feedback_response("12.5 -512.25", &pos, &vel));
    REQUIRE(pos == Approx(12.5f));
    REQUIRE(vel == Approx(-512.25f));
}

TEST_CASE("OdriveAsciiProtocol parses scalar responses", "[protocol][odrive]") {
    float f = 0.0f;
    uint32_t u = 0;
    REQUIRE(robot::protocol::parse_float_response("24.125", &f));
    REQUIRE(robot::protocol::parse_u32_response("0x10", &u));
    REQUIRE(f == Approx(24.125f));
    REQUIRE(u == 0x10u);
}
