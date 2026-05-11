#include <catch2/catch.hpp>

#include <rapidjson/document.h>
#include <string_view>

#include "pv_cleaning_robot/service/health_service.h"

TEST_CASE("HealthPayloadBuilder emits diagnostics payload into caller buffer",
          "[service][health][payload]") {
    char out[4096];
    robot::service::HealthPayloadBuilder::DiagnosticsView view{};
    view.ts_ms = 1714202400123ULL;
    view.walk.wheel[0].speed_rpm = 11.0f;
    view.walk.wheel[0].torque_a = 1.5f;
    view.walk.wheel[0].fault = robot::protocol::WalkMotorFault::OVER_CURRENT;
    view.walk.ctrl_frame_count = 12;
    view.brush.actual_rpm = 800;
    view.gps.fix_quality = 2;
    view.imu.pitch_deg = 1.2f;
    view.imu.roll_deg = 2.3f;
    view.imu.yaw_deg = 3.4f;

    const size_t len = robot::service::HealthPayloadBuilder::build_diagnostics(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc.HasMember("ts"));
    REQUIRE(doc["ts"].IsUint64());
    REQUIRE(doc["ts"].GetUint64() == 1714202400123ULL);
    REQUIRE(doc.HasMember("values"));
    REQUIRE(doc["values"].IsObject());
    REQUIRE(doc["values"].HasMember("lt_rpm"));
    REQUIRE(doc["values"].HasMember("lt_cur"));
    REQUIRE(doc["values"].HasMember("lt_err"));
    REQUIRE(doc["values"].HasMember("imu_p"));
    REQUIRE(doc["values"].HasMember("imu_r"));
    REQUIRE(doc["values"].HasMember("imu_y"));
    REQUIRE(doc["values"].HasMember("br_rpm"));
    REQUIRE(doc["values"].HasMember("gps_fix"));
}

TEST_CASE("HealthPayloadBuilder emits health payload with per-wheel walk values",
          "[service][health][payload]") {
    char out[4096];
    robot::service::HealthPayloadBuilder::HealthView view{};
    view.ts_ms = 1714202400456ULL;
    view.walk.wheel[0].speed_rpm = 10.0f;
    view.walk.wheel[1].speed_rpm = 20.0f;
    view.walk.wheel[2].speed_rpm = 30.0f;
    view.walk.wheel[3].speed_rpm = 40.0f;
    view.walk.wheel[0].torque_a = 1.0f;
    view.walk.wheel[1].torque_a = 2.0f;
    view.walk.wheel[2].torque_a = 3.0f;
    view.walk.wheel[3].torque_a = 4.0f;
    view.imu.pitch_deg = 5.0f;
    view.imu.roll_deg = 6.0f;
    view.imu.yaw_deg = 7.0f;

    const size_t len = robot::service::HealthPayloadBuilder::build_health(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc.HasMember("ts"));
    REQUIRE(doc["ts"].IsUint64());
    REQUIRE(doc["ts"].GetUint64() == 1714202400456ULL);
    REQUIRE(doc.HasMember("values"));
    REQUIRE(doc["values"].IsObject());
    REQUIRE(doc["values"].HasMember("lt_rpm"));
    REQUIRE(doc["values"].HasMember("rt_rpm"));
    REQUIRE(doc["values"].HasMember("lb_rpm"));
    REQUIRE(doc["values"].HasMember("rb_rpm"));
    REQUIRE(doc["values"].HasMember("lt_cur"));
    REQUIRE(doc["values"].HasMember("rt_cur"));
    REQUIRE(doc["values"].HasMember("lb_cur"));
    REQUIRE(doc["values"].HasMember("rb_cur"));
    REQUIRE(doc["values"].HasMember("imu_p"));
    REQUIRE(doc["values"].HasMember("imu_r"));
    REQUIRE(doc["values"].HasMember("imu_y"));
}
