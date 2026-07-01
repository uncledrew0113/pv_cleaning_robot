#include <catch2/catch.hpp>

#include <memory>
#include <rapidjson/document.h>
#include <string_view>

#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/diagnostics_collector.h"

namespace {

struct MockTransport final : robot::middleware::INetworkTransport {
    std::string last_publish_topic;
    std::string last_publish_payload;

    bool connect() override { return true; }
    void disconnect() override {}
    bool is_connected() const override { return true; }
    bool publish(const std::string& topic, const std::string& payload) override {
        last_publish_topic = topic;
        last_publish_payload = payload;
        return true;
    }
    bool subscribe(const std::string&, MessageCallback) override { return true; }
};

}  // namespace

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

TEST_CASE("HealthPayloadBuilder emits health payload from diagnostics snapshot",
          "[service][health][payload]") {
    char out[4096];
    robot::service::DiagnosticsCollector::Snapshot snapshot{};
    snapshot.ts_ms = 1234ULL;
    snapshot.epoch_ms = 1714202400789ULL;
    snapshot.walk_status.wheel[0].speed_rpm = 12.0f;
    snapshot.walk_status.wheel[1].speed_rpm = 13.0f;
    snapshot.brush_status.actual_rpm = 700;
    snapshot.bms_data.soc_pct = 88.0f;
    snapshot.imu_data.pitch_deg = 1.0f;
    snapshot.gps_data.fix_quality = 2;

    const size_t len =
        robot::service::HealthPayloadBuilder::build_health(snapshot, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc["ts"].GetUint64() == 1714202400789ULL);
    REQUIRE(doc["values"].HasMember("lt_rpm"));
    CHECK(doc["values"]["lt_rpm"].GetFloat() == 12.0f);
    CHECK(doc["values"]["br_rpm"].GetInt() == 700);
}

TEST_CASE("HealthPayloadBuilder emits diagnostics payload from diagnostics snapshot",
          "[service][health][payload]") {
    char out[4096];
    robot::service::DiagnosticsCollector::Snapshot snapshot{};
    snapshot.ts_ms = 1234ULL;
    snapshot.epoch_ms = 1714202400999ULL;
    snapshot.walk_diagnostics.wheel[0].feedback_frame_count = 42;
    snapshot.brush_diagnostics.comm_error_count = 3;
    snapshot.bms_diagnostics.error_count = 4;
    snapshot.imu_diagnostics.frame_count = 5;
    snapshot.gps_diagnostics.sentence_count = 6;

    const size_t len =
        robot::service::HealthPayloadBuilder::build_diagnostics(snapshot, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc["ts"].GetUint64() == 1714202400999ULL);
    CHECK(doc["values"]["br_ce"].GetUint() == 3);
    CHECK(doc["values"]["gps_sent"].GetUint() == 6);
}

TEST_CASE("HealthService publishes telemetry from shared diagnostics collector",
          "[service][health]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<robot::middleware::NetworkManager>(
        mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<robot::middleware::DataCache>(
        "/tmp/health_service_snapshot_provider_test.jsonl");
    auto cloud = std::make_shared<robot::service::CloudService>(net, cache);

    auto diagnostics = std::make_shared<robot::service::DiagnosticsCollector>();
    robot::service::DiagnosticsCollector::Input input{};
    input.brush_status.actual_rpm = 650;
    input.gps_data.fix_quality = 2;
    diagnostics->update_from_input(input, 1234ULL, 1714202401111ULL);

    robot::service::HealthService service(
        diagnostics,
        cloud,
        robot::service::HealthService::Mode::HEALTH);
    service.update();

    REQUIRE(mqtt->last_publish_topic == "v1/devices/me/telemetry");
    rapidjson::Document doc;
    doc.Parse(mqtt->last_publish_payload.c_str(), mqtt->last_publish_payload.size());
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc["ts"].GetUint64() == 1714202401111ULL);
    CHECK(doc["values"]["br_rpm"].GetInt() == 650);
}
