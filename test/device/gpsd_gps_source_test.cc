#include <catch2/catch.hpp>

#include <vector>

#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/gps_source.h"

using robot::device::DeviceError;
using robot::device::GpsDevice;
using robot::device::GpsdGpsSource;
using robot::device::GpsdSourceConfig;
using robot::device::IGpsSource;
using robot::protocol::GpsData;

struct GpsdCallbackProbe {
    GpsData latest{};
    int     update_count{0};
    int     parse_error_count{0};
    int     message_count{0};

    GpsdGpsSource make_source() {
        return GpsdGpsSource(
            GpsdSourceConfig{},
            [&](const GpsData& d) {
                latest = d;
                ++update_count;
            },
            [&]() { ++parse_error_count; },
            [&]() { ++message_count; });
    }
};

TEST_CASE("GpsdGpsSource: TPV maps mode 3 to valid fix", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(
        R"({"class":"TPV","lat":30.5,"lon":114.2,"speed":1.2,"track":91.0,"mode":3})");

    REQUIRE(probe.update_count == 1);
    REQUIRE(probe.latest.valid);
    REQUIRE(probe.latest.fix_quality == 2);
    REQUIRE(probe.latest.latitude == Approx(30.5));
    REQUIRE(probe.latest.longitude == Approx(114.2));
    REQUIRE(probe.latest.speed_m_s == Approx(1.2f));
    REQUIRE(probe.latest.course_deg == Approx(91.0f));
}

TEST_CASE("GpsdGpsSource: SKY updates satellite stats without clearing TPV", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":2})");
    src.ingest_json_line_for_test(
        R"({"class":"SKY","hdop":0.8,"satellites":[{"used":true},{"used":false},{"used":true}]})");

    REQUIRE(probe.update_count == 2);
    REQUIRE(probe.latest.latitude == Approx(30.5));
    REQUIRE(probe.latest.longitude == Approx(114.2));
    REQUIRE(probe.latest.hdop == Approx(0.8f));
    REQUIRE(probe.latest.satellites_used == 2);
    REQUIRE(probe.latest.satellites_in_view == 3);
}

TEST_CASE("GpsdGpsSource: TPV mode 2 maps to valid 2D fix", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":2})");

    REQUIRE(probe.latest.valid);
    REQUIRE(probe.latest.fix_quality == 1);
}

TEST_CASE("GpsdGpsSource: TPV mode 1 clears valid fix", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":3})");
    src.ingest_json_line_for_test(R"({"class":"TPV","mode":1})");

    REQUIRE(probe.update_count == 2);
    REQUIRE_FALSE(probe.latest.valid);
    REQUIRE(probe.latest.fix_quality == 0);
    REQUIRE(probe.latest.latitude == Approx(30.5));
    REQUIRE(probe.latest.longitude == Approx(114.2));
}

TEST_CASE("GpsdGpsSource: TPV time parses to UTC epoch milliseconds", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(
        R"({"class":"TPV","mode":3,"time":"2026-04-25T12:34:56.789Z"})");

    REQUIRE(probe.parse_error_count == 0);
    REQUIRE(probe.latest.utc_timestamp_ms == 1777120496789ULL);
}

TEST_CASE("GpsdGpsSource: invalid TPV time increments parse errors but still updates fix", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(
        R"({"class":"TPV","mode":2,"time":"not-a-real-time"})");

    REQUIRE(probe.update_count == 1);
    REQUIRE(probe.parse_error_count == 1);
    REQUIRE(probe.latest.valid);
    REQUIRE(probe.latest.fix_quality == 1);
}

TEST_CASE("GpsdGpsSource: VERSION and WATCH messages are ignored", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"VERSION","release":"3.25"})");
    src.ingest_json_line_for_test(R"({"class":"WATCH","enable":true,"json":true})");

    REQUIRE(probe.update_count == 0);
    REQUIRE(probe.parse_error_count == 0);
    REQUIRE(probe.message_count == 0);
}

TEST_CASE("GpsdGpsSource: malformed JSON increments parse errors", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV",)");

    REQUIRE(probe.update_count == 0);
    REQUIRE(probe.parse_error_count == 1);
}

TEST_CASE("GpsdGpsSource: ERROR message increments parse errors", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"ERROR","message":"bad watch"})");

    REQUIRE(probe.update_count == 0);
    REQUIRE(probe.parse_error_count == 1);
}

TEST_CASE("GpsdGpsSource: unknown class increments parse errors", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"DEVICE","path":"/dev/ttyS2"})");

    REQUIRE(probe.update_count == 0);
    REQUIRE(probe.parse_error_count == 1);
}

TEST_CASE("GpsdGpsSource: TPV missing fields keeps previous values", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(
        R"({"class":"TPV","lat":30.5,"lon":114.2,"speed":1.2,"track":91.0,"mode":3})");
    src.ingest_json_line_for_test(R"({"class":"TPV","mode":3})");

    REQUIRE(probe.latest.latitude == Approx(30.5));
    REQUIRE(probe.latest.longitude == Approx(114.2));
    REQUIRE(probe.latest.speed_m_s == Approx(1.2f));
    REQUIRE(probe.latest.course_deg == Approx(91.0f));
}

TEST_CASE("GpsdGpsSource: altHAE is used when altMSL is absent", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV","mode":3,"altHAE":123.4})");

    REQUIRE(probe.latest.altitude_m == Approx(123.4f));
}

TEST_CASE("GpsdGpsSource: altMSL takes precedence when both altitude fields exist", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(R"({"class":"TPV","mode":3,"altMSL":88.8,"altHAE":123.4})");

    REQUIRE(probe.latest.altitude_m == Approx(88.8f));
}

TEST_CASE("GpsdGpsSource: SKY nSat overrides satellites array size for in-view count", "[device][gps][gpsd]") {
    GpsdCallbackProbe probe;
    auto src = probe.make_source();

    src.ingest_json_line_for_test(
        R"({"class":"SKY","nSat":9,"satellites":[{"used":true},{"used":true},{"used":false}]})");

    REQUIRE(probe.latest.satellites_used == 2);
    REQUIRE(probe.latest.satellites_in_view == 9);
}

TEST_CASE("GpsDevice(gpsd): commands are not supported", "[device][gps][gpsd]") {
    struct FakeGpsdSource final : IGpsSource {
        bool open() override { return true; }
        void close() override {}
        DeviceError set_output_rate(int) override { return DeviceError::NOT_SUPPORTED; }
        DeviceError hot_restart() override { return DeviceError::NOT_SUPPORTED; }
        DeviceError cold_restart() override { return DeviceError::NOT_SUPPORTED; }
    };

    GpsDevice gps(std::make_unique<FakeGpsdSource>());
    REQUIRE(gps.set_output_rate(5) == DeviceError::NOT_SUPPORTED);
    REQUIRE(gps.hot_restart() == DeviceError::NOT_SUPPORTED);
    REQUIRE(gps.cold_restart() == DeviceError::NOT_SUPPORTED);
}
