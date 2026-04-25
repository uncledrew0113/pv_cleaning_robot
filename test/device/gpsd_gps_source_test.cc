#include <catch2/catch.hpp>

#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/gps_source.h"

using robot::device::DeviceError;
using robot::device::GpsDevice;
using robot::device::GpsdGpsSource;
using robot::device::GpsdSourceConfig;
using robot::device::IGpsSource;
using robot::protocol::GpsData;

TEST_CASE("GpsdGpsSource: TPV maps mode 3 to valid fix", "[device][gps][gpsd]") {
    GpsData latest{};
    GpsdGpsSource src(
        GpsdSourceConfig{},
        [&](const GpsData& d) { latest = d; },
        []() {},
        []() {});

    src.ingest_json_line_for_test(
        R"({"class":"TPV","lat":30.5,"lon":114.2,"speed":1.2,"track":91.0,"mode":3})");

    REQUIRE(latest.valid);
    REQUIRE(latest.fix_quality == 2);
    REQUIRE(latest.latitude == Approx(30.5));
    REQUIRE(latest.longitude == Approx(114.2));
    REQUIRE(latest.speed_m_s == Approx(1.2f));
    REQUIRE(latest.course_deg == Approx(91.0f));
}

TEST_CASE("GpsdGpsSource: SKY updates satellite stats without clearing TPV", "[device][gps][gpsd]") {
    GpsData latest{};
    GpsdGpsSource src(
        GpsdSourceConfig{},
        [&](const GpsData& d) { latest = d; },
        []() {},
        []() {});

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":2})");
    src.ingest_json_line_for_test(
        R"({"class":"SKY","hdop":0.8,"satellites":[{"used":true},{"used":false},{"used":true}]})");

    REQUIRE(latest.latitude == Approx(30.5));
    REQUIRE(latest.longitude == Approx(114.2));
    REQUIRE(latest.hdop == Approx(0.8f));
    REQUIRE(latest.satellites_used == 2);
    REQUIRE(latest.satellites_in_view == 3);
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
