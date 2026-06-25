#include <chrono>
#include <limits>

#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/gps_stuck_service.h"

using robot::device::GpsDevice;
using robot::service::GpsStuckService;

namespace {

GpsDevice::GpsData good_gps(uint64_t utc_timestamp_ms) {
    GpsDevice::GpsData gps{};
    gps.latitude = 24.685062;
    gps.longitude = 118.219479;
    gps.altitude_m = 18.5f;
    gps.speed_m_s = 0.004f;
    gps.course_deg = 0.0f;
    gps.hdop = 0.64f;
    gps.satellites_used = 12;
    gps.satellites_in_view = 14;
    gps.fix_quality = 2;
    gps.pdop = 0.0f;
    gps.vdop = 0.0f;
    gps.valid = true;
    gps.utc_timestamp_ms = utc_timestamp_ms;
    return gps;
}

}  // namespace

TEST_CASE("GpsStuckService detects robot stuck after stationary timeout", "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);
    REQUIRE(service.get_status().state == GpsStuckService::State::kMonitoring);

    now += seconds(7);
    latest = good_gps(8000);
    service.update();
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);

    now += seconds(1);
    latest = good_gps(9000);
    service.update();

    const auto status = service.get_status();
    REQUIRE(status.state == GpsStuckService::State::kStuck);
    REQUIRE(status.robot_stuck_detected);
}

TEST_CASE("GpsStuckService refreshes motion timeout after confirmed GPS speed",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();

    now += seconds(7);
    latest = good_gps(8000);
    latest.speed_m_s = 0.20f;
    service.update();
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);

    now += seconds(1);
    latest = good_gps(9000);
    latest.speed_m_s = 0.21f;
    service.update();
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);

    now += seconds(7);
    latest = good_gps(16000);
    service.update();
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);
}

TEST_CASE("GpsStuckService ignores horizontal displacement when GPS speed stays low",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();

    now += seconds(8);
    latest = good_gps(9000);
    latest.latitude += 0.000009;
    service.update();

    const auto status = service.get_status();
    REQUIRE(status.state == GpsStuckService::State::kStuck);
    REQUIRE(status.robot_stuck_detected);
}

TEST_CASE("GpsStuckService does not report robot stuck while GPS quality is poor",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    latest.hdop = 2.0f;
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();
    REQUIRE(service.get_status().state == GpsStuckService::State::kGpsQualityPoor);

    now += seconds(20);
    latest = good_gps(21000);
    latest.hdop = 2.0f;
    service.update();

    REQUIRE(service.get_status().state == GpsStuckService::State::kGpsQualityPoor);
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);
}

TEST_CASE("GpsStuckService treats stale cached GPS samples as waiting for GPS",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();

    now += seconds(4);
    service.update();

    REQUIRE(service.get_status().state == GpsStuckService::State::kWaitingForGps);
    REQUIRE_FALSE(service.get_status().robot_stuck_detected);
}

TEST_CASE("GpsStuckService restarts tracking after stale GPS samples",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();

    now += seconds(4);
    service.update();
    REQUIRE(service.get_status().state == GpsStuckService::State::kWaitingForGps);

    now += seconds(5);
    latest = good_gps(10000);
    service.update();

    const auto status = service.get_status();
    REQUIRE(status.state == GpsStuckService::State::kMonitoring);
    REQUIRE_FALSE(status.robot_stuck_detected);
}

TEST_CASE("GpsStuckService rejects invalid GPS speed and restarts tracking",
          "[service][gps_stuck]") {
    using namespace std::chrono;

    GpsDevice::GpsData latest = good_gps(1000);
    auto now = steady_clock::time_point{};
    GpsStuckService service([&]() { return latest; }, [&]() { return now; });

    service.update();

    now += seconds(7);
    latest = good_gps(8000);
    latest.speed_m_s = std::numeric_limits<float>::quiet_NaN();
    service.update();
    REQUIRE(service.get_status().state == GpsStuckService::State::kWaitingForGps);
    REQUIRE(service.get_status().reason == "invalid_gps_speed");

    now += seconds(2);
    latest = good_gps(10000);
    service.update();

    const auto status = service.get_status();
    REQUIRE(status.state == GpsStuckService::State::kMonitoring);
    REQUIRE_FALSE(status.robot_stuck_detected);
}
