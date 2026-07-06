/**
 * @file gpsd_hw_test.cc
 * @brief gpsd 数据源真实硬件集成测试。
 *
 * 本文件验证 GPSD 连接、WATCH 配置和定位数据解析链路。
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

static const hw::HwParams kp = hw::load_hw_test_config();

using namespace robot;
using namespace std::chrono_literals;

TEST_CASE("集成测试 - GPSD TCP 服务可获得真实有效定位", "[hw_gpsd][fix]") {
    device::GpsdSourceConfig cfg;
    cfg.host = kp.gpsd_host;
    cfg.port = kp.gpsd_port;
    cfg.watch = kp.gpsd_watch;

    auto gps = device::GpsDevice::create_gpsd(cfg);
    REQUIRE(gps != nullptr);
    REQUIRE(gps->open());

    INFO("已连接 gpsd TCP 服务，等待有效 GPS fix...");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(kp.gpsd_fix_timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        if (gps->get_latest().valid)
            break;
        std::this_thread::sleep_for(200ms);
    }

    const auto diag = gps->get_diagnostics();
    const auto data = gps->get_latest();
    gps->close();

    spdlog::info(
        "[hw_gpsd][fix] valid={} lat={:.7f} lon={:.7f} alt={:.2f}m speed={:.3f}m/s "
        "course={:.2f} hdop={:.2f} fix={} utc_ms={} sentences={} parse_err={}",
        data.valid,
        data.latitude,
        data.longitude,
        data.altitude_m,
        data.speed_m_s,
        data.course_deg,
        data.hdop,
        static_cast<int>(data.fix_quality),
        static_cast<unsigned long long>(data.utc_timestamp_ms),
        diag.sentence_count,
        diag.parse_error_count);

    REQUIRE(diag.sentence_count > 0u);
    REQUIRE(data.valid);
}

TEST_CASE("集成测试 - GPSD TCP 服务连续读取并打印真实数据", "[hw_gpsd][stream]") {
    device::GpsdSourceConfig cfg;
    cfg.host = kp.gpsd_host;
    cfg.port = kp.gpsd_port;
    cfg.watch = kp.gpsd_watch;

    auto gps = device::GpsDevice::create_gpsd(cfg);
    REQUIRE(gps != nullptr);
    REQUIRE(gps->open());

    const auto start = std::chrono::steady_clock::now();
    auto next_print = start;
    constexpr auto kDuration = 60s;
    constexpr auto kPrintInterval = 1s;

    while (std::chrono::steady_clock::now() - start < kDuration) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_print) {
            const auto data = gps->get_latest();
            const auto diag = gps->get_diagnostics();
            spdlog::info(
                "[hw_gpsd][stream] valid={} lat={:.7f} lon={:.7f} alt={:.2f} speed={:.3f} "
                "course={:.2f} hdop={:.2f} sats_used={} sats_view={} fix={} sentences={} err={}",
                data.valid,
                data.latitude,
                data.longitude,
                data.altitude_m,
                data.speed_m_s,
                data.course_deg,
                data.hdop,
                static_cast<int>(data.satellites_used),
                static_cast<int>(data.satellites_in_view),
                static_cast<int>(data.fix_quality),
                diag.sentence_count,
                diag.parse_error_count);
            next_print += kPrintInterval;
        }
        std::this_thread::sleep_for(100ms);
    }

    const auto diag = gps->get_diagnostics();
    gps->close();

    REQUIRE(diag.sentence_count > 0u);
}
