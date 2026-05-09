#include <catch2/catch.hpp>

#include <string_view>

#include "pv_cleaning_robot/service/health_service.h"

TEST_CASE("HealthPayloadBuilder emits diagnostics payload into caller buffer",
          "[service][health][payload]") {
    char out[2048];
    robot::service::HealthPayloadBuilder::DiagnosticsView view{};
    view.ts_iso8601 = "2026-04-26T10:00:00Z";
    view.walk.ctrl_frame_count = 12;
    view.brush.actual_rpm = 800;
    view.gps.fix_quality = 2;

    const size_t len = robot::service::HealthPayloadBuilder::build_diagnostics(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    REQUIRE(std::string_view(out, len).find("\"brush\":{\"rpm\":800") != std::string_view::npos);
    REQUIRE(std::string_view(out, len).find("\"gps\":{\"lat\":0.0") != std::string_view::npos);
}
