#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/diagnostics_collector.h"

using robot::service::DiagnosticsCollector;

TEST_CASE("DiagnosticsCollector updates stream timestamp only when source count changes",
          "[service][diagnostics_collector]") {
    DiagnosticsCollector collector;
    DiagnosticsCollector::Input input{};
    input.gps_enabled = true;
    input.gps_sentence_count = 1;
    input.gps_diagnostics.sentence_count = 1;
    input.gps_diagnostics.latitude = 31.25;

    auto snapshot = collector.update_from_input(input, 1000);
    CHECK(snapshot.ts_ms == 1000);
    CHECK(snapshot.error.gps.enabled);
    CHECK(snapshot.error.gps.last_update_ms == 1000);
    CHECK(snapshot.gps_diagnostics.sentence_count == 1);
    CHECK(snapshot.gps_diagnostics.latitude == 31.25);

    snapshot = collector.update_from_input(input, 2000);
    CHECK(snapshot.error.gps.last_update_ms == 1000);

    input.gps_sentence_count = 2;
    input.gps_diagnostics.sentence_count = 2;
    snapshot = collector.update_from_input(input, 2500);
    CHECK(snapshot.error.gps.last_update_ms == 2500);
}

TEST_CASE("DiagnosticsCollector keeps steady and epoch timestamps separately",
          "[service][diagnostics_collector]") {
    DiagnosticsCollector collector;
    DiagnosticsCollector::Input input{};

    const auto snapshot = collector.update_from_input(input, 1234, 1714202400789ULL);

    CHECK(snapshot.ts_ms == 1234);
    CHECK(snapshot.epoch_ms == 1714202400789ULL);
}

TEST_CASE("DiagnosticsCollector carries error counters and walk feedback timestamps",
          "[service][diagnostics_collector]") {
    DiagnosticsCollector collector;
    DiagnosticsCollector::Input input{};
    input.bms_update_count = 7;
    input.brush_comm_error_count = 9;
    input.bms_diagnostics.update_count = 7;
    input.brush_diagnostics.comm_error_count = 9;
    input.brush_diagnostics.fault_code = 0x12;
    input.walk_feedback_expected = true;
    input.walk_feedback_enabled = {true, true, false, true};
    input.walk_feedback_frame_count = {1, 2, 0, 4};
    input.walk_diagnostics.wheel[0].feedback_frame_count = 1;
    input.walk_diagnostics.wheel[1].feedback_frame_count = 2;
    input.walk_diagnostics.wheel[3].feedback_frame_count = 4;

    auto snapshot = collector.update_from_input(input, 3000);
    CHECK(snapshot.error.bms_update.enabled);
    CHECK(snapshot.error.bms_update.last_update_ms == 3000);
    CHECK(snapshot.error.brush.error_count == 9);
    CHECK(snapshot.error.brush_fault_active);
    CHECK(snapshot.walk_diagnostics.wheel[0].feedback_frame_count == 1);
    CHECK(snapshot.bms_diagnostics.update_count == 7);
    CHECK(snapshot.brush_diagnostics.comm_error_count == 9);
    CHECK(snapshot.error.walk_feedback_expected);
    CHECK(snapshot.error.walk_feedback[0].last_update_ms == 3000);
    CHECK(snapshot.error.walk_feedback[1].last_update_ms == 3000);
    CHECK_FALSE(snapshot.error.walk_feedback[2].enabled);
    CHECK(snapshot.error.walk_feedback[3].last_update_ms == 3000);

    snapshot = collector.update_from_input(input, 3500);
    CHECK(snapshot.error.bms_update.last_update_ms == 3000);
    CHECK(snapshot.error.walk_feedback[0].last_update_ms == 3000);

    input.bms_update_count = 8;
    input.bms_diagnostics.update_count = 8;
    input.walk_feedback_frame_count[0] = 2;
    input.walk_diagnostics.wheel[0].feedback_frame_count = 2;
    snapshot = collector.update_from_input(input, 4000);
    CHECK(snapshot.error.bms_update.last_update_ms == 4000);
    CHECK(snapshot.error.walk_feedback[0].last_update_ms == 4000);
}

TEST_CASE("DiagnosticsCollector stores latest full and error snapshots",
          "[service][diagnostics_collector]") {
    DiagnosticsCollector collector;
    DiagnosticsCollector::Input input{};
    input.brush_status.actual_rpm = 500;
    input.gps_enabled = true;
    input.gps_sentence_count = 1;

    collector.update_from_input(input, 1000);

    auto full = collector.snapshot();
    auto error = collector.error_snapshot();
    CHECK(full.ts_ms == 1000);
    CHECK(full.brush_status.actual_rpm == 500);
    CHECK(error.gps.last_update_ms == 1000);
}
