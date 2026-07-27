/**
 * @file system_hw_diagnostics_tests.cc
 * @brief 真实硬件诊断与上报系统测试。
 *
 * 本文件验证 DiagnosticsCollector、HealthService 和本地/云端上报在系统级夹具中的输出。
 */
#include "system_hw_common.h"

#include "pv_cleaning_robot/service/diagnostics_collector.h"

TEST_CASE("DiagnosticsCollector 汇聚真实硬件诊断快照",
          "[diag][diag.snapshot]") {
    SystemHwFixture f;
    REQUIRE(f.init());

    auto collector = std::make_shared<robot::service::DiagnosticsCollector>(
        f.walk_group, f.brush, f.bms, f.imu, f.gps, f.gps_stuck);

    bool bms_ready = false;
    bool gps_ready = false;
    bool imu_ready = false;
    bool walk_ready = false;
    robot::service::DiagnosticsCollector::Snapshot snapshot{};

    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while (std::chrono::steady_clock::now() < deadline &&
           (!bms_ready || !gps_ready || !imu_ready || !walk_ready)) {
        f.walk_group->update();
        f.bms->update();
        f.gps_stuck->update();
        collector->update();
        snapshot = collector->snapshot();

        bms_ready = snapshot.bms_diagnostics.update_count > 0;
        gps_ready = snapshot.gps_diagnostics.sentence_count > 0;
        imu_ready = snapshot.imu_diagnostics.frame_count > 0;

        walk_ready = snapshot.error.walk_feedback_expected;
        for (const auto& wheel : snapshot.walk_diagnostics.wheel) {
            walk_ready = walk_ready && wheel.feedback_frame_count > 0;
        }
        std::this_thread::sleep_for(100ms);
    }

    INFO("bms_update_count=" << snapshot.bms_diagnostics.update_count);
    INFO("gps_sentence_count=" << snapshot.gps_diagnostics.sentence_count);
    INFO("imu_frame_count=" << snapshot.imu_diagnostics.frame_count);
    INFO("walk_feedback=["
         << snapshot.walk_diagnostics.wheel[0].feedback_frame_count << ", "
         << snapshot.walk_diagnostics.wheel[1].feedback_frame_count << ", "
         << snapshot.walk_diagnostics.wheel[2].feedback_frame_count << ", "
         << snapshot.walk_diagnostics.wheel[3].feedback_frame_count << "]");

    REQUIRE(bms_ready);
    REQUIRE(gps_ready);
    REQUIRE(imu_ready);
    REQUIRE(walk_ready);

    const auto error_snapshot = collector->error_snapshot();
    CHECK(error_snapshot.bms_update.enabled);
    CHECK(error_snapshot.gps.enabled);
    CHECK(error_snapshot.imu.enabled);
    CHECK(error_snapshot.walk_feedback_expected);
    for (const auto& stream : error_snapshot.walk_feedback) {
        CHECK(stream.enabled);
        CHECK(stream.last_update_ms > 0);
    }
}
