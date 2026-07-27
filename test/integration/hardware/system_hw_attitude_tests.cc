/**
 * @file system_hw_attitude_tests.cc
 * @brief 真实硬件姿态限位与回中恢复测试。
 *
 * 本文件验证姿态限位触发、下轮回中、上轮保持 0 速和恢复后任务继续等实机行为。
 * 运行测试前必须确认机器人周围安全，并准备人工急停手段。
 */
#include "system_hw_common.h"

TEST_CASE("上轮 0 速通过姿态接近边界回中",
          "[attitude][attitude.center][manual][long]") {
    run_lower_attitude_center_test();
}

TEST_CASE("真实 UDS 与 IMU 融合偏差角连续输出", "[heading][heading.fusion]") {
    run_uds_gyro_fusion_probe_test();
}

namespace {

struct CorrectionCompareCase {
    const char* tag;
    robot::service::HeadingCorrector::AngleSource angle_source;
    robot::service::HeadingCorrector::WheelStrategy wheel_strategy;
    bool slow_on_error;
    int brush_direction_sign{1};
};

void run_correction_compare_case(const CorrectionCompareCase& c) {
    spdlog::warn(
        "[{}] correction_compare slow_base={:.1f} threshold={:.2f} kp={:.2f} ki={:.2f} "
        "kd={:.2f} max_output={:.1f} fusion(q_angle={:.4f} q_bias={:.4f} r_uds={:.4f} "
        "gyro_only_ms={})",
        c.tag,
        kp.correction_compare.slow_base_rpm,
        kp.correction_compare.yaw_slow_threshold_deg,
        kp.correction_compare.kp,
        kp.correction_compare.ki,
        kp.correction_compare.kd,
        kp.correction_compare.max_output,
        kp.correction_compare.fusion.process_noise_angle,
        kp.correction_compare.fusion.process_noise_bias,
        kp.correction_compare.fusion.measurement_noise_uds,
        kp.correction_compare.fusion.max_gyro_only_ms);

    SystemHwFixture f;
    auto motion_cfg =
        make_correction_compare_motion_config(c.angle_source, c.wheel_strategy, c.slow_on_error);
    motion_cfg.brush_direction_sign = c.brush_direction_sign;
    run_configured_system_chain(f, c.tag, kp.combined_passes, true, true, true, motion_cfg);
}

}  // namespace

TEST_CASE("原始 UDS 降速，只纠偏下轮",
          "[heading][heading.raw-lower][manual][long]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 true});
}

TEST_CASE("融合角不降速，上下轮共同纠偏",
          "[heading][heading.fused-all][manual][long]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_all",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 false});
}

TEST_CASE("融合角不降速，上下轮共同纠偏，滚刷反向",
          "[heading][heading.fused-all-brush-reverse][manual][long]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_all_brush_reverse",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 false,
                                 -1});
}

TEST_CASE("融合角降速，上轮只允许减速",
          "[heading][heading.fused-top-decel][manual][long]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_top_decel_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY,
                                 true});
}
