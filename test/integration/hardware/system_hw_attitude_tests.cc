#include "system_hw_common.h"

TEST_CASE("下轨道轮低速采样 IMU pitch 峰值", "[hw_system][lower_pitch_peak]") {
    const auto sample = sample_lower_wheel_pitch();
    INFO("samples=" << sample.samples);
    INFO("pitch_min=" << sample.pitch_min);
    INFO("pitch_max=" << sample.pitch_max);
    CHECK(sample.samples > 5);
    CHECK(sample.pitch_max >= sample.pitch_min);
}

TEST_CASE("下轨道轮低速采样姿态评分", "[hw_system][lower_pitch_score]") {
    const auto sample = sample_lower_wheel_pitch();
    const float pitch_span = sample.pitch_max - sample.pitch_min;
    const float score = pitch_span - 0.25f * sample.roll_abs_max;
    INFO("pitch_span=" << pitch_span);
    INFO("roll_abs_max=" << sample.roll_abs_max);
    INFO("score=" << score);
    CHECK(sample.samples > 5);
    CHECK(std::isfinite(score));
}

TEST_CASE("lower_uds_zero 调速随误差缩小并保持方向", "[hw_system][lower_uds_zero][logic]") {
    constexpr float kDeadband = 0.2f;
    constexpr float kMinRpm = 0.8f;
    constexpr float kMaxRpm = 5.0f;
    constexpr float kKp = 1.5f;

    CHECK(compute_lower_uds_zero_cmd(0.1f, kDeadband, kMinRpm, kMaxRpm, kKp) == 0.0f);
    CHECK(compute_lower_uds_zero_cmd(0.4f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(-0.8f));
    CHECK(compute_lower_uds_zero_cmd(-0.4f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(0.8f));
    CHECK(std::abs(compute_lower_uds_zero_cmd(3.0f, kDeadband, kMinRpm, kMaxRpm, kKp)) >
          std::abs(compute_lower_uds_zero_cmd(0.4f, kDeadband, kMinRpm, kMaxRpm, kKp)));
    CHECK(compute_lower_uds_zero_cmd(10.0f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(-5.0f));
}

TEST_CASE("lower_attitude_center 根据触发侧选择下轮方向",
          "[hw_system][lower_attitude_center][logic]") {
    const float rpm = 2.0f;
    const robot::device::AttitudeLimitSwitch::Status none_left{
        robot::device::AttitudeLimitSide::LEFT_LOWER, true, false, false};
    const robot::device::AttitudeLimitSwitch::Status none_right{
        robot::device::AttitudeLimitSide::RIGHT_LOWER, true, false, false};
    const robot::device::AttitudeLimitSwitch::Status active_left{
        robot::device::AttitudeLimitSide::LEFT_LOWER, false, true, false};
    const robot::device::AttitudeLimitSwitch::Status active_right{
        robot::device::AttitudeLimitSide::RIGHT_LOWER, false, true, false};

    CHECK(classify_lower_attitude_state(none_left, none_right) == LowerAttitudeState::NONE);
    CHECK(classify_lower_attitude_state(active_left, none_right) == LowerAttitudeState::LEFT);
    CHECK(classify_lower_attitude_state(none_left, active_right) == LowerAttitudeState::RIGHT);
    CHECK(classify_lower_attitude_state(active_left, active_right) == LowerAttitudeState::BOTH);
    CHECK_FALSE(active_side_from_state(LowerAttitudeState::NONE).has_value());
    CHECK_FALSE(active_side_from_state(LowerAttitudeState::BOTH).has_value());

    const auto left_plan =
        make_lower_attitude_center_plan(robot::device::AttitudeLimitSide::LEFT_LOWER, rpm);
    CHECK(left_plan.initial_lower_rpm == Approx(2.0f));
    CHECK(left_plan.return_lower_rpm == Approx(-2.0f));
    CHECK(left_plan.release_side == robot::device::AttitudeLimitSide::LEFT_LOWER);
    CHECK(left_plan.opposite_side == robot::device::AttitudeLimitSide::RIGHT_LOWER);

    const auto right_plan =
        make_lower_attitude_center_plan(robot::device::AttitudeLimitSide::RIGHT_LOWER, rpm);
    CHECK(right_plan.initial_lower_rpm == Approx(-2.0f));
    CHECK(right_plan.return_lower_rpm == Approx(2.0f));
    CHECK(right_plan.release_side == robot::device::AttitudeLimitSide::RIGHT_LOWER);
    CHECK(right_plan.opposite_side == robot::device::AttitudeLimitSide::LEFT_LOWER);
}

TEST_CASE("上轮 0 速仅调下轮使 UDS yaw 进入死区", "[hw_system][lower_uds_zero]") {
    run_lower_uds_zero_test();
}

TEST_CASE("上轮 0 速通过姿态接近边界回中", "[hw_system][lower_attitude_center]") {
    run_lower_attitude_center_test();
}

TEST_CASE("真实 UDS 与 IMU 融合偏差角连续输出", "[hw_system][uds_gyro_fusion_probe]") {
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
    run_configured_system_chain(
        f, c.tag, kp.combined_passes, true, true, true, motion_cfg);
}

}  // namespace

TEST_CASE("原始 UDS 降速，上下轮共同纠偏", "[hw_system][corr_raw_slow_all]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_all",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 true});
}

TEST_CASE("原始 UDS 降速，只纠偏下轮", "[hw_system][corr_raw_slow_lower_only]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 true});
}

TEST_CASE("原始 UDS 降速，上轮只允许减速", "[hw_system][corr_raw_slow_top_decel_only]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_top_decel_only",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY,
                                 true});
}

TEST_CASE("融合角不降速，上下轮共同纠偏", "[hw_system][corr_fused_fast_all]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_all",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 false});
}

TEST_CASE("融合角不降速，上下轮共同纠偏，滚刷反向",
          "[hw_system][corr_fused_fast_all_brush_reverse]") {
    run_correction_compare_case(
        {"hw_system][corr_fused_fast_all_brush_reverse",
         robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
         robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
         false,
         -1});
}

TEST_CASE("融合角不降速，只纠偏下轮", "[hw_system][corr_fused_fast_lower_only]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 false});
}

TEST_CASE("融合角降速，上下轮共同纠偏", "[hw_system][corr_fused_slow_all]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_all",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 true});
}

TEST_CASE("融合角降速，只纠偏下轮", "[hw_system][corr_fused_slow_lower_only]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 true});
}

TEST_CASE("融合角降速，上轮只允许减速", "[hw_system][corr_fused_slow_top_decel_only]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_top_decel_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY,
                                 true});
}
