#include <catch2/catch.hpp>

#include <cmath>

#include "pv_cleaning_robot/service/heading_corrector.h"

using robot::service::HeadingCorrector;

namespace {

HeadingCorrector::Params test_params() {
    HeadingCorrector::Params p;
    p.pitch_alpha = 1.0f;
    p.roll_alpha = 1.0f;
    p.gyro_alpha = 1.0f;
    p.pitch_drop_threshold = 0.10f;
    p.roll_threshold = 0.50f;
    p.learn_improve_eps = 0.02f;
    p.best_decay_per_s = 0.0f;
    p.freeze_gyro_z_threshold = 30.0f;
    p.freeze_pitch_rate_threshold = 20.0f;
    p.freeze_roll_rate_threshold = 20.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.warmup_ms = 40;
    p.hold_ms = 40;
    p.freeze_release_ms = 60;
    return p;
}

}  // namespace

TEST_CASE("HeadingCorrector: raw samples are accepted directly", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.raw_yaw_deg = -8.0f;
    input.raw_gyro_z_dps = 0.5f;
    input.dt_s = 0.02f;

    const auto out = ctrl.compute(input);
    REQUIRE_FALSE(out.has_speed_command);
}

TEST_CASE("HeadingCorrector: wheel feedback remains part of raw input", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.raw_yaw_deg = -8.0f;
    input.raw_gyro_z_dps = 0.5f;
    input.dt_s = 0.02f;
    input.wheel_feedback.valid = true;
    input.wheel_feedback.rpm = {25.0f, 24.0f, -25.0f, -24.0f};

    REQUIRE(ctrl.compute(input).correction_rpm == Approx(0.0f));
}

TEST_CASE("HeadingCorrector: disabled controller outputs zero", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.dt_s = 0.02f;

    const auto out = ctrl.compute(input);
    REQUIRE(out.correction_rpm == Approx(0.0f));
    REQUIRE_FALSE(out.has_speed_command);
}

TEST_CASE("HeadingCorrector: warmup needs a stable window, not one long sample",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.dt_s = 0.10f;
    REQUIRE(ctrl.compute(input).correction_rpm == Approx(0.0f));

    const auto state = ctrl.debug_state();
    REQUIRE(state.mode == HeadingCorrector::Mode::UNINITIALIZED);
    REQUIRE(state.pitch_abs_best == Approx(0.0f));
}

TEST_CASE("HeadingCorrector: learns local pitch best before tracking", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.dt_s = 0.02f;

    for (int i = 0; i < 5; ++i) {
        input.raw_pitch_deg = -34.2f;
        input.raw_roll_deg = -4.5f;
        REQUIRE(ctrl.compute(input).correction_rpm == Approx(0.0f));
    }

    for (int i = 0; i < 5; ++i) {
        input.raw_pitch_deg = -34.8f;
        input.raw_roll_deg = -2.0f;
        REQUIRE(ctrl.compute(input).correction_rpm == Approx(0.0f));
    }

    const auto state = ctrl.debug_state();
    REQUIRE(state.mode == HeadingCorrector::Mode::TRACK);
    REQUIRE(state.pitch_abs_best == Approx(34.8f).margin(0.05f));
    REQUIRE(state.roll_at_best == Approx(-2.0f).margin(0.1f));
}

TEST_CASE("HeadingCorrector: right-biased sample commands negative correction toward center",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.base_command = {100.0f, 100.0f, -100.0f, -100.0f};

    for (int i = 0; i < 10; ++i) {
        input.raw_pitch_deg = -34.83f;
        input.raw_roll_deg = -1.95f;
        ctrl.compute(input);
    }

    HeadingCorrector::Output output;
    for (int i = 0; i < 4; ++i) {
        input.raw_pitch_deg = -34.17f;
        input.raw_roll_deg = -5.98f;
        output = ctrl.compute(input);
    }

    REQUIRE(output.correction_rpm < 0.0f);
    REQUIRE(output.has_speed_command);
    REQUIRE(output.speed_command.lt_rpm < 100.0f);
    REQUIRE(output.speed_command.rt_rpm < 100.0f);
    REQUIRE(output.speed_command.lb_rpm < -100.0f);
    REQUIRE(output.speed_command.rb_rpm < -100.0f);
}

TEST_CASE("HeadingCorrector: debug state reflects controller-owned filtering",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -35.0f;
    input.raw_roll_deg = -1.5f;
    input.raw_yaw_deg = -7.0f;
    input.raw_gyro_z_dps = 1.0f;
    input.dt_s = 0.02f;

    ctrl.compute(input);
    const auto state = ctrl.debug_state();
    REQUIRE(state.filtered_pitch == Approx(-35.0f));
    REQUIRE(state.filtered_roll == Approx(-1.5f));
    REQUIRE(state.filtered_yaw == Approx(-7.0f));
    REQUIRE(state.filtered_gyro_z == Approx(1.0f));
}

TEST_CASE("HeadingCorrector: final wheel target generation stays inside controller",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.base_command = {100.0f, 100.0f, -100.0f, -100.0f};

    for (int i = 0; i < 10; ++i) {
        input.raw_pitch_deg = -34.83f;
        input.raw_roll_deg = -1.95f;
        ctrl.compute(input);
    }

    input.raw_pitch_deg = -34.17f;
    input.raw_roll_deg = -5.98f;

    const auto output = ctrl.compute(input);
    REQUIRE(output.has_speed_command);
    REQUIRE(std::isfinite(output.speed_command.lt_rpm));
    REQUIRE(std::isfinite(output.speed_command.rt_rpm));
    REQUIRE(std::isfinite(output.speed_command.lb_rpm));
    REQUIRE(std::isfinite(output.speed_command.rb_rpm));
}
