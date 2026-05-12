#include <catch2/catch.hpp>

#include <cmath>
#include <tuple>
#include <vector>

#include "pv_cleaning_robot/service/heading_pid_controller.h"

using robot::service::HeadingPidController;

namespace {

HeadingPidController::Params test_params() {
    HeadingPidController::Params p;
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

TEST_CASE("HeadingPidController: disabled controller outputs zero", "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());

    REQUIRE(ctrl.compute(-34.8f, -2.0f, 0.0f, 0.02f) == Approx(0.0f));
}

TEST_CASE("HeadingPidController: warmup needs a stable window, not one long sample",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    REQUIRE(ctrl.compute(-34.8f, -2.0f, 0.0f, 0.10f) == Approx(0.0f));

    const auto state = ctrl.debug_state();
    REQUIRE(state.mode == HeadingPidController::Mode::UNINITIALIZED);
    REQUIRE(state.pitch_abs_best == Approx(0.0f));
}

TEST_CASE("HeadingPidController: learns local pitch best before tracking",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 5; ++i) {
        REQUIRE(ctrl.compute(-34.2f, -4.5f, 0.0f, 0.02f) == Approx(0.0f));
    }

    for (int i = 0; i < 5; ++i) {
        REQUIRE(ctrl.compute(-34.8f, -2.0f, 0.0f, 0.02f) == Approx(0.0f));
    }

    const auto state = ctrl.debug_state();
    REQUIRE(state.mode == HeadingPidController::Mode::TRACK);
    REQUIRE(state.pitch_abs_best == Approx(34.8f).margin(0.05f));
    REQUIRE(state.roll_at_best == Approx(-2.0f).margin(0.1f));
}

TEST_CASE("HeadingPidController: right-biased sample commands negative correction toward center",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 10; ++i) {
        ctrl.compute(-34.83f, -1.95f, 0.0f, 0.02f);
    }

    float correction = 0.0f;
    for (int i = 0; i < 4; ++i) {
        correction = ctrl.compute(-34.17f, -5.98f, 0.0f, 0.02f);
    }

    REQUIRE(correction < 0.0f);

}

TEST_CASE("HeadingPidController: best reference waits for a new stable window after freeze",
          "[service][heading_pid]") {
    auto params = test_params();
    params.warmup_ms = 80;
    params.freeze_release_ms = 60;

    HeadingPidController ctrl(params);
    ctrl.enable(true);

    for (int i = 0; i < 12; ++i) {
        ctrl.compute(-34.8f, -2.0f, 0.0f, 0.02f);
    }

    const auto learned = ctrl.debug_state();
    REQUIRE(learned.mode == HeadingPidController::Mode::TRACK);
    REQUIRE(learned.pitch_abs_best == Approx(34.8f).margin(0.05f));

    REQUIRE(ctrl.compute(-34.5f, 0.0f, 80.0f, 0.02f) == Approx(0.0f));
    REQUIRE(ctrl.debug_state().mode == HeadingPidController::Mode::FREEZE);

    for (int i = 0; i < 3; ++i) {
        REQUIRE(ctrl.compute(-35.2f, -1.0f, 0.0f, 0.02f) == Approx(0.0f));
    }

    const auto still_frozen = ctrl.debug_state();
    REQUIRE(still_frozen.mode == HeadingPidController::Mode::TRACK);
    REQUIRE(still_frozen.pitch_abs_best == Approx(34.8f).margin(0.05f));
    REQUIRE(still_frozen.roll_at_best == Approx(-2.0f).margin(0.1f));

    REQUIRE(ctrl.compute(-35.2f, -1.0f, 0.0f, 0.02f) == Approx(0.0f));
    const auto refreshed = ctrl.debug_state();
    REQUIRE(refreshed.pitch_abs_best == Approx(35.2f).margin(0.05f));
    REQUIRE(refreshed.roll_at_best == Approx(-1.0f).margin(0.1f));
}

TEST_CASE("HeadingPidController: disturbance enters freeze and suppresses output",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 10; ++i) {
        ctrl.compute(-34.83f, -1.95f, 0.0f, 0.02f);
    }

    const float correction = ctrl.compute(-34.5f, 0.0f, 80.0f, 0.02f);
    REQUIRE(correction == Approx(0.0f));
    REQUIRE(ctrl.debug_state().mode == HeadingPidController::Mode::FREEZE);
}

TEST_CASE("HeadingPidController: replayed IMU sequence tracks a local pitch maximum",
          "[service][heading_pid][replay]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    const std::vector<std::tuple<float, float, float>> samples = {
        {-34.17f, -5.98f, 7.81f},
        {-34.22f, -5.67f, 7.42f},
        {-34.50f, -3.00f, 1.58f},
        {-34.83f, -1.95f, -0.49f},
        {-34.74f, -1.16f, -1.12f},
        {-34.66f, -0.73f, -3.59f},
        {-34.54f, 0.65f, -6.23f},
        {-34.58f, 1.44f, -7.36f},
    };

    float correction = 0.0f;
    for (const auto& [pitch, roll, gyro_z] : samples) {
        correction = ctrl.compute(pitch, roll, gyro_z, 0.02f);
    }

    const auto state = ctrl.debug_state();
    REQUIRE(state.pitch_abs_best >= Approx(34.7f));
    REQUIRE(state.mode != HeadingPidController::Mode::UNINITIALIZED);
    REQUIRE(std::isfinite(correction));
}
