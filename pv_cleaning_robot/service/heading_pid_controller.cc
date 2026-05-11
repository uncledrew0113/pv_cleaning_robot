#include "pv_cleaning_robot/service/heading_pid_controller.h"

#include <cmath>

namespace robot::service {

HeadingPidController::HeadingPidController(const Params& p) : params_(p) {}

void HeadingPidController::set_params(const Params& p) {
    params_ = p;
}

void HeadingPidController::enable(bool en) {
    if (enabled_ == en)
        return;

    enabled_ = en;
    reset();
}

void HeadingPidController::reset() {
    filters_initialized_ = false;
    learned_once_ = false;
    mode_ = Mode::UNINITIALIZED;
    filtered_pitch_ = 0.0f;
    filtered_roll_ = 0.0f;
    filtered_gyro_z_ = 0.0f;
    pitch_abs_best_ = 0.0f;
    roll_at_best_ = 0.0f;
    stable_ms_acc_ = 0.0f;
    learn_ms_acc_ = 0.0f;
    hold_ms_acc_ = 0.0f;
    freeze_stable_ms_ = 0.0f;
    stable_sample_count_ = 0;
}

float HeadingPidController::compute(float pitch_deg,
                                    float roll_deg,
                                    float gyro_z_dps,
                                    float dt_s) {
    if (!enabled_)
        return 0.0f;

    const float dt_ms = dt_s > 0.0f ? dt_s * 1000.0f : 0.0f;
    const float prev_pitch = filtered_pitch_;
    const float prev_roll = filtered_roll_;
    const bool had_filters = filters_initialized_;

    update_filters(pitch_deg, roll_deg, gyro_z_dps);

    const float pitch_rate_dps =
        (had_filters && dt_s > 0.0f) ? std::abs(filtered_pitch_ - prev_pitch) / dt_s : 0.0f;
    const float roll_rate_dps =
        (had_filters && dt_s > 0.0f) ? std::abs(filtered_roll_ - prev_roll) / dt_s : 0.0f;
    const bool disturbed =
        std::abs(filtered_gyro_z_) > params_.freeze_gyro_z_threshold ||
        pitch_rate_dps > params_.freeze_pitch_rate_threshold ||
        roll_rate_dps > params_.freeze_roll_rate_threshold;

    if (disturbed) {
        reset_stability_window();
    } else {
        mark_stable_sample(dt_ms);
    }

    if (mode_ == Mode::FREEZE) {
        if (disturbed) {
            freeze_stable_ms_ = 0.0f;
        } else {
            freeze_stable_ms_ += dt_ms;
            if (freeze_stable_ms_ >= static_cast<float>(params_.freeze_release_ms)) {
                mode_ = learned_once_ ? Mode::TRACK : Mode::LEARN;
                hold_ms_acc_ = 0.0f;
            }
        }
        return 0.0f;
    }

    if (disturbed) {
        mode_ = Mode::FREEZE;
        freeze_stable_ms_ = 0.0f;
        learn_ms_acc_ = 0.0f;
        hold_ms_acc_ = 0.0f;
        return 0.0f;
    }

    switch (mode_) {
        case Mode::UNINITIALIZED:
            if (!stable_window_ready())
                return 0.0f;
            mode_ = Mode::LEARN;
            learn_ms_acc_ = 0.0f;
            return 0.0f;

        case Mode::LEARN:
            if (!stable_window_ready())
                return 0.0f;
            update_best_reference(dt_s);
            learn_ms_acc_ += dt_ms;
            if (learned_once_ && learn_ms_acc_ >= static_cast<float>(params_.warmup_ms)) {
                mode_ = Mode::TRACK;
                hold_ms_acc_ = 0.0f;
            }
            return 0.0f;

        case Mode::TRACK:
            if (!stable_window_ready()) {
                hold_ms_acc_ = 0.0f;
                return 0.0f;
            }
            update_best_reference(dt_s);
            return track_correction(dt_s);

        case Mode::FREEZE:
            return 0.0f;
    }

    return 0.0f;
}

void HeadingPidController::reset_stability_window() {
    stable_ms_acc_ = 0.0f;
    stable_sample_count_ = 0;
}

void HeadingPidController::mark_stable_sample(float dt_ms) {
    stable_ms_acc_ += dt_ms;
    ++stable_sample_count_;
}

bool HeadingPidController::stable_window_ready() const {
    return stable_sample_count_ >= 2 &&
           stable_ms_acc_ >= static_cast<float>(params_.warmup_ms);
}

HeadingPidController::DebugState HeadingPidController::debug_state() const {
    return {mode_, filtered_pitch_, filtered_roll_, filtered_gyro_z_, pitch_abs_best_,
            roll_at_best_};
}

float HeadingPidController::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float HeadingPidController::clamp_alpha(float alpha) {
    return clamp(alpha, 0.0f, 1.0f);
}

float HeadingPidController::low_pass(float previous, float sample, float alpha) {
    return previous + clamp_alpha(alpha) * (sample - previous);
}

void HeadingPidController::update_filters(float pitch_deg, float roll_deg, float gyro_z_dps) {
    if (!filters_initialized_) {
        filtered_pitch_ = pitch_deg;
        filtered_roll_ = roll_deg;
        filtered_gyro_z_ = gyro_z_dps;
        filters_initialized_ = true;
        return;
    }

    filtered_pitch_ = low_pass(filtered_pitch_, pitch_deg, params_.pitch_alpha);
    filtered_roll_ = low_pass(filtered_roll_, roll_deg, params_.roll_alpha);
    filtered_gyro_z_ = low_pass(filtered_gyro_z_, gyro_z_dps, params_.gyro_alpha);
}

void HeadingPidController::update_best_reference(float dt_s) {
    if (learned_once_ && params_.best_decay_per_s > 0.0f && dt_s > 0.0f) {
        pitch_abs_best_ = std::max(0.0f, pitch_abs_best_ - params_.best_decay_per_s * dt_s);
    }

    const float pitch_abs = std::abs(filtered_pitch_);
    if (!learned_once_ || pitch_abs > pitch_abs_best_ + params_.learn_improve_eps) {
        pitch_abs_best_ = pitch_abs;
        roll_at_best_ = filtered_roll_;
        learned_once_ = true;
    }
}

float HeadingPidController::track_correction(float dt_s) {
    (void)dt_s;

    const float pitch_abs = std::abs(filtered_pitch_);
    const float pitch_drop = pitch_abs_best_ - pitch_abs;
    if (pitch_drop <= params_.pitch_drop_threshold) {
        hold_ms_acc_ = 0.0f;
        return 0.0f;
    }

    hold_ms_acc_ += dt_s > 0.0f ? dt_s * 1000.0f : 0.0f;
    if (hold_ms_acc_ < static_cast<float>(params_.hold_ms))
        return 0.0f;

    const float roll_delta = filtered_roll_ - roll_at_best_;
    if (std::abs(roll_delta) <= params_.roll_threshold)
        return 0.0f;

    float output = clamp(pitch_drop, 0.0f, params_.max_output);
    if (output > 0.0f && output < params_.min_effective_output) {
        output = params_.min_effective_output;
    }

    // 现场标定数据表明：
    //   右偏  -> roll 比 roll_at_best 更小（roll_delta < 0），应施加负修正
    //   左偏  -> roll 比 roll_at_best 更大（roll_delta > 0），应施加正修正
    return roll_delta > 0.0f ? output : -output;
}

}  // namespace robot::service
