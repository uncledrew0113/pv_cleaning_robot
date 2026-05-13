#include "pv_cleaning_robot/service/heading_corrector.h"

#include <cmath>

namespace robot::service {

HeadingCorrector::HeadingCorrector(const Params& p) : params_(p) {}

void HeadingCorrector::set_params(const Params& p) {
    params_ = p;
}

void HeadingCorrector::enable(bool en) {
    if (enabled_ == en)
        return;

    enabled_ = en;
    reset();
}

void HeadingCorrector::reset() {
    filters_initialized_ = false;
    learned_once_ = false;
    mode_ = Mode::UNINITIALIZED;
    filtered_pitch_ = 0.0f;
    filtered_roll_ = 0.0f;
    filtered_yaw_ = 0.0f;
    filtered_gyro_z_ = 0.0f;
    yaw_initialized_ = false;
    pitch_abs_best_ = 0.0f;
    roll_at_best_ = 0.0f;
    last_pitch_drop_ = 0.0f;
    last_roll_delta_ = 0.0f;
    last_correction_ = 0.0f;
    stable_ms_acc_ = 0.0f;
    learn_ms_acc_ = 0.0f;
    hold_ms_acc_ = 0.0f;
    freeze_stable_ms_ = 0.0f;
    stable_sample_count_ = 0;
}

HeadingCorrector::Output HeadingCorrector::compute(const Input& input) {
    Output output{};
    if (!enabled_)
        return output;

    const float dt_ms = input.dt_s > 0.0f ? input.dt_s * 1000.0f : 0.0f;

    update_filters(input.raw_pitch_deg, input.raw_roll_deg, input.raw_gyro_z_dps);
    if (!yaw_initialized_) {
        filtered_yaw_ = input.raw_yaw_deg;
        yaw_initialized_ = true;
    } else {
        filtered_yaw_ = low_pass(filtered_yaw_, input.raw_yaw_deg, params_.gyro_alpha);
    }

    float pitch_reference = pitch_abs_best_;
    float roll_reference = roll_at_best_;

    refresh_debug_terms(pitch_reference, roll_reference);
    last_correction_ = 0.0f;

    mark_stable_sample(dt_ms);

    switch (mode_) {
        case Mode::UNINITIALIZED:
            if (!stable_window_ready())
                break;
            mode_ = Mode::LEARN;
            learn_ms_acc_ = 0.0f;
            break;

        case Mode::LEARN:
            if (!stable_window_ready())
                break;
            update_best_reference(input.dt_s);
            pitch_reference = pitch_abs_best_;
            roll_reference = roll_at_best_;
            refresh_debug_terms(pitch_reference, roll_reference);
            learn_ms_acc_ += dt_ms;
            if (learned_once_ && learn_ms_acc_ >= static_cast<float>(params_.warmup_ms)) {
                mode_ = Mode::TRACK;
                hold_ms_acc_ = 0.0f;
            }
            break;

        case Mode::TRACK:
            if (!stable_window_ready()) {
                hold_ms_acc_ = 0.0f;
                break;
            }
            update_best_reference(input.dt_s);
            pitch_reference = pitch_abs_best_;
            roll_reference = roll_at_best_;
            refresh_debug_terms(pitch_reference, roll_reference);
            output.feedback_correction_rpm =
                track_correction(input.dt_s, pitch_reference, roll_reference);
            break;

        case Mode::FREEZE:
            mode_ = learned_once_ ? Mode::TRACK : Mode::LEARN;
            hold_ms_acc_ = 0.0f;
            break;
    }

    output.feedforward_correction_rpm = 0.0f;
    output.correction_rpm = output.feedback_correction_rpm + output.feedforward_correction_rpm;
    output.applied_pitch_abs_reference_deg = pitch_abs_best_;
    output.applied_roll_reference_deg = roll_at_best_;
    if (input.has_base_command) {
        output.has_speed_command = true;
        output.speed_command = apply_correction(input.base_command, output.correction_rpm);
    }
    return output;
}

void HeadingCorrector::reset_stability_window() {
    stable_ms_acc_ = 0.0f;
    stable_sample_count_ = 0;
}

void HeadingCorrector::mark_stable_sample(float dt_ms) {
    stable_ms_acc_ += dt_ms;
    ++stable_sample_count_;
}

bool HeadingCorrector::stable_window_ready() const {
    return stable_sample_count_ >= 2 &&
           stable_ms_acc_ >= static_cast<float>(params_.warmup_ms);
}

HeadingCorrector::DebugState HeadingCorrector::debug_state() const {
    return {mode_,
            filtered_pitch_,
            filtered_roll_,
            filtered_yaw_,
            filtered_gyro_z_,
            pitch_abs_best_,
            roll_at_best_,
            last_pitch_drop_,
            last_roll_delta_,
            last_correction_};
}

float HeadingCorrector::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float HeadingCorrector::clamp_alpha(float alpha) {
    return clamp(alpha, 0.0f, 1.0f);
}

float HeadingCorrector::low_pass(float previous, float sample, float alpha) {
    return previous + clamp_alpha(alpha) * (sample - previous);
}

void HeadingCorrector::refresh_debug_terms(float pitch_reference, float roll_reference) {
    if (!learned_once_ && pitch_reference <= 0.0f) {
        last_pitch_drop_ = 0.0f;
        last_roll_delta_ = 0.0f;
        return;
    }

    const float active_pitch_reference = pitch_reference > 0.0f ? pitch_reference : pitch_abs_best_;
    last_pitch_drop_ = active_pitch_reference - std::abs(filtered_pitch_);
    last_roll_delta_ = filtered_roll_ - roll_reference;
}

void HeadingCorrector::update_filters(float pitch_deg, float roll_deg, float gyro_z_dps) {
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

void HeadingCorrector::update_best_reference(float dt_s) {
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

float HeadingCorrector::track_correction(float dt_s, float pitch_reference, float roll_reference) {
    const float pitch_abs = std::abs(filtered_pitch_);
    const float active_pitch_reference = pitch_reference > 0.0f ? pitch_reference : pitch_abs_best_;
    const float pitch_drop = active_pitch_reference - pitch_abs;
    last_pitch_drop_ = pitch_drop;
    if (pitch_drop <= params_.pitch_drop_threshold) {
        hold_ms_acc_ = 0.0f;
        last_correction_ = 0.0f;
        return 0.0f;
    }

    hold_ms_acc_ += dt_s > 0.0f ? dt_s * 1000.0f : 0.0f;
    if (hold_ms_acc_ < static_cast<float>(params_.hold_ms)) {
        last_correction_ = 0.0f;
        return 0.0f;
    }

    const float roll_delta = filtered_roll_ - roll_reference;
    last_roll_delta_ = roll_delta;
    if (std::abs(roll_delta) <= params_.roll_threshold) {
        last_correction_ = 0.0f;
        return 0.0f;
    }

    float output = clamp(pitch_drop, 0.0f, params_.max_output);
    if (output > 0.0f && output < params_.min_effective_output) {
        output = params_.min_effective_output;
    }

    last_correction_ = roll_delta > 0.0f ? output : -output;
    return last_correction_;
}

HeadingCorrector::SpeedCommand HeadingCorrector::apply_correction(const SpeedCommand& base,
                                                                  float correction_rpm) {
    SpeedCommand corrected = base;
    corrected.lt_rpm = clamp(base.lt_rpm + correction_rpm, -210.0f, 210.0f);
    corrected.rt_rpm = clamp(base.rt_rpm + correction_rpm, -210.0f, 210.0f);
    corrected.lb_rpm = clamp(base.lb_rpm + correction_rpm, -210.0f, 210.0f);
    corrected.rb_rpm = clamp(base.rb_rpm + correction_rpm, -210.0f, 210.0f);
    return corrected;
}

}  // namespace robot::service
