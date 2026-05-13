#pragma once

/// @file heading_corrector.h
/// @brief 基于 IMU 姿态的方向纠偏控制器。

#include <array>
#include <cstdint>

namespace robot::service {

class HeadingCorrector {
   public:
    struct Params {
        float pitch_alpha{0.2f};
        float roll_alpha{0.2f};
        float gyro_alpha{0.2f};
        float pitch_drop_threshold{0.12f};
        float roll_threshold{0.6f};
        float learn_improve_eps{0.03f};
        float best_decay_per_s{0.01f};
        float freeze_gyro_z_threshold{30.0f};
        float freeze_pitch_rate_threshold{20.0f};
        float freeze_roll_rate_threshold{20.0f};
        float max_output{30.0f};
        float min_effective_output{0.0f};
        int warmup_ms{400};
        int hold_ms{400};
        int freeze_release_ms{300};
    };

    enum class Mode : uint8_t {
        UNINITIALIZED = 0,
        LEARN,
        TRACK,
        FREEZE,
    };

    struct WheelFeedback {
        bool valid{false};
        std::array<float, 4> rpm{};
        std::array<float, 4> current{};
    };

    struct SpeedCommand {
        float lt_rpm{0.0f};
        float rt_rpm{0.0f};
        float lb_rpm{0.0f};
        float rb_rpm{0.0f};
    };

    struct Input {
        float raw_pitch_deg{0.0f};
        float raw_roll_deg{0.0f};
        float raw_yaw_deg{0.0f};
        float raw_gyro_z_dps{0.0f};
        float dt_s{0.0f};
        WheelFeedback wheel_feedback{};
        bool has_base_command{false};
        SpeedCommand base_command{};
    };

    struct Output {
        float feedback_correction_rpm{0.0f};
        float feedforward_correction_rpm{0.0f};
        float correction_rpm{0.0f};
        float applied_pitch_abs_reference_deg{0.0f};
        float applied_roll_reference_deg{0.0f};
        bool has_speed_command{false};
        SpeedCommand speed_command{};
    };

    struct DebugState {
        Mode mode{Mode::UNINITIALIZED};
        float filtered_pitch{0.0f};
        float filtered_roll{0.0f};
        float filtered_yaw{0.0f};
        float filtered_gyro_z{0.0f};
        float pitch_abs_best{0.0f};
        float roll_at_best{0.0f};
        float pitch_drop{0.0f};
        float roll_delta{0.0f};
        float last_correction{0.0f};
    };

    HeadingCorrector() = default;
    explicit HeadingCorrector(const Params& p);

    void set_params(const Params& p);
    void enable(bool en);
    void reset();

    bool is_enabled() const {
        return enabled_;
    }

    Output compute(const Input& input);
    DebugState debug_state() const;

   private:
    static float clamp(float v, float lo, float hi);
    static float clamp_alpha(float alpha);
    static float low_pass(float previous, float sample, float alpha);

    void refresh_debug_terms(float pitch_reference, float roll_reference);
    void reset_stability_window();
    void mark_stable_sample(float dt_ms);
    bool stable_window_ready() const;
    void update_filters(float pitch_deg, float roll_deg, float gyro_z_dps);
    void update_best_reference(float dt_s);
    float track_correction(float dt_s, float pitch_reference, float roll_reference);
    static SpeedCommand apply_correction(const SpeedCommand& base, float correction_rpm);

    Params params_{};
    bool enabled_{false};
    bool filters_initialized_{false};
    bool learned_once_{false};
    Mode mode_{Mode::UNINITIALIZED};

    float filtered_pitch_{0.0f};
    float filtered_roll_{0.0f};
    float filtered_yaw_{0.0f};
    float filtered_gyro_z_{0.0f};
    bool yaw_initialized_{false};
    float pitch_abs_best_{0.0f};
    float roll_at_best_{0.0f};
    float last_pitch_drop_{0.0f};
    float last_roll_delta_{0.0f};
    float last_correction_{0.0f};

    float stable_ms_acc_{0.0f};
    float learn_ms_acc_{0.0f};
    float hold_ms_acc_{0.0f};
    float freeze_stable_ms_{0.0f};
    uint32_t stable_sample_count_{0};
};

}  // namespace robot::service
