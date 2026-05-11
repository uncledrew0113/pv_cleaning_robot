#pragma once

#include <cstdint>

namespace robot::service {

class HeadingPidController {
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

    struct DebugState {
        Mode mode{Mode::UNINITIALIZED};
        float filtered_pitch{0.0f};
        float filtered_roll{0.0f};
        float filtered_gyro_z{0.0f};
        float pitch_abs_best{0.0f};
        float roll_at_best{0.0f};
    };

    HeadingPidController() = default;
    explicit HeadingPidController(const Params& p);

    void set_params(const Params& p);
    void enable(bool en);
    void reset();

    bool is_enabled() const {
        return enabled_;
    }

    float compute(float pitch_deg, float roll_deg, float gyro_z_dps, float dt_s);

    DebugState debug_state() const;

   private:
    static float clamp(float v, float lo, float hi);
    static float clamp_alpha(float alpha);
    static float low_pass(float previous, float sample, float alpha);

    void reset_stability_window();
    void mark_stable_sample(float dt_ms);
    bool stable_window_ready() const;
    void update_filters(float pitch_deg, float roll_deg, float gyro_z_dps);
    void update_best_reference(float dt_s);
    float track_correction(float dt_s);

    Params params_{};
    bool enabled_{false};
    bool filters_initialized_{false};
    bool learned_once_{false};
    Mode mode_{Mode::UNINITIALIZED};

    float filtered_pitch_{0.0f};
    float filtered_roll_{0.0f};
    float filtered_gyro_z_{0.0f};
    float pitch_abs_best_{0.0f};
    float roll_at_best_{0.0f};

    float stable_ms_acc_{0.0f};
    float learn_ms_acc_{0.0f};
    float hold_ms_acc_{0.0f};
    float freeze_stable_ms_{0.0f};
    uint32_t stable_sample_count_{0};
};

}  // namespace robot::service
