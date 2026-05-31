#pragma once

/// @file heading_corrector.h
/// @brief 基于视觉 UDS 结果的横向纠偏控制器。

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::service {

class HeadingCorrector {
   public:
    struct Params {
        std::string uds_path{"/tmp/pv_edge_tracker.sock"};
        int reconnect_interval_ms{500};
        int result_timeout_ms{500};
        float min_confidence{0.60f};
        float deadband_yaw_deg{1.0f};
        float kp{0.8f};
        float ki{0.0f};
        float kd{0.0f};
        float integral_limit{1.0f};
        float max_output{8.0f};
        float min_effective_output{1.0f};
        float yaw_alpha{0.35f};
        float output_sign{1.0f};
    };

    enum class Mode : uint8_t {
        UNINITIALIZED = 0,
        DISCONNECTED,
        STALE,
        TRACK,
    };

    enum class MotionPhase : uint8_t {
        ToFarEnd = 0,
        ToParkingSide,
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
        float dt_s{0.0f};
        WheelFeedback wheel_feedback{};
        bool has_base_command{false};
        SpeedCommand base_command{};
        MotionPhase motion_phase{MotionPhase::ToFarEnd};
        ParkingSide parking_side{ParkingSide::Right};
    };

    struct Output {
        float feedback_correction_rpm{0.0f};
        float feedforward_correction_rpm{0.0f};
        float correction_rpm{0.0f};
        bool has_speed_command{false};
        SpeedCommand speed_command{};
    };

    struct DebugState {
        Mode mode{Mode::UNINITIALIZED};
        bool connected{false};
        bool latest_valid{false};
        float latest_yaw_deg{0.0f};
        float latest_confidence{0.0f};
        // 控制误差（已按 parking_side / motion_phase 归一化并做低通滤波），不是原始视觉 yaw。
        float filtered_yaw_deg{0.0f};
        float integral_term{0.0f};
        float last_correction{0.0f};
        int64_t result_age_ms{-1};
    };

    HeadingCorrector();
    explicit HeadingCorrector(const Params& p);
    ~HeadingCorrector();

    void set_params(const Params& p);
    void enable(bool en);
    void reset();

    bool is_enabled() const {
        return enabled_.load(std::memory_order_relaxed);
    }

    Output compute(const Input& input);
    DebugState debug_state() const;

   private:
    struct VisionResult {
        bool available{false};
        bool valid{false};
        float yaw_deg{0.0f};
        float confidence{0.0f};
        std::chrono::steady_clock::time_point received_at{};
    };

    static float clamp(float v, float lo, float hi);
    static float clamp_alpha(float alpha);
    static float low_pass(float previous, float sample, float alpha);
    static SpeedCommand apply_correction(const SpeedCommand& base, float correction_rpm);

    void start_io_thread_if_needed();
    void stop_io_thread();
    void io_loop();
    void close_socket_locked();
    bool connect_locked(const Params& params);
    void consume_socket_locked();
    void ingest_json_line_locked(const std::string& line);
    void reset_control_state_locked();
    int64_t latest_result_age_ms_locked(std::chrono::steady_clock::time_point now) const;

    mutable std::mutex mu_;
    Params params_{};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread io_thread_;
    bool io_thread_started_{false};

    int socket_fd_{-1};
    std::string connected_path_;
    std::string rx_buffer_;

    Mode mode_{Mode::UNINITIALIZED};
    bool connected_{false};
    VisionResult latest_result_{};
    bool filter_initialized_{false};
    float filtered_yaw_deg_{0.0f};
    float integral_term_{0.0f};
    float last_error_{0.0f};
    float last_correction_{0.0f};
};

}  // namespace robot::service
