#pragma once

/// @file heading_pid_controller.h
/// @brief 方向控制 PID 逻辑，用于根据 IMU 姿态保持方向稳定。

#include <cstdint>

namespace robot::service {

class HeadingPidController {
   public:
    struct Params {
        /// 俯仰角滤波系数
        float pitch_alpha{0.2f};
        /// 横滚角滤波系数
        float roll_alpha{0.2f};
        /// 陀螺仪角速度滤波系数
        float gyro_alpha{0.2f};
        /// 俯仰角稳定阈值，用于进入学习阶段
        float pitch_drop_threshold{0.12f};
        /// 横滚角误差阈值，用于姿态跟踪
        float roll_threshold{0.6f};
        /// 学习阶段改进阈值，用于判断是否更新最优参考值
        float learn_improve_eps{0.03f};
        /// 最优参考值衰减速率（每秒）
        float best_decay_per_s{0.01f};
        /// 陀螺仪 Z 轴冻结判定阈值
        float freeze_gyro_z_threshold{30.0f};
        /// 俯仰角速度冻结判定阈值
        float freeze_pitch_rate_threshold{20.0f};
        /// 横滚角速度冻结判定阈值
        float freeze_roll_rate_threshold{20.0f};
        /// 输出最大值限制
        float max_output{30.0f};
        /// 最小有效输出阈值
        float min_effective_output{0.0f};
        /// 学习阶段预热时间（毫秒）
        int warmup_ms{400};
        /// 保持稳定状态时间（毫秒）
        int hold_ms{400};
        /// 冻结释放延迟（毫秒）
        int freeze_release_ms{300};
    };

    enum class Mode : uint8_t {
        UNINITIALIZED = 0,  ///< 未初始化
        LEARN,              ///< 通过稳定姿态学习基准值
        TRACK,              ///< 运行中基于参考姿态执行修正
        FREEZE,             ///< 冻结状态，暂不输出控制量
    };

    struct DebugState {
        Mode mode{Mode::UNINITIALIZED};  ///< 当前控制器模式
        float filtered_pitch{0.0f};      ///< 滤波后的俯仰角
        float filtered_roll{0.0f};       ///< 滤波后的横滚角
        float filtered_gyro_z{0.0f};     ///< 滤波后的陀螺仪 Z 轴角速度
        float pitch_abs_best{0.0f};      ///< 当前最佳俯仰绝对值参考
        float roll_at_best{0.0f};        ///< 当前最佳横滚参考值
    };

    HeadingPidController() = default;
    explicit HeadingPidController(const Params& p);

    /// @brief 更新 PID 控制器参数。
    void set_params(const Params& p);

    /// @brief 启用或禁用方向控制。
    void enable(bool en);

    /// @brief 重置内部滤波和学习状态。
    void reset();

    bool is_enabled() const {
        return enabled_;  ///< 只能在启用状态下计算输出
    }

    /// @brief 计算当前姿态的方向修正输出。
    /// @param pitch_deg 当前俯仰角（度）
    /// @param roll_deg 当前横滚角（度）
    /// @param gyro_z_dps Z 轴角速度（度/秒）
    /// @param dt_s 自上次调用以来的秒数
    /// @return 输出修正量，范围由 params.max_output 限制
    float compute(float pitch_deg, float roll_deg, float gyro_z_dps, float dt_s);

    /// @brief 获取当前调试状态，用于诊断和日志。
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
