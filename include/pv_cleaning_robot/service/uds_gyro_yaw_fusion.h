/**
 * @file uds_gyro_yaw_fusion.h
 * @brief UDS 与 IMU 陀螺 yaw 融合接口。
 *
 * 本模块用于将视觉/UDS 航向与 IMU 角速度融合为运动纠偏可用的 yaw 估计，避免单一低频传感器
 * 在控制周期内产生突变。
 */
#pragma once

#include <cstdint>

namespace robot::service {

class UdsGyroYawFusion {
   public:
    struct Params {
        float process_noise_angle{0.05f};
        float process_noise_bias{0.001f};
        float measurement_noise_uds{0.5f};
        float initial_angle_variance{1.0f};
        float initial_bias_variance{1.0f};
        int max_gyro_only_ms{300};
    };

    struct Input {
        float dt_s{0.0f};
        float uds_yaw_deg{0.0f};
        bool uds_valid{false};
        float uds_confidence{0.0f};
        int64_t uds_age_ms{-1};
        float gyro_z_rad_s{0.0f};
        bool imu_valid{false};
    };

    struct Output {
        bool valid{false};
        float fused_yaw_deg{0.0f};
        float gyro_z_dps{0.0f};
        float gyro_bias_dps{0.0f};
        float innovation_deg{0.0f};
        float kalman_gain_angle{0.0f};
        float kalman_gain_bias{0.0f};
    };

    void set_params(const Params& params);
    void reset();
    Output update(const Input& input);

   private:
    Params params_{};
    bool initialized_{false};
    int gyro_only_elapsed_ms_{0};
    float angle_deg_{0.0f};
    float gyro_bias_dps_{0.0f};
    float p00_{1.0f};
    float p01_{0.0f};
    float p10_{0.0f};
    float p11_{1.0f};
};

}  // namespace robot::service
