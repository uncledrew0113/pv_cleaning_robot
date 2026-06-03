#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

#include <algorithm>
#include <cmath>

namespace robot::service {

namespace {

constexpr float kRadToDeg = 57.29577951308232f;

float non_negative(float v) {
    return std::max(0.0f, v);
}

}  // namespace

void UdsGyroYawFusion::set_params(const Params& params) {
    params_ = params;
    params_.process_noise_angle = non_negative(params_.process_noise_angle);
    params_.process_noise_bias = non_negative(params_.process_noise_bias);
    params_.measurement_noise_uds = std::max(1e-4f, params_.measurement_noise_uds);
    params_.initial_angle_variance = std::max(1e-4f, params_.initial_angle_variance);
    params_.initial_bias_variance = std::max(1e-4f, params_.initial_bias_variance);
    params_.max_gyro_only_ms = std::max(0, params_.max_gyro_only_ms);
}

void UdsGyroYawFusion::reset() {
    initialized_ = false;
    gyro_only_elapsed_ms_ = 0;
    angle_deg_ = 0.0f;
    gyro_bias_dps_ = 0.0f;
    p00_ = params_.initial_angle_variance;
    p01_ = 0.0f;
    p10_ = 0.0f;
    p11_ = params_.initial_bias_variance;
}

UdsGyroYawFusion::Output UdsGyroYawFusion::update(const Input& input) {
    Output out{};
    const float dt = std::max(0.0f, input.dt_s);
    const float gyro_z_dps = input.imu_valid ? input.gyro_z_rad_s * kRadToDeg : 0.0f;
    out.gyro_z_dps = gyro_z_dps;

    if (!initialized_) {
        if (!input.uds_valid) {
            return out;
        }
        initialized_ = true;
        gyro_only_elapsed_ms_ = 0;
        angle_deg_ = input.uds_yaw_deg;
        gyro_bias_dps_ = 0.0f;
        p00_ = params_.initial_angle_variance;
        p01_ = 0.0f;
        p10_ = 0.0f;
        p11_ = params_.initial_bias_variance;
    } else if (dt > 0.0f) {
        angle_deg_ += (gyro_z_dps - gyro_bias_dps_) * dt;

        const float old_p00 = p00_;
        const float old_p01 = p01_;
        const float old_p10 = p10_;
        const float old_p11 = p11_;

        p00_ = old_p00 - dt * (old_p10 + old_p01) + dt * dt * old_p11 +
               params_.process_noise_angle;
        p01_ = old_p01 - dt * old_p11;
        p10_ = old_p10 - dt * old_p11;
        p11_ = old_p11 + params_.process_noise_bias;
    }

    if (input.uds_valid) {
        gyro_only_elapsed_ms_ = 0;
        const float innovation = input.uds_yaw_deg - angle_deg_;
        const float s = std::max(1e-4f, p00_ + params_.measurement_noise_uds);
        const float k0 = p00_ / s;
        const float k1 = p10_ / s;

        angle_deg_ += k0 * innovation;
        gyro_bias_dps_ += k1 * innovation;

        const float old_p00 = p00_;
        const float old_p01 = p01_;
        p00_ -= k0 * old_p00;
        p01_ -= k0 * old_p01;
        p10_ -= k1 * old_p00;
        p11_ -= k1 * old_p01;

        out.innovation_deg = innovation;
        out.kalman_gain_angle = k0;
        out.kalman_gain_bias = k1;
    } else {
        gyro_only_elapsed_ms_ += static_cast<int>(std::lround(dt * 1000.0f));
    }

    out.valid = gyro_only_elapsed_ms_ <= params_.max_gyro_only_ms;
    out.fused_yaw_deg = angle_deg_;
    out.gyro_bias_dps = gyro_bias_dps_;
    return out;
}

}  // namespace robot::service
