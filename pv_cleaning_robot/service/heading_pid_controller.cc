#include "pv_cleaning_robot/service/heading_pid_controller.h"

#include <cmath>

namespace robot::service {

HeadingPidController::HeadingPidController(const Params& p) : params_(p) {}

void HeadingPidController::set_params(const Params& p) {
    params_ = p;
}

void HeadingPidController::enable(bool en) {
    enabled_ = en;
    if (!en) {
        integral_ = 0.0f;
        prev_err_ = 0.0f;
    }
}

void HeadingPidController::reset() {
    integral_ = 0.0f;
    prev_err_ = 0.0f;
}

float HeadingPidController::compute(float omega_z_dps, float dt_s) {
    if (!enabled_)
        return 0.0f;

    // 死区：轨道几何慢速漂移（通常 <1°/s）落在死区内，不触发纠偏；
    // 突发偏转（出轨/接缝冲击，通常 >5°/s）超出死区，立即纠偏。
    if (params_.deadband_rate_dps > 0.0f &&
        std::abs(omega_z_dps) <= params_.deadband_rate_dps) {
        // 死区内：复位积分，避免在死区附近长期停留时产生积分饱和
        integral_ = 0.0f;
        prev_err_ = 0.0f;
        return 0.0f;
    }

    // 误差 = 0（目标角速度）− omega_z（当前角速度）= −omega_z
    // 直观：omega_z > 0（CCW）→ err < 0 → correction < 0 → 下轨加速 → CW 纠偏 ✓
    float err = -omega_z_dps;

    // 积分过零清零：防止系统越过目标后，已累积的积分继续施加错误方向的纠偏力
    if (prev_err_ * err < 0.0f)
        integral_ = 0.0f;

    float derivative = (dt_s > 0.0f) ? (err - prev_err_) / dt_s : 0.0f;
    prev_err_ = err;

    integral_ += err * dt_s;
    integral_ = clamp(integral_, -params_.integral_limit, params_.integral_limit);

    float output = params_.kp * err + params_.ki * integral_ + params_.kd * derivative;
    return clamp(output, -params_.max_output, params_.max_output);
}

float HeadingPidController::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace robot::service
