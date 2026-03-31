#include "pv_cleaning_robot/service/heading_pid_controller.h"

namespace robot::service {

HeadingPidController::HeadingPidController(const Params& p) : params_(p) {}

void HeadingPidController::set_params(const Params& p) {
    params_ = p;
}

void HeadingPidController::enable(bool en) {
    enabled_ = en;
    if (!en) {
        integral_    = 0.0f;
        prev_err_    = 0.0f;
        initialized_ = false;
    }
}

void HeadingPidController::set_target(float yaw_deg) {
    target_      = yaw_deg;
    initialized_ = true;
    integral_    = 0.0f;
    prev_err_    = 0.0f;
}

void HeadingPidController::reset() {
    integral_    = 0.0f;
    prev_err_    = 0.0f;
    initialized_ = false;
}

float HeadingPidController::compute(float yaw_deg, float dt_s) {
    if (!enabled_)
        return 0.0f;

    // 首次调用时自动将当前航向锁定为目标（保持直线）
    if (!initialized_) {
        target_      = yaw_deg;
        initialized_ = true;
    }

    float err = norm_angle(target_ - yaw_deg);

    // 积分（带限幅）
    integral_ += err * dt_s;
    integral_ = clamp(integral_, -params_.integral_limit, params_.integral_limit);

    // 微分
    float derivative = (dt_s > 0.0f) ? (err - prev_err_) / dt_s : 0.0f;
    prev_err_ = err;

    float output = params_.kp * err + params_.ki * integral_ + params_.kd * derivative;
    return clamp(output, -params_.max_output, params_.max_output);
}

float HeadingPidController::norm_angle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float HeadingPidController::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace robot::service
