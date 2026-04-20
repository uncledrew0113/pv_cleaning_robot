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

    // 自适应目标跟踪（alpha < 1.0 时生效）：
    //   target += (1 - alpha) × norm_angle(yaw - target)
    // 作用：轨道几何引起的慢速漂移（速率 << 1/τ）自动被目标吸收，不产生大误差；
    //       突发偏转（速率 >> 1/τ，如出轨）目标跟不上，PID 立即响应纠正。
    // 时间常数 τ ≈ -T / ln(alpha)，T = 0.02s；alpha=0.99 → τ ≈ 2s
    if (params_.target_tracking_alpha < 1.0f - 1e-4f) {
        float track_diff = norm_angle(yaw_deg - target_);
        target_ += (1.0f - params_.target_tracking_alpha) * track_diff;
    }

    float err = norm_angle(target_ - yaw_deg);

    // Fix 1：误差过零（航向越过目标）时清零积分，防止收敛阶段累积的正/负积分
    // 在系统越过目标后继续施加错误方向的纠偏力（积分超调/windup）。
    // 条件：prev_err_ 与 err 异号，且两者均不为 0（排除初始状态）。
    if (prev_err_ * err < 0.0f)
        integral_ = 0.0f;

    // 微分（先计算，prev_err_ 更新前使用旧值；死区内也更新，确保退出死区时导数平滑）
    float derivative = (dt_s > 0.0f) ? (err - prev_err_) / dt_s : 0.0f;
    prev_err_ = err;

    // Fix 2：死区内不累积积分，防止在死区附近长期停留时产生积分饱和。
    // 退出死区时导数仍然平滑（prev_err_ 在上方已更新）。
    if (params_.deadband_deg > 0.0f && std::abs(err) <= params_.deadband_deg)
        return 0.0f;

    // 积分（带限幅；仅在死区外且误差未过零时累积）
    integral_ += err * dt_s;
    integral_ = clamp(integral_, -params_.integral_limit, params_.integral_limit);

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
