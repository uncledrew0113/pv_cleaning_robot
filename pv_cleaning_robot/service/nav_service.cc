#include <cmath>

#include "pv_cleaning_robot/service/nav_service.h"

namespace robot::service {

NavService::NavService(std::shared_ptr<device::WalkMotorGroup> walk,
                       std::shared_ptr<device::ImuDevice>      imu,
                       std::shared_ptr<device::GpsDevice>      gps,
                       float wheel_circumference_m)
    : walk_(std::move(walk))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , wheel_circ_m_(wheel_circumference_m) {}

void NavService::reset_odometry() {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    pose_.distance_m = 0.0;
    pose_.speed_mps = 0.0f;
}

NavService::Pose NavService::get_pose() const {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    return pose_;
}

bool NavService::is_slope_too_steep(float threshold_deg) const {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    return std::abs(pose_.pitch_deg) > threshold_deg;
}

void NavService::update() {
    // 取 4 轮转速平均値作为车体线速度（降低单轮打滑弹性）
    auto gs = walk_->get_group_status();
    float avg_rpm = 0.0f;
    for (auto& w : gs.wheel) avg_rpm += w.speed_rpm;
    avg_rpm /= static_cast<float>(device::WalkMotorGroup::kWheelCount);

    auto imu_data = imu_->get_latest();

    std::lock_guard<robot::hal::PiMutex> lk(mtx_);

    // 速度积分里程计：avg_rpm × 周长 / 60 × 周期（10ms）
    constexpr float kDt = 0.010f;
    float speed_rps = avg_rpm / 60.0f;
    float delta_m = speed_rps * wheel_circ_m_ * kDt;
    pose_.distance_m += delta_m;
    pose_.speed_mps = speed_rps * wheel_circ_m_;

    // IMU 姿态
    if (imu_data.valid) {
        pose_.pitch_deg = imu_data.pitch_deg;
        pose_.roll_deg = imu_data.roll_deg;
    }

    pose_.valid = true;
}

}  // namespace robot::service
