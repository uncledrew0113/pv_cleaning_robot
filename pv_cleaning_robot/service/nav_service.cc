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
    spin_free_ticks_ = 0;
    pose_.spin_free_detected = false;
}

void NavService::clear_spin_detection() {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    spin_free_ticks_ = 0;
    pose_.spin_free_detected = false;
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
    // 取 4 轮诊断数据（含 target_value 用于悬空检测）
    auto gd = walk_->get_group_diagnostics();

    // 物理安装修正：LB/RB 安装方向相反，反转符号后与 LT/RT 同向
    // 车辆向前 = LT(+) + RT(+) + (-LB) + (-RB)  平均圱
    const float avg_rpm = (gd.wheel[0].speed_rpm + gd.wheel[1].speed_rpm
                         - gd.wheel[2].speed_rpm - gd.wheel[3].speed_rpm) / 4.0f;

    // 悬空检测：任一轮有命令且所有轮实际转速 ≈ 0，持续 kSpinMaxTicks 节拍
    bool any_commanded = false;
    bool all_stopped   = true;
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        if (std::abs(gd.wheel[i].target_value) > kSpinCmdThreshold)
            any_commanded = true;
        if (std::abs(gd.wheel[i].speed_rpm) > kSpinStopThreshold)
            all_stopped = false;
    }

    auto imu_data = imu_->get_latest();

    // 坡度修正：车面进程 delta_m × cos(纵坡角) = 水平地面距离
    constexpr float kDt = 0.010f;
    const float speed_rps = avg_rpm / 60.0f;
    float delta_m = speed_rps * wheel_circ_m_ * kDt;
    float cur_pitch = 0.0f;
    float cur_roll  = 0.0f;
    if (imu_data.valid) {
        cur_pitch = imu_data.pitch_deg;
        cur_roll  = imu_data.roll_deg;
        const float cos_pitch = std::cos(cur_pitch * static_cast<float>(M_PI) / 180.0f);
        delta_m *= cos_pitch;
    }

    std::lock_guard<robot::hal::PiMutex> lk(mtx_);

    // 更新悬空计数器
    if (any_commanded && all_stopped)
        ++spin_free_ticks_;
    else
        spin_free_ticks_ = 0;
    pose_.spin_free_detected = (spin_free_ticks_ >= kSpinMaxTicks);

    pose_.distance_m += static_cast<double>(delta_m);
    pose_.speed_mps  = speed_rps * wheel_circ_m_;
    if (imu_data.valid) {
        pose_.pitch_deg = cur_pitch;
        pose_.roll_deg  = cur_roll;
    }
    pose_.valid = true;
}

}  // namespace robot::service
