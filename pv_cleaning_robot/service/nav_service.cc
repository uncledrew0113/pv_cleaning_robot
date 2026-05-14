#include <cmath>

#include "pv_cleaning_robot/service/nav_service.h"

namespace robot::service {

namespace {

constexpr float kDt = 0.010f;
constexpr float kGyroSupportFullScaleDps = 20.0f;
constexpr float kAccelSupportFullScaleMps2 = 1.5f;
constexpr float kBaseDiffAlpha = 0.15f;
constexpr float kExtraDiffAlpha = 0.55f;
constexpr float kGpsPositionGain = 0.05f;
constexpr float kGpsMinCourseSpeedMps = 0.3f;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;
constexpr float kGravityMps2 = 9.80665f;

float clamp01(float v) {
    if (v < 0.0f)
        return 0.0f;
    if (v > 1.0f)
        return 1.0f;
    return v;
}

void project_gps_to_track(const device::GpsDevice::GpsData& gps,
                          bool& gps_origin_valid,
                          double& gps_origin_lat_deg,
                          double& gps_origin_lon_deg,
                          bool& track_axis_valid,
                          double& track_axis_x,
                          double& track_axis_y,
                          bool& gps_reference_valid,
                          double& gps_reference_s,
                          bool& last_gps_track_valid,
                          double& last_gps_track_s,
                          double* gps_rel_s_out) {
    if (!gps.valid) {
        return;
    }

    if (!gps_origin_valid) {
        gps_origin_valid = true;
        gps_origin_lat_deg = gps.latitude;
        gps_origin_lon_deg = gps.longitude;
    }

    const double lat0_rad = gps_origin_lat_deg * kPi / 180.0;
    const double lat_rad = gps.latitude * kPi / 180.0;
    const double lon_rad = gps.longitude * kPi / 180.0;
    const double lon0_rad = gps_origin_lon_deg * kPi / 180.0;
    const double x_m = (lon_rad - lon0_rad) * std::cos((lat_rad + lat0_rad) * 0.5) * kEarthRadiusM;
    const double y_m = (lat_rad - lat0_rad) * kEarthRadiusM;

    if (!track_axis_valid) {
        if (gps.speed_m_s > kGpsMinCourseSpeedMps && std::isfinite(gps.course_deg)) {
            const double course_rad = static_cast<double>(gps.course_deg) * kPi / 180.0;
            track_axis_x = std::sin(course_rad);
            track_axis_y = std::cos(course_rad);
            track_axis_valid = true;
        } else {
            const double norm = std::hypot(x_m, y_m);
            if (norm >= 1.0) {
                track_axis_x = x_m / norm;
                track_axis_y = y_m / norm;
                track_axis_valid = true;
            }
        }
    }

    if (!track_axis_valid) {
        return;
    }

    const double track_s = x_m * track_axis_x + y_m * track_axis_y;
    last_gps_track_valid = true;
    last_gps_track_s = track_s;
    if (!gps_reference_valid) {
        gps_reference_valid = true;
        gps_reference_s = track_s;
    }
    if (gps_rel_s_out) {
        *gps_rel_s_out = track_s - gps_reference_s;
    }
}

}  // namespace

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
    fused_.top_distance_m = 0.0;
    fused_.bottom_distance_m = 0.0;
    fused_.fused_distance_m = 0.0;
    fused_.distance_diff_m = 0.0;
    fused_.top_speed_mps = 0.0f;
    fused_.bottom_speed_mps = 0.0f;
    fused_.fused_speed_mps = 0.0f;
    spin_free_ticks_ = 0;
    pose_.spin_free_detected = false;
    diff_filter_initialized_ = false;
    filtered_diff_speed_mps_ = 0.0f;
    if (last_gps_track_valid_) {
        gps_reference_valid_ = true;
        gps_reference_s_ = last_gps_track_s_;
    }
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

NavService::FusedOdometry NavService::get_fused_odometry() const {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    return fused_;
}

bool NavService::is_slope_too_steep(float threshold_deg) const {
    std::lock_guard<robot::hal::PiMutex> lk(mtx_);
    return std::abs(pose_.pitch_deg) > threshold_deg;
}

void NavService::update() {
    const auto gd = walk_ ? walk_->get_group_diagnostics()
                          : device::WalkMotorGroup::GroupDiagnostics{};
    const auto imu_data = imu_ ? imu_->get_latest() : device::ImuDevice::ImuData{};
    const auto gps_data = gps_ ? gps_->get_latest() : device::GpsDevice::GpsData{};

    const float top_avg_rpm = (gd.wheel[0].speed_rpm + gd.wheel[1].speed_rpm) * 0.5f;
    const float bottom_avg_rpm = (-gd.wheel[2].speed_rpm - gd.wheel[3].speed_rpm) * 0.5f;
    const float top_speed_raw = (top_avg_rpm / 60.0f) * wheel_circ_m_;
    const float bottom_speed_raw = (bottom_avg_rpm / 60.0f) * wheel_circ_m_;
    const float common_speed_raw = 0.5f * (top_speed_raw + bottom_speed_raw);
    const float diff_speed_raw = top_speed_raw - bottom_speed_raw;

    // 悬空检测：任一轮有命令且所有轮实际转速 ≈ 0，持续 kSpinMaxTicks 节拍
    bool any_commanded = false;
    bool all_stopped   = true;
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        if (std::abs(gd.wheel[i].target_value) > kSpinCmdThreshold)
            any_commanded = true;
        if (std::abs(gd.wheel[i].speed_rpm) > kSpinStopThreshold)
            all_stopped = false;
    }

    const float gyro_z_dps = imu_data.valid
                                 ? imu_data.gyro[2] * static_cast<float>(180.0 / kPi)
                                 : 0.0f;
    const float accel_norm = imu_data.valid
                                 ? std::sqrt(imu_data.accel[0] * imu_data.accel[0]
                                             + imu_data.accel[1] * imu_data.accel[1]
                                             + imu_data.accel[2] * imu_data.accel[2])
                                 : kGravityMps2;
    const float gyro_support =
        clamp01(std::abs(gyro_z_dps) / kGyroSupportFullScaleDps);
    const float accel_support =
        clamp01(std::abs(accel_norm - kGravityMps2) / kAccelSupportFullScaleMps2);
    const float diff_alpha =
        kBaseDiffAlpha + kExtraDiffAlpha * std::max(gyro_support, accel_support);
    if (!diff_filter_initialized_) {
        filtered_diff_speed_mps_ = diff_speed_raw;
        diff_filter_initialized_ = true;
    } else {
        filtered_diff_speed_mps_ += diff_alpha * (diff_speed_raw - filtered_diff_speed_mps_);
    }

    const float top_speed = common_speed_raw + 0.5f * filtered_diff_speed_mps_;
    const float bottom_speed = common_speed_raw - 0.5f * filtered_diff_speed_mps_;
    double gps_rel_s = 0.0;
    bool has_gps_rel_s = false;
    project_gps_to_track(gps_data,
                         gps_origin_valid_,
                         gps_origin_lat_deg_,
                         gps_origin_lon_deg_,
                         track_axis_valid_,
                         track_axis_x_,
                         track_axis_y_,
                         gps_reference_valid_,
                         gps_reference_s_,
                         last_gps_track_valid_,
                         last_gps_track_s_,
                         &gps_rel_s);
    has_gps_rel_s = gps_data.valid && track_axis_valid_ && gps_reference_valid_;

    float cur_pitch = 0.0f;
    float cur_roll  = 0.0f;
    if (imu_data.valid) {
        cur_pitch = imu_data.pitch_deg;
        cur_roll  = imu_data.roll_deg;
    }

    std::lock_guard<robot::hal::PiMutex> lk(mtx_);

    // 更新悬空计数器
    if (any_commanded && all_stopped)
        ++spin_free_ticks_;
    else
        spin_free_ticks_ = 0;
    pose_.spin_free_detected = (spin_free_ticks_ >= kSpinMaxTicks);

    fused_.top_speed_mps = top_speed;
    fused_.bottom_speed_mps = bottom_speed;
    fused_.fused_speed_mps = common_speed_raw;

    double top_distance_pred = fused_.top_distance_m + static_cast<double>(top_speed) * kDt;
    double bottom_distance_pred =
        fused_.bottom_distance_m + static_cast<double>(bottom_speed) * kDt;
    const double common_distance_pred = 0.5 * (top_distance_pred + bottom_distance_pred);
    double fused_distance = common_distance_pred;
    if (has_gps_rel_s) {
        fused_distance += kGpsPositionGain * (gps_rel_s - common_distance_pred);
    }
    const double common_correction = fused_distance - common_distance_pred;
    top_distance_pred += common_correction;
    bottom_distance_pred += common_correction;

    fused_.top_distance_m = top_distance_pred;
    fused_.bottom_distance_m = bottom_distance_pred;
    fused_.fused_distance_m = fused_distance;
    fused_.distance_diff_m = fused_.top_distance_m - fused_.bottom_distance_m;
    fused_.valid = true;

    pose_.distance_m = fused_.fused_distance_m;
    pose_.speed_mps = fused_.fused_speed_mps;
    if (imu_data.valid) {
        pose_.pitch_deg = cur_pitch;
        pose_.roll_deg  = cur_roll;
    }
    pose_.valid = true;
}

}  // namespace robot::service
