#pragma once
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::service {

/// @brief 导航服务——里程计 + IMU 融合，坡度估计
///
/// 基于编码器里程计 + IMU 姿态角进行航位推算（Dead Reckoning）。
/// GPS 数据作为定期校正（非强依赖）。
class NavService : public middleware::IRunnable {
   public:
    struct Pose {
        double distance_m{0.0};  ///< 沿轨道方向累计位移（米）
        float pitch_deg{0.0f};   ///< 纵坡角（来自 IMU，正=上坡）
        float roll_deg{0.0f};    ///< 横坡角（来自 IMU）
        float speed_mps{0.0f};   ///< 当前速度（m/s，由编码器微分得到）
        bool valid{false};
    };

    NavService(std::shared_ptr<device::WalkMotor> walk,
               std::shared_ptr<device::ImuDevice> imu,
               std::shared_ptr<device::GpsDevice> gps,
               float wheel_circumference_m = 0.3f);

    /// 重置里程计（返回基准点后调用）
    void reset_odometry();

    /// 获取当前位姿（线程安全）
    Pose get_pose() const;

    /// 当前坡度超过安全阈值？（用于 FSM 判断）
    bool is_slope_too_steep(float threshold_deg = 15.0f) const;

    void update() override;  ///< 由 ThreadExecutor 10ms 调用

   private:
    std::shared_ptr<device::WalkMotor> walk_;
    std::shared_ptr<device::ImuDevice> imu_;
    std::shared_ptr<device::GpsDevice> gps_;
    float wheel_circ_m_;

    mutable std::shared_mutex mtx_;
    Pose pose_;
};

}  // namespace robot::service
