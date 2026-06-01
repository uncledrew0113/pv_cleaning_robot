#pragma once
#include <memory>

#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::service {

/// @brief 导航服务——里程计、融合里程和坡度/悬空检测
///
/// 当前实现包含两套输出：
/// - `Pose`：对外兼容的导航快照，`distance_m/speed_mps` 来自融合后的整体里程与速度，
///   `pitch_deg/roll_deg` 保留 IMU 当前读数，供安全与业务逻辑使用。
/// - `FusedOdometry`：上下轮组独立积分 + GPS 轨道投影低频校正 + IMU 陀螺/加速度短时约束
///   得到的精简融合里程快照。
///
/// 注意：受现场磁场干扰约束，融合里程当前不依赖 IMU 姿态角做绝对方向修正。
class NavService : public middleware::IRunnable {
   public:
    struct Pose {
        double distance_m{0.0};         ///< 沿轨道方向累计位移（米，当前取整体融合里程）
        float pitch_deg{0.0f};          ///< 纵坡角（来自 IMU，正=上坡）
        float roll_deg{0.0f};           ///< 横坡角（来自 IMU）
        float speed_mps{0.0f};          ///< 当前速度（m/s，由编码器微分得到）
        bool spin_free_detected{false}; ///< 悬空检测：电机有命令但轮速≈0 超过500ms
        bool valid{false};
    };

    struct FusedOdometry {
        double top_distance_m{0.0};
        double bottom_distance_m{0.0};
        double fused_distance_m{0.0};
        double distance_diff_m{0.0};
        float top_speed_mps{0.0f};
        float bottom_speed_mps{0.0f};
        float fused_speed_mps{0.0f};
        bool valid{false};
    };

    /// @param walk              行走电机组，用于读取各轮速度反馈
    /// @param imu               IMU 设备，用于读取陀螺/加速度和当前姿态读数
    /// @param gps               GPS 设备，用于读取经纬度坐标并做整体里程低频校正
    /// @param wheel_circumference_m 车轮周长（米），用于里程积分；
    ///        典型值：0.628f（直径 200mm 橡胶轮，π × 0.2）；默认 0.3f（小型测试轮）
    NavService(std::shared_ptr<device::WalkMotorGroup> walk,
               std::shared_ptr<device::ImuDevice>      imu,
               std::shared_ptr<device::GpsDevice>      gps,
               float wheel_circumference_m = 0.3f);

    /// 重置里程计和融合里程基准
    void reset_odometry();

    /// 仅清除悬空检测计数器（故障处理后调用，不影响里程计数值）
    void clear_spin_detection();

    /// 获取当前位姿（线程安全）
    Pose get_pose() const;
    /// 获取融合后的上下轮组/整体里程快照（线程安全）
    FusedOdometry get_fused_odometry() const;

    /// 当前坡度超过安全阈值？（用于业务控制器判断）
    bool is_slope_too_steep(float threshold_deg = 15.0f) const;

    void update() override;  ///< 由 ThreadExecutor 10ms 调用

   private:
    std::shared_ptr<device::WalkMotorGroup> walk_;
    std::shared_ptr<device::ImuDevice>      imu_;
    std::shared_ptr<device::GpsDevice>      gps_;
    float wheel_circ_m_;

    mutable robot::hal::PiMutex mtx_;
    Pose pose_;
    FusedOdometry fused_;

    // 悬空检测：在 mtx_ 保护下访问
    static constexpr float kSpinCmdThreshold  = 10.0f;  ///< 有效命令阈值 (RPM)
    static constexpr float kSpinStopThreshold =  5.0f;  ///< 视为静止阈值 (RPM)
    static constexpr int   kSpinMaxTicks      = 50;     ///< 50×10ms = 500ms
    int spin_free_ticks_{0};

    bool gps_origin_valid_{false};
    double gps_origin_lat_deg_{0.0};
    double gps_origin_lon_deg_{0.0};
    bool track_axis_valid_{false};
    double track_axis_x_{1.0};
    double track_axis_y_{0.0};
    bool gps_reference_valid_{false};
    double gps_reference_s_{0.0};
    bool last_gps_track_valid_{false};
    double last_gps_track_s_{0.0};
    bool diff_filter_initialized_{false};
    float filtered_diff_speed_mps_{0.0f};
};

}  // namespace robot::service
