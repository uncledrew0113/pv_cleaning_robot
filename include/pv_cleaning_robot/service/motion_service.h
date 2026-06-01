#pragma once
#include <atomic>
#include <functional>
#include <memory>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/heading_corrector.h"

namespace robot::service {

/// @brief 运动服务：把业务任务段转换成行走电机和滚刷命令。
///
/// 边界约束：
///   - app 层只能调用 start_segment()/stop_cleaning()/emergency_stop()；
///   - 具体轮向和滚刷方向在本服务内根据目标端点和主停机端统一换算；
///   - update() 负责周期心跳和姿态纠偏，override 激活时不抢占安全停车。
class MotionService : public middleware::IRunnable, public domain::EmergencyStopPort {
   public:
    struct Config {
        float clean_speed_rpm{30.0f};   ///< 清扫行进速度（绝对值 RPM）
        float return_speed_rpm{30.0f};  ///< 返回速度（绝对值 RPM）
        int brush_rpm{1200};            ///< 滚刷转速绝对值
        bool heading_pid_en{true};      ///< 是否使能视觉纠偏
        HeadingCorrector::Params pid{};  ///< 视觉纠偏参数
    };

    /// @param group    4轮行走电机组（构造时已配置 comm_timeout_ms）
    /// @param brush    滚刷电机
    /// @param imu      IMU 设备（当前保留构造依赖，视觉纠偏不直接使用）
    /// @param bus      事件总线
    /// @param cfg      运动配置
    MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                  std::shared_ptr<device::BrushMotor> brush,
                  std::shared_ptr<device::ImuDevice> imu,
                  middleware::EventBus& bus,
                  Config cfg);

    /// 注入主停机端配置，用于速度选择、无刷返航和纠偏镜像符号。
    void set_primary_dock_query(std::function<domain::Endpoint()> query);
    /// 新任务启动前从 active runtime 同步速度/滚刷参数；所有速度配置统一按绝对值解释。
    void set_runtime_config_query(std::function<RuntimeConfig()> query);

    /// @brief 按业务任务段启动运动，是 app 层唯一的任务段运动入口。
    bool start_segment(const domain::MissionSegment& segment);

    /// 停止当前运动段（停滚刷，行走归零，禁用姿态纠偏）。
    void stop_cleaning();

    /// 原地急停（失能行走，停滚刷）
    void emergency_stop() override;

    void update() override;  ///< 由 ThreadExecutor 20ms 调用（50Hz 姿态纠偏）
    HeadingCorrector::DebugState heading_pid_debug_state() const;

   private:
    std::shared_ptr<device::WalkMotorGroup> group_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::ImuDevice> imu_;
    middleware::EventBus& bus_;
    Config cfg_;
    std::function<domain::Endpoint()> primary_dock_query_;
    std::function<RuntimeConfig()> runtime_config_query_;
    HeadingCorrector heading_corrector_{};
    device::WalkMotorGroup::SpeedCmd base_speed_cmd_{};
    bool walk_command_active_{false};
    domain::TravelDirection travel_direction_{domain::TravelDirection::AToB};
    uint32_t last_override_clear_generation_{0};

    domain::Endpoint primary_dock() const;
    int target_direction_sign(domain::Endpoint target) const;
    void sync_runtime_config();
    bool enable_speed_mode();
    void sync_heading_pid_enabled();
    bool start_cleaning_to(domain::Endpoint target);
    bool start_brush_off_return_to(domain::Endpoint target);
    void set_base_speed_command(const device::WalkMotorGroup::SpeedCmd& cmd);
};

}  // namespace robot::service
