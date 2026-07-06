/**
 * @file motion_service.h
 * @brief 机器人运动控制服务接口。
 *
 * MotionService 将任务段、姿态回中和恢复动作转换为行走电机、滚刷和 IMU 相关控制命令。
 * 安全急停和恢复入口通过明确接口暴露，状态机只调用服务边界，不直接拼装电机协议。
 */
#pragma once
#include <atomic>
#include <chrono>
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
        int brush_direction_sign{1};     ///< 测试/调试用滚刷方向乘子：1=默认，-1=反向
        bool heading_pid_en{true};      ///< 是否使能视觉纠偏
        float control_dt_s{0.05f};      ///< 姿态纠偏控制周期，必须与 walk_ctrl 调度周期保持一致
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

    /// 注入主停机端配置，用于速度选择和纠偏镜像符号。
    void set_primary_dock_query(std::function<domain::Endpoint()> query);
    /// 新任务启动前从 active runtime 同步速度/滚刷参数；所有速度配置统一按绝对值解释。
    void set_runtime_config_query(std::function<RuntimeConfig()> query);

    /// @brief 按业务任务段启动运动，是 app 层唯一的任务段运动入口。
    bool start_segment(const domain::MissionSegment& segment);

    /// 停止当前运动段（停滚刷，行走归零，禁用姿态纠偏）。
    void stop_cleaning();

    /// 原地急停（失能行走，停滚刷）
    void emergency_stop() override;

    /// @brief 按当前运动方向反向短距离恢复。
    ///
    /// 用于 GpsStuck / WalkMotorStall 这类“通信正常但现场卡住”的恢复。
    /// 运动命令仍由 MotionService 统一换算并下发，调用方只提供时长和中断条件。
    bool reverse_for_recovery(std::chrono::milliseconds duration,
                              std::chrono::milliseconds tick,
                              std::function<bool()> interrupted);

    /// @brief 进入姿态回中运动模式。
    ///
    /// 只负责准备运动侧状态：停止滚刷、关闭清扫段命令、解除安全覆盖并切到速度模式。
    /// 姿态限位状态、回中时序和超时判断由 AttitudeLimitService 负责。
    bool begin_attitude_center_motion();

    /// @brief 姿态回中期间只驱动下轮 LB/RB，上轮 LT/RT 保持 0。
    bool command_lower_wheels_for_attitude_center(float lower_rpm);

    /// @brief 停止姿态回中运动并失能行走电机。
    bool stop_attitude_center_motion();

    void update() override;  ///< 由 walk_ctrl 线程按 control_dt_s 对应周期调用
    HeadingCorrector::DebugState heading_pid_debug_state() const;

   private:
    std::shared_ptr<device::WalkMotorGroup> group_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::ImuDevice> imu_;
    middleware::EventBus& bus_;

    mutable hal::PiMutex state_mtx_;
    Config cfg_;
    std::function<domain::Endpoint()> primary_dock_query_;
    std::function<RuntimeConfig()> runtime_config_query_;
    HeadingCorrector heading_corrector_{};
    device::WalkMotorGroup::SpeedCmd base_speed_cmd_{};
    bool walk_command_active_{false};
    domain::TravelDirection travel_direction_{domain::TravelDirection::AToB};
    uint32_t last_override_clear_generation_{0};
    uint32_t command_generation_{0};

    struct StateSnapshot {
        Config cfg{};
        std::function<domain::Endpoint()> primary_dock_query{};
        device::WalkMotorGroup::SpeedCmd base_speed_cmd{};
        bool walk_command_active{false};
        domain::TravelDirection travel_direction{domain::TravelDirection::AToB};
        uint32_t command_generation{0};
    };

    struct OverrideClearAction {
        bool changed{false};
        bool restore_base_speed{false};
        device::WalkMotorGroup::SpeedCmd base_speed_cmd{};
        uint32_t command_generation{0};
    };

    StateSnapshot snapshot_state() const;
    domain::Endpoint primary_dock() const;
    int target_direction_sign(domain::Endpoint target) const;
    void sync_runtime_config();
    bool enable_speed_mode();
    void sync_heading_pid_enabled();
    bool start_cleaning_to(domain::Endpoint target);
    void activate_walk_command(const device::WalkMotorGroup::SpeedCmd& cmd,
                               domain::TravelDirection direction);
    void deactivate_walk_command();
    void update_heading_correction(const StateSnapshot& state, bool override_active);
    void handle_override_clear();
    void apply_speed_if_command_current(uint32_t generation,
                                        const device::WalkMotorGroup::SpeedCmd& cmd);
    OverrideClearAction observe_override_clear(uint32_t clear_generation);
};

}  // namespace robot::service
