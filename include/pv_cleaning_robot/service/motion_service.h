#pragma once
#include <atomic>
#include <functional>
#include <memory>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/heading_corrector.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::service {

/// @brief 运动控制服务——协调行走电机组与滚刷电机
///
/// 新增功能：
///   1. 通信超时保活：WalkMotorGroup 在 open() 时自动下发 comm_timeout_ms
///      给每台电机；update() 每 20ms 重发设定值维持心跳，超时自停
///   2. 主动上报+温度查询：反馈方式使用主动上报（100Hz），
///      无法上报温度时由 WalkMotorGroup::update() 定期发 0x107 查询补采
///   3. IMU 姿态纠偏：MotionService 持有 HeadingCorrector，
///      update() 每 20ms 计算最终4轮目标速度并下发给 WalkMotorGroup
///   4. 冲突保护：override 激活期间 WalkMotorGroup::update() 跳过心跳重发，
///      防止控制心跳干扰边缘停车指令
class MotionService : public middleware::IRunnable {
   public:
    struct Config {
        float clean_speed_rpm{30.0f};   ///< 清扫行进速度（RPM）
        float return_speed_rpm{30.0f};  ///< 返回速度（RPM，快速）
        int brush_rpm{1200};            ///< 滚刷正向转速
        int return_brush_rpm{1200};     ///< 滚刷返程转速（绝对值，实际方向取反）
        float edge_reverse_rpm{30.0f};  ///< 边缘触发后反转速度（RPM，0=原地停）
        bool heading_pid_en{true};      ///< 是否使能姿态纠偏
        HeadingCorrector::Params pid{};  ///< 姿态纠偏参数
    };

    /// @param group    4轮行走电机组（构造时已配置 comm_timeout_ms）
    /// @param brush    滚刷电机
    /// @param imu      IMU 设备（提供 pitch/roll/yaw/gyro）
    /// @param bus      事件总线
    /// @param cfg      运动配置
    MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                  std::shared_ptr<device::BrushMotor> brush,
                  std::shared_ptr<device::ImuDevice> imu,
                  middleware::EventBus& bus,
                  Config cfg);

    /// 以“停机位在右侧”为运动方向基线；provider 返回 Left 时整体取反。
    void set_parking_side_provider(std::function<ParkingSide()> provider);
    /// 新任务启动前从 active runtime 同步速度/滚刷参数。
    void set_runtime_config_provider(std::function<TbRuntimeConfig()> provider);

    // ── 运动控制 ──────────────────────────────────────────────────────────
    /// 开始清扫前进（使能行走 + 滚刷，启用姿态纠偏）
    bool start_cleaning();

    /// 停止清扫（停滚刷，行走归零，禁用姿态纠偏）
    void stop_cleaning();

    /// 以返回速度反向行进（滚刷反向运行，保持姿态纠偏）
    bool start_returning();

    /// @brief P1 故障路径：先停滚刷，再以返回速度倒退回停机位。
    ///
    /// 由 RobotFsm::dispatch<EvFaultP1>() 调用；与 start_returning() 的区别是
    /// 滚刷立即停止（不反向运行），适用于需要避免滚刷二次损伤的故障场景。
    bool start_returning_no_brush();

    /// 原地急停（失能行走，停滚刷）
    void emergency_stop();

    void update() override;  ///< 由 ThreadExecutor 20ms 调用（50Hz 姿态纠偏）

   private:
    std::shared_ptr<device::WalkMotorGroup> group_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::ImuDevice> imu_;
    middleware::EventBus& bus_;
    Config cfg_;
    std::function<ParkingSide()> parking_side_provider_;
    std::function<TbRuntimeConfig()> runtime_config_provider_;
    HeadingCorrector heading_corrector_{};
    device::WalkMotorGroup::SpeedCmd base_speed_cmd_{};
    bool walk_command_active_{false};
    uint32_t last_override_clear_generation_{0};

    int task_direction_sign() const;
    void sync_runtime_config();
    bool enable_speed_mode();
    void sync_heading_pid_enabled();
    bool start_returning_impl(bool run_brush);
    void set_base_speed_command(const device::WalkMotorGroup::SpeedCmd& cmd);
};

}  // namespace robot::service
