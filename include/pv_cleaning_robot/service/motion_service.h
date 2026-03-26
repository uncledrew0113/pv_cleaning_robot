#pragma once
#include <atomic>
#include <functional>
#include <memory>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::service {

/// @brief 运动控制服务——协调行走电机组与辊刷电机
///
/// 新增功能：
///   1. 通信超时保活：WalkMotorGroup 在 open() 时自动下发 comm_timeout_ms
///      给每台电机；update() 每 20ms 重发设定值维持心跳，超时自停
///   2. 主动上报+温度查询：反馈方式使用主动上报（100Hz），
///      无法上报温度时由 WalkMotorGroup::update() 定期发 0x107 查询补采
///   3. 航向 PID：start_cleaning() 时锁定当前 IMU yaw 为目标，
///      update() 每 20ms 用最新 IMU yaw 计算左右差速补偿
///   4. 边缘紧急响应：register_edge_callback() 注册后，
///      边缘触发时 emergency_override() 立即发帧（不经过 update() 调度），
///      并用 cancel_edge_override() 解除，恢复正常行驶
///   5. 冲突保护：override 激活期间 update() 跳过心跳重发，
///      防止1/4号提案（心跳/PID帧）干扰边缘停车指令
class MotionService : public middleware::IRunnable {
   public:
    struct Config {
        float clean_speed_rpm{300.0f};   ///< 清扫行进速度（RPM）
        float return_speed_rpm{500.0f};  ///< 返回速度（RPM，快速）
        int brush_rpm{1200};             ///< 辊刷转速
        float edge_reverse_rpm{150.0f};  ///< 边缘触发后反转速度（RPM，0=原地停）
        bool heading_pid_en{true};       ///< 是否使能航向 PID
        device::WalkMotorGroup::HeadingPidParams pid{};  ///< PID 参数
    };

    /// @param group    4轮行走电机组（构造时已配置 comm_timeout_ms）
    /// @param brush    辊刷电机
    /// @param imu      IMU 设备（提供 yaw_deg）
    /// @param bus      事件总线
    /// @param cfg      运动配置
    MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                  std::shared_ptr<device::BrushMotor> brush,
                  std::shared_ptr<device::ImuDevice> imu,
                  middleware::EventBus& bus,
                  Config cfg);

    // ── 运动控制 ──────────────────────────────────────────────────────────
    /// 开始清扫前进（使能行走 + 辊刷，锁定航向目标）
    bool start_cleaning();

    /// 停止清扫（停辊刷，行走归零，禁用 PID）
    void stop_cleaning();

    /// 以返回速度反向行进
    bool start_returning();

    /// 原地急停（失能行走，停辊刷）
    void emergency_stop();

    /// 直接设置行走速度（RPM），同时更新 PID 基准速度
    bool set_walk_speed(float rpm);

    // ── 边缘触发接口 ─────────────────────────────────────────────────────
    /// 边缘触发时立即调用：override 帧直接发送，抑制心跳
    void on_edge_triggered();
    /// 恢复正常行驶（由 FSM/SafetyMonitor 在确认安全后调用）
    void cancel_edge_override();

    // ── 状态查询 ─────────────────────────────────────────────────────────
    bool is_moving() const;
    bool is_brush_running() const;
    bool is_edge_override_active() const;

    void update() override;  ///< 由 ThreadExecutor 20ms 调用（50Hz PID）

   private:
    std::shared_ptr<device::WalkMotorGroup> group_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::ImuDevice> imu_;
    middleware::EventBus& bus_;
    Config cfg_;
};

}  // namespace robot::service
