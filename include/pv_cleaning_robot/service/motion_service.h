#pragma once
#include <atomic>
#include <functional>
#include <memory>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::service {

/// @brief 运动控制服务——协调行走电机组与滚刷电机
///
/// 新增功能：
///   1. 通信超时保活：WalkMotorGroup 在 open() 时自动下发 comm_timeout_ms
///      给每台电机；update() 每 20ms 重发设定值维持心跳，超时自停
///   2. 主动上报+温度查询：反馈方式使用主动上报（100Hz），
///      无法上报温度时由 WalkMotorGroup::update() 定期发 0x107 查询补采
///   3. 航向速率 PID：基于 IMU 陀螺仪 Z 轴角速度（deg/s），
///      update() 每 20ms 计算差速补偿，目标角速度始终为 0
///   4. 边缘紧急响应：register_edge_callback() 注册后，
///      边缘触发时 emergency_override() 立即发帧（不经过 update() 调度），
///      并用 cancel_edge_override() 解除，恢复正常行驶
///   5. 冲突保护：override 激活期间 update() 跳过心跳重发，
///      防止1/4号提案（心跳/PID帧）干扰边缘停车指令
class MotionService : public middleware::IRunnable {
   public:
    struct Config {
        float clean_speed_rpm{30.0f};   ///< 清扫行进速度（RPM）
        float return_speed_rpm{30.0f};  ///< 返回速度（RPM，快速）
        int brush_rpm{1200};            ///< 滚刷正向转速
        int return_brush_rpm{1200};     ///< 滚刷返程转速（绝对值，实际方向取反）
        float edge_reverse_rpm{30.0f};  ///< 边缘触发后反转速度（RPM，0=原地停）
        bool heading_pid_en{true};      ///< 是否使能航向 PID
        device::WalkMotorGroup::HeadingPidParams pid{};  ///< PID 参数
    };

    /// @param group    4轮行走电机组（构造时已配置 comm_timeout_ms）
    /// @param brush    滚刷电机
    /// @param imu      IMU 设备（提供 yaw_deg）
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
    /// 开始清扫前进（使能行走 + 滚刷，锁定航向目标）
    bool start_cleaning();

    /// 停止清扫（停滚刷，行走归零，禁用 PID）
    void stop_cleaning();

    /// 暂停任务（停滚刷，速度归零，保留驱动可恢复状态）
    void pause_task();

    /// 以返回速度反向行进（滚刷反向运行，保持航向 PID）
    bool start_returning();

    /// @brief P1 故障路径：先停滚刷，再以返回速度倒退回停机位。
    ///
    /// 由 RobotFsm::dispatch<EvFaultP1>() 调用；与 start_returning() 的区别是
    /// 滚刷立即停止（不反向运行），适用于需要避免滚刷二次损伤的故障场景。
    bool start_returning_no_brush();

    /// 原地急停（失能行走，停滚刷）
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
    std::function<ParkingSide()> parking_side_provider_;
    std::function<TbRuntimeConfig()> runtime_config_provider_;

    /// EMA 滤波后的 yaw（替代 update() 中的 static 局部变量，解决多实例共享和重启污染）
    float filtered_yaw_{0.0f};
    bool filtered_yaw_inited_{false};
    float filtered_omega_z_{0.0f};   ///< EMA 滤波后的 Z 轴角速度（°/s），用于速率 PID 输入
    bool filtered_omega_z_inited_{false};  ///< 首次调用时硬初始化标志

    int task_direction_sign() const;
    void sync_runtime_config();
};

}  // namespace robot::service
