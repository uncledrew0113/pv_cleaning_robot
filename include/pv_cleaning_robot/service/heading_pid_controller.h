/*
 * @Author: UncleDrew
 * @Date: 2026-03-31 10:54:14
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-31 11:03:49
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/service/heading_pid_controller.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once

namespace robot::service {

/// 航向 PID 控制器：根据当前航向与目标航向误差计算差速修正量（RPM）。
///
/// **非线程安全**：所有方法须在调用方的锁保护下调用（通常为 WalkMotorGroup::mtx_）。
/// 本类不含任何硬件/CAN 依赖，便于独立单元测试。
///
/// 差速约定：correction > 0 → 偏右（yaw < target），左轮加速/右轮减速；
///           correction < 0 → 偏左，右轮加速/左轮减速。
class HeadingPidController {
   public:
    /// PID 调参（与原 WalkMotorGroup::HeadingPidParams 字段完全一致）
    struct Params {
        float kp{0.5f};               ///< 比例系数
        float ki{0.05f};              ///< 积分系数
        float kd{0.1f};               ///< 微分系数
        float max_output{30.0f};      ///< 最大差速输出（RPM），防止饱和
        float integral_limit{20.0f};  ///< 积分限幅（RPM）
    };

    /// 使用默认参数构造
    HeadingPidController() = default;
    /// 使用指定参数构造
    explicit HeadingPidController(const Params& p);

    /// 热更新参数，不复位积分
    void set_params(const Params& p);

    /// 使能/禁用；禁用时自动复位积分状态
    void enable(bool en);

    /// 设置目标航向（同时复位积分，首次调用时解除自动锁定）
    void set_target(float yaw_deg);

    /// 复位积分状态（不改变使能状态）
    void reset();

    bool is_enabled() const {
        return enabled_;
    }

    /// 计算差速修正值（RPM）。
    /// 首次调用时若尚未 set_target()，自动将当前 yaw 锁定为目标（保持直行）。
    /// @param yaw_deg  当前航向角（来自 IMU，°）
    /// @param dt_s     控制周期（秒），≤0 时微分项置 0
    /// @return  差速修正量（RPM），未使能时始终返回 0.0f
    float compute(float yaw_deg, float dt_s);

   private:
    Params params_;
    bool enabled_{false};
    float target_{0.0f};
    bool initialized_{false};
    float integral_{0.0f};
    float prev_err_{0.0f};

    static float norm_angle(float deg);
    static float clamp(float v, float lo, float hi);
};

}  // namespace robot::service
