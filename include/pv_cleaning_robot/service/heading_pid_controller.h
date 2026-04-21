/*
 * @Author: UncleDrew
 * @Date: 2026-03-31 10:54:14
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-04-21 00:00:00
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/service/heading_pid_controller.h
 * @Description: 基于 IMU 陀螺仪 Z 轴角速度的航向速率 PID 控制器
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once

namespace robot::service {

/// 航向速率 PID 控制器：基于 IMU 陀螺仪 Z 轴角速度（deg/s）计算差速修正量（RPM）。
///
/// **设计原理（角速度控制 vs 绝对航向控制）：**
///
/// 旧方案（绝对航向）：error = target_yaw - current_yaw
///   缺陷：轨道几何强制使 yaw 沿轨道自然变化 ~10°，固定目标与轨道物理对抗，
///   导致返程 PID 持续输出最大差速，将机器人推出轨道。
///
/// 新方案（角速度控制）：error = 0 - omega_z = -omega_z
///   原理：机器人在直线轨道上行走时不应旋转（omega_z ≈ 0）。
///   - 轨道几何慢速变化 → omega_z < deadband_rate_dps → 不纠偏（物理自然复位）
///   - 突发偏转/出轨 → omega_z 大 → 立即纠偏
///   - 不需要初始化参考航向，正返程完全对称，无需区分处理。
///
/// **差速约定（IMU 右手法则，Z 向上，Y 为行进方向）：**
///   omega_z > 0（CCW，yaw 增大）→ correction < 0 → 下轨加速/上轨减速 → CW 纠偏 ✓
///   omega_z < 0（CW，yaw 减小）→ correction > 0 → 上轨加速/下轨减速 → CCW 纠偏 ✓
///   正返程均适用（公式对称）。
///
/// **非线程安全**：所有方法须在调用方的锁保护下调用（通常为 WalkMotorGroup::mtx_）。
class HeadingPidController {
   public:
    /// PID 参数
    struct Params {
        float kp{2.0f};              ///< 比例系数 [RPM/(deg/s)]；kp=2 在 omega_z=5°/s 时输出 10 RPM
        float ki{0.0f};              ///< 积分系数；速率控制通常不需要积分，默认 0；
                                     ///< 若存在持续单侧扭矩偏差（如坡度），可适当增大（≤0.1）
        float kd{0.0f};              ///< 微分系数；速率控制已含微分特性，通常不需要，默认 0
        float max_output{30.0f};     ///< 最大差速输出（RPM），防止饱和；建议设为基速的 50%
        float integral_limit{10.0f}; ///< 积分限幅（RPM）
        float deadband_rate_dps{2.0f}; ///< 死区角速度（°/s）；|omega_z| ≤ 此值时输出 0；
                                       ///< 建议 2~5°/s：可过滤轨道几何慢速漂移（通常 <1°/s），
                                       ///< 同时响应突发偏转（出轨时通常 >10°/s）
    };

    /// 使用默认参数构造
    HeadingPidController() = default;
    /// 使用指定参数构造
    explicit HeadingPidController(const Params& p);

    /// 热更新参数，不复位积分
    void set_params(const Params& p);

    /// 使能/禁用；禁用时自动复位积分状态
    void enable(bool en);

    /// 复位积分状态（不改变使能状态）
    void reset();

    bool is_enabled() const {
        return enabled_;
    }

    /// 计算差速修正值（RPM）。
    ///
    /// error = -omega_z_dps（目标角速度为 0，偏差 = 当前角速度取反）
    ///
    /// 积分抗饱和：误差过零时自动清零积分；死区内不累积积分。
    ///
    /// @param omega_z_dps  当前 Z 轴角速度（°/s），来自 IMU gyro[2] × (180/π)；
    ///                     正值 = CCW（yaw 增大），负值 = CW（yaw 减小）
    /// @param dt_s         控制周期（秒），≤0 时微分项置 0
    /// @return  差速修正量（RPM），未使能时始终返回 0.0f
    float compute(float omega_z_dps, float dt_s);

   private:
    Params params_;
    bool enabled_{false};
    float integral_{0.0f};
    float prev_err_{0.0f};

    /// 通用限幅：将 v 限制在 [lo, hi] 范围内
    static float clamp(float v, float lo, float hi);
};

}  // namespace robot::service
