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
/// 差速约定（IMU 顺时针为正/CW+，上轨=LT+RT，下轨=LB+RB）：
///   correction > 0 → 偏左（yaw < target，向左偏转未达目标），上轨加速/下轨减速 → 机器人向右纠偏；
///   correction < 0 → 偏右（yaw > target，向右偏转超过目标），上轨减速/下轨加速 → 机器人向左纠偏。
/// 同一轨道两轮物理约束相同，速度必须一致（lt==rt，lb==rb）；下轨 base 取反体现反向安装。
class HeadingPidController {
   public:
    /// PID 调参（与原 WalkMotorGroup::HeadingPidParams 字段完全一致）
    struct Params {
        float kp{1.5f};               ///< 比例系数；kp=1.5 在 err=2° 时输出 3 RPM，可抵消接缝处 ~3°/s 扰动
        float ki{0.05f};              ///< 积分系数；配合符号翻转清零策略，用于消除稳态摩擦偏差
        float kd{0.3f};               ///< 微分系数；提高对接缝冲击的预测阻尼（原 0.1 太弱）
        float max_output{30.0f};      ///< 最大差速输出（RPM），防止饱和；测试时应改为 base_speed×50%
        float integral_limit{5.0f};   ///< 积分限幅（RPM）；符号翻转清零后可降至 5，避免残余积分压制 P 项
        float deadband_deg{0.5f};     ///< 死区（°），|err| ≤ deadband_deg 时输出 0 且不累积积分；0.0f = 关闭

        /// 目标 yaw 自适应跟踪系数 [0, 1]，每个控制周期（20ms）执行一步指数平滑：
        ///   target += (1 - alpha) × norm_angle(yaw - target)
        /// 效果：
        ///   alpha = 1.0：禁用跟踪，目标固定不变（旧行为，向后兼容）
        ///   alpha = 0.99：时间常数 τ ≈ 2s @50Hz；0.3°/s 几何漂移的稳态误差 ≈ 0.6°（接近死区）
        ///   alpha = 0.98：时间常数 τ ≈ 1s @50Hz；对突发偏移响应更快但几何漂移误差略大
        /// 原理：
        ///   轨道几何慢速漂移（速率 << 1/τ）→ 目标跟上 → PID 误差小 → 不对抗轨道自然复位力
        ///   突发偏转（速率 >> 1/τ，例如出轨）→ 目标跟不上 → 误差保持 → PID 大力纠正
        /// 推荐生产值：0.99（适合板间桥架过渡，τ=2s 可吸收 0.3°/s 的常规几何漂移）
        float target_tracking_alpha{1.0f};
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
    /// 积分抗饱和：误差过零（越过目标）时自动清零积分；死区内不累积积分。
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

    /// 将角度规范化到 (-180, +180] 区间，处理 0°/360° 跨越边界的误差计算
    static float norm_angle(float deg);
    /// 通用限幅：将 v 限制在 [lo, hi] 范围内
    static float clamp(float v, float lo, float hi);
};

}  // namespace robot::service
