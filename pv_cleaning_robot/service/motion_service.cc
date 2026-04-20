/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-30 16:02:45
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/motion_service.cc
 * @Description: 运动服务——集成 WalkMotorGroup + IMU 航向 PID + 边缘触发覆盖
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include <cmath>

#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::service {

MotionService::MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                             std::shared_ptr<device::BrushMotor> brush,
                             std::shared_ptr<device::ImuDevice> imu,
                             middleware::EventBus& bus,
                             Config cfg)
    : group_(std::move(group))
    , brush_(std::move(brush))
    , imu_(std::move(imu))
    , bus_(bus)
    , cfg_(cfg) {}

// ── 运动控制 ──────────────────────────────────────────────────────────────

bool MotionService::start_cleaning() {
    // 解除可能由 SafetyMonitor::on_limit_trigger() 触发的 emergency_override 锁
    group_->clear_override();

    // 先使能，再切换速度环模式
    // M1502E 硬件确认：ENABLE 帧（0x01）会覆盖 SPEED 模式位 → 必须先 ENABLE 再 SPEED
    if (group_->enable_all() != device::DeviceError::OK)
        return false;
    if (group_->set_mode_all(protocol::WalkMotorMode::SPEED) != device::DeviceError::OK)
        return false;

    // 如果 heading PID 使能，以当前 IMU yaw 为目标航向
    if (cfg_.heading_pid_en) {
        group_->set_heading_pid_params(cfg_.pid);
        const float cur_yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;
        group_->set_target_heading(cur_yaw);
        group_->enable_heading_control(true);
    }

    // 物理安装：LT/RT 正转=前进，LB/RB 因安装方向相反，负转=前进
    // 车辆前进：LT=+spd, RT=+spd, LB=-spd, RB=-spd
    const float spd = cfg_.clean_speed_rpm;
    if (group_->set_speeds(spd, spd, -spd, -spd) != device::DeviceError::OK)
        return false;

    brush_->set_rpm(cfg_.brush_rpm);
    brush_->start();
    return true;
}

void MotionService::stop_cleaning() {
    brush_->stop();
    group_->enable_heading_control(false);
    group_->set_speed_uniform(0.0f);
    group_->disable_all();
}

bool MotionService::start_returning() {
    // 解除可能由 SafetyMonitor 触发的 emergency_override 锁，确保心跳正常恢复
    group_->clear_override();

    // 返程滚刷反向运行（清洁板面残留，绝对值同 brush_rpm，方向取反）
    brush_->set_rpm(-static_cast<float>(cfg_.return_brush_rpm));
    brush_->start();

    // 返程保持航向 PID，但配合自适应目标跟踪（target_tracking_alpha < 1.0）：
    //   - 以当前 yaw（正向终点）为初始目标，PID 在目标自适应跟踪下不再以固定偏移值对抗轨道复位力
    //   - 轨道几何引起的慢速 yaw 变化（τ < 2s）：目标自动跟踪 → PID 误差小 → 纠偏力弱 → 不脱轨
    //   - 突发偏转（出轨等异常）：目标跟不上 → PID 大力纠正 → 保证安全
    // 根因分析（doc/pid.txt）：
    //   旧做法以正向终点 yaw（~141°）为固定返程目标，轨道自然将 yaw 压回 134°，
    //   PID 对抗轨道复位力产生 20+ RPM 差速（LT≈-9, LB≈+30）→ 脱轨。
    //   自适应跟踪（alpha=0.99）下稳态误差 ≤ 0.6°，纠偏力 ≤ 0.9 RPM，彻底解决此问题。
    if (cfg_.heading_pid_en) {
        const float cur_yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;
        group_->set_target_heading(cur_yaw);
        group_->enable_heading_control(true);
    }

    // 先使能，再切换速度环（M1502E：ENABLE 帧覆盖模式位，Q5 修复）
    if (group_->enable_all() != device::DeviceError::OK)
        return false;
    if (group_->set_mode_all(protocol::WalkMotorMode::SPEED) != device::DeviceError::OK)
        return false;

    // 物理安装：LT/RT 负转=后退，LB/RB 安装相反正转=后退
    // 车辆后退：LT=-spd, RT=-spd, LB=+spd, RB=+spd
    const float spd = cfg_.return_speed_rpm;
    if (group_->set_speeds(-spd, -spd, +spd, +spd) != device::DeviceError::OK)
        return false;

    return true;
}

bool MotionService::start_returning_no_brush() {
    // P1 故障路径：停刷再反向返回（保持航向 PID 防止返程漂移）
    brush_->stop();
    group_->clear_override();

    // 保持航向 PID（与 start_returning() 一致，自适应目标跟踪；
    // 原 Q9 修复"保持 PID"依然正确，但原因更新为：靠自适应跟踪而非固定目标）
    if (cfg_.heading_pid_en) {
        const float cur_yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;
        group_->set_target_heading(cur_yaw);
        group_->enable_heading_control(true);
    }

    // 先使能，再切换速度环（M1502E：ENABLE 帧覆盖模式位，Q5 修复）
    if (group_->enable_all() != device::DeviceError::OK)
        return false;
    if (group_->set_mode_all(protocol::WalkMotorMode::SPEED) != device::DeviceError::OK)
        return false;

    const float spd = cfg_.return_speed_rpm;
    if (group_->set_speeds(-spd, -spd, +spd, +spd) != device::DeviceError::OK)
        return false;

    return true;
}

void MotionService::emergency_stop() {
    brush_->stop();
    group_->emergency_override(0.0f);  // 原地停止 + 抑制心跳
    group_->disable_all();
}

bool MotionService::set_walk_speed(float rpm) {
    // 物理安装：LB/RB 安装方向与 LT/RT 相反，需取反才能同向运动
    return group_->set_speeds(rpm, rpm, -rpm, -rpm) == device::DeviceError::OK;
}

// ── 边缘触发接口 ──────────────────────────────────────────────────────────

void MotionService::on_edge_triggered() {
    // 直接调用 WalkMotorGroup::emergency_override()
    // 该函数立即向 CAN 总线发送停车/反转帧，并设置 override_active_ 标志，
    // 阻止 update() 继续重发正常心跳帧，保证安全状态不被覆盖
    group_->emergency_override(cfg_.edge_reverse_rpm);
    brush_->stop();
}

void MotionService::cancel_edge_override() {
    // 由上层 FSM 或 SafetyMonitor 在确认安全后调用
    group_->clear_override();
}

// ── 状态查询 ──────────────────────────────────────────────────────────────

bool MotionService::is_moving() const {
    const auto st = group_->get_group_status();
    for (const auto& w : st.wheel) {
        if (std::abs(w.speed_rpm) > 5.0f)
            return true;
    }
    return false;
}

bool MotionService::is_brush_running() const {
    return brush_->get_status().running;
}

bool MotionService::is_edge_override_active() const {
    return group_->is_override_active();
}

// ── 周期心跳（50 ms，由 ThreadExecutor 调用）──────────────────────────────

void MotionService::update() {
    // 读取最新 IMU yaw（已由 ImuDevice 后台线程更新，无阻塞）
    const float raw_yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;
    // EMA 低通滤波（α=0.8，τ≈100ms @50ms 周期），抑制 IMU 高频噪声对 PID 的扰动
    // 首次调用硬初始化（直接赋值 raw_yaw），避免从 0 缓慢收敛引发 PID 初始抖动；
    // 使用成员变量（非 static）解决多实例共享和 IMU 重启后的污染问题。
    if (!filtered_yaw_inited_) {
        filtered_yaw_ = raw_yaw;
        filtered_yaw_inited_ = true;
    } else {
        filtered_yaw_ = 0.8f * filtered_yaw_ + 0.2f * raw_yaw;
    }

    // 传入 yaw，由 WalkMotorGroup::update() 完成：
    //   1. 更新 online 超时状态
    //   2. 排干命令队列（消费 set_speeds/clear_override 投递的 Cmd）
    //   3. override 激活时跳过重发（不干扰紧急停车帧）
    //   4. 若 heading_ctrl_en_，计算 PID 差速修正并发帧
    group_->update(filtered_yaw_);

    // 注意：brush_->update() 已移到 bms_exec 线程（SCHED_OTHER, 500ms）
    // 原因：Modbus RTU 读取寄存器需 5~10ms 阻塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
    // 中将占用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期变化，
    // 移至低优先级 bms_exec 线程可完全消除对运动控制周期的干扰。
}

}  // namespace robot::service
