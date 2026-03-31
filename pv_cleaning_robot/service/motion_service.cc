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

    // 返程辊刷反向运行（清洁板面残留，绝对值同 brush_rpm，方向取反）
    brush_->set_rpm(-static_cast<float>(cfg_.return_brush_rpm));
    brush_->start();

    // 保持航向 PID：锁定当前 yaw，防止返程漂移
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

    // 保持航向 PID（与 start_returning() 一致；Q9 修复：原来错误地禁用了 PID）
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
    return group_->set_speeds(rpm, rpm, rpm, rpm) == device::DeviceError::OK;
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
    static float filtered_yaw = raw_yaw;
    filtered_yaw = 0.8f * filtered_yaw + 0.2f * raw_yaw;

    // 传入 yaw，由 WalkMotorGroup::update() 完成：
    //   1. 更新 online 超时状态
    //   2. 排干命令队列（消费 set_speeds/clear_override 投递的 Cmd）
    //   3. override 激活时跳过重发（不干扰紧急停车帧）
    //   4. 若 heading_ctrl_en_，计算 PID 差速修正并发帧
    group_->update(filtered_yaw);

    // 注意：brush_->update() 已移到 bms_exec 线程（SCHED_OTHER, 500ms）
    // 原因：Modbus RTU 读取寄存器需 5~10ms阶塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
    // 中将住用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期平候
}

}  // namespace robot::service
