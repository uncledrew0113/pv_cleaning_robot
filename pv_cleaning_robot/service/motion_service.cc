/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-25 18:39:24
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
    // 使能全部行走电机
    if (group_->enable_all() != device::DeviceError::OK)
        return false;

    // 如果 heading PID 使能，以当前 IMU yaw 为目标航向
    if (cfg_.heading_pid_en) {
        group_->set_heading_pid_params(cfg_.pid);
        const float cur_yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;
        group_->set_target_heading(cur_yaw);
        group_->enable_heading_control(true);
    }

    // 同步设定四轮清扫速度（左右同向前进，正值=前进）
    const float spd = cfg_.clean_speed_rpm;
    if (group_->set_speeds(spd, spd, spd, spd) != device::DeviceError::OK)
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
    brush_->stop();
    group_->enable_heading_control(false);

    if (group_->enable_all() != device::DeviceError::OK)
        return false;

    const float spd = -cfg_.return_speed_rpm;  // 负值=后退
    if (group_->set_speeds(spd, spd, spd, spd) != device::DeviceError::OK)
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
    const float yaw = imu_ ? imu_->get_latest().yaw_deg : 0.0f;

    // 传入 yaw，由 WalkMotorGroup::update() 完成：
    //   1. 更新 online 超时状态
    //   2. override 激活时跳过重发（不干扰紧急停车帧）
    //   3. 若 heading_ctrl_en_，计算 PID 差速修正并发帧
    //   4. 每 1000ms 发一次温度查询帧（主动上报不含温度）
    group_->update(yaw);

    brush_->update();
}

}  // namespace robot::service
