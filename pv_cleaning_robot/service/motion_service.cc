/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-05-09 15:23:06
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/motion_service.cc
 * @Description: 运动服务——集成 WalkMotorGroup + IMU 姿态纠偏 + 边缘触发覆盖
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

void MotionService::set_parking_side_provider(std::function<ParkingSide()> provider) {
    parking_side_provider_ = std::move(provider);
}

void MotionService::set_runtime_config_provider(std::function<TbRuntimeConfig()> provider) {
    runtime_config_provider_ = std::move(provider);
}

int MotionService::task_direction_sign() const {
    if (!parking_side_provider_)
        return 1;
    return parking_side_provider_() == ParkingSide::Left ? -1 : 1;
}

void MotionService::sync_runtime_config() {
    if (!runtime_config_provider_) {
        return;
    }
    const auto runtime_cfg = runtime_config_provider_();
    cfg_.clean_speed_rpm = static_cast<float>(runtime_cfg.clean_speed_rpm);
    cfg_.return_speed_rpm = static_cast<float>(runtime_cfg.return_speed_rpm);
    cfg_.brush_rpm = runtime_cfg.brush_rpm;
    cfg_.return_brush_rpm = runtime_cfg.return_brush_rpm;
}

bool MotionService::enable_speed_mode() {
    // M1502E 硬件确认：ENABLE 帧（0x01）会覆盖 SPEED 模式位 → 必须先 ENABLE 再 SPEED。
    if (group_->enable_all() != device::DeviceError::OK) {
        return false;
    }
    return group_->set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK;
}

void MotionService::sync_heading_pid_enabled() {
    if (!cfg_.heading_pid_en) {
        group_->enable_heading_control(false);
        return;
    }
    group_->set_heading_pid_params(cfg_.pid);
    group_->enable_heading_control(true);
}

bool MotionService::start_returning_impl(bool run_brush) {
    sync_runtime_config();
    group_->clear_override();

    const int dir = task_direction_sign();
    // brush_->set_mode_speed();
    if (run_brush) {
        // 返程刷反向转，用 return_brush_rpm 单独表达返程刷速。
        brush_->set_rpm(-cfg_.return_brush_rpm * dir);
    } else {
        brush_->stop();
    }

    sync_heading_pid_enabled();
    if (!enable_speed_mode()) {
        return false;
    }

    const float spd = cfg_.return_speed_rpm * static_cast<float>(dir);
    return group_->set_speeds(-spd, -spd, +spd, +spd) == device::DeviceError::OK;
}

// ── 运动控制 ──────────────────────────────────────────────────────────────

bool MotionService::start_cleaning() {
    sync_runtime_config();
    // 解除可能由 SafetyMonitor::on_limit_trigger() 触发的 emergency_override 锁
    group_->clear_override();

    if (!enable_speed_mode()) {
        return false;
    }

    // 清扫和返程都使用同一套 IMU 姿态纠偏，只是轮速方向不同。
    sync_heading_pid_enabled();

    // 物理安装：LT/RT 正转=前进，LB/RB 因安装方向相反，负转=前进
    // 车辆前进：LT=+spd, RT=+spd, LB=-spd, RB=-spd
    const int dir = task_direction_sign();
    const float spd = cfg_.clean_speed_rpm * static_cast<float>(dir);
    if (group_->set_speeds(spd, spd, -spd, -spd) != device::DeviceError::OK)
        return false;

    // brush_->set_mode_speed();
    brush_->set_rpm(cfg_.brush_rpm * dir);
    return true;
}

void MotionService::stop_cleaning() {
    brush_->stop();
    group_->enable_heading_control(false);
    group_->set_speed_uniform(0.0f);
    group_->disable_all();
}

void MotionService::pause_task() {
    brush_->stop();
    group_->enable_heading_control(false);
    group_->set_speed_uniform(0.0f);
}

bool MotionService::start_returning() {
    return start_returning_impl(true);
}

bool MotionService::start_returning_no_brush() {
    // P1 故障返回只和正常返程差一个动作：停刷，不再反向继续扫。
    return start_returning_impl(false);
}

void MotionService::emergency_stop() {
    brush_->stop();
    group_->emergency_override(0.0f);  // 原地停止 + 抑制心跳
    group_->disable_all();
}

bool MotionService::set_walk_speed(float rpm) {
    // 正值语义统一为“从停机位驶向对侧端点”。
    const float directed_rpm = rpm * static_cast<float>(task_direction_sign());
    return group_->set_speeds(directed_rpm, directed_rpm, -directed_rpm, -directed_rpm) ==
           device::DeviceError::OK;
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
    device::ImuDevice::ImuData imu_data{};
    if (imu_) {
        imu_data = imu_->get_latest();
    }
    const float raw_pitch = imu_data.pitch_deg;
    const float raw_roll = imu_data.roll_deg;
    const float raw_yaw = imu_data.yaw_deg;
    const float raw_omega_z = imu_data.gyro[2] * (180.0f / 3.14159265f);

    // EMA 低通滤波（α=0.8，τ≈100ms @50ms 周期），抑制 IMU 高频噪声对姿态纠偏的扰动。
    // 首次调用硬初始化，避免从 0 缓慢收敛引发控制初始抖动；
    // 使用成员变量（非 static）解决多实例共享和 IMU 重启后的污染问题。
    if (!filtered_pitch_inited_) {
        filtered_pitch_ = raw_pitch;
        filtered_pitch_inited_ = true;
    } else {
        filtered_pitch_ = 0.8f * filtered_pitch_ + 0.2f * raw_pitch;
    }

    if (!filtered_roll_inited_) {
        filtered_roll_ = raw_roll;
        filtered_roll_inited_ = true;
    } else {
        filtered_roll_ = 0.8f * filtered_roll_ + 0.2f * raw_roll;
    }

    if (!filtered_yaw_inited_) {
        filtered_yaw_ = raw_yaw;
        filtered_yaw_inited_ = true;
    } else {
        filtered_yaw_ = 0.8f * filtered_yaw_ + 0.2f * raw_yaw;
    }

    if (!filtered_omega_z_inited_) {
        filtered_omega_z_ = raw_omega_z;
        filtered_omega_z_inited_ = true;
    } else {
        filtered_omega_z_ = 0.8f * filtered_omega_z_ + 0.2f * raw_omega_z;
    }

    // 传入过滤后的 pitch/roll/yaw/gyro_z，由 WalkMotorGroup::update() 完成：
    //   1. 更新 online 超时状态
    //   2. 排干命令队列（消费 set_speeds/clear_override 投递的 Cmd）
    //   3. override 激活时跳过重发（不干扰紧急停车帧）
    //   4. 若 heading_ctrl_en_，计算姿态差速修正并发帧
    group_->update(filtered_pitch_, filtered_roll_, filtered_yaw_, filtered_omega_z_);

    // 注意：brush_->update() 已移到 bms_exec 线程（SCHED_OTHER, 500ms）
    // 原因：Modbus RTU 读取寄存器需 5~10ms 阻塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
    // 中将占用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期变化，
    // 移至低优先级 bms_exec 线程可完全消除对运动控制周期的干扰。
}

}  // namespace robot::service
