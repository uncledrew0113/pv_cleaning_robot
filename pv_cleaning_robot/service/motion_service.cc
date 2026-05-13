/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-05-12 12:13:29
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/motion_service.cc
 * @Description: 运动服务——集成 WalkMotorGroup + IMU 姿态纠偏 + 边缘触发覆盖
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::service {

namespace {

HeadingCorrector::SpeedCommand to_corrector_command(
    const device::WalkMotorGroup::SpeedCmd& cmd) {
    return {cmd.lt_rpm, cmd.rt_rpm, cmd.lb_rpm, cmd.rb_rpm};
}

device::WalkMotorGroup::SpeedCmd to_group_command(const HeadingCorrector::SpeedCommand& cmd) {
    return {cmd.lt_rpm, cmd.rt_rpm, cmd.lb_rpm, cmd.rb_rpm};
}

}  // namespace

MotionService::MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                             std::shared_ptr<device::BrushMotor> brush,
                             std::shared_ptr<device::ImuDevice> imu,
                             middleware::EventBus& bus,
                             Config cfg)
    : group_(std::move(group))
    , brush_(std::move(brush))
    , imu_(std::move(imu))
    , bus_(bus)
    , cfg_(cfg)
    , heading_corrector_(cfg.pid)
    , last_override_clear_generation_(group_ ? group_->override_clear_generation() : 0u) {}

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
    heading_corrector_.set_params(cfg_.pid);
    if (!cfg_.heading_pid_en) {
        heading_corrector_.enable(false);
        return;
    }
    if (heading_corrector_.is_enabled()) {
        heading_corrector_.reset();
    } else {
        heading_corrector_.enable(true);
    }
}

void MotionService::set_base_speed_command(const device::WalkMotorGroup::SpeedCmd& cmd) {
    base_speed_cmd_ = cmd;
    walk_command_active_ = true;
}

bool MotionService::start_returning_impl(bool run_brush) {
    sync_runtime_config();
    group_->clear_override();
    walk_command_active_ = false;

    const int dir = task_direction_sign();
    if (run_brush) {
        // 返程刷反向转，用 return_brush_rpm 单独表达返程刷速。
        brush_->set_rpm(cfg_.return_brush_rpm * dir);
    } else {
        brush_->stop();
    }

    sync_heading_pid_enabled();
    if (!enable_speed_mode()) {
        return false;
    }

    const float spd = cfg_.return_speed_rpm * static_cast<float>(dir);
    const device::WalkMotorGroup::SpeedCmd cmd{-spd, -spd, +spd, +spd};
    if (group_->set_speeds(cmd) != device::DeviceError::OK) {
        return false;
    }
    set_base_speed_command(cmd);
    return true;
}

// ── 运动控制 ──────────────────────────────────────────────────────────────

bool MotionService::start_cleaning() {
    sync_runtime_config();
    // 解除可能由 SafetyMonitor::on_limit_trigger() 触发的 emergency_override 锁
    group_->clear_override();
    walk_command_active_ = false;

    if (!enable_speed_mode()) {
        return false;
    }

    // 清扫和返程都使用同一套 IMU 姿态纠偏，只是轮速方向不同。
    sync_heading_pid_enabled();

    // 物理安装：LT/RT 正转=前进，LB/RB 因安装方向相反，负转=前进
    // 车辆前进：LT=+spd, RT=+spd, LB=-spd, RB=-spd
    const int dir = task_direction_sign();
    const float spd = cfg_.clean_speed_rpm * static_cast<float>(dir);
    const device::WalkMotorGroup::SpeedCmd cmd{spd, spd, -spd, -spd};
    if (group_->set_speeds(cmd) != device::DeviceError::OK)
        return false;
    set_base_speed_command(cmd);

    brush_->set_rpm(-cfg_.brush_rpm * dir);
    return true;
}

void MotionService::stop_cleaning() {
    brush_->stop();
    heading_corrector_.enable(false);
    walk_command_active_ = false;
    group_->set_speed_uniform(0.0f);
    group_->disable_all();
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
    heading_corrector_.enable(false);
    walk_command_active_ = false;
    group_->emergency_override(0.0f);  // 原地停止 + 抑制心跳
    group_->disable_all();
}

// ── 周期心跳（50 ms，由 ThreadExecutor 调用）──────────────────────────────

void MotionService::update() {
    device::ImuDevice::ImuData imu_data{};
    if (imu_) {
        imu_data = imu_->get_latest();
    }
    const bool override_active = group_->is_override_active();

    if (walk_command_active_ && cfg_.heading_pid_en && !override_active) {
        HeadingCorrector::Input input;
        input.raw_pitch_deg = imu_data.pitch_deg;
        input.raw_roll_deg = imu_data.roll_deg;
        input.raw_yaw_deg = imu_data.yaw_deg;
        input.raw_gyro_z_dps = imu_data.gyro[2] * (180.0f / 3.14159265f);
        input.dt_s = 0.02f;
        input.has_base_command = true;
        input.base_command = to_corrector_command(base_speed_cmd_);

        const auto status = group_->get_group_status();
        const auto diagnostics = group_->get_group_diagnostics();
        input.wheel_feedback.valid = true;
        input.wheel_feedback.rpm = {status.wheel[0].speed_rpm,
                                    status.wheel[1].speed_rpm,
                                    status.wheel[2].speed_rpm,
                                    status.wheel[3].speed_rpm};
        input.wheel_feedback.current = {diagnostics.wheel[0].torque_a,
                                        diagnostics.wheel[1].torque_a,
                                        diagnostics.wheel[2].torque_a,
                                        diagnostics.wheel[3].torque_a};

        const auto heading_output = heading_corrector_.compute(input);
        if (heading_output.has_speed_command) {
            group_->set_speeds(to_group_command(heading_output.speed_command));
        }
    }

    group_->update();

    const auto clear_generation = group_->override_clear_generation();
    if (clear_generation != last_override_clear_generation_) {
        last_override_clear_generation_ = clear_generation;
        heading_corrector_.reset();
        if (walk_command_active_) {
            group_->set_speeds(base_speed_cmd_);
        }
    }

    // 注意：brush_->update() 已移到 bms_exec 线程（SCHED_OTHER, 500ms）
    // 原因：Modbus RTU 读取寄存器需 5~10ms 阻塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
    // 中将占用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期变化，
    // 移至低优先级 bms_exec 线程可完全消除对运动控制周期的干扰。
}

}  // namespace robot::service
