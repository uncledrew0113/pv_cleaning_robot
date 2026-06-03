/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-05-27 16:35:16
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/motion_service.cc
 * @Description: 运动服务——集成 WalkMotorGroup + 视觉纠偏 + 边缘触发覆盖
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include <cmath>

#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::service {

namespace {

HeadingCorrector::SpeedCommand to_corrector_command(const device::WalkMotorGroup::SpeedCmd& cmd) {
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

void MotionService::set_primary_dock_query(std::function<domain::Endpoint()> query) {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    primary_dock_query_ = std::move(query);
}

void MotionService::set_runtime_config_query(std::function<RuntimeConfig()> query) {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    runtime_config_query_ = std::move(query);
}

MotionService::StateSnapshot MotionService::snapshot_state() const {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    return StateSnapshot{cfg_,
                         primary_dock_query_,
                         base_speed_cmd_,
                         walk_command_active_,
                         travel_direction_,
                         command_generation_};
}

domain::Endpoint MotionService::primary_dock() const {
    const auto query = snapshot_state().primary_dock_query;
    return query ? query() : domain::Endpoint::B;
}

int MotionService::target_direction_sign(domain::Endpoint target) const {
    // 物理标定：向 A 端运行时上轨道轮正转、下轨道轮反转；向 B 端整体取反。
    return target == domain::Endpoint::A ? 1 : -1;
}

void MotionService::sync_runtime_config() {
    std::function<RuntimeConfig()> query;
    {
        std::lock_guard<hal::PiMutex> lk(state_mtx_);
        query = runtime_config_query_;
    }
    if (!query) {
        return;
    }
    const auto runtime_cfg = query();
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    cfg_.clean_speed_rpm = std::abs(static_cast<float>(runtime_cfg.clean_speed_rpm));
    cfg_.return_speed_rpm = std::abs(static_cast<float>(runtime_cfg.return_speed_rpm));
    cfg_.brush_rpm = std::abs(runtime_cfg.brush_rpm);
}

bool MotionService::enable_speed_mode() {
    // M1502E 硬件确认：ENABLE 帧（0x01）会覆盖 SPEED 模式位 → 必须先 ENABLE 再 SPEED。
    if (group_->enable_all() != device::DeviceError::OK) {
        return false;
    }
    return group_->set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK;
}

void MotionService::sync_heading_pid_enabled() {
    const auto state = snapshot_state();
    heading_corrector_.set_params(state.cfg.pid);
    if (!state.cfg.heading_pid_en) {
        heading_corrector_.enable(false);
        return;
    }
    if (heading_corrector_.is_enabled()) {
        heading_corrector_.reset();
    } else {
        heading_corrector_.enable(true);
    }
}

void MotionService::activate_walk_command(const device::WalkMotorGroup::SpeedCmd& cmd,
                                          domain::TravelDirection direction) {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    travel_direction_ = direction;
    base_speed_cmd_ = cmd;
    walk_command_active_ = true;
    ++command_generation_;
}

void MotionService::deactivate_walk_command() {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    walk_command_active_ = false;
    ++command_generation_;
}

void MotionService::update_heading_correction(const StateSnapshot& state, bool override_active) {
    if (!state.walk_command_active || !state.cfg.heading_pid_en || override_active) {
        return;
    }

    HeadingCorrector::Input input;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.base_command = to_corrector_command(state.base_speed_cmd);
    input.travel_direction = state.travel_direction;
    input.primary_dock =
        state.primary_dock_query ? state.primary_dock_query() : domain::Endpoint::B;

    const auto imu_data = imu_ ? imu_->get_latest() : robot::device::ImuDevice::ImuData{};
    input.imu_valid = imu_data.valid;
    input.gyro_z_rad_s = imu_data.valid ? imu_data.gyro[2] : 0.0f;

    const auto diagnostics = group_->get_group_diagnostics();
    input.wheel_feedback.valid = true;
    input.wheel_feedback.rpm = {diagnostics.wheel[0].speed_rpm,
                                diagnostics.wheel[1].speed_rpm,
                                diagnostics.wheel[2].speed_rpm,
                                diagnostics.wheel[3].speed_rpm};
    input.wheel_feedback.current = {diagnostics.wheel[0].torque_a,
                                    diagnostics.wheel[1].torque_a,
                                    diagnostics.wheel[2].torque_a,
                                    diagnostics.wheel[3].torque_a};

    const auto heading_output = heading_corrector_.compute(input);
    if (heading_output.has_speed_command) {
        apply_speed_if_command_current(state.command_generation,
                                       to_group_command(heading_output.speed_command));
    }
}

void MotionService::apply_speed_if_command_current(
    uint32_t generation,
    const device::WalkMotorGroup::SpeedCmd& cmd) {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    if (!walk_command_active_ || command_generation_ != generation) {
        return;
    }
    // set_speeds() only updates WalkMotorGroup's normal control slot; CAN TX remains in update().
    static_cast<void>(group_->set_speeds(cmd));
}

MotionService::OverrideClearAction MotionService::observe_override_clear(uint32_t clear_generation) {
    std::lock_guard<hal::PiMutex> lk(state_mtx_);
    if (clear_generation == last_override_clear_generation_) {
        return {};
    }
    last_override_clear_generation_ = clear_generation;
    return OverrideClearAction{true,
                               walk_command_active_,
                               base_speed_cmd_,
                               command_generation_};
}

void MotionService::handle_override_clear() {
    const auto clear_action = observe_override_clear(group_->override_clear_generation());
    if (clear_action.changed) {
        heading_corrector_.reset();
    }
    if (clear_action.restore_base_speed) {
        apply_speed_if_command_current(clear_action.command_generation,
                                       clear_action.base_speed_cmd);
    }
}

// ── 运动控制 ──────────────────────────────────────────────────────────────

bool MotionService::start_cleaning_to(domain::Endpoint target) {
    sync_runtime_config();
    // 解除可能由 SafetyMonitor::on_limit_trigger() 触发的 emergency_override 锁
    group_->clear_override();
    deactivate_walk_command();

    if (!enable_speed_mode()) {
        return false;
    }

    // 清扫和返程都使用同一套视觉纠偏，只是轮速方向不同。
    sync_heading_pid_enabled();

    // 物理安装：LT/RT 与 LB/RB 方向相反；target_direction_sign() 决定沿 A/B 轴正反向。
    const int dir = target_direction_sign(target);
    const auto state = snapshot_state();
    const auto dock = state.primary_dock_query ? state.primary_dock_query() : domain::Endpoint::B;
    const float speed = target == dock ? state.cfg.return_speed_rpm : state.cfg.clean_speed_rpm;
    const float spd = std::abs(speed) * static_cast<float>(dir);
    const device::WalkMotorGroup::SpeedCmd cmd{spd, spd, -spd, -spd};
    if (group_->set_speeds(cmd) != device::DeviceError::OK)
        return false;
    activate_walk_command(cmd, domain::travel_direction_to(target));

    brush_->set_rpm(std::abs(state.cfg.brush_rpm) * dir);
    return true;
}

void MotionService::stop_cleaning() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    brush_->stop();
    group_->set_speed_uniform(0.0f);
    group_->disable_all();
}

bool MotionService::start_segment(const domain::MissionSegment& segment) {
    switch (segment.mode) {
    case domain::SegmentMode::Cleaning:
        return start_cleaning_to(segment.target);
    }
    return false;
}

void MotionService::emergency_stop() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    brush_->stop();
    group_->emergency_override(0.0f);  // 原地停止 + 抑制心跳
    group_->disable_all();
}

// ── 周期心跳（50 ms，由 ThreadExecutor 调用）──────────────────────────────

void MotionService::update() {
    const bool override_active = group_->is_override_active();
    const auto state = snapshot_state();
    update_heading_correction(state, override_active);
    group_->update();
    handle_override_clear();

    // 注意：brush_->update() 已移到 bms_exec 线程（SCHED_OTHER, 500ms）
    // 原因：Modbus RTU 读取寄存器需 5~10ms 阻塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
    // 中将占用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期变化，
    // 移至低优先级 bms_exec 线程可完全消除对运动控制周期的干扰。
}

HeadingCorrector::DebugState MotionService::heading_pid_debug_state() const {
    return heading_corrector_.debug_state();
}

}  // namespace robot::service
