/**
 * @file motion_service.cc
 * @brief 机器人运动控制服务实现。
 *
 * 本文件实现清扫段启动、周期纠偏、滚刷控制、急停和姿态回中所需的行走电机命令封装。
 * 安全相关入口保持短路径，复杂恢复策略由 AttitudeLimitService 和 RecoveryExecutor 编排。
 */
#include <cmath>
#include <thread>

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
    // PID / 融合预测必须使用真实控制周期；写死 20ms 会导致 50ms walk_ctrl 下
    // 积分和微分时间基准错误。非法配置回退到当前主程序默认的 50ms。
    input.dt_s = state.cfg.control_dt_s > 0.0f ? state.cfg.control_dt_s : 0.05f;
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
    // set_speeds() 只更新 WalkMotorGroup 的 normal 控制槽；实际 CAN 发送仍由 update() 周期完成。
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

bool MotionService::start_cleaning_to(domain::Endpoint target) {
    sync_runtime_config();
    // 解除可能由 SafetyMonitor 触发的安全覆盖；随后由当前任务段重新接管 normal 电机命令。
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

    const int brush_direction_sign = state.cfg.brush_direction_sign < 0 ? -1 : 1;
    brush_->set_rpm(std::abs(state.cfg.brush_rpm) * dir * brush_direction_sign);
    return true;
}

void MotionService::stop_cleaning() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    // 正常停机也先清行走轮命令，避免滚刷串口锁或写超时拖慢车辆停止。
    group_->set_speed_uniform(0.0f);
    group_->disable_all();
    brush_->stop();
}

bool MotionService::start_segment(const domain::MissionSegment& segment) {
    switch (segment.mode) {
    case domain::SegmentMode::Cleaning:
        return start_cleaning_to(segment.target);
    }
    return false;
}

bool MotionService::hold_at_endpoint() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    const bool initial_zero_ok =
        group_->emergency_override(0.0f) == device::DeviceError::OK;
    const bool brush_ok = brush_->stop() == device::DeviceError::OK;
    if (!initial_zero_ok) {
        return false;
    }
    const bool speed_mode_ok = enable_speed_mode();
    const bool confirmed_zero_ok =
        group_->emergency_override(0.0f) == device::DeviceError::OK;
    return speed_mode_ok && confirmed_zero_ok && brush_ok;
}

void MotionService::emergency_stop() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    // 急停优先级：行走轮先进入 override 停车，避免滚刷串口锁或写超时拖慢行走制动。
    group_->emergency_override(0.0f);  // 原地停止 + 抑制心跳
    group_->disable_all();
    brush_->stop();
}

bool MotionService::reverse_for_recovery(std::chrono::milliseconds duration,
                                         std::chrono::milliseconds tick,
                                         std::function<bool()> interrupted) {
    if (duration <= std::chrono::milliseconds::zero() || tick <= std::chrono::milliseconds::zero()) {
        return false;
    }

    const auto state = snapshot_state();
    const auto reverse_target = state.travel_direction == domain::TravelDirection::AToB
                                    ? domain::Endpoint::A
                                    : domain::Endpoint::B;
    const auto deadline = std::chrono::steady_clock::now() + duration;
    auto wait_until_deadline = [&] {
        while (std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(tick);
        }
    };

    // 恢复后退复用正常任务段的方向换算和 PID 使能逻辑，只额外关闭滚刷。
    if (!start_cleaning_to(reverse_target)) {
        emergency_stop();
        wait_until_deadline();
        return true;
    }
    brush_->stop();

    while (std::chrono::steady_clock::now() < deadline) {
        if (interrupted && interrupted()) {
            emergency_stop();
            wait_until_deadline();
            return true;
        }
        std::this_thread::sleep_for(tick);
    }

    emergency_stop();
    return true;
}

bool MotionService::begin_attitude_center_motion() {
    // 姿态回中属于恢复动作，不继承清扫段 PID/滚刷状态；这里只准备最小运动能力。
    heading_corrector_.enable(false);
    deactivate_walk_command();
    brush_->stop();
    group_->clear_override();
    return enable_speed_mode();
}

bool MotionService::command_lower_wheels_for_attitude_center(float lower_rpm) {
    // AttitudeLimitService 负责决定 lower_rpm 的方向和时长；
    // MotionService 只保证所有行走电机命令仍从统一运动入口下发。
    return group_->set_speeds(0.0f, 0.0f, lower_rpm, lower_rpm) ==
           device::DeviceError::OK;
}

bool MotionService::stop_attitude_center_motion() {
    heading_corrector_.enable(false);
    deactivate_walk_command();
    brush_->stop();
    const bool speed_zero_ok =
        group_->set_speeds(0.0f, 0.0f, 0.0f, 0.0f) == device::DeviceError::OK;
    const bool disable_ok = group_->disable_all() == device::DeviceError::OK;
    return speed_zero_ok && disable_ok;
}

void MotionService::update() {
    const bool override_active = group_->is_override_active();
    const auto state = snapshot_state();
    update_heading_correction(state, override_active);
    group_->update();
    handle_override_clear();

    // 滚刷状态采集由独立 brush_exec 线程执行。Modbus RTU 读寄存器可能阻塞数毫秒，
    // 不放入 walk_ctrl 实时路径，避免 Modbus 阻塞占用行走控制周期预算。
}

HeadingCorrector::DebugState MotionService::heading_pid_debug_state() const {
    return heading_corrector_.debug_state();
}

}  // namespace robot::service
