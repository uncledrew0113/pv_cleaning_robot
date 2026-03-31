#pragma once
#include <memory>
#include <tuple>
#include <vector>

#include "pv_cleaning_robot/device/walk_motor_group.h"

/// @brief WalkMotorGroup 可注入替身（不继承，供模板或 shared_ptr 替换使用）
///
/// 由于 WalkMotorGroup 无虚接口，MotionService/SafetyMonitor 测试中使用真实
/// WalkMotorGroup(MockCanBus) 组合；此结构体用于直接验证 emergency_override
/// 和 set_speeds 调用序列，在需要 duck-type 替代的测试中使用。
struct MockWalkMotorGroup {
    // ── 可配置返回值 ──────────────────────────────────────────
    robot::device::DeviceError open_result{robot::device::DeviceError::OK};
    robot::device::DeviceError set_modes_result{robot::device::DeviceError::OK};
    robot::device::DeviceError enable_result{robot::device::DeviceError::OK};
    robot::device::DeviceError set_speeds_result{robot::device::DeviceError::OK};
    robot::device::DeviceError emergency_result{robot::device::DeviceError::OK};
    robot::device::WalkMotorGroup::GroupStatus status_result{};
    bool override_active_result{false};

    // ── 调用记录 ──────────────────────────────────────────────
    int open_count{0};
    int close_count{0};
    int enable_all_count{0};
    int disable_all_count{0};
    int emergency_override_count{0};
    int clear_override_count{0};
    float last_emergency_rpm{-999.0f};
    std::vector<std::tuple<float, float, float, float>> set_speeds_calls;
    float last_uniform_rpm{0.0f};
    int set_uniform_count{0};
    float last_target_heading{0.0f};
    bool heading_ctrl_en{false};

    robot::device::DeviceError open() {
        ++open_count;
        return open_result;
    }
    void close() {
        ++close_count;
    }

    robot::device::DeviceError set_mode_all(robot::protocol::WalkMotorMode) {
        return set_modes_result;
    }
    robot::device::DeviceError enable_all() {
        ++enable_all_count;
        return enable_result;
    }
    robot::device::DeviceError disable_all() {
        ++disable_all_count;
        return enable_result;
    }

    robot::device::DeviceError set_speeds(float lt, float rt, float lb, float rb) {
        set_speeds_calls.emplace_back(lt, rt, lb, rb);
        return set_speeds_result;
    }
    robot::device::DeviceError set_speed_uniform(float rpm) {
        last_uniform_rpm = rpm;
        ++set_uniform_count;
        return set_speeds_result;
    }

    robot::device::DeviceError emergency_override(float rpm = 0.0f) {
        ++emergency_override_count;
        last_emergency_rpm = rpm;
        return emergency_result;
    }
    void clear_override() {
        ++clear_override_count;
    }
    bool is_override_active() const {
        return override_active_result;
    }

    void set_heading_pid_params(const robot::device::WalkMotorGroup::HeadingPidParams&) {}
    void enable_heading_control(bool en) {
        heading_ctrl_en = en;
    }
    void set_target_heading(float yaw) {
        last_target_heading = yaw;
    }

    robot::device::WalkMotorGroup::GroupStatus get_group_status() const {
        return status_result;
    }
    robot::device::WalkMotor::Status get_wheel_status(robot::device::WalkMotorGroup::Wheel) const {
        return {};
    }

    void update(float = 0.0f) {}
};
