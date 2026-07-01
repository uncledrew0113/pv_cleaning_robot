#include "pv_cleaning_robot/service/attitude_limit_service.h"

#include <cmath>
#include <thread>
#include <utility>

namespace robot::service {

AttitudeLimitService::AttitudeLimitService(
    std::shared_ptr<device::AttitudeLimitSwitch> left,
    std::shared_ptr<device::AttitudeLimitSwitch> right,
    MotionPorts motion_ports)
    : left_(std::move(left))
    , right_(std::move(right))
    , motion_ports_(std::move(motion_ports)) {}

void AttitudeLimitService::start_monitoring() {
    if (!left_ || !right_) {
        return;
    }

    // GPIO 回调只记录确定性事件，不直接调用 ErrorManager，避免在 GPIO 监控线程中修改错误队列。
    left_->set_trigger_callback(
        [this](device::AttitudeLimitSide side) { handle_trigger(side); });
    right_->set_trigger_callback(
        [this](device::AttitudeLimitSide side) { handle_trigger(side); });
    left_->start_monitoring();
    right_->start_monitoring();
}

void AttitudeLimitService::stop_monitoring() {
    if (left_) {
        left_->stop_monitoring();
    }
    if (right_) {
        right_->stop_monitoring();
    }
}

std::optional<AttitudeLimitService::Event> AttitudeLimitService::consume_pending_event() {
    std::lock_guard<std::mutex> lk(event_mtx_);
    auto out = pending_event_;
    pending_event_.reset();
    return out;
}

void AttitudeLimitService::handle_trigger(device::AttitudeLimitSide side) {
    // 姿态限位属于安全事件：急停必须在 GPIO 回调线程内立即执行，
    // 后续 ErrorManager 决策和回中恢复可以由主循环按统一路径处理。
    if (motion_ports_.emergency_stop) {
        motion_ports_.emergency_stop();
    }

    const auto status = read_status();
    std::lock_guard<std::mutex> lk(event_mtx_);
    if (status.left_active && status.right_active) {
        pending_event_ = Event{EventType::AttitudeLimitBoth, side};
        return;
    }
    if (centering_active_) {
        // 回中期间的单侧姿态限位是回中算法的测量信号，不再作为新错误提交。
        // 急停已经在本函数入口执行，回中流程随后通过 read_status() 继续推进或超时退出。
        return;
    }
    pending_event_ = Event{EventType::AttitudeLimit, side};
}

AttitudeLimitService::Status AttitudeLimitService::read_status() const {
    const bool left_active = left_ && !left_->read_current_level();
    const bool right_active = right_ && !right_->read_current_level();
    return Status{left_active, right_active};
}

std::optional<device::AttitudeLimitSide> AttitudeLimitService::active_side(Status status) {
    if (status.left_active && status.right_active) {
        return std::nullopt;
    }
    if (status.left_active) {
        return device::AttitudeLimitSide::LEFT_LOWER;
    }
    if (status.right_active) {
        return device::AttitudeLimitSide::RIGHT_LOWER;
    }
    return std::nullopt;
}

AttitudeLimitService::CenterPlan AttitudeLimitService::make_center_plan(
    device::AttitudeLimitSide side,
    float lower_rpm) {
    const float rpm = std::abs(lower_rpm);
    if (side == device::AttitudeLimitSide::LEFT_LOWER) {
        return CenterPlan{device::AttitudeLimitSide::LEFT_LOWER,
                          device::AttitudeLimitSide::RIGHT_LOWER,
                          rpm,
                          -rpm};
    }
    return CenterPlan{device::AttitudeLimitSide::RIGHT_LOWER,
                      device::AttitudeLimitSide::LEFT_LOWER,
                      -rpm,
                      rpm};
}

bool AttitudeLimitService::command_lower_wheels(float lower_rpm) {
    if (!motion_ports_.command_lower_wheels) {
        return false;
    }
    return motion_ports_.command_lower_wheels(lower_rpm);
}

bool AttitudeLimitService::stop_lower_wheels() {
    if (!command_lower_wheels(0.0f)) {
        return false;
    }
    return motion_ports_.stop_center_motion && motion_ports_.stop_center_motion();
}

bool AttitudeLimitService::lower_attitude_center() {
    return lower_attitude_center(CenterConfig{});
}

bool AttitudeLimitService::lower_attitude_center(const CenterConfig& config) {
    if (config.lower_rpm <= 0.0f || config.stable_samples_required <= 0 ||
        config.tick <= std::chrono::milliseconds::zero()) {
        return false;
    }
    if (!left_ || !right_ || !motion_ports_.prepare_center_motion ||
        !motion_ports_.prepare_center_motion()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(event_mtx_);
        centering_active_ = true;
        pending_event_.reset();
    }
    auto finish_centering = [this]() {
        std::lock_guard<std::mutex> lk(event_mtx_);
        centering_active_ = false;
    };

    auto initial_side = std::optional<device::AttitudeLimitSide>{};
    auto deadline = std::chrono::steady_clock::now() + config.search_timeout;
    // 阶段 1：确认当前触发侧。若进入恢复时两侧都未触发，则先向默认方向低速搜索。
    while (std::chrono::steady_clock::now() < deadline && !initial_side.has_value()) {
        const auto status = read_status();
        if (status.left_active && status.right_active) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        initial_side = active_side(status);
        if (!initial_side.has_value() && !command_lower_wheels(-std::abs(config.lower_rpm))) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        std::this_thread::sleep_for(config.tick);
    }
    if (!initial_side.has_value()) {
        stop_lower_wheels();
        finish_centering();
        return false;
    }

    const auto plan = make_center_plan(*initial_side, config.lower_rpm);
    int stable_release_samples = 0;
    auto release_at = std::chrono::steady_clock::time_point{};
    deadline = std::chrono::steady_clock::now() + config.release_timeout;
    // 阶段 2：沿离开触发侧的方向运动，记录触发侧稳定释放的时间点。
    while (std::chrono::steady_clock::now() < deadline &&
           stable_release_samples < config.stable_samples_required) {
        if (!command_lower_wheels(plan.initial_lower_rpm)) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        const auto status = read_status();
        if (status.left_active && status.right_active) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        const bool released = plan.release_side == device::AttitudeLimitSide::LEFT_LOWER
                                  ? !status.left_active
                                  : !status.right_active;
        if (released) {
            ++stable_release_samples;
            if (release_at == std::chrono::steady_clock::time_point{}) {
                release_at = std::chrono::steady_clock::now();
            }
        } else {
            stable_release_samples = 0;
            release_at = {};
        }
        std::this_thread::sleep_for(config.tick);
    }
    if (release_at == std::chrono::steady_clock::time_point{}) {
        stop_lower_wheels();
        finish_centering();
        return false;
    }

    auto opposite_at = std::chrono::steady_clock::time_point{};
    deadline = std::chrono::steady_clock::now() + config.opposite_timeout;
    // 阶段 3：继续同向运动直到对侧触发，release_at -> opposite_at 的时间代表全行程宽度。
    while (std::chrono::steady_clock::now() < deadline &&
           opposite_at == std::chrono::steady_clock::time_point{}) {
        if (!command_lower_wheels(plan.initial_lower_rpm)) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        const auto status = read_status();
        if (status.left_active && status.right_active) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        const bool opposite_active = plan.opposite_side == device::AttitudeLimitSide::LEFT_LOWER
                                         ? status.left_active
                                         : status.right_active;
        if (opposite_active) {
            opposite_at = std::chrono::steady_clock::now();
            break;
        }
        std::this_thread::sleep_for(config.tick);
    }
    if (opposite_at == std::chrono::steady_clock::time_point{}) {
        stop_lower_wheels();
        finish_centering();
        return false;
    }

    const auto return_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>((opposite_at - release_at) / 2);
    deadline = std::chrono::steady_clock::now() + return_duration;
    // 阶段 4：反向运行半程时间，回到两侧限位之间的中间位置。
    while (std::chrono::steady_clock::now() < deadline) {
        if (!command_lower_wheels(plan.return_lower_rpm)) {
            stop_lower_wheels();
            finish_centering();
            return false;
        }
        std::this_thread::sleep_for(config.tick);
    }

    const bool stopped = stop_lower_wheels();
    finish_centering();
    return stopped;
}

}  // namespace robot::service
