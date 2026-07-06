/**
 * @file attitude_limit_service.cc
 * @brief 姿态限位监测与下轮回中恢复实现。
 *
 * 本文件实现姿态限位 GPIO 回调、事件缓存和下轮回中策略。回中策略使用整体超时保护，
 * 单侧姿态限位触发作为测量信号，双侧姿态限位仍提交错误事件。
 */
#include "pv_cleaning_robot/service/attitude_limit_service.h"

#include <cmath>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

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
        centering_triggered_side_ = side;
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

AttitudeLimitService::CenterResult AttitudeLimitService::lower_attitude_center() {
    return lower_attitude_center(CenterConfig{});
}

AttitudeLimitService::CenterResult AttitudeLimitService::lower_attitude_center(
    const CenterConfig& config) {
    const auto tick = config.tick > std::chrono::milliseconds::zero()
                          ? config.tick
                          : std::chrono::milliseconds(20);
    const auto started_at = std::chrono::steady_clock::now();
    const auto deadline = started_at + config.overall_timeout;
    auto timed_out = [&] { return std::chrono::steady_clock::now() >= deadline; };
    auto wait_until_timeout = [&] {
        while (!timed_out()) {
            std::this_thread::sleep_for(tick);
        }
    };

    auto finish_centering = [this]() {
        std::lock_guard<std::mutex> lk(event_mtx_);
        centering_active_ = false;
        centering_triggered_side_.reset();
    };
    auto emergency_stop_after_timeout = [&] {
        if (motion_ports_.emergency_stop) {
            motion_ports_.emergency_stop();
        }
    };
    auto timeout_after_problem = [&](const char* reason) {
        spdlog::warn("[AttitudeLimitService] lower_attitude_center timed out: {}", reason);
        stop_lower_wheels();
        wait_until_timeout();
        emergency_stop_after_timeout();
        finish_centering();
        return CenterResult{CenterOutcome::TimedOut};
    };

    if (config.lower_rpm <= 0.0f || config.stable_samples_required <= 0 ||
        config.overall_timeout <= std::chrono::milliseconds::zero()) {
        return timeout_after_problem("invalid config");
    }
    if (!left_ || !right_ || !motion_ports_.prepare_center_motion) {
        return timeout_after_problem("missing dependency");
    }
    if (!motion_ports_.prepare_center_motion()) {
        return timeout_after_problem("prepare_center_motion failed");
    }
    {
        std::lock_guard<std::mutex> lk(event_mtx_);
        centering_active_ = true;
        centering_triggered_side_.reset();
        pending_event_.reset();
    }
    auto timeout_finish = [&](const char* reason) {
        spdlog::warn("[AttitudeLimitService] lower_attitude_center timed out: {}", reason);
        stop_lower_wheels();
        wait_until_timeout();
        emergency_stop_after_timeout();
        finish_centering();
        return CenterResult{CenterOutcome::TimedOut};
    };
    auto side_active = [](Status status, device::AttitudeLimitSide side) {
        return side == device::AttitudeLimitSide::LEFT_LOWER ? status.left_active
                                                             : status.right_active;
    };
    auto take_center_trigger = [this]() -> std::optional<device::AttitudeLimitSide> {
        std::lock_guard<std::mutex> lk(event_mtx_);
        auto out = centering_triggered_side_;
        centering_triggered_side_.reset();
        return out;
    };
    auto prepare_after_expected_trigger = [&]() -> bool {
        return motion_ports_.prepare_center_motion && motion_ports_.prepare_center_motion();
    };
    auto wait_for_expected_trigger =
        [&](device::AttitudeLimitSide expected,
            float lower_rpm,
            const char* command_failed_reason,
            const char* unexpected_trigger_reason) -> std::optional<std::chrono::steady_clock::time_point> {
        while (!timed_out()) {
            if (const auto triggered = take_center_trigger(); triggered.has_value()) {
                if (*triggered != expected) {
                    spdlog::warn(
                        "[AttitudeLimitService] lower_attitude_center unexpected trigger side");
                    return std::nullopt;
                }
                return std::chrono::steady_clock::now();
            }

            const auto status = read_status();
            if (status.left_active && status.right_active) {
                spdlog::warn(
                    "[AttitudeLimitService] lower_attitude_center both attitude limits active");
                return std::nullopt;
            }
            if (!command_lower_wheels(lower_rpm)) {
                spdlog::warn("[AttitudeLimitService] lower_attitude_center {}",
                             command_failed_reason);
                return std::nullopt;
            }
            std::this_thread::sleep_for(tick);
        }
        spdlog::warn("[AttitudeLimitService] lower_attitude_center {}",
                     unexpected_trigger_reason);
        return std::nullopt;
    };

    auto release_at = std::chrono::steady_clock::time_point{};
    auto opposite_at = std::chrono::steady_clock::time_point{};
    auto return_lower_rpm = 0.0f;
    const auto initial_status = read_status();
    if (initial_status.left_active && initial_status.right_active) {
        return timeout_finish("both attitude limits active");
    }

    const auto initial_side = active_side(initial_status);
    if (initial_side.has_value()) {
        const auto plan = make_center_plan(*initial_side, config.lower_rpm);
        int stable_release_samples = 0;
        // 初始已有一侧触发：先向对侧运动，记录触发侧稳定释放的时间点。
        while (!timed_out() && stable_release_samples < config.stable_samples_required) {
            if (!command_lower_wheels(plan.initial_lower_rpm)) {
                return timeout_finish("command release failed");
            }
            const auto status = read_status();
            if (status.left_active && status.right_active) {
                return timeout_finish("both attitude limits active");
            }
            const bool released = !side_active(status, plan.release_side);
            if (released) {
                ++stable_release_samples;
                if (release_at == std::chrono::steady_clock::time_point{}) {
                    release_at = std::chrono::steady_clock::now();
                }
            } else {
                stable_release_samples = 0;
                release_at = {};
            }
            std::this_thread::sleep_for(tick);
        }
        if (release_at == std::chrono::steady_clock::time_point{}) {
            return timeout_finish("release side not released");
        }

        const auto triggered_at = wait_for_expected_trigger(plan.opposite_side,
                                                            plan.initial_lower_rpm,
                                                            "command opposite search failed",
                                                            "opposite side not found");
        if (!triggered_at.has_value()) {
            return timeout_finish("opposite side not found");
        }
        opposite_at = *triggered_at;
        return_lower_rpm = plan.return_lower_rpm;
    } else {
        constexpr auto default_side = device::AttitudeLimitSide::RIGHT_LOWER;
        constexpr auto opposite_side = device::AttitudeLimitSide::LEFT_LOWER;
        const float default_lower_rpm = -std::abs(config.lower_rpm);
        const float reverse_lower_rpm = std::abs(config.lower_rpm);

        const auto first_at = wait_for_expected_trigger(default_side,
                                                        default_lower_rpm,
                                                        "command default search failed",
                                                        "default side not found");
        if (!first_at.has_value()) {
            return timeout_finish("default side not found");
        }
        release_at = *first_at;
        if (timed_out()) {
            return timeout_finish("default side found after timeout");
        }
        if (!prepare_after_expected_trigger()) {
            return timeout_finish("prepare after default trigger failed");
        }

        const auto second_at = wait_for_expected_trigger(opposite_side,
                                                         reverse_lower_rpm,
                                                         "command reverse search failed",
                                                         "opposite side not found");
        if (!second_at.has_value()) {
            return timeout_finish("opposite side not found");
        }
        opposite_at = *second_at;
        return_lower_rpm = default_lower_rpm;
    }

    if (timed_out()) {
        return timeout_finish("opposite side found after timeout");
    }
    if (!prepare_after_expected_trigger()) {
        return timeout_finish("prepare after opposite trigger failed");
    }

    const auto return_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>((opposite_at - release_at) / 2);
    const auto return_deadline = std::chrono::steady_clock::now() + return_duration;
    // 阶段 4：反向运行半程时间，回到两侧限位之间的中间位置。
    while (!timed_out() && std::chrono::steady_clock::now() < return_deadline) {
        if (!command_lower_wheels(return_lower_rpm)) {
            return timeout_finish("command return failed");
        }
        std::this_thread::sleep_for(tick);
    }
    if (timed_out()) {
        return timeout_finish("return motion not completed");
    }

    const bool stopped = stop_lower_wheels();
    if (!stopped) {
        spdlog::warn("[AttitudeLimitService] lower_attitude_center timed out: stop failed");
        wait_until_timeout();
        emergency_stop_after_timeout();
        finish_centering();
        return CenterResult{CenterOutcome::TimedOut};
    }
    finish_centering();
    return CenterResult{CenterOutcome::Completed};
}

}  // namespace robot::service
