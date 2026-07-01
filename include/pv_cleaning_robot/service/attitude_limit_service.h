#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "pv_cleaning_robot/device/attitude_limit_switch.h"

namespace robot::service {

/// @brief 姿态限位服务：监测下轮姿态限位，并执行下轮低速回中恢复。
///
/// 边界约束：
///   - 本服务只处理 attitude_limit_switch，不处理主限位；
///   - 错误决策仍由 ErrorManager 完成，本服务只提供可消费的确定性事件；
///   - GPIO 触发时先立即调用急停端口，再记录事件，保证安全动作不等待主循环；
///   - 回中流程只编排限位开关判定和时序，实际电机命令通过 MotionPorts 交给 MotionService。
class AttitudeLimitService {
public:
    enum class EventType {
        AttitudeLimit,
        AttitudeLimitBoth,
    };

    struct Event {
        EventType type{EventType::AttitudeLimit};
        device::AttitudeLimitSide side{device::AttitudeLimitSide::LEFT_LOWER};
    };

    struct Status {
        bool left_active{false};
        bool right_active{false};
    };

    struct CenterConfig {
        float lower_rpm{5.0f};  ///< 下轮回中速度绝对值；方向由触发侧决定。
        int stable_samples_required{2};  ///< 释放侧连续稳定样本数，避免电平抖动影响时间点。
        std::chrono::milliseconds search_timeout{std::chrono::seconds(30)};  ///< 搜索初始触发侧超时。
        std::chrono::milliseconds release_timeout{std::chrono::seconds(10)};  ///< 从触发侧退出的超时。
        std::chrono::milliseconds opposite_timeout{std::chrono::seconds(30)};  ///< 搜索对侧触发超时。
        std::chrono::milliseconds tick{std::chrono::milliseconds(20)};  ///< 回中轮询和命令刷新周期。
    };

    struct MotionPorts {
        /// 姿态限位触发后的首响急停。
        std::function<void()> emergency_stop;
        /// 准备回中运动模式，通常会停滚刷、清任务命令并切速度模式。
        std::function<bool()> prepare_center_motion;
        /// 下发下轮回中速度；上轮保持 0。
        std::function<bool(float)> command_lower_wheels;
        /// 停止回中运动并收敛行走电机。
        std::function<bool()> stop_center_motion;
    };

    AttitudeLimitService(std::shared_ptr<device::AttitudeLimitSwitch> left,
                         std::shared_ptr<device::AttitudeLimitSwitch> right,
                         MotionPorts motion_ports);

    void start_monitoring();
    void stop_monitoring();

    /// @brief 取出最近一次姿态限位事件；主循环消费后再提交给 ErrorManager。
    std::optional<Event> consume_pending_event();

    /// @brief 执行“下轮回中”：寻找触发侧、释放、到对侧、回半程。
    ///
    /// 本流程使用轮询读取 GPIO 当前电平，不依赖中断精确时间戳；在 RK3576 GPIO 无可靠
    /// 中断能力的现场环境下，20ms tick 足够用于机械回中。
    bool lower_attitude_center();
    bool lower_attitude_center(const CenterConfig& config);

private:
    struct CenterPlan {
        device::AttitudeLimitSide release_side{device::AttitudeLimitSide::LEFT_LOWER};
        device::AttitudeLimitSide opposite_side{device::AttitudeLimitSide::RIGHT_LOWER};
        float initial_lower_rpm{0.0f};
        float return_lower_rpm{0.0f};
    };

    void handle_trigger(device::AttitudeLimitSide side);
    Status read_status() const;
    static std::optional<device::AttitudeLimitSide> active_side(Status status);
    static CenterPlan make_center_plan(device::AttitudeLimitSide side, float lower_rpm);
    bool command_lower_wheels(float lower_rpm);
    bool stop_lower_wheels();

    std::shared_ptr<device::AttitudeLimitSwitch> left_;
    std::shared_ptr<device::AttitudeLimitSwitch> right_;
    MotionPorts motion_ports_;

    mutable std::mutex event_mtx_;
    std::optional<Event> pending_event_;
    bool centering_active_{false};
};

}  // namespace robot::service
