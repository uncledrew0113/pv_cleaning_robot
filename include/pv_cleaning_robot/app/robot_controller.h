/**
 * @file robot_controller.h
 * @brief 机器人任务状态机接口。
 *
 * RobotController 负责接受调度/RPC/本地命令，维护任务状态、限位切段、恢复返回和故障锁存。
 * 硬件动作通过 ActionPorts 注入，并在状态机锁外执行，避免硬件 IO 阻塞状态查询和事件处理。
 */
#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "pv_cleaning_robot/app/error_manager.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {

/// @brief 机器人主任务状态。
enum class RobotState {
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    Recovering,
    Charging,
    FaultStopped,
};

/// @brief 命令提交结果。
struct CommandResult {
    bool accepted{false};
    std::string reason;
};

/// @brief 状态机只读快照，用于上报和测试断言。
struct RobotControllerSnapshot {
    std::string state{"Idle"};
    std::optional<uint32_t> fault;
    uint32_t repeat_count{0};
    int completed_cycles{0};
    uint64_t cfg_ver{0};
    std::optional<domain::Endpoint> current_segment_target;
    std::optional<domain::SegmentMode> current_segment_mode;
};

/**
 * @brief 机器人任务状态机。
 *
 * 状态机运行在内部工作线程中，所有外部事件先进入队列再串行处理。状态更新受 mtx_ 保护；
 * 电机、锁止和急停等外部动作通过端口回调在锁外执行。
 */
class RobotController {
public:
    struct ActionPorts {
        /// 启动当前任务段；具体电机命令由 MotionService 负责换算和下发。
        std::function<bool(const domain::MissionSegment&)> start_segment;
        /// 正常停止当前任务段，用于 stop 命令、端点完成和进入恢复流程前的业务停机。
        std::function<void()> stop_motion;
        /// 故障停机兜底急停；状态机只调用端口，不直接操作电机。
        std::function<void()> emergency_stop;
        /// 进入 Recovering 时的外部通知钩子，当前用于上层观测，不承载恢复动作。
        std::function<void()> start_recovery;
        /// 故障复位时的外部清理钩子；故障锁存仍由 RobotController 自身维护。
        std::function<void()> clear_fault;
        /// 任务完成回到停机位后执行锁止；失败会锁存 FaultStopped。
        std::function<bool()> open_lock_motor;
        /// 自检通过、清扫启动前解除锁止；失败会锁存 FaultStopped。
        std::function<bool()> close_lock_motor;
    };

    struct ConfigPorts {
        /// 当前生效运行配置，任务启动和运动服务同步都以它为基准。
        std::function<domain::RuntimeConfig()> active_runtime_config;
        /// 待生效运行配置；启动任务前提升为 active，避免任务中途变更参数。
        std::function<std::optional<domain::RuntimeConfig>()> pending_runtime_config;
        std::function<uint64_t(const domain::RuntimeConfig&)> runtime_config_version;
        std::function<bool()> promote_pending_runtime_config;
        /// 固定车道配置，描述主停机端和端点几何关系。
        std::function<domain::LaneConfig()> lane_config;
    };

    RobotController() = default;
    explicit RobotController(ActionPorts ports);
    ~RobotController();

    void start();
    void stop();
    void set_config_ports(ConfigPorts ports);
    void set_position_state_query(std::function<domain::PositionState()> query);
    void set_battery_soc_query(std::function<float()> query);

    CommandResult submit_command(const domain::RobotCommand& command);
    RobotControllerSnapshot snapshot() const;

    /// SafetyMonitor 去抖确认主限位后投递；状态机线程内再决定切段或忽略。
    void post_limit_settled(domain::Endpoint endpoint);
    /// RecoveryExecutor 返回后投递；只有 Recovering 状态会消费该事件。
    void post_recovery_finished(bool ok);
    void post_schedule_window_hit();
    void post_tick();
    /// ErrorHandlingService 调用的状态机入口；只处理切状态和故障锁存，不执行恢复动作。
    void apply_error_decision(const ErrorDecision& decision);

    void complete_self_check(bool ok);
    void complete_self_check_for_test(bool ok);
    void handle_limit_settled_for_test(domain::Endpoint endpoint);
    void post_for_test(std::function<void()> fn);
    void drain_for_test();

private:
    static const char* state_name(RobotState state) noexcept;
    void loop();
    void post(std::function<void()> fn);
    bool mission_active() const noexcept;
    CommandResult submit_command_locked(const domain::RobotCommand& command);
    CommandResult start_command_locked(const domain::RobotCommand& command);
    CommandResult validate_start_command_locked(const domain::RobotCommand& command,
                                                const domain::LaneConfig& lane,
                                                domain::PositionState position_state) const;
    domain::MissionContext build_start_mission_locked(const domain::RobotCommand& command,
                                                      const domain::LaneConfig& lane,
                                                      domain::PositionState position_state) const;
    CommandResult stop_locked();
    struct StartSegmentResult {
        bool ok{false};
        std::optional<domain::Endpoint> already_at_target;
    };
    StartSegmentResult start_current_segment_locked();
    void complete_self_check_unlocked(bool ok);
    void handle_limit_settled_unlocked(domain::Endpoint endpoint);
    std::optional<domain::Endpoint> handle_source_limit_repeat_locked(domain::Endpoint endpoint);
    std::optional<domain::Endpoint> handle_recovery_finished_locked(bool ok);

    mutable std::mutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
    std::optional<uint32_t> active_fault_;
    std::optional<RobotState> recovery_return_state_;
    ActionPorts actions_{};
    ConfigPorts config_{};
    std::function<domain::PositionState()> position_state_query_;
    std::function<float()> battery_soc_query_;

    mutable std::mutex queue_mtx_;
    std::condition_variable queue_cv_;
    std::condition_variable idle_cv_;
    std::deque<std::function<void()>> queue_;
    bool running_{false};
    bool stop_requested_{false};
    bool handling_event_{false};
    std::thread worker_;
};

}  // namespace robot::app
