#pragma once

/// @file robot_supervisor.h
/// @brief 机器人监督器，协调 FSM、导航、云端配置与命令状态。

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::middleware {
class EventBus;
}

namespace robot::service {
class ConfigService;
struct FaultEvent;
class FaultReporter;
class MotionService;
class NavService;
class RecoveryMotion;
class SchedulerService;
}  // namespace robot::service

namespace robot::app {

class RobotFsm;
enum class RobotState;
struct RobotAction;
using robot::domain::RobotRuntimeSnapshot;
using robot::domain::RuntimeConfig;

class RobotSupervisor {
   public:
    struct CommandResult {
        bool accepted{false};
        std::string reason;
    };

    struct StartupPositionAssessment {
        domain::PositionState position_state{domain::PositionState::Unknown};
        bool should_request_return{false};
        const char* status_reason{"ok"};
    };

    /// @brief 构建机器人监督器。
    RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                    service::ConfigService& config,
                    std::shared_ptr<service::FaultReporter> fault,
                    std::shared_ptr<service::NavService> nav);

    /// @brief 提交统一机器人业务命令。RPC、调度、本地按钮都必须收敛到这里。
    CommandResult submit_command(const domain::RobotCommand& command);

    /// @brief 注入当前位置估计。未注入时返回 Unknown，商业启动会被拒绝。
    void set_position_state_query(std::function<domain::PositionState()> query);

    /// @brief 注入运动服务，用于分发 FSM 产生的运动动作。
    void set_motion_service(std::shared_ptr<service::MotionService> motion);

    /// @brief 注入恢复动作编排器，用于 Recovering 状态的多阶段姿态恢复。
    void set_recovery_motion(std::shared_ptr<service::RecoveryMotion> recovery);

    /// @brief 注入 BMS SOC 查询，用于启动前自检；未注入时跳过电量门禁。
    void set_battery_soc_query(std::function<float()> query);

    /// @brief 自检通过后推进 FSM 并分发启动动作。
    CommandResult handle_self_check_passed();

    /// @brief 将 FaultService 事件转换为 FSM 事件并分发安全动作。
    void handle_fault_event(const service::FaultEvent& event);

    /// @brief 按“下一次任务将使用的 primary_dock”处理调度启动。
    bool handle_scheduler_window_hit(bool left_limit_active,
                                     bool right_limit_active,
                                     float battery_soc);

    /// @brief 运行时安全检查函数，应周期性调用。
    void tick_safety();

    /// @brief 推进恢复动作；Recovering 状态下周期性调用。
    void tick_recovery();

    /// @brief 查询当前 FSM 状态字符串。
    std::string current_state() const;

    /// @brief 获取当前运行时快照。
    RobotRuntimeSnapshot snapshot() const;

    /// @brief 按固定 A/B 标定解释当前物理限位状态。
    domain::PositionState active_position_state(bool left_limit_active,
                                                bool right_limit_active) const;

    /// @brief 按固定 A/B 标定解释当前物理限位状态。
    domain::PositionState start_position_state(bool left_limit_active,
                                               bool right_limit_active) const;

    /// @brief 评估上电时当前位置是否异常；必要时触发返回停机位。
    StartupPositionAssessment handle_startup_position(bool left_limit_active,
                                                      bool right_limit_active);

    /// @brief 将物理限位稳定事件翻译为业务事件并分发给 FSM。
    void handle_limit_settled(domain::Endpoint endpoint, float battery_soc);
    void handle_limit_settled(domain::Endpoint endpoint,
                              bool left_limit_active,
                              bool right_limit_active,
                              float battery_soc);

    /// @brief 注册限位稳定事件到业务语义的桥接。
    void register_limit_settled_bridge(middleware::EventBus& event_bus,
                                       std::function<std::pair<bool, bool>()> current_limit_levels,
                                       std::function<float()> current_battery_soc);

    /// @brief 注册调度窗口命中后的业务启动入口。
    void register_scheduler_window(service::SchedulerService& scheduler,
                                   std::function<std::pair<bool, bool>()> current_limit_levels,
                                   std::function<float()> current_battery_soc);

   private:
    static bool can_trigger_spin_free_fault(RobotState state);
    RuntimeConfig start_runtime_config() const;
    domain::PositionState current_position_state() const;
    domain::LaneConfig start_lane_config() const;
    CommandResult handle_stop_command(const domain::RobotCommand& command);
    CommandResult handle_fault_reset_command(const domain::RobotCommand& command);
    CommandResult handle_start_command(const domain::RobotCommand& command);
    CommandResult validate_start_command(const domain::RobotCommand& command,
                                         const domain::LaneConfig& lane,
                                         domain::PositionState position_state);
    domain::MissionContext build_start_mission(
        const domain::RobotCommand& command,
        domain::PositionState position_state);
    void dispatch_actions(const std::vector<RobotAction>& actions);

    std::shared_ptr<RobotFsm> fsm_;
    service::ConfigService& config_;
    std::shared_ptr<service::FaultReporter> fault_;
    std::shared_ptr<service::NavService> nav_;
    std::shared_ptr<service::MotionService> motion_;
    std::shared_ptr<service::RecoveryMotion> recovery_;
    std::function<domain::PositionState()> position_state_query_;
    std::function<float()> battery_soc_query_;
};

}  // namespace robot::app
