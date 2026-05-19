#pragma once

/// @file robot_supervisor.h
/// @brief 机器人监督器，协调 FSM、导航、云端配置与命令状态。

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "pv_cleaning_robot/app/parking_side_runtime.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"

namespace robot::middleware {
class EventBus;
}

namespace robot::service {
class CommandTracker;
class ConfigService;
class FaultService;
class NavService;
class SchedulerService;
}  // namespace robot::service

namespace robot::app {

class RobotFsm;

class RobotSupervisor {
   public:
    struct StartupPositionAssessment {
        ParkingSideFacts facts;
        bool should_request_return{false};
        const char* status_reason{"ok"};
    };

    /// @brief 构建机器人监督器。
    RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                    service::ConfigService& config,
                    std::shared_ptr<service::CommandTracker> command_tracker,
                    std::shared_ptr<service::FaultService> fault,
                    std::shared_ptr<service::NavService> nav);

    /// @brief 从停机侧位置开始清扫任务。
    bool start_task(bool at_parking_side, bool position_valid, float battery_soc);

    /// @brief 直接从当前位置启动任务。
    bool start_task_from_current_position(bool position_valid, float battery_soc);

    /// @brief 停止当前任务并返回安全状态。
    bool stop_task();

    /// @brief 请求机器人返回停机侧。
    bool return_task(bool at_parking_side);

    /// @brief 按“下一次任务将使用的 parking_side”处理调度启动。
    bool handle_scheduler_window_hit(bool left_limit_active,
                                     bool right_limit_active,
                                     float battery_soc);

    /// @brief 运行时安全检查函数，应周期性调用。
    void tick_safety();

    /// @brief 查询当前 FSM 状态字符串。
    std::string current_state() const;

    /// @brief 获取当前运行时快照。
    RobotRuntimeSnapshot snapshot() const;

    /// @brief 按 active parking_side 解释当前物理限位状态。
    ParkingSideFacts active_parking_facts(bool left_limit_active, bool right_limit_active) const;

    /// @brief 按“下一次任务将使用的 parking_side”解释当前物理限位状态。
    ParkingSideFacts start_parking_facts(bool left_limit_active, bool right_limit_active) const;

    /// @brief 评估上电时当前位置是否异常；必要时触发返回停机位。
    StartupPositionAssessment handle_startup_position(bool left_limit_active,
                                                      bool right_limit_active);

    /// @brief 将物理限位稳定事件翻译为业务事件并分发给 FSM。
    void handle_limit_settled(device::LimitSide side, float battery_soc);

    /// @brief 注册限位稳定事件到业务语义的桥接。
    void register_limit_settled_bridge(middleware::EventBus& event_bus,
                                       std::function<float()> current_battery_soc);

    /// @brief 注册调度窗口命中后的业务启动入口。
    void register_scheduler_window(service::SchedulerService& scheduler,
                                   std::function<std::pair<bool, bool>()> current_limit_levels,
                                   std::function<float()> current_battery_soc);

   private:
    static bool is_new_task_start_state(const std::string& state);
    static bool is_cleaning_state(const std::string& state);
    static bool is_return_allowed_state(const std::string& state);
    static bool can_trigger_spin_free_fault(const std::string& state);
    static std::string task_state_from_device_state(const std::string& device_state);
    service::RuntimeConfig start_runtime_config() const;

    std::shared_ptr<RobotFsm> fsm_;
    service::ConfigService& config_;
    std::shared_ptr<service::CommandTracker> command_tracker_;
    std::shared_ptr<service::FaultService> fault_;
    std::shared_ptr<service::NavService> nav_;
};

}  // namespace robot::app
