#pragma once

/// @file robot_supervisor.h
/// @brief 机器人监督器，协调 FSM、导航、云端配置与命令状态。

#include <memory>
#include <string>

#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"

namespace robot::service {
class CommandTracker;
class FaultService;
class NavService;
class ThingsBoardConfigManager;
}  // namespace robot::service

namespace robot::app {

class RobotFsm;

class RobotSupervisor {
   public:
    /// @brief 构建机器人监督器。
    RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg,
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

    /// @brief 运行时安全检查函数，应周期性调用。
    void tick_safety();

    /// @brief 查询当前 FSM 状态字符串。
    std::string current_state() const;

    /// @brief 获取当前运行时快照。
    RobotRuntimeSnapshot snapshot() const;

   private:
    static bool is_new_task_start_state(const std::string& state);
    static bool is_cleaning_state(const std::string& state);
    static bool is_return_allowed_state(const std::string& state);
    static bool can_trigger_spin_free_fault(const std::string& state);
    static std::string task_state_from_device_state(const std::string& device_state);
    static uint64_t runtime_config_version(const service::TbRuntimeConfig& config);
    service::TbRuntimeConfig start_runtime_config() const;

    std::shared_ptr<RobotFsm> fsm_;
    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg_;
    std::shared_ptr<service::CommandTracker> command_tracker_;
    std::shared_ptr<service::FaultService> fault_;
    std::shared_ptr<service::NavService> nav_;
};

}  // namespace robot::app
