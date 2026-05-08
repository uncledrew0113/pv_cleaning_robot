#pragma once

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
    RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg,
                    std::shared_ptr<service::CommandTracker> command_tracker,
                    std::shared_ptr<service::FaultService> fault,
                    std::shared_ptr<service::NavService> nav);

    bool start_task(bool at_parking_side);
    bool resume_paused_task();
    bool pause_task();
    bool return_task();
    bool terminate_task();
    bool reset_task(bool at_parking_side);
    void tick_safety(bool low_battery);
    std::string current_state() const;
    RobotRuntimeSnapshot snapshot() const;

private:
    static std::string task_state_from_device_state(const std::string& device_state);
    static uint64_t runtime_config_version(const service::TbRuntimeConfig& config);

    std::shared_ptr<RobotFsm> fsm_;
    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg_;
    std::shared_ptr<service::CommandTracker> command_tracker_;
    std::shared_ptr<service::FaultService> fault_;
    std::shared_ptr<service::NavService> nav_;
};

}  // namespace robot::app
