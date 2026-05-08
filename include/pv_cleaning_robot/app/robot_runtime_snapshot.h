#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::app {

struct RobotRuntimeSnapshot {
    std::string device_state;
    std::string task_state;
    int target_passes{0};
    int completed_passes{0};
    int clean_count{0};
    uint64_t active_config_version{0};
    std::optional<robot::service::TbRuntimeConfig> active_config;
    std::optional<robot::service::TbRuntimeConfig> pending_config;
    std::optional<robot::service::CommandSnapshot> active_command;
    std::optional<robot::service::CommandSnapshot> last_command;
};

}  // namespace robot::app
