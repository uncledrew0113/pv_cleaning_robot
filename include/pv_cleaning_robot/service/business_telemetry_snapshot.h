#pragma once

#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::service {

struct BusinessTelemetrySnapshot {
    std::string device_state;
    std::string task_state;
    int target_half_passes{0};
    int completed_half_passes{0};
    int clean_count{0};
    uint64_t active_config_version{0};
    std::optional<TbRuntimeConfig> active_config;
    std::optional<TbRuntimeConfig> pending_config;
    std::optional<CommandSnapshot> active_command;
    std::optional<CommandSnapshot> last_command;

    nlohmann::json to_json() const;
};

}  // namespace robot::service
