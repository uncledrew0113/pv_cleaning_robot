#pragma once

/// @file robot_runtime_snapshot.h
/// @brief 运行时状态快照，用于上报、监控和 UI 显示。

#include <cstdint>
#include <optional>
#include <string>

#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::app {

struct RobotRuntimeSnapshot {
    /// 设备当前工作状态，例如 Idle、Cleaning、Returning
    std::string device_state;
    /// 任务当前阶段，例如 Pending、Running、Completed
    std::string task_state;
    /// 期望通过次数
    int target_passes{0};
    /// 已完成的通过次数
    int completed_passes{0};
    /// 累计清扫次数
    int clean_count{0};
    /// 当前激活运行配置的版本号
    uint64_t active_config_version{0};
    /// 当前正在使用的云端运行时配置
    std::optional<robot::service::TbRuntimeConfig> active_config;
    /// 当前待生效的云端运行时配置
    std::optional<robot::service::TbRuntimeConfig> pending_config;
    /// 当前处于进行中的命令快照
    std::optional<robot::service::CommandSnapshot> active_command;
    /// 最近完成的命令快照
    std::optional<robot::service::CommandSnapshot> last_command;
};

}  // namespace robot::app
