/**
 * @file robot_application.h
 * @brief 机器人应用生命周期编排接口。
 *
 * RobotApplication 负责从配置加载到硬件、服务、状态机和线程执行器的组合根搭建。
 * 具体业务逻辑由各模块实现，本类只负责依赖接线、启动顺序和退出清理。
 */
#pragma once

#include <atomic>

#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::service {
class ConfigService;
}

namespace robot::app {

/// @brief 从 ConfigService 读取主程序使用的运动控制配置。
///
/// 独立成函数是为了让硬件测试验证过的纠偏参数可以被单元测试覆盖，避免主程序
/// 漏读配置字段时只在真实机器人运行时才暴露。
service::MotionService::Config make_motion_config_from_config(service::ConfigService& cfg);

/// @brief 机器人应用生命周期编排器。
///
/// 本类集中负责配置加载、硬件/服务/app 对象接线、线程启动、主循环推进和退出清理。
/// main.cc 只保留进程级职责（信号处理和启动 run），避免入口函数继续承载业务编排。
class RobotApplication {
public:
    /// @brief 运行完整机器人应用，直到 running 被信号处理器置为 false。
    /// @return 0 表示正常退出；非 0 表示初始化阶段出现致命错误。
    int run(const std::atomic<bool>& running);
};

}  // namespace robot::app
