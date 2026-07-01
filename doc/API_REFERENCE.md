# PV 清扫机器人 API 参考

本文档只描述当前 MVP 中仍由主程序接入的接口与运行契约。

## 1. 可执行目标

当前构建产物：

- `pv_cleaning_robot`
- `unit_tests`
- `hw_tests`

构建定义：

- 主程序目标：[pv_cleaning_robot/CMakeLists.txt](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/CMakeLists.txt:1)
- 测试目标：[test/CMakeLists.txt](/home/tronlong/pv_cleaning_robot/test/CMakeLists.txt:1)

## 2. 进程入口

入口文件：[main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc:1)

当前 `main` 只负责：

- 安装退出信号。
- 创建 `RobotApplication`。
- 调用 `RobotApplication::run()`。

硬件、服务、线程、状态机、错误处理和云端控制均由 `RobotApplication` 组装。

## 3. 配置服务

配置服务类：`robot::service::ConfigService`

当前配置模型：

- `config.runtime.json`：当前生效的运行配置。
- `config.fixed.json`：固定硬件和系统配置。
- `config.runtime.pending.json`：下一次任务启动前生效的运行配置补丁。
- `config.runtime.backup.json`：runtime 最近一次备份。

读取规则：

- `get()` 先读 runtime，再回退到 fixed。
- `get_fixed()` 只读 fixed。

当前 shared attributes 支持字段：

- `repeat_count`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `primary_dock`
- `min_battery_soc`
- `charge_stop_soc`
- `schedules`

## 4. RobotApplication

`RobotApplication::run()` 当前负责：

- 加载配置。
- 初始化日志、内存锁定和 watchdog。
- 构造 CAN、串口、GPIO、GPSD、MQTT 等驱动。
- 构造 `WalkMotorGroup`、`BrushMotor`、`BMS`、`ImuDevice`、`GpsDevice`、
  `LockMotor`。
- 构造 `MotionService`、`GpsStuckService`、`AttitudeLimitService`、
  `DiagnosticsCollector`、`HealthService`、`CloudService`、
  `ThingsBoardControlPlane`。
- 构造 `RobotController`、`ErrorManager`、`RecoveryExecutor`、
  `ErrorHandlingService`。
- 启动运行线程并在退出时按顺序关闭。

## 5. 主状态机

控制器类：`robot::app::RobotController`

当前状态：

- `Idle`
- `SelfChecking`
- `ExecutingMission`
- `SettlingEndpoint`
- `Recovering`
- `Charging`
- `FaultStopped`

核心规则：

- RPC、调度和本地命令统一通过 `submit_command()`。
- `StartConfiguredMission` 从配置任务起点启动。
- `CleanTowardOppositeEndpoint` / `CleanTowardPrimaryDock` 用于定向维护清扫。
- `Stop` 只在任务运行中接受。
- `FaultReset` 只在 `FaultStopped` 接受。
- 错误恢复只通过 `ErrorManager + RecoveryExecutor` 闭环。

## 6. 运动控制

服务类：`robot::service::MotionService`

主程序使用接口：

- `start_segment()`：按任务段启动运动。
- `stop_cleaning()`：停止清扫运动。
- `emergency_stop()`：系统级急停。
- `reverse_for_recovery()`：卡住或堵转后的反向恢复运动。
- `begin_attitude_center_motion()` / `command_lower_wheels_for_attitude_center()` /
  `stop_attitude_center_motion()`：姿态回中运动编排。
- `update()`：周期运动控制和纠偏。

## 7. 错误处理

错误处理入口：

- `ErrorHandlingService::update()`

处理链路：

1. 读取外部错误事件和 `DiagnosticsCollector` 快照。
2. `ErrorManager` 进行错误合并、去重和决策。
3. 每轮只处理一个最高优先级错误。
4. 需要状态切换时调用 `RobotController::apply_error_decision()`。
5. 需要恢复时调用 `RecoveryExecutor::execute()`。
6. 恢复完成后调用 `RobotController::post_recovery_finished()`。

## 8. 诊断与健康上报

诊断采集：

- `DiagnosticsCollector` 是设备诊断数据的统一输出源。

健康上报：

- `HealthService::Mode::HEALTH`：精简状态字段。
- `HealthService::Mode::DIAGNOSTICS`：完整诊断字段。

上报周期：

- 运行态使用 `diagnostics.publish_interval_active_ms`。
- 空闲态使用 `diagnostics.publish_interval_idle_ms`。

## 9. ThingsBoard 控制面

服务类：`robot::service::ThingsBoardControlPlane`

当前注册 RPC：

- `clean_to_return`
- `clean_to_parking`
- `start_configured`
- `stop`
- `fault_reset`

当前业务遥测字段：

- `state`
- `fault`
- `cfg_ver`
- `repeat_count`
- `completed_cycles`

启动属性字段：

- `software_version`
- `hardware_version`
- `device_model`
- `device_id`
