# 当前 MVP 调用链

本文档只记录当前 MVP 主程序仍在使用的核心调用链。历史改造计划中的旧故障模块、
旧导航模块和旧恢复模块不再作为当前代码依据。

## 进程入口

- `main.cc`
  - 安装 `SIGINT` / `SIGTERM` 退出信号。
  - 创建 `robot::app::RobotApplication`。
  - 调用 `RobotApplication::run()`。

## 应用装配

- `RobotApplication::run()`
  - 加载 `config.runtime.json` 和 `config.fixed.json`。
  - 构造硬件驱动、设备、服务、控制器和线程执行器。
  - 启动运行线程。
  - 进入主循环，周期推进调度、自检和云端上报周期切换。
  - 退出时按顺序停止线程、服务和硬件设备。

## 主状态机

- ThingsBoard RPC / Scheduler / SafetyMonitor / ErrorHandlingService
  - 调用或投递事件到 `RobotController`。
- `RobotController`
  - 统一维护 `Idle`、`SelfChecking`、`ExecutingMission`、`SettlingEndpoint`、
    `Recovering`、`Charging`、`FaultStopped`。
  - 启动任务前读取配置、位置和电量。
  - 任务段启动通过 `MotionService::start_segment()` 执行。
  - 任务完成后执行停机动作和锁止电机开动作。

## 运动控制

- `ThreadExecutor(walk_ctrl)`
  - 周期调用 `MotionService::update()`。
  - 周期上报 `walk_ctrl` 看门狗心跳。
- `MotionService`
  - 根据任务段目标端点换算行走方向。
  - 控制行走电机组和滚刷。
  - 执行视觉/融合角纠偏。
  - 提供系统急停、清扫停止、反向恢复运动、姿态回中运动接口。
- `WalkMotorGroup`
  - 负责 CAN 帧发送、反馈接收和四轮状态缓存。

## 安全限位

- `SafetyMonitor`
  - 轮询或 GPIO 回调接收主限位触发。
  - 触发后立即执行急停路径。
  - 稳定后发布 `LimitSettledEvent`。
- `RobotApplication`
  - 订阅 `LimitSettledEvent`。
  - 将稳定端点传给 `RobotController::post_limit_settled()`。

## 姿态限位

- `AttitudeLimitService`
  - 轮询姿态限位开关。
  - 任一单侧姿态限位触发时立即急停。
  - 仅在任务运行态提交 `AttitudeLimit` 错误。
  - 回中流程中，单侧姿态限位事件用于回中算法，不再重复提交错误。

## 错误处理

- `ThreadExecutor(error_mgr)`
  - 周期调用 `ErrorHandlingService::update()`。
- `ErrorHandlingService`
  - 读取外部错误事件和 `DiagnosticsCollector` 快照。
  - 调用 `ErrorManager` 进行错误仲裁。
  - 每轮只处理一个最高优先级决策。
  - 需要恢复时调用 `RobotController::apply_error_decision()` 和
    `RecoveryExecutor::execute()`。
  - 恢复完成后调用 `RobotController::post_recovery_finished()`。
- `ErrorManager`
  - 合并同类错误。
  - 根据错误源生成确定性恢复计划或故障停止决策。
  - 记录恢复失败次数和短时间内重复原始错误。
- `RecoveryExecutor`
  - 只执行恢复动作，不决定状态机。
  - 设备类恢复负责停止相关执行器、重启驱动、恢复执行器。
  - 运动类恢复负责急停、暂停 GpsStuck、执行反向或回中动作、恢复 GpsStuck。

## 诊断与健康上报

- `ThreadExecutor(diagnostics)`
  - 周期调用 `DiagnosticsCollector::update()`。
- `DiagnosticsCollector`
  - 统一采集设备状态、诊断计数和错误检测快照。
  - `HealthService` 读取 health/diagnostics 快照。
  - `ErrorManager` 读取错误快照。
- `ThreadExecutor(cloud)`
  - 根据当前机器人状态动态切换上报周期。
  - 周期调用 `HealthService::update()`。
  - 周期调用 `ThingsBoardControlPlane::publish_business_telemetry()`。
  - 周期调用 `CloudService::update()`。

## 云端控制

- `CloudService`
  - 负责 MQTT 连接、订阅、发布、RPC 路由和离线缓存。
- `ThingsBoardControlPlane`
  - 处理 shared attributes。
  - 注册 `clean_to_return`、`clean_to_parking`、`start_configured`、`stop`、
    `fault_reset`。
  - 将 RPC 转换为 `RobotCommand` 后交给 `RobotController`。
  - 周期发布业务遥测。

## 看门狗

- `WatchdogMgr`
  - 监控 `walk_ctrl`、`gps_stuck`、`bms`、`brush`、`cloud` 等线程心跳。
  - 超时后通过回调提交给 `ErrorManager`。
  - 具体恢复策略由 `ErrorManager + RecoveryExecutor` 决定。
