# MVP 旧残留保留清单

本文档记录本轮旧残留扫描中发现、并经人工确认继续保留的项目。后续旧残留清理不应再把这些内容当作待删除项处理。

## Charging 状态

- 文件：
  - `include/pv_cleaning_robot/app/robot_controller.h`
  - `pv_cleaning_robot/app/robot_controller.cc`
  - `pv_cleaning_robot/app/error_manager.cc`
- 保留内容：
  - `RobotState::Charging`
- 证据：
  - 当前主流程没有进入 `Charging` 的状态切换路径。
  - `ErrorHandlingService` 和上报周期判断仍显式识别该状态。
- 保留原因：
  - 属于状态机保留状态，且和错误恢复边界、遥测周期有关。
  - 已人工确认不删除。

## RobotController::mission_active()

- 文件：
  - `include/pv_cleaning_robot/app/robot_controller.h`
  - `pv_cleaning_robot/app/robot_controller.cc`
- 保留内容：
  - `RobotController::mission_active() const noexcept`
- 证据：
  - 当前全项目只剩声明和定义，没有调用方。
- 保留原因：
  - 位于 `RobotController` 主状态机公开接口内，后续测试夹具或外部维护工具可能仍依赖该查询语义。
  - 已人工确认不删除。
