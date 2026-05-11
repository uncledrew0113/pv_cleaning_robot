# RPC Start Override Design

## Goal

把云端 RPC `start` 从“普通业务启动”里拆出来，定义成更高权限的远程控制命令。

本次只解决以下问题：

- 保留普通业务 `start` 的原语义：必须在停机位启动新任务
- 允许云端 RPC `start` 在非停机位也能启动完整清扫流程
- 明确 RPC `stop` / `return` 与普通任务控制的区别
- 让状态机、控制平面、测试语义保持一致

本次不做以下事情：

- 不修改调度器启动语义
- 不修改 shared-attribute 配置结构
- 不引入新的云端 RPC 方法名
- 不放宽 P0 / 悬空检测等安全保护

## Current Problem

当前代码把“普通业务启动”和“云端 RPC start”都收敛到同一条路径：

- `RobotSupervisor::start_task(...)`
- `EvScheduleStart`

这条路径的定义是“从停机位开始一个正常清扫任务”。

因此现在的行为是：

- 普通业务 `start` 必须在停机位
- 云端 RPC `start` 也必须在停机位
- 一旦用户想从当前所在位置直接开始一次完整清扫，现有语义无法表达

这不是单纯删一个判断就能解决的问题，因为：

- `RobotSupervisor` 在入口层要求 `position_valid && at_parking_side`
- `RobotFsm::dispatch<EvScheduleStart>` 内部也只接受 `at_parking_side == true`

如果只删掉一层限制，会造成“控制平面允许，但 FSM 仍拒绝”的语义不一致。

## Target Semantics

### 1. 普通业务 `start`

保持现状：

- 只允许从停机位启动
- 仍然表示“开始一个正常清扫任务”
- 使用配置中的 `passes`

这里的“普通业务 `start`”包括：

- 调度器启动
- 任何复用现有 `EvScheduleStart` 的内部业务入口

### 2. 云端 RPC `start`

重新定义为高权限启动：

- 不要求当前在停机位
- 不要求 `at_parking_side == true`
- 从当前位置直接进入 `CleanFwd`
- 到达对侧限位后进入 `CleanReturn`
- 返程继续带刷清扫
- 回到停机位后按正常完整趟数计数
- 总趟数仍取配置中的 `passes`

也就是说，RPC `start` 的语义是：

- “从当前位置强制开始一轮完整的前进清扫 + 返程清扫任务”
- 只是首段不再要求从停机位出发

### 3. 云端 RPC `stop`

保持“强制停机”语义：

- 四轮停止
- 滚刷停止
- 进入 `Stopped`

它不负责回停机位，也不负责继续任务。

### 4. 云端 RPC `return`

保持“回停机位”语义：

- 回停机位
- 调用 `start_returning_no_brush()`
- 返程不带刷
- 不作为正常清扫趟数推进的一部分

它和 RPC `start` 明确区分：

- RPC `start` 的返程是带刷、属于正常任务的一部分
- RPC `return` 的返程是不带刷、属于人工回收动作

## Design

### 1. Keep `EvScheduleStart` For Normal Business Start

现有 `EvScheduleStart` 保持原职责不变：

- 普通业务启动
- 从停机位启动
- 启动前做当前定义下的自检与约束检查

这样可以保证：

- 调度逻辑不变
- 现有“正常任务必须从停机位开始”的业务规则不变

### 2. Add A Separate RPC-Only Start Path

新增一条只供云端 RPC `start` 使用的特权路径。

设计原则：

- 不复用“必须在停机位”的普通入口
- 不改变普通业务 `start` 的定义
- 显式表达“这是云端强制开始完整清扫任务”

建议实现为：

- `RobotSupervisor` 增加一个 RPC 专用启动入口
- `RobotFsm` 增加一个 RPC 专用启动事件

该事件进入状态机后的行为为：

1. 直接把任务初始化为配置中的 `passes`
2. 把状态推进到 `CleanFwd`
3. 调用 `motion_->start_cleaning()`
4. 后续继续复用已有：
   - `EvFarEndLimitSettled -> CleanReturn`
   - `EvParkingSideLimitSettled -> 完成趟数 / 继续下一趟 / 完成任务`

这样首段放宽，后段复用现有完整清扫链。

### 3. Pass Counting Rule

RPC `start` 虽然可能从非停机位开始，但回到停机位后仍按“完成 1 趟”计数。

这里采用最简单、最稳定的规则：

- 每次从 `CleanFwd` 到 `CleanReturn` 再回到停机位，算完成 1 趟
- 与当前整数趟计数规则保持一致

这意味着：

- 首次从非停机位介入不会引入“半趟”“补趟”之类的新概念
- 任务语义仍然简单：回到停机位一次，就完成一整趟

### 4. RPC Response Semantics

`ThingsBoardControlPlane` 中：

- RPC `start` 不再因为“不在停机位”而拒绝
- 但仍然要保留以下拒绝条件：
  - 当前状态不允许启动
  - 电量不足
  - pending 配置提升失败
  - 状态机最终未进入有效清扫状态

也就是说，只移除“必须在停机位”这一条 RPC 特有限制，不放宽其他安全和状态约束。

### 5. Safety Boundary

本设计不修改安全边界：

- `RobotSupervisor::tick_safety()` 的悬空检测仍然有效
- P0 / P1 故障仍可中断 RPC `start` 发起的任务
- 限位防抖与停机位/对侧限位桥接规则不变

因此：

- RPC `start` 被允许，不等于绕过安全系统
- 如果现场处于悬空、卡滞、故障或其他危险状态，任务仍应被打断

## Scope

预计涉及以下文件：

- [include/pv_cleaning_robot/app/robot_supervisor.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/app/robot_supervisor.h)
- [pv_cleaning_robot/app/robot_supervisor.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/app/robot_supervisor.cc)
- [include/pv_cleaning_robot/app/robot_fsm.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/app/robot_fsm.h)
- [pv_cleaning_robot/app/robot_fsm.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/app/robot_fsm.cc)
- [pv_cleaning_robot/service/thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc)
- 相关单元测试与 ThingsBoard 集成测试

## Testing

至少补齐以下验证：

- 普通业务 `start` 在非停机位仍然拒绝
- RPC `start` 在非停机位可接受
- RPC `start` 接受后进入 `CleanFwd`
- RPC `start` 到达对侧后进入 `CleanReturn`
- RPC `start` 返程继续带刷
- RPC `start` 回到停机位后完成趟数计数
- RPC `return` 仍走 `start_returning_no_brush()`
- RPC `stop` 仍使四轮和滚刷停机

## Risks

最大风险不是代码复杂度，而是语义混淆。

因此本设计坚持两点：

- 普通业务 `start` 不变
- RPC `start` 单独建模

这样用户、测试、日志、云端行为都更容易解释，不会出现“同一个 start 在不同入口含义不同但代码里看不出来”的问题。
