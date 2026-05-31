# Unattended Robot Architecture Refactor Design

## Goal

重构 `app`、`middleware`、`service`、`main` 与相关测试，使无人值守干挂式光伏清扫机器人业务保持完整，同时依赖方向清晰、职责单一、可扩展、运行稳定可靠。

## Non-Negotiable Constraints

- 不增加代码文件数量；允许删除、改名、合并、迁移现有代码文件。
- 不新增降级运行模式。
- 保留 HEALTH 和 DIAGNOSTICS 健康上报字段语义。
- 保留当前业务能力：单停机位往返清扫、双停机位任一端启动单段清扫、RPC start/stop/return/reset、调度启动、P0/P1/P2 故障链、限位急停与稳定到位确认、视觉 PID 辅助纠偏、BMS 正式环境启动约束。
- 每个阶段必须能独立编译验证。

## Target Dependency Direction

最终依赖方向：

```text
main.cc
  -> app use cases
  -> domain types + ports
  -> service adapters
  -> middleware/device/HAL
```

禁止方向：

- `service -> app`
- `middleware -> service/app`
- `device -> service/app`
- `app -> concrete device`
- `FSM -> concrete MotionService/NavService/FaultService`

允许方向：

- `main.cc` 作为组合根可以依赖所有层，用于构造对象和绑定端口。
- `app` 可以依赖 domain 类型和端口接口。
- `service` 可以实现 app 需要的端口，并依赖 device/HAL/middleware。
- `middleware` 只能依赖 HAL 或基础接口，不承载业务策略。

## File Strategy

不新增代码文件数量。通过以下方式实现分层：

- 将现有小文件改名/复用为 domain/ports 承载文件。
- 合并过窄或历史桥接文件，抵消必要的改名。
- 不为单一实现创建多余工厂、框架、容器。

候选迁移：

- `app/parking_side_runtime.h` 扩展/改名为业务 domain 类型承载文件，集中 `ParkingSideFacts`、`MissionPlan`、`SegmentSpec`、`FaultCode`、限位语义。
- `app/robot_runtime_snapshot.h` 扩展/改名为 app ports 承载文件，集中 `MotionPort`、`FaultPort`、`RobotControlPort`、`EmergencyStopPort`、`RuntimeSnapshot`。
- `ThingsBoardControlPlane` 不再 forward app 类型，只依赖 port 和纯 snapshot 类型。
- `SafetyMonitor` 不再 include `WalkMotorGroup`，只依赖急停端口。

## Domain Model

核心业务类型应是纯类型或纯函数：

- `ParkingSide`
- `ParkingSideFacts`
- `SegmentDirection`
- `SegmentMode`
- `CompletionCondition`
- `SegmentSpec`
- `MissionType`
- `MissionContext`
- `FaultCode`
- `FaultLevel`
- `RuntimeSnapshot`

domain 不读取配置文件、不调用硬件、不发布 MQTT、不启动线程。

## App Layer

### RobotFsm

职责：

- 维护任务段状态。
- 根据事件推进 `Idle/SelfCheck/ExecutingSegment/SegmentBoundary/Charging/FaultStopped`。
- 只通过端口启动/停止段动作。

不再负责：

- 读取配置。
- 解释物理限位。
- 直接依赖 `MotionService`、`NavService`、`FaultService`。
- 处理云控协议。

### RobotSupervisor

职责：

- 唯一业务用例入口。
- RPC/调度启动准入。
- 上电位置评估。
- 单停机位/双停机位任务建模。
- 错端限位、双限位、非预期限位统一 P0。
- P0/P1/P2 故障链协调。
- 安全 tick 中读取导航/传感器告警并上报 fault。

不再负责：

- 云协议 JSON。
- 线程启动/停止。
- 设备 open/close。

### FaultHandler

职责：

- 将故障事件转换为业务动作。
- P0：急停并进入 FaultStopped。
- P1：停滚刷并无刷返航。
- P2：只告警；指定 P2 可提升为 P1。

不再负责：

- 持有完整 MotionService。
- 发布云端协议。

## Service Layer

### MotionService

职责：

- 实现 `MotionPort`。
- 统一处理行走、滚刷、PID、override 清理。

边界：

- FSM 只能看到 `MotionPort`，不能看到完整 MotionService。
- PID 仍是辅助纠偏，不改变任务状态。

### ThingsBoardControlPlane

职责：

- 只处理 ThingsBoard 协议和 payload。
- 通过 `RobotControlPort` 调用 app 层业务。
- 通过 status event 上报命令结果。

不再负责：

- 依赖 `RobotSupervisor` 具体类型。
- 维护业务真相。

### ConfigService

职责：

- 管理 fixed/runtime/pending 配置。
- 负责字段校验、active/pending 生效规则。

不负责：

- 决定业务启动。
- 解释当前位置。

## Middleware Layer

### SafetyMonitor

职责：

- 限位触发后立即调用 `EmergencyStopPort`。
- 等待限位持续触发稳定后发布 `LimitSettledEvent`。
- release 稳定后重新 arm。

不负责：

- 知道 WalkMotorGroup。
- 知道 parking_side。
- 上报 FaultService。

## Main

`main.cc` 只保留：

- 配置加载。
- 日志初始化。
- HAL/driver/device/service/app 构造。
- port 绑定。
- 线程启动/停止。
- 优雅关停。

业务桥接应迁移到 app/service 现有文件中。`main.cc` 不再承载业务判断分支。

## Fault Strategy

关键故障源表：

- `0x0002 kWheelSpinFree`：电机空转/打滑，P0。
- `0x1001 kCanCommunicationLost`：CAN 通信失联，P0。
- `0x1003 kUnexpectedLimitSide`：错端/非预期限位，P0。
- `0x1004 kConflictingLimitSides`：双限位同时有效，P0。
- `0x1101 kSegmentStartFailed`：段启动失败，P0。
- `0x1102 kP1DuringReturnEscalatedToP0`：P1 返航中再次 P1，升级 P0。
- `0x2001 kBrushFaultReturnRequired`：滚刷故障需返航，P1。
- `0x2002 kReturnPathBlocked`：返航路径阻塞，P1/P0，按发生阶段判定。
- `0x3001 kSelfCheckFailed`：自检失败，P2/status event；不启动任务。
- `0x3002 kGpsLostRequiresReturn`：GPS 丢失需返航，P2 提升 P1。

故障处理同步规则：

- active fault 的唯一真相在 `FaultService`。
- FSM 状态转换通过 `FaultHandler` 订阅 fault event 完成。
- 云端故障上报不能阻塞安全线程；必须通过普通线程的 status/telemetry 周期上报或后续异步队列。

## Testing Strategy

按边界测试：

- domain：纯函数和任务段建模。
- FSM：状态转换和段动作端口调用。
- Supervisor：业务准入、限位策略、故障策略。
- ThingsBoardControlPlane：RPC 协议、payload、回调结果。
- SafetyMonitor：立即急停、稳定触发、release re-arm。
- main 相关：尽量通过 fixture 或集成测试验证装配，不复制业务逻辑。

每阶段验证：

- `cmake --build --preset rk3576-build --target unit_tests`
- `cmake --build --preset rk3576-build --target hw_tests`
- `git diff --check`

qemu 运行测试受 `PiMutex pthread_mutex_init failed: Operation not supported` 限制时，记录为环境限制，不作为代码通过证据。

## Phased Execution

1. Domain/ports extraction：只搬类型和接口，行为不变。
2. App orchestration cleanup：FSM/Supervisor/FaultHandler 分责。
3. Service/middleware dependency cleanup：ControlPlane/SafetyMonitor/MotionService 去反向依赖和宽接口。
4. Main/test cleanup：main 缩成组合根，测试 fixture 按层收口。

