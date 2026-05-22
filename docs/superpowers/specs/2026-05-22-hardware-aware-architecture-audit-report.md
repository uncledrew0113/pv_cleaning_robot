# Hardware-Aware Architecture Audit Report

## Executive Summary

当前项目已经具备清晰的工程分层雏形：`hal -> driver -> protocol -> device -> middleware -> service -> app -> main`。硬件装配、运动控制、安全限位、FSM、云控、健康遥测等主链路基本完整，能够支撑真实设备清扫往返、限位急停、BMS/IMU/GPS/滚刷/行走电机健康采集和 ThingsBoard 控制。

但当前架构还不是严格分层架构，主要靠目录约定维持边界，没有通过编译目标或接口约束阻止跨层依赖。对硬件项目来说，最高优先级问题不是“代码风格”，而是业务语义与硬件处置策略不一致：

- P1 故障目标语义是“先停，再无刷返航到停机侧”，当前实现是急停后进入 `Fault`。
- app 级故障码和自检失败原因没有形成统一云端上报链路。
- FSM 只支持整数完整往返趟，不能表达 0.5 趟清扫。
- `MotionService` 控制周期注释/实现存在 20 ms 与 50 ms 混用，未来启用 `ki/kd` 会影响 PID 调参。
- `ThingsBoardControlPlane` 反向依赖 `app::RobotSupervisor`，`ParkingSide` 等领域类型放置不当，导致跨层耦合。

建议先修业务安全闭环，再做架构清理：P1/P2/fault 上报、自检失败上报、BMS 无数据策略、FSM 半趟模型、RPC start 准入规则，应优先于纯粹的目录重构。

## Layer And Call Chain

### Current Layer Map

当前目录表达的层级如下：

1. `hal`
   - 系统资源接口：CAN、串口、GPIO、Modbus、互斥量。
2. `driver`
   - Linux 具体实现：SocketCAN、libserialport、libgpiod、libmodbus。
3. `protocol`
   - 设备协议：行走电机 CAN、ODrive ASCII、BMS、IMU、NMEA/GPSD、距离传感器。
4. `device`
   - 硬件对象：`WalkMotorGroup`、`BrushMotor`、`BMS`、`ImuDevice`、`GpsDevice`、`LimitSwitch`。
5. `middleware`
   - 运行支撑：`EventBus`、`ThreadExecutor`、`SafetyMonitor`、`NetworkManager`、`DataCache`。
6. `service`
   - 业务服务：`MotionService`、`NavService`、`HealthService`、`CloudService`、`ThingsBoardControlPlane`、`FaultService`、`HeadingCorrector`、`SchedulerService`。
7. `app`
   - 机器人业务状态：`RobotFsm`、`RobotSupervisor`、`FaultHandler`、`WatchdogMgr`、`ParkingSideRuntime`。
8. `main`
   - 组合根：配置加载、对象构造、线程启动、事件桥接、RPC 注册、关停。

`pv_cleaning_robot/CMakeLists.txt` 使用 `GLOB_RECURSE` 把 driver/protocol/device/middleware/service/app 全部编进一个可执行文件，边界没有被编译系统强制。因此跨层 include 可以自然发生，当前架构是“约定式分层”，不是“强约束分层”。

### Startup Chain

主链路：

`main -> ConfigService -> HAL/driver/device construction -> SafetyMonitor -> Network/Cloud -> Motion/Nav/Fault/FSM/Supervisor/ThingsBoard -> ThreadExecutor start -> main loop`

关键硬件点：

- 行走电机 CAN 初始化失败会直接退出。
- IMU/GPS/BMS/滚刷初始化失败多数只是 warning，系统仍可能继续运行。
- 限位开关通过 `SafetyMonitor` 绑定高优先级线程，触发后直接控制行走电机急停。
- BMS 数据通过 `current_battery_soc()` 影响启动门槛和充电完成。

风险：

- 设备是否“关键”没有统一策略表达。行走电机是 fatal，BMS 在主程序里是 non-fatal，但业务要求正式环境 BMS 无数据禁止启动。这个策略目前分散在启动日志、Supervisor 启动门槛和测试夹具差异里。
- `main.cc` 同时承担装配、业务桥接、云控注册、安全 tick、线程策略和关停顺序，变更风险高。

### Scheduled Start Chain

主链路：

`SchedulerService -> RobotSupervisor::handle_scheduler_window_hit -> start_parking_facts -> start_task -> RobotFsm::EvScheduleStart -> MotionService::start_cleaning -> WalkMotorGroup/BrushMotor`

硬件业务语义：

- 计划启动必须在停机位。
- `parking_side` 决定左/右限位哪个是停机侧。
- 电池 SOC 必须高于 `start_battery_soc`。

风险：

- GPIO 左右接线、`parking_side` 配置、限位低有效解释任一错误，都会让计划启动从错误端开始。
- 如果 BMS 无数据返回 0，计划启动会被电量门槛拒绝；如果未来绕过该检查，则有低电启动风险。

### RPC Start Chain

主链路：

`ThingsBoard RPC start -> ThingsBoardControlPlane -> RobotSupervisor::start_task_from_current_position -> RobotFsm::EvRpcStartTask -> MotionService::start_cleaning`

已确认业务语义：

- RPC start 是特权启动，不要求停机位。
- 只要电池满足、自检通过、当前位置/运动条件满足，就开启一次清扫任务。

当前代码状态：

- `RobotSupervisor::start_task_from_current_position()` 不检查 `at_parking_side`，符合该方向。
- `RobotFsm::EvRpcStartTask` 只支持整数趟，且总是从 `CleanFwd` 开始。

风险：

- 非停机位启动后，“下一次限位事件”如何解释完全依赖当前物理位置与 FSM 状态。如果机器人实际上靠近远端但 FSM 从 `CleanFwd` 开始，限位事件顺序需要硬件验证。
- RPC start 与计划 start 的准入规则应该明确拆开，不应复用同一个“停机位启动”语义。

### Cleaning Cycle Chain

主链路：

`RobotFsm CleanFwd/CleanReturn -> MotionService base wheel command -> optional HeadingCorrector -> WalkMotorGroup -> SafetyMonitor LimitSettledEvent -> RobotFsm transition`

硬件业务语义：

- `CleanFwd`：从停机侧向对侧端点。
- `CleanReturn`：从对侧端点返回停机侧。
- 停机侧命中后完成一整趟计数。

当前限制：

- `passes` 只接受大于等于 1 的整数。
- 没有 0.5 趟模型，不能表达“只清扫单程”或“半趟后停在对侧”。

风险：

- 当前 FSM 的 `completed_passes_` 是完整往返计数，半趟能力不是简单允许 `passes=0.5` 就能完成，必须重定义任务段和结束端点。
- `CleanFwd/CleanReturn` 是方向名，不是“去停机侧/去远端”的业务名；在支持非停机位 RPC start 和半趟后，命名容易误导。

### Limit Safety Chain

主链路：

`LimitSwitch trigger -> SafetyMonitor::on_limit_trigger -> WalkMotorGroup::emergency_override(0) -> monitor_loop debounce -> EventBus LimitSettledEvent -> RobotSupervisor/RobotFsm`

硬件业务语义：

- 限位触发后只急停。
- 不在 SafetyMonitor 自动反向脱离限位。
- 后续继续清扫、返程或结束任务由 FSM 决定。

架构判断：

- `SafetyMonitor` 从 middleware 直接调用 `WalkMotorGroup` 是跨层调用，但这是安全时延要求下的合理例外。
- 该例外应被明确隔离为 `EmergencyStopPort` 或类似窄接口，避免 middleware 任意扩大对 device 的依赖。

### Fault Chain

主链路：

`FaultService::report -> EventBus FaultEvent -> FaultHandler -> MotionService + RobotFsm`

当前代码状态：

- `FaultService::report()` 保存 `level/code/description/timestamp_ms` 并发布 EventBus。
- `FaultHandler` 对 P0 和 P1 都执行 `motion_->emergency_stop()`，P1 进入 `Fault`。
- `MotionService` 已有 `start_returning_no_brush()`，注释目标就是 P1 无刷返回，但当前 P1 链路没有用上。

业务结论：

- P1 当前实现与已确认业务不一致。目标应是：先停当前清扫动作，再无刷返航到停机侧，同时上报故障码。
- P2 当前只记录告警，不转换状态；“降速继续”没有完整控制闭环。

### Cloud And Telemetry Chain

主链路：

`HealthService -> CloudService publish_telemetry`

`RobotSupervisor::snapshot -> ThingsBoardJsonCodec::build_business_telemetry -> CloudService publish_telemetry`

当前代码状态：

- business telemetry 只输出 `device_state`、`task_state`、`active_config_version`。
- `RobotRuntimeSnapshot` 不含 fault 字段。
- `ThingsBoardControlPlane` 不订阅 FaultEvent。
- `HealthService` 能输出硬件级错误：轮毂电机 fault/error code、滚刷 fault、BMS alarm、IMU/GPS 状态等。

业务结论：

- 当前云端不能直接看到 app 级故障码、故障描述、自检失败原因和处置动作。
- 硬件健康遥测不能替代业务 fault 上报，因为硬件故障和业务故障不是同一个层级。

## Cross-Layer Dependency Findings

### Must Fix

1. `ThingsBoardControlPlane` 位于 service 层，但直接持有 `app::RobotSupervisor`。
   - 影响：云控协议层变成 app 编排入口，service -> app 反向依赖。
   - 建议：抽一个窄接口，例如 `TaskCommandPort` / `RuntimeSnapshotProvider`，由 app 实现，service 只依赖接口。

2. `ParkingSide` 领域类型放在 `ThingsBoardControlPlane`/service 相关头里。
   - 影响：`ParkingSideRuntime`、`HeadingCorrector` 等为了使用停机侧概念依赖云控头或 service 类型。
   - 建议：新增 domain 层或独立基础头，例如 `domain/robot_domain.h`，放 `ParkingSide`、`TaskPhase`、`FaultSeverity`、`LimitEndpoint`。

3. `RobotFsm` 直接依赖 `MotionService`、`NavService`、`FaultService`。
   - 影响：FSM 既管状态，又直接调硬件动作服务，难以单测纯业务状态。
   - 建议：拆 `RobotFsm` 为纯状态机 + action executor，或让 `RobotSupervisor` 作为动作编排者。

### Acceptable Exception

1. `SafetyMonitor` 直接依赖 `WalkMotorGroup`。
   - 原因：限位触发到急停有硬实时要求。
   - 保留条件：只能暴露急停窄接口，不能让 SafetyMonitor 承担业务状态决策。

### Later Optimization

1. `protocol/walk_motor_can_codec.h` 依赖 HAL 的 `CanFrame`。
   - 当前可接受，但协议层更理想的方向是使用协议自己的 frame DTO，HAL 适配在 driver/device 边界完成。

2. `main.cc` include 全层对象。
   - 组合根本来允许依赖全层，但当前业务逻辑过多，建议保留装配职责，迁出业务桥接。

## Hardware-Aware Business Inventory

### 1. 启动与自检

涉及硬件：

- CAN 行走电机
- 左右限位 GPIO
- BMS
- IMU/GPS
- 滚刷串口
- 网络链路

当前能力：

- 行走电机初始化失败退出。
- 启动位置异常会发布 `startup_position_invalid`。
- 不在任一端点时会尝试 `EvManualReturn`。

风险：

- 内部自检失败缺少统一云端事件。
- BMS 无数据在正式环境应禁止启动，但当前“BMS open 失败”本身不阻止主程序运行。
- 位置有效性与 RPC start 的关系需要更严格定义。

### 2. 计划任务启动

涉及硬件：

- 限位 GPIO
- BMS
- 行走电机
- 滚刷

当前能力：

- 要求停机侧有效。
- 要求电量高于启动阈值。
- pending runtime config 在启动前 promotion。

风险：

- BMS 无数据时的“0 SOC”与真实低电不可区分。
- 限位线序或停机侧配置错误会导致自动任务完全反向。

### 3. RPC 特权启动

涉及硬件：

- 云控网络
- BMS
- 行走电机
- 滚刷
- 限位/定位输入

当前能力：

- 不要求停机侧，符合已确认业务方向。
- 仍要求 position valid 和电量门槛。

风险：

- FSM 缺少“从当前位置决定下一段目标端点”的模型，当前总是进入 `CleanFwd`。
- 需要云端事件明确记录“特权启动来源、起点状态、起点是否停机侧”。

### 4. 清扫往返

涉及硬件：

- 四个行走电机
- 滚刷
- 限位
- 视觉 UDS

当前能力：

- 支持完整往返整数趟。
- 支持停机侧镜像改变速度符号。
- 支持视觉 PID 辅助纠偏。

风险：

- 不支持 0.5 趟。
- PID dt 固定为 `0.02f`，而 `walk_ctrl` 当前配置周期为 50 ms；`ki/kd=0` 时影响小，但未来启用 I/D 项会影响行为。
- 视觉 PID 是辅助纠偏，不应参与安全决策。

### 5. 限位急停

涉及硬件：

- 左右限位 GPIO
- 行走电机 CAN

当前能力：

- 触发后立即 `emergency_override(0)`。
- 180 ms 后发布限位稳定事件。
- 等待释放稳定后同侧重新 armed。

风险：

- 同时触发两侧限位时，业务层如何处理仍需统一定义。
- SafetyMonitor 层不自动反退是正确方向，但 FSM 必须能处理急停后恢复。

### 6. 手动 stop / return

涉及硬件：

- 行走电机
- 滚刷
- 限位

当前能力：

- stop 在清扫/返航态可进入 `Stopped` 并急停。
- return 在清扫/Stopped/Idle 且不在停机侧时进入 `Returning`。

风险：

- `Stopped` 后允许 start，但恢复路径和云端语义需要明确。
- return 从中间位置出发依赖方向判断，缺少明确的“当前位置可信度”字段。

### 7. 故障处置

涉及硬件：

- 所有执行器
- CAN/串口/GPIO
- BMS/IMU/GPS
- 看门狗

当前能力：

- P0 可急停进入 Fault。
- Watchdog 超时会上报 P0。
- 硬件测试有 P0/P1 链路验证意图。

风险：

- P1 实现与目标业务不一致。
- P2 缺降速闭环。
- app fault 未上报云端故障码。
- `FaultService::last_fault()` 返回 const 引用，锁释放后引用可被并发更新，接口存在并发安全隐患。

### 8. 电池与充电

涉及硬件：

- BMS
- 停机侧限位
- 充电位置/机构

当前能力：

- 启动按 `start_battery_soc` 过滤。
- 返回停机侧后按 `charge_start_soc` 进入 Charging 或 Idle。
- Charging 达到 `charge_stop_soc` 进入 Idle。

风险：

- BMS 无数据策略没有被显式建模为生产/测试两种模式。
- BMS 通信失败与真实 0 SOC 在业务上需要区分。

### 9. 健康遥测

涉及硬件：

- 行走电机
- 滚刷
- BMS
- IMU
- GPS
- 本地存储/云链路

当前能力：

- health/diagnostics 两种 payload。
- 支持本地 JSONL 轮转。
- 支持云端上报周期随任务状态切换。

风险：

- health telemetry 与 business telemetry 是不同采样链路，不是原子快照。
- development 模式上报硬件诊断更多，但 app 业务故障依然缺字段。
- 本地日志轮转配置正确，但长期现场部署仍需监控磁盘空间。

### 10. 配置与云控

涉及硬件：

- 所有端口、地址、速度、阈值、PID 参数
- ThingsBoard/MQTT/LoRa

当前能力：

- fixed/runtime config 分离。
- shared attributes 可修改 runtime 配置。
- RPC 支持 start/stop/return/reset。

风险：

- 哪些 runtime 字段立即生效、哪些下任务生效，需要更明确。
- PID 参数实时变更可能造成运动突变。
- `hw_test_config.json` 和生产 fixed config 的 `termination_motor_id` 不一致，应作为配置差异追溯项记录到硬件部署清单。

## Business Bug / Risk List

### Deterministic Bugs

1. P1 故障链错误
   - 目标：先停，再无刷返航到停机侧。
   - 当前：P1 与 P0 类似，急停并进入 `Fault`。
   - 硬件影响：机器人可能停在板面/轨道中间，无法按业务自动回到停机侧。

2. App fault 未上报故障码到云端
   - 目标：云端可见故障等级、故障码、描述、时间、处置动作。
   - 当前：FaultEvent 只在 EventBus/本地 FaultService 状态中流转。
   - 硬件影响：现场只能看到状态变成 Fault，无法远程定位 CAN、看门狗、悬空、坡度等原因。

3. FSM 不支持 0.5 趟
   - 目标：支持半趟业务。
   - 当前：`passes` 必须是大于等于 1 的整数。
   - 硬件影响：无法配置“只清扫单程/停在对侧”等业务。

4. PID dt 与控制周期不一致
   - 当前：`MotionService::update()` 给 HeadingCorrector 的 `dt_s=0.02f`。
   - 当前线程：`walk_ctrl` 配置为 50 ms。
   - 硬件影响：P-only 时主要影响小，启用 I/D 后会直接影响积分/微分量。

### Semantic / Design Risks

1. P2 “降速继续”只有语义，没有运动控制闭环。
2. BMS 无数据没有生产/测试策略开关。
3. RPC start 从非停机位启动后，FSM 缺少“从当前位置推导目标端点”的显式模型。
4. `CleanFwd/CleanReturn` 命名与半趟、非停机位启动不天然兼容。
5. 云端 business telemetry 过瘦，难以支撑运维。
6. SafetyMonitor 的安全例外没有接口化，后续重构容易误改。

## SOLID Assessment

### Single Responsibility

不完全满足。

- `main.cc` 同时承担装配、业务桥接、云控回调、线程策略、安全 tick 和关停。
- `MotionService` 同时管理任务速度、执行器命令、滚刷、PID enable/reset、base command 和 override 恢复。
- `ThingsBoardControlPlane` 同时处理云协议、RPC、CommandTracker 和 app Supervisor 调用。

### Open/Closed

部分满足。

- HAL/driver/device 对硬件替换较友好。
- 新增 fault 上报字段、P1 策略、半趟 FSM、RPC 准入规则时，需要修改多个核心类，扩展成本偏高。

### Liskov Substitution

基本满足但生命周期语义需继续收紧。

- mock 与真实 HAL/设备在测试中可替换。
- 但生产与硬件测试对 BMS/IMU/GPS 失败容忍度不同，应显式通过环境策略表达，而不是散落在 fixture 和 main。

### Interface Segregation

不充分。

- 云控依赖完整 `RobotSupervisor`。
- FSM 依赖完整 `MotionService`。
- SafetyMonitor 依赖完整 `WalkMotorGroup`，虽有安全理由，但仍应收窄为急停接口。

### Dependency Inversion

不充分。

- service -> app 反向依赖存在。
- app -> device 物理类型依赖存在。
- domain 类型归属不清，导致 `ParkingSide` 被多个层级绕路引用。

## Architecture Verdict

当前架构“能跑、能测、能接硬件”，但还不是稳定的业务架构。它更像是一个逐步演进出来的全栈硬件控制程序：

- 优点：真实硬件链路完整，限位安全路径优先级明确，主程序关停顺序较谨慎，健康遥测覆盖主要设备。
- 缺点：业务语义分散，云端真相不完整，故障策略不一致，层级约束弱，FSM 扩展到半趟会牵动较多逻辑。

对硬件项目来说，建议先修“会影响机器怎么动、故障怎么停、云端怎么看”的问题，再做纯架构整理。

## Recommended Remediation Order

### Phase 1: Safety And Business Truth

1. 修 P1：`FaultHandler/FSM` 改为先停再 `start_returning_no_brush()`，进入 `Returning`。
2. 补 app fault 云端上报：fault event 或 business telemetry 增加 level/code/description/timestamp/action。
3. 补自检失败上报：计划启动、RPC start、FSM 内部 self-check fail 均上报 reason。
4. 明确 BMS 无数据策略：production 禁止启动，hardware test 允许降级并标记。

### Phase 2: FSM Hardening

1. 重定义任务单位：segment/pass 分离。
2. 支持 `0.5` 趟：明确起点、终点、是否回停机侧、是否充电。
3. 将 `CleanFwd/CleanReturn` 逐步迁移为更业务化的“去远端/回停机侧”语义。
4. 统一异常限位事件处理：重复限位、双限位、错端限位、非预期限位。

### Phase 3: Architecture Boundary Cleanup

1. 抽 domain 基础类型。
2. 拆 `ThingsBoardControlPlane -> RobotSupervisor` 反向依赖。
3. 给 SafetyMonitor 引入急停窄接口。
4. 让 FSM 更纯，动作执行移到 Supervisor 或 action executor。
5. 用 CMake 子目标或 include 规则约束分层。

### Phase 4: Control And Telemetry Polish

1. 修正 PID `dt_s` 来源，使用实际周期或实测时间差。
2. 扩展 business telemetry，至少包括 target/completed passes、command、fault、自检状态、parking_side。
3. 把 HealthService 的硬件诊断与业务 telemetry 的状态关联起来，方便现场排障。

## Immediate Next Questions

报告生成后，下一步实施前建议只确认两个点：

1. 0.5 趟的结束位置：是允许停在远端，还是半趟完成后必须再自动返航？
2. P1 返航期间如果再次触发故障或错端限位，优先级是进入 Fault 原地停，还是继续尝试回停机侧？
