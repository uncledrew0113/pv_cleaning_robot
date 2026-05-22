# Hardware-Aware Architecture Audit Design

## Goal

本设计定义一次只读审计，不改代码。审计目标是把当前项目的调用链、业务清单、跨层调用、业务缺陷风险和架构质量讲清楚，并且把硬件设备纳入业务判断，而不是只从软件模块角度评价。

审计输出面向后续重构决策，必须回答三类问题：

1. 当前各级调用链是什么，是否存在跨级调用。
2. 当前所有业务有哪些，结合硬件设备判断是否有业务逻辑风险或 bug。
3. 当前架构是否清晰，是否满足 SOLID 原则，哪些问题需要优先处理。

## Scope

审计覆盖当前主程序和硬件测试体现出来的真实运行链路：

- 主程序装配与生命周期：`pv_cleaning_robot/main.cc`
- app 层：`RobotFsm`、`RobotSupervisor`、`FaultHandler`、`WatchdogMgr`、`ParkingSideRuntime`
- service 层：`MotionService`、`NavService`、`HealthService`、`CloudService`、`ThingsBoardControlPlane`、`ConfigService`、`FaultService`、`HeadingCorrector`、`SchedulerService`
- middleware 层：`EventBus`、`DataCache`、`SafetyMonitor`、`ThreadExecutor`、网络/MQTT/LoRa 支撑
- device/driver/hal/protocol 层：行走电机、滚刷、BMS、IMU、GPS、限位开关、距离传感器、CAN、串口、GPIO、Modbus、gpsd/NMEA
- 硬件集成测试：`test/integration/hardware/*`
- 配置：`config/config.fixed.json`、`config/config.runtime.json`、`test/integration/hardware/hw_test_config.json`

审计不做以下事情：

- 不修复 PID、FSM、云控或硬件驱动代码
- 不重构目录、接口或编译目标
- 不把历史文档当成真相；文档只作为意图和差异来源
- 不评价未接入代码路径之外的外部平台能力

## Current Layer Model

当前项目大体按以下层次组织：

1. `hal`
   - 抽象 CAN、串口、GPIO、Modbus、互斥量等系统资源。
2. `driver`
   - Linux 具体实现，例如 SocketCAN、libserialport、libgpiod、libmodbus。
3. `protocol`
   - 设备协议编解码，例如 BMS、IMU、GPS/NMEA、ODrive ASCII、行走电机 CAN。
4. `device`
   - 硬件设备对象，例如 `WalkMotorGroup`、`BrushMotor`、`BMS`、`ImuDevice`、`GpsDevice`、`LimitSwitch`。
5. `middleware`
   - 通用运行支撑，例如事件总线、线程执行器、缓存、网络、MQTT、LoRa、安全监视。
6. `service`
   - 业务服务和算法编排，例如运动控制、导航融合、健康采集、云控、调度、故障服务、视觉纠偏。
7. `app`
   - 机器人业务状态、任务准入、故障处理、运行快照和看门狗。
8. `main`
   - 组合根，负责配置加载、对象构造、线程启动、事件桥接、RPC 注册和关停。

理想依赖方向应为上层依赖下层接口，底层不依赖上层业务语义。当前代码基本按此组织，但存在若干明显跨层依赖和安全路径例外。

## Runtime Call Chains To Audit

### 1. Startup And Wiring

主链路：

`main -> ConfigService -> HAL/driver/device construction -> service construction -> app construction -> event/RPC bridge -> ThreadExecutor start`

审计点：

- `main.cc` 目前同时承担硬件装配、业务桥接、线程策略、云控 RPC、遥测周期切换和安全 tick，职责过重。
- CMake 当前将大部分源文件直接编进单个可执行目标，架构边界主要靠约定，不靠编译规则强制。

### 2. Scheduled Start

主链路：

`SchedulerService -> main callback -> RobotSupervisor::start_task -> RobotFsm::EvStartTask -> MotionService::start_cleaning -> WalkMotorGroup/BrushMotor`

硬件含义：

- 只有停机侧限位确认有效时，计划任务才应启动。
- BMS 电量低于启动阈值时应拒绝启动。
- 停机位左右由 `parking_side` 决定，GPIO 物理左右线接反或配置错误会直接导致任务从错误端启动。

### 3. RPC Start

主链路：

`ThingsBoard RPC start -> ThingsBoardControlPlane -> RobotSupervisor::start_task_from_current_position -> RobotFsm -> MotionService -> motors/brush`

硬件含义：

- RPC start 允许从“当前位置有效”处启动，不强制停机侧。这是功能能力，也是硬件风险：云端或人工误触可能让机器人从中途直接进入清扫段。
- 审计应明确这是设计选择还是需要收紧为只允许停机侧启动。

### 4. Cleaning Outbound And Return

主链路：

`RobotFsm CleanFwd/CleanReturn -> MotionService base command -> HeadingCorrector optional correction -> WalkMotorGroup set_speed -> SafetyMonitor limit event -> RobotFsm transition`

硬件含义：

- CleanFwd 与 CleanReturn 的速度符号依赖 `parking_side` 和上下轮物理安装方向。
- 视觉纠偏通过 UDS 结果改变四轮目标速度；`filtered_yaw_deg` 是控制误差，不等于原始视觉 `yaw_deg`，符号可因停机侧和阶段归一化而相反。
- 限位触发先由 `SafetyMonitor` 直接执行急停，再通过稳定事件驱动 FSM 进入下一段。

### 5. Limit Safety Path

主链路：

`LimitSwitch physical trigger -> SafetyMonitor::on_limit_trigger -> WalkMotorGroup::emergency_override(0) -> debounce -> EventBus LimitSettledEvent -> FSM`

硬件含义：

- 这是有意跨层：middleware 直接控制 device，是为了限位响应时间，不应简单视为错误。
- 风险在于该例外必须被清晰记录；否则后续重构可能把安全急停改回异步业务链路，拉长响应时间。

### 6. Manual Stop And Return

主链路：

`ThingsBoard RPC stop/return -> ThingsBoardControlPlane -> RobotSupervisor -> RobotFsm -> MotionService emergency_stop/start_returning_no_brush`

硬件含义：

- stop 必须保证行走电机和滚刷停机。
- return 应无刷返回停机侧，依赖限位正确识别。
- 如果当前位置无效，return 的策略必须谨慎，避免盲目运动。

### 7. Fault Handling

主链路：

`FaultService report -> FaultHandler -> RobotFsm fault event -> MotionService emergency_stop or return`

硬件含义：

- P0 是硬急停语义。
- P1 文档/测试倾向“安全返回停机位”，但当前 app 逻辑表现为进入 Fault 并急停。该差异需要作为高优先级业务风险确认。
- P2 文档说“降速继续”，但当前未看到完整降速业务闭环，需要作为能力缺口确认。

### 8. Navigation And Position

主链路：

`WalkMotorGroup feedback + IMU + GPS -> NavService -> pose/fused odom -> Supervisor position validity / telemetry`

硬件含义：

- IMU/GPS 在硬件测试中可非致命失败，系统仍可能依靠轮速/编码器运行。
- GPSD、NMEA、IMU 串口异常不一定阻断运动，但会降低定位和远程诊断可信度。
- 如果定位有效性被过度放宽，云端 RPC start 的风险会放大。

### 9. Battery And Charging

主链路：

`BMS serial -> BMS device -> Supervisor start gate / charging condition -> FSM Charging/Idle -> telemetry`

硬件含义：

- BMS 初始读取失败或返回 0 值时，启动门槛、充电状态和健康遥测都可能失真。
- 审计需要区分“BMS 通信失败时禁止启动”与“硬件测试允许非致命继续”的不同场景。

### 10. Cloud And Telemetry

主链路：

`HealthService + RobotSupervisor snapshot + DataCache -> ThingsBoardControlPlane/CloudService -> MQTT/LoRa/local JSONL`

硬件含义：

- 云端看到的任务状态、硬件健康、PID 调试值和故障状态必须能对应真实设备行为。
- 如果业务状态和硬件状态分别采样，遥测可能不是同一时刻的原子快照，应在审计中标明。

## Cross-Layer Calls To Report

当前需要明确分类的跨层调用包括：

- `ThingsBoardControlPlane` 位于 service 层，但直接持有并调用 `app::RobotSupervisor`，属于 service -> app 的反向依赖。
- `ParkingSideRuntime` 位于 app 层，但使用 `service::ParkingSide` 类型，说明领域基础类型放置位置不理想。
- `HeadingCorrector` 位于 service 层，但通过 `thingsboard_control_plane.h` 间接拿 `ParkingSide`，属于不必要的重依赖。
- `RobotSupervisor` 使用 `device::LimitSide`，app 层直接依赖设备层物理类型。
- `SafetyMonitor` 位于 middleware 层，直接调用 `WalkMotorGroup::emergency_override()`，这是安全响应的有意跨层例外，应保留但隔离说明。
- `protocol/walk_motor_can_codec.h` 依赖 HAL 的 CAN frame 类型，协议层与 HAL 绑定较紧。

审计结论应把跨层调用分为三类：

- 必须修复：反向依赖或领域类型放错位置导致的耦合。
- 可接受但需文档化：安全急停等硬件时延要求导致的例外。
- 可后续优化：不影响当前行为但削弱测试和维护边界的依赖。

## Hardware-Aware Business Inventory

审计需要按业务列出软件流程、硬件设备和主要风险。

### 1. 任务启动业务

涉及硬件：

- 左/右限位 GPIO
- BMS
- 行走电机 CAN
- 滚刷串口
- 云控网络

主要规则：

- 计划启动应要求在停机位。
- RPC 启动当前允许从有效当前位置启动。
- 电量不足应拒绝启动。

风险：

- `parking_side`、GPIO 线序、限位物理安装任一错误，都会把停机侧/远端判断反过来。
- BMS 无数据时若被当作 0，可能拒绝启动；若被忽略，可能低电启动。
- RPC start 的权限和位置约束需要产品层确认。

### 2. 清扫往返业务

涉及硬件：

- 四个行走电机
- 滚刷电机
- 限位开关
- 视觉 UDS 纠偏输入
- IMU/GPS/轮速反馈

主要规则：

- CleanFwd 从停机侧向远端。
- CleanReturn 从远端回停机侧。
- 每到端点通过限位稳定事件切换状态。
- 达到目标趟数后进入 Idle 或 Charging。

风险：

- 电机编号、上下轮定义、速度符号、停机侧镜像任何一个不一致，都会造成纠偏方向反。
- 视觉 UDS 频率低于控制 tick 时，系统会复用最近有效结果直到超时；超时后应回到基础速度。
- 硬件日志中出现过单个电机实际转速严重掉速的现象，需要把电机通信/堵转/限流与 PID 过调区分开。

### 3. 端点限位与急停业务

涉及硬件：

- 左/右限位 GPIO
- 行走电机 CAN

主要规则：

- 任一端点触发后，`SafetyMonitor` 立即对行走电机下发 emergency override 0。
- 稳定后 FSM 决定下一段或结束。

风险：

- 当前 safety override 总是 0，没有使用配置中的反向脱离速度；若业务期待“触发后倒退离开限位”，当前未实现。
- 如果限位抖动或线接反，机器人可能频繁急停或状态跳转错误。
- safety 直接调用 device 是安全需要，但必须避免其它 middleware 路径继续扩大这种例外。

### 4. 手动停止业务

涉及硬件：

- 行走电机
- 滚刷电机
- 云控网络

主要规则：

- stop 进入 Stopped 并急停。

风险：

- 需要确认 stop 后是否允许直接 start，还是必须 reset/return。
- 云控 stop 响应与真实电机停机之间可能存在采样延迟，遥测需要能显示硬件实际速度。

### 5. 手动返航业务

涉及硬件：

- 行走电机
- 限位开关
- BMS
- IMU/GPS/轮速

主要规则：

- return 应无刷返回停机侧。

风险：

- 若当前位置无效或不在端点，返航方向依赖当前配置与里程/姿态判断，错误会导致朝远离停机位运动。
- 停机侧配置变更在任务中途生效会造成返航方向风险；审计应确认 active/pending 配置边界。

### 6. 故障业务

涉及硬件：

- 所有执行器
- BMS
- IMU/GPS
- CAN/串口/GPIO
- 看门狗

主要规则：

- P0 应急停。
- P1/P2 的目标语义需要与代码统一。

风险：

- P1 当前实现与测试/文档语义不一致，是直接影响硬件处置策略的风险：到底是原地故障停机，还是安全返回停机侧。
- P2 若声明为降速继续但没有实际降速路径，会误导云端和运维判断。
- `FaultService` 故障清除和线程安全接口需要审计，否则 reset 后可能残留故障状态。

### 7. 电池与充电业务

涉及硬件：

- BMS
- 充电机构或停机位充电条件
- 限位开关

主要规则：

- 启动前检查 SOC。
- 到停机侧后按阈值进入 Charging 或 Idle。

风险：

- BMS 首次读取失败在日志中常见，必须明确“无 BMS 数据”时业务如何处理。
- 如果 `bat_soc=0` 是通信失败而非真实低电，云端会误判。
- 如果停机侧限位错误，系统可能在非充电位进入 Charging。

### 8. 定位与健康遥测业务

涉及硬件：

- IMU
- GPS/gpsd
- 行走电机反馈
- BMS
- 滚刷
- 本地存储和云链路

主要规则：

- 周期采集并上报健康数据。
- 任务中记录电机、电池、IMU/GPS、PID、刷子状态。

风险：

- 多线程采样不是原子快照，跨设备字段可能对应不同时间点。
- GPS/IMU 全 0 或失联时，如果仍标记为有效，会影响远程判断。
- 本地 JSONL 日志轮转必须避免占满存储。

### 9. 配置与云控业务

涉及硬件：

- 所有受配置影响的设备
- ThingsBoard/MQTT/LoRa

主要规则：

- 固定硬件配置应本地固定。
- 运行参数可由云端更新，部分立即生效，部分下个任务生效。

风险：

- 硬件端口、GPIO、CAN ID 等固定配置若被云端误改，风险高。
- 清扫速度、滚刷速度、PID 参数实时改变可能造成运动突变；需要明确哪些字段只能下个任务生效。
- 测试配置和生产配置存在差异时，应明确是否 deliberate，例如 termination motor id。

### 10. 复位/重启业务

涉及硬件：

- Linux 系统
- 所有执行器和通信外设
- 云控链路

主要规则：

- reset RPC 触发设备重启。

风险：

- reset 是高风险远程动作，必须先确保执行器停机和日志落盘。
- 若重启发生在运动中，依赖析构或系统断电并不等同于安全停机。

## High-Priority Risks To Validate

审计报告应至少确认以下风险是否成立：

1. P1 故障语义不一致：文档/测试倾向返航，当前 app 路径倾向 Fault + emergency stop。
2. P2 降速继续缺少完整闭环：声明存在，但运动服务未体现统一降速策略。
3. PID 控制周期风险：主运动线程和硬件测试常见周期为 50 ms，但控制代码/注释存在 20 ms 假设；若未来启用 `ki/kd` 会影响调参。
4. 视觉 PID 符号容易误解：`filtered_yaw_deg` 是归一化控制误差，不是原始视觉角；日志需要明确字段语义。
5. 限位急停绕过层级是必要例外，但边界未被架构显式保护。
6. `ThingsBoardControlPlane` 反向依赖 app 层，导致云控服务成为业务编排入口。
7. `ParkingSide` 这种领域类型放在 service/cloud 相关头里，造成 app/service/控制算法多处不自然依赖。
8. 固定硬件配置、运行配置、测试配置的差异需要逐项说明，否则硬件现场问题难以定位。
9. RPC start 从当前位置启动可能是功能，也可能是安全风险，需要产品确认。
10. BMS/IMU/GPS 非致命失败策略需要按生产和测试场景分开定义。

## SOLID And Architecture Assessment Criteria

### Single Responsibility

重点检查：

- `main.cc` 是否承担过多业务编排。
- `MotionService` 是否混合任务速度、执行器控制、PID 输入、纠偏生命周期。
- `ThingsBoardControlPlane` 是否同时负责云协议、RPC 解释、业务调度和 app 调用。

### Open/Closed

重点检查：

- 新增云控命令、硬件设备、故障等级时，是否需要修改过多核心类。
- PID、定位、健康遥测是否可替换或扩展，而不是散落在 main 和服务层。

### Liskov Substitution

重点检查：

- HAL/driver 接口是否可被 mock 与真实硬件替换。
- 设备对象是否在测试和生产中遵守相同生命周期语义。

### Interface Segregation

重点检查：

- 上层是否依赖过宽的设备对象，而不是只依赖需要的能力接口。
- 云控是否依赖完整 `RobotSupervisor`，而不是一个窄的任务控制接口。

### Dependency Inversion

重点检查：

- service -> app、app -> device 这类跨层依赖是否可通过领域类型和窄接口拆开。
- 安全急停这种例外是否有专用接口表达，而不是直接扩散 `WalkMotorGroup` 依赖。

## Expected Audit Output

最终审计报告应包含：

1. 完整调用链图谱
   - 按启动、任务、限位、故障、云控、遥测、关停分段。
2. 跨层调用清单
   - 标明必须修复、可接受例外、可后续优化。
3. 硬件业务清单
   - 每项业务列出涉及设备、软件入口、状态变化和主要风险。
4. 业务逻辑 bug / 风险列表
   - 区分确定性 bug、语义不一致、硬件配置风险、调参风险。
5. 架构质量判断
   - 按 SOLID 和实际硬件工程约束给结论。
6. 后续改造建议
   - 按高优先级风险、边界清理、测试补强分批。

## Recommended Follow-Up Direction

审计完成后，建议先处理高风险业务语义，再处理纯架构清理：

1. 先确认 P1/P2 故障策略、RPC start 位置约束、限位/parking_side 语义。
2. 再把领域基础类型抽到独立 domain 层，例如 `ParkingSide`、`TaskPhase`、`LimitEndpoint`、`FaultSeverity`。
3. 再拆云控到 app 的反向依赖，引入窄接口或命令端口。
4. 最后用 CMake 目标或 include 规则约束层级，避免跨层依赖回流。

## Review Questions

正式审计前需要用户确认的业务问题：

1. P1 故障到底应“原地故障停机”还是“无刷返航到停机侧”？
2. RPC start 是否允许从非停机侧但位置有效处直接启动？
3. 限位触发后是否只需要急停为 0，还是需要支持反向脱离限位？
4. BMS 无数据时，生产环境应禁止启动还是允许降级运行？
5. PID/视觉纠偏是否只作为可选增强，还是未来必须参与安全相关决策？
