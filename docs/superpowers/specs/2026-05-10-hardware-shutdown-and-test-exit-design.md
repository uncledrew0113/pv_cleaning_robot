# Hardware Shutdown And Test Exit Design

## Goal

补齐设备生命周期接口，收紧执行器停机顺序，并为硬件测试增加统一的优雅退出路径。

本设计只覆盖以下目标：

- 给缺失 `close()` / 析构的设备补齐生命周期接口
- 明确执行器类设备比传感器类设备更严格的关停语义
- 补齐 `main()` 的显式关停缺口
- 给硬件测试增加统一的可捕获退出清理逻辑

本设计不做以下事情：

- 不引入项目级 `ShutdownManager` / `ResourceRegistry`
- 不尝试处理 `SIGKILL`、断电、内核强杀等不可捕获退出
- 不重构现有线程执行器或事件总线模型

## Scope

涉及模块：

- 设备层
  - `BrushMotor`
  - `DistanceSensor`
  - `WalkMotorGroup`
- 主程序
  - `pv_cleaning_robot/main.cc`
- 硬件测试
  - `test/integration/hardware/hw_config.h`
  - `test/integration/hardware/system_hw_test.cc`
  - 必要时补一个公共硬件测试退出守卫头/源文件

## Design

### 1. Device Lifecycle Policy

设备分两类：

- 执行器类：`WalkMotorGroup`、`BrushMotor`
- 采集类：`DistanceSensor`、`BMS`、`ImuDevice`、`GpsDevice`、`LimitSwitch`

执行器类要求：

- `close()` 必须是“先停机，再关通信”
- `close()` 必须幂等
- 析构函数调用 `close()` 作为兜底，但正常路径仍由调用方显式关闭

采集类要求：

- `close()` 只负责停线程、关句柄、关连接
- 不引入额外停机动作
- `close()` 同样要求幂等

### 2. BrushMotor Changes

`BrushMotor` 当前缺少统一生命周期出口，只能依赖：

- `motion->emergency_stop()` 间接调用 `brush_->stop()`
- 串口对象析构时关闭串口

这不够明确，也无法被硬件测试退出守卫统一调用。

新增：

- `~BrushMotor()`
- `void close()`

语义固定为：

1. 若串口未打开，直接返回
2. 调用 `stop()`
3. 直接关闭串口
4. 不调用 `enter_idle()`

原因：

- 用户已明确要求 `BrushMotor` 只做 `stop()` 后关串口
- 避免在退出路径里额外切换 ODrive 状态机，扩大副作用

幂等要求：

- 多次 `close()` 不崩溃
- 串口未打开时可安全调用
- `stop()` 失败不阻止后续串口关闭

### 3. DistanceSensor Changes

`DistanceSensor` 当前没有显式 `close()` / 析构。

新增：

- `~DistanceSensor()`
- `void close()`

语义：

- 若 `modbus_` 存在，则调用其 `close()`
- 不增加任何停机动作

这是纯资源释放，不改变业务行为。

### 4. WalkMotorGroup Close Semantics

`WalkMotorGroup` 已有析构和 `close()`，但 `close()` 目前偏资源释放语义。

收紧要求：

- `close()` 进入时，若总线已打开，先尝试让电机进入安全静止状态
- 再停接收线程
- 最后关闭 CAN

建议顺序：

1. 关闭姿态纠偏
2. 若当前可通信，尝试 `set_speed_uniform(0.0f)`
3. 再尝试 `disable_all()`
4. 停 `recv_thread_`
5. `can_->close()`

说明：

- 这里采用“尽力而为”的安全停机，不要求每一步都成功才继续
- 即使停车命令失败，也要继续完成线程和句柄关闭

### 5. Main Shutdown Sequence

`main()` 保留手动关闭，不依赖对象析构顺序。

最终顺序固定为：

1. 停执行线程
   - `walk_exec.stop()`
   - `nav_exec.stop()`
   - `bms_exec.stop()`
   - `cloud_exec.stop()`
2. 停管理组件
   - `safety_monitor.stop()`
   - `watchdog.stop()`
3. 停执行器
   - `motion->emergency_stop()`
   - `brush_motor->close()`
   - `walk_group->close()`
4. 关采集设备
   - `imu->close()`
   - `gps->close()`
   - `bms->close()`
   - `left_switch->close()`
   - `right_switch->close()`
5. 断外围
   - `net_mgr->disconnect()`
   - `data_cache->close()`

设计原则：

- 先停线程，避免并发访问已关闭设备
- 先停执行器，再关传感器
- 显式关停作为主路径，析构仅兜底

### 6. Hardware Test Graceful Exit

硬件测试增加统一退出守卫，不把信号处理散落到每个测试里。

新增一个公共守卫组件，职责：

- 安装 `SIGINT` / `SIGTERM` 处理器
- 维护全局退出标志
- 跟踪当前活动的硬件测试夹具
- 在测试轮询循环中提供“是否请求退出”判断

实现原则：

- 信号处理函数内只做异步安全动作：
  - 记录退出标志
- 不在信号处理函数里直接调设备 I/O
- 真正的设备关停在测试线程上下文完成

### 7. FullSystemFixture Shutdown Contract

`FullSystemFixture` 新增统一 `shutdown()`，由以下路径共用：

- 析构函数
- 测试发现退出标志后主动调用
- 未来其他硬件夹具可复用同一风格

`shutdown()` 顺序：

1. 停后台循环线程
2. 停 `safety`
3. `motion->emergency_stop()`
4. `brush->close()`
5. `walk_group->close()`
6. `gps->close()`
7. `imu->close()`
8. `bms->close()`
9. `left_sw->close()`
10. `right_sw->close()`
11. `watchdog->stop()`

要求：

- 幂等
- 可在部分初始化失败后安全调用
- 不依赖对象一定已成功 `open()`

### 8. Signal Coverage Boundary

本设计承诺覆盖：

- `SIGINT`
- `SIGTERM`
- 测试断言失败后的栈展开析构
- 普通异常展开析构

本设计不承诺覆盖：

- `SIGKILL`
- 断电
- 内核直接终止进程

## Risks

主要风险：

- 退出路径增加更多 I/O，若底层驱动阻塞，关停时间可能变长
- `WalkMotorGroup::close()` 中新增停车动作后，关闭时会多发 CAN 帧
- 硬件测试信号守卫若实现不当，可能和 Catch2 自己的终止流程冲突

控制措施：

- 所有 `close()` 都保持幂等
- 执行器停机采用“尽力而为”，不因单步失败阻塞整个关停
- 信号处理函数只置位标志，不直接做设备操作

## Verification

本次实现完成后至少验证：

- `pv_cleaning_robot` 可编译
- `unit_tests` 可编译
- `hw_tests` 可编译

不在交叉编译环境里执行测试程序。

后续现场验证重点：

- `Ctrl+C` 结束硬件测试时，行走和滚刷是否能停住
- `SIGTERM` 结束硬件测试时，是否走到统一 `shutdown()`
- `main` 正常退出时，设备关闭顺序是否按设计执行
