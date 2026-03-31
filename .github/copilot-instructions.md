# PV 清扫机器人 — Copilot 工作区指令

RK3576 PREEMPT_RT Linux，aarch64，C++17。光伏板面清扫机器人固件——主轨道清扫、故障自检、云端遥测。

---

## 构建与测试

### 前置条件

交叉编译工具链须在 PATH 中：`aarch64-linux-gnu-gcc`。  
Sysroot 固定路径：`/home/tronlong/RK3576/sysroots/armv8a-linux`（见 [cmake/toolchain.cmake](cmake/toolchain.cmake)）。

### 构建命令

```bash
# 首次配置（交叉编译 → build/）
cmake --preset rk3576-cross-linux

# 编译所有目标（主程序 + 单元测试）
cmake --build --preset rk3576-build

# 仅编译主程序
cmake --build build --target pv_cleaning_robot

# 仅编译单元测试
cmake --build build --target unit_tests
```

构建产物：
- 主程序：`build/aarch64/bin/pv_cleaning_robot`
- 单元测试：`build/aarch64/bin/unit_tests`

### 运行测试

测试须在 aarch64 目标板上运行（或 QEMU 模拟）。

```bash
# 所有测试
./build/aarch64/bin/unit_tests

# 指定测试段
./build/aarch64/bin/unit_tests --section "driver can驱动"

# CTest（在目标板上）
cd build && ctest --verbose
```

测试位于 [test/](test/)，使用 **Catch2**。Mock 对象在 [test/mock/](test/mock/)（`MockCanBus`、`MockGpioPin`、`MockModbusMaster`、`MockSerialPort`）。

---

## 架构

6 层严格向下依赖，**禁止跨层跳用**：

```
main.cc          配置驱动的依赖注入，优雅关闭
  App            robot_fsm / clean_task / fault_handler / watchdog_mgr
  Service        motion / nav / cloud / fault / health
  Middleware     event_bus / thread_executor / safety_monitor / data_cache / network
  Device         walk_motor_group / bms / imu_device / gps_device / brush_motor / limit_switch
  Driver         linux_can_socket / libgpiod_pin / libserialport_port / libmodbus_master
  HAL            i_can_bus.h / i_gpio_pin.h / i_serial_port.h / i_modbus_master.h（纯虚接口）
```

详细 API 见 [doc/API_REFERENCE.md](doc/API_REFERENCE.md)。

### 关键组件边界

| 组件 | 职责边界注意事项 |
|------|----------------|
| `WalkMotorGroup` | 4 轮统一控制（替代单轮 `WalkMotor`）。急停用 `emergency_override(0.0f)`，**不要**直接操作单轮 |
| `SafetyMonitor` | 持有 `WalkMotorGroup` 引用，GPIO 触发时直接调用 `emergency_override`，**不经过** `MotionService` |
| `EventBus` | `publish()` 热路径：`const void*` + `static_cast`，零堆分配；订阅者回调须极短（<100μs） |
| `HealthService` | HEALTH/DIAGNOSTICS 双模式由 `config.json::diagnostics.mode` 决定，**不要新建** `DiagnosticsCollector` |
| `DataCache` | JSONL 原子写（`.tmp` → `rename`），断电安全 |

---

## 线程模型与 RT 调度

线程调度策略和 CPU 亲和性见 [doc/API_REFERENCE.md §10](doc/API_REFERENCE.md)。

| 线程名 | 策略 | 优先级 | CPU | 周期 |
|--------|------|--------|-----|------|
| `gpio_front/rear` | SCHED_FIFO | **95** | 4 | 事件驱动 |
| `safety_monitor` | SCHED_FIFO | **94** | 4 | 5ms |
| `group_recv` | SCHED_FIFO | **82** | 5 | 流式 |
| `walk_ctrl` | SCHED_FIFO | **80** | 5 | 50ms |
| `imu_read` | SCHED_FIFO | **68** | 6 | 流式 |
| `nav` | SCHED_FIFO | **65** | 6 | 10ms |
| `watchdog_mon` | SCHED_FIFO | **50** | 7 | 200ms |
| `bms` / `cloud` | SCHED_OTHER | - | 0-3 | 500ms / 配置 |

**RT 设置规则**：调度策略和 CPU 亲和性在**线程内部**（`pthread_self()`）设置，不在主线程中设置。

---

## 并发规则（必须遵守）

> 详见 [doc/dev-guide/CONCURRENCY.md](doc/dev-guide/CONCURRENCY.md)。

1. **只用 `PiMutex`**（`PTHREAD_PRIO_INHERIT` + Robust）。RT 线程路径上**禁用** `std::mutex`，否则出现优先级反转。
2. **EventBus 回调不能阻塞**，不能再调用 `publish()`（防重入死锁）。
3. **持锁时不做 I/O**（Modbus/CAN/Serial），先释放锁再通信。
4. **关闭顺序**：`request_stop()` → `join()` → 释放硬件句柄（见 CONCURRENCY.md 关闭顺序节）。
5. 所有内存映射（实时性要求）在 `main()` 启动时调用 `mlockall(MCL_CURRENT | MCL_FUTURE)`。

---

## 代码规范

### 命名

```cpp
ClassName           // PascalCase
method_name()       // snake_case
member_var_         // 下划线后缀（Private 成员）
kConstName          // k 前缀 + PascalCase（constexpr / static const）
```

### 接口

- HAL 纯虚接口以 `I` 前缀命名（`ICanBus`、`IGpioPin`），设备类依赖接口而非具体实现
- 实现类通过构造函数注入依赖（**禁止**在类内部 `new` 具体驱动）
- `get_*()` 方法返回缓存值（无 I/O）；`update()` 执行实际 I/O

### `update()` 周期

`IRunnable::update()` 由 `ThreadExecutor` 调用，实现类中：
- **禁止**在 `update()` 内阻塞（>2× 周期）
- 读取状态用缓存（`get_latest()`、`get_data()`），不直接调 I/O

### 错误处理

- 驱动层返回 `DeviceError` 枚举（`OK` / `TIMEOUT` / `BUS_ERROR` 等），不抛异常
- 服务层 P0 故障通过 `FaultService::report()` + `EventBus` 向上传播
- 日志用 `middleware::Logger::get()` 获取 spdlog 实例，格式：`[模块名] 消息`

---

## 配置文件

运行时配置：`/opt/robot/config/config.json`（部署）/ `config/config.json`（开发回退）。

读取用 `ConfigService::get<T>(path, default)`，路径为 `.` 分隔的 JSON 键（如 `"network.mqtt.broker_uri"`）。

关键配置项：

```jsonc
"diagnostics.mode"          // "production" | "development"
"diagnostics.local_path"    // 本地遥测 JSONL 路径（空 = 禁用）
"robot.clean_speed_rpm"     // 清扫速度
"system.hw_watchdog"        // 硬件看门狗设备路径
"network.transport_mode"    // "mqtt_only" | "lorawan_only" | "dual_parallel"
```

---

## 常见陷阱

| 陷阱 | 正确做法 |
|------|---------|
| 在 RT 路径上分配堆内存 | 预分配 JSON 键树（见 HealthService 构造），publish 路径用 `static_cast<const void*>` |
| 在外部（主线程）设置 RT 优先级 | 在线程内 `pthread_setschedparam(pthread_self(), ...)` |
| 用 `WalkMotor` 单轮急停 | 用 `WalkMotorGroup::emergency_override(0.0f)`（同时停4轮+锁定心跳） |
| 新增 `DiagnosticsCollector` | 直接用 `HealthService::Mode::DIAGNOSTICS` |
| `std::ofstream` 未创建父目录 | `std::filesystem::create_directories(path.parent_path())` 再 `open()` |
| 在 `EventBus` 回调中调用 `publish()` | 不可以，会死锁；异步需求投递到外部队列 |
| `PiMutex` `EOWNERDEAD` 被吞掉 | 捕获后必须 `pthread_mutex_consistent()` 或重建受保护状态 |

---

## 参考文档

- [doc/API_REFERENCE.md](doc/API_REFERENCE.md) — 完整 API 文档（所有类接口、线程表、数据流）
- [doc/dev-guide/CONCURRENCY.md](doc/dev-guide/CONCURRENCY.md) — 并发契约与关闭顺序
- [config/config.json](config/config.json) — 运行时配置参考
