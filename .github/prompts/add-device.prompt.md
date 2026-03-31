---
description: "向导式新增硬件设备：从 HAL 接口 → 驱动实现 → 设备抽象 → Mock → 单元测试 → main.cc 接入，遵循项目 6 层架构与并发规则"
argument-hint: "设备名称 通信接口（can|uart|modbus|gpio） [简要说明]"
agent: "agent"
---

# 新增硬件设备向导

在 PV 清扫机器人固件中，按 **HAL → Driver → Device → 测试 → main.cc 接入** 的顺序新增一个硬件设备。

## 输入参数

用户提供（从对话或参数中提取）：

- **$DEVICE_NAME** — 设备名，PascalCase，例如 `UltrasonicSensor`
- **$IFACE_TYPE** — 通信接口：`can` | `uart`(串口) | `modbus`(RS485) | `gpio`
- **$DESCRIPTION** — 一句话功能描述，例如"超声波障碍物检测，UART 115200"

如果参数缺失，先询问用户再继续。

---

## 架构约束（必须遵守）

参考 [工作区指令](../copilot-instructions.md) 和 [并发规范](../../doc/dev-guide/CONCURRENCY.md)：

- 6 层严格向下依赖：HAL ← Driver ← Device ← Middleware/Service ← App
- 设备类只依赖 HAL 接口（`I*`），**禁止** 在设备类内部 `new` 具体驱动
- `get_*()` 方法返回缓存，无 I/O；`update()` 执行 I/O
- RT 路径禁用 `std::mutex`，用 `PiMutex`（`PTHREAD_PRIO_INHERIT`）
- 驱动层返回 `DeviceError` 枚举，不抛异常

---

## 执行步骤

按以下顺序生成代码，每步完成后等待确认（或一次性全部输出由用户选择）。

### Step 1 — HAL 接口（仅 `uart` / `modbus` / `gpio` 新接口时需要）

> CAN 设备复用已有 `ICanBus`；若通信接口已有对应 `I*.h`，跳过此步。

文件：`include/pv_cleaning_robot/hal/i_$device_snake.h`

```cpp
// 参考 include/pv_cleaning_robot/hal/i_serial_port.h 的格式
// 纯虚接口，I 前缀，无成员变量，析构 = default
```

### Step 2 — 驱动实现

文件：
- `include/pv_cleaning_robot/driver/$driver_name.h`
- `pv_cleaning_robot/driver/$driver_name.cc`

要点：
- 构造函数接收底层系统资源（fd / port / ctx）
- 实现对应 HAL 接口
- 不持有业务状态

对应关系：

| $IFACE_TYPE | 参考驱动 |
|-------------|---------|
| `can` | `linux_can_socket.h/.cc` |
| `uart` | `libserialport_port.h/.cc` |
| `modbus` | `libmodbus_master.h/.cc` |
| `gpio` | `libgpiod_pin.h/.cc` |

### Step 3 — 设备抽象

文件：
- `include/pv_cleaning_robot/device/$device_snake.h`
- `pv_cleaning_robot/device/$device_snake.cc`

必须包含：

```cpp
// 1. 数据结构
struct $DEVICE_NAMEData {
    // 缓存字段
    bool valid{false};
};

// 2. 构造（依赖注入，接收 HAL 接口指针）
$DEVICE_NAME(std::shared_ptr<hal::I***> iface, /* config params */);

// 3. 生命周期
bool open();            ///< 初始化，返回 false 表示失败（不抛异常）
void close();

// 4. 缓存读取（无 I/O，线程安全）
$DEVICE_NAMEData get_latest() const;

// 5. 周期更新（由 ThreadExecutor 调用，不阻塞超过 2× 周期）
void update();          // 或 IRunnable::update()

// 6. 私有
mutable robot::hal::PiMutex mtx_;
$DEVICE_NAMEData cache_;
```

**如果设备有后台读取线程**（如 UART 流式），还需：

```cpp
void read_loop();       ///< 线程函数，RT 优先级在内部用 pthread_self() 设置
std::thread read_thread_;
std::atomic<bool> running_{false};
```

### Step 4 — Mock 对象（用于单元测试）

文件：`test/mock/mock_$device_snake.h`

```cpp
// 参考 test/mock/mock_can_bus.h
// 继承设备接口或整个设备类，提供可注入的测试数据
class Mock$DEVICE_NAME {
public:
    void inject($DEVICE_NAMEData data) { injected_ = data; }
    $DEVICE_NAMEData get_latest() const { return injected_; }
    void open() {}
    void update() {}
private:
    $DEVICE_NAMEData injected_;
};
```

### Step 5 — 单元测试

文件：`test/$device_snake_test.cc`

测试用例清单（最少）：

1. `get_latest()` 返回 `valid=false` 在 `open()` 前
2. `update()` 解析一帧正常数据 → `get_latest().valid == true`
3. `update()` 超时（Mock 驱动返回 `TIMEOUT`）→ `valid` 保持旧值，不崩溃
4. `open()` 失败时 `update()` 不崩溃

参考 `test/imu_test.cc` 或 `test/bms_test.cc` 了解 Catch2 TEST_CASE 结构。

### Step 6 — main.cc 接入

在 `pv_cleaning_robot/main.cc` 中：

1. **构造驱动**（参考相同接口类型的已有代码块，如 `// ── 6. IMU ──`）：
   ```cpp
   // ── N. $DESCRIPTION ───────────────────────────────────────
   auto $device_var = std::make_shared<robot::device::$DEVICE_NAME>(
       /* driver */, /* config from cfg.get<T>(...) */);
   if (!$device_var->open())
       log->warn("[Main] $DEVICE_NAME 初始化失败");
   ```

2. **读取配置**（所有参数从 `config.json` 读，不硬编码）：
   ```cpp
   auto port = cfg.get<std::string>("$device_key.port", "/dev/ttyS?");
   ```

3. **注册到 ThreadExecutor**（选择合适的执行器和周期）：
   ```cpp
   bms_exec.add_runnable($device_var);  // 500ms，SCHED_OTHER
   // 或 nav_exec.add_runnable(...)      // 10ms，SCHED_FIFO 65
   ```

4. **注入 Service**（如需上层使用，修改对应 Service 的构造参数）。

### Step 7 — config.json 配置键

在 `config/config.json` 中新增设备配置节：

```jsonc
"$device_key": {
  "port":     "/dev/ttyS?",    // UART/Modbus 设备文件
  "baudrate": 9600,
  // 其他设备特定参数
  "enabled":  true
}
```

### Step 8 — API_REFERENCE.md 更新

在 `doc/API_REFERENCE.md` 第 5 节（Device 层）新增小节，包含：
- 类接口签名（`Status` / `Diagnostics` struct + 公有方法）
- 通信协议说明（寄存器地址 / 帧格式）
- 线程模型（`update()` 周期、后台线程 RT 优先级）

---

## 完成检查清单

生成所有文件后，执行：

```bash
cmake --build --preset rk3576-build 2>&1 | tail -20
```

确认 `[100%] Built target pv_cleaning_robot` 且无 warning/error。

出现编译错误时，先读取完整报错再修复，不要盲目修改。
