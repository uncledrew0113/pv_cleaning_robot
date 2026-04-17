# HW Test Config Externalization & Test Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 hw_tests 所有硬件参数从编译期 constexpr 迁移到运行时 JSON 配置文件，并整理测试标签体系与重复测试。

**Architecture:** 新建 `hw_test_config.json` 提供可覆盖的硬件参数；`hw_config.h` 用 `HwParams` struct + `load_hw_test_config()` 替换全部 `constexpr`，复用现有 `ConfigService`；各 hw 测试文件改用 `HwParams` 成员。`test_base.cc` 中重复的硬件 TEST_CASE 全部删除，仅保留无锁队列测试并提取为 `lockfree_queue_test.cc`。

**Tech Stack:** C++17, Catch2 v2.13.8, nlohmann/json (via `ConfigService`), spdlog, cmake

---

## 文件映射

| 文件 | 操作 |
|------|------|
| `test/integration/hardware/hw_test_config.json` | **新建** — 运行时配置 |
| `test/integration/hardware/hw_config.h` | **重构** — 删 constexpr，加 HwParams + load_hw_test_config() |
| `test/integration/hardware/walk_motor_group_hw_test.cc` | **修改** — 用 file-level HwParams 替代 hw::kXxx |
| `test/integration/hardware/limit_switch_hw_test.cc` | **修改** — 同上 |
| `test/integration/hardware/system_hw_test.cc` | **修改** — 用 f.p.xxx 替代 hw::kXxx |
| `test/integration/hardware/clean_cycle_hw_test.cc` | **修改** — 用 fx.p.xxx 替代 hw::kXxx |
| `test/integration/hardware/distance_sensor_hw_test.cc` | **修改** — 用 file-level HwParams + 修注释端口 |
| `test/integration/hardware/full_sweep_hw_test.cc` | **重构** — 删本地 constexpr，include hw_config.h，改标签 |
| `test/integration/hardware/bms_hw_test.cc` | **修改** — 用 HwParams 端口，改标签 [integration][bms]→[hw_bms] |
| `test/integration/hardware/imu_hw_test.cc` | **修改** — 用 HwParams 端口，改标签 [integration][imu]→[hw_imu] |
| `test/integration/hardware/driver_hw_test.cc` | **修改** — 修正文件头 @FilePath 注释 |
| `test/driver/lockfree_queue_test.cc` | **新建** — 从 test_base.cc 提取无锁队列测试 |
| `test/test_base.cc` | **删除** — 删除所有重复 TEST_CASE 后整个文件删除 |
| `test/CMakeLists.txt` | **修改** — 替换 test_base.cc → lockfree_queue_test.cc |

---

## Task 1: 创建 hw_test_config.json

**Files:**
- Create: `test/integration/hardware/hw_test_config.json`

- [ ] **Step 1.1: 创建配置文件**

```bash
cat > /home/tronlong/pv_cleaning_robot/test/integration/hardware/hw_test_config.json << 'EOF'
{
  "hardware": {
    "can_iface":           "can0",
    "motor_id_base":       1,
    "imu_port":            "/dev/ttyS1",
    "imu_baud":            9600,
    "bms_port":            "/dev/ttyS8",
    "bms_baud":            9600,
    "dist_port":           "/dev/ttyS9",
    "dist_baud":           9600,
    "dist_slave_id":       1,
    "dist_channel_count":  2,
    "gpio_chip":           "gpiochip5",
    "front_limit_line":    0,
    "rear_limit_line":     1
  },
  "timing": {
    "limit_timeout_sec":   60,
    "online_timeout_ms":   600,
    "comm_timeout_ms":     500,
    "sweep_duration_ms":   5000,
    "loop_period_ms":      50
  },
  "behavior": {
    "test_speed_rpm":      10.0,
    "test_return_rpm":     10.0,
    "sweep_rpm":           20.0,
    "limit_test_rpm":      10.0,
    "combined_passes":     50.0,
    "health_jsonl_path":   "/tmp/hw_system_test_health.jsonl"
  }
}
EOF
```

- [ ] **Step 1.2: 验证 JSON 有效性**

```bash
python3 -c "import json; json.load(open('test/integration/hardware/hw_test_config.json')); print('OK')"
```
Expected: `OK`

- [ ] **Step 1.3: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/hw_test_config.json
git commit -m "test: add hw_test_config.json for runtime hw parameter configuration

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 2: 重构 hw_config.h

**Files:**
- Modify: `test/integration/hardware/hw_config.h`

- [ ] **Step 2.1: 用完整重写替换 hw_config.h**

用以下内容**完整替换** `test/integration/hardware/hw_config.h`（保留所有现有 struct 成员和方法，只改 namespace hw 开头的常量区块 + DeviceFixture/FullSystemFixture 构造逻辑）：

在文件顶部 `#pragma once` 之后、现有 `#include` 列表之前，**插入**：

```cpp
#include "pv_cleaning_robot/service/config_service.h"
```

在 `namespace hw {` 之后，**删除**以下全部旧 constexpr（第 61-87 行）：

```cpp
// 【删除】以下所有 constexpr 常量
constexpr char kCanIface[] = "can0";
constexpr uint8_t kMotorIdBase = 1u;
constexpr uint16_t kCommTimeoutMs = 500u;
constexpr float kTestSpeedRpm = 10.0f;
constexpr float kTestReturnRpm = 10.0f;
constexpr char kImuPort[] = "/dev/ttyS1";
constexpr int kImuBaud = 9600;
constexpr char kBmsPort[] = "/dev/ttyS8";
constexpr int kBmsBaud = 9600;
constexpr char kDistSensorPort[] = "/dev/ttyS9";
constexpr int kDistSensorBaud = 9600;
constexpr int kDistSensorSlaveId = 1;
constexpr uint8_t kDistSensorChannelCount = 2;
constexpr char kGpioChip[] = "gpiochip5";
constexpr unsigned kFrontLine = 0u;
constexpr unsigned kRearLine = 1u;
constexpr int kLimitTimeoutSec = 60;
constexpr int kOnlineTimeoutMs = 600;
constexpr float kCombinedPasses = 50.0f;
constexpr char kHealthJsonlPath[] = "/tmp/hw_system_test_health.jsonl";
```

**替换为**以下 HwParams struct + load_hw_test_config()：

```cpp
/// 运行时可配置的硬件测试参数（从 hw_test_config.json 加载，缺失则用内嵌默认值）
struct HwParams {
    // hardware mapping
    std::string can_iface          = "can0";
    uint8_t     motor_id_base      = 1u;
    uint16_t    comm_timeout_ms    = 500u;
    std::string imu_port           = "/dev/ttyS1";
    int         imu_baud           = 9600;
    std::string bms_port           = "/dev/ttyS8";
    int         bms_baud           = 9600;
    std::string dist_port          = "/dev/ttyS9";
    int         dist_baud          = 9600;
    uint8_t     dist_slave_id      = 1u;
    uint8_t     dist_channel_count = 2u;
    std::string gpio_chip          = "gpiochip5";
    unsigned    front_limit_line   = 0u;
    unsigned    rear_limit_line    = 1u;
    // timing
    int         limit_timeout_sec  = 60;
    int         online_timeout_ms  = 600;
    int         sweep_duration_ms  = 5000;
    int         loop_period_ms     = 50;
    // behavior
    float       test_speed_rpm     = 10.0f;
    float       test_return_rpm    = 10.0f;
    float       sweep_rpm          = 20.0f;
    float       limit_test_rpm     = 10.0f;
    float       combined_passes    = 50.0f;
    std::string health_jsonl_path  = "/tmp/hw_system_test_health.jsonl";
};

/// 按优先级查找 hw_test_config.json：
///   1. 环境变量 HW_TEST_CONFIG
///   2. 当前工作目录 hw_test_config.json
///   缺失时打印 warning 并返回内嵌默认值
inline HwParams load_hw_test_config() {
    HwParams p;
    std::string path;
    const char* env = std::getenv("HW_TEST_CONFIG");
    if (env && std::filesystem::exists(env))
        path = env;
    else if (std::filesystem::exists("hw_test_config.json"))
        path = "hw_test_config.json";

    if (path.empty()) {
        spdlog::warn("[hw_config] hw_test_config.json not found — using built-in defaults");
        return p;
    }
    robot::service::ConfigService cfg(path);
    if (!cfg.load()) {
        spdlog::warn("[hw_config] Failed to load {} — using built-in defaults", path);
        return p;
    }
    p.can_iface         = cfg.get<std::string>("hardware.can_iface",        p.can_iface);
    p.motor_id_base     = static_cast<uint8_t>(cfg.get<int>("hardware.motor_id_base",    (int)p.motor_id_base));
    p.comm_timeout_ms   = static_cast<uint16_t>(cfg.get<int>("timing.comm_timeout_ms",   (int)p.comm_timeout_ms));
    p.imu_port          = cfg.get<std::string>("hardware.imu_port",         p.imu_port);
    p.imu_baud          = cfg.get<int>        ("hardware.imu_baud",         p.imu_baud);
    p.bms_port          = cfg.get<std::string>("hardware.bms_port",         p.bms_port);
    p.bms_baud          = cfg.get<int>        ("hardware.bms_baud",         p.bms_baud);
    p.dist_port         = cfg.get<std::string>("hardware.dist_port",        p.dist_port);
    p.dist_baud         = cfg.get<int>        ("hardware.dist_baud",        p.dist_baud);
    p.dist_slave_id     = static_cast<uint8_t>(cfg.get<int>("hardware.dist_slave_id",    (int)p.dist_slave_id));
    p.dist_channel_count= static_cast<uint8_t>(cfg.get<int>("hardware.dist_channel_count",(int)p.dist_channel_count));
    p.gpio_chip         = cfg.get<std::string>("hardware.gpio_chip",        p.gpio_chip);
    p.front_limit_line  = static_cast<unsigned>(cfg.get<int>("hardware.front_limit_line",(int)p.front_limit_line));
    p.rear_limit_line   = static_cast<unsigned>(cfg.get<int>("hardware.rear_limit_line", (int)p.rear_limit_line));
    p.limit_timeout_sec = cfg.get<int>        ("timing.limit_timeout_sec",  p.limit_timeout_sec);
    p.online_timeout_ms = cfg.get<int>        ("timing.online_timeout_ms",  p.online_timeout_ms);
    p.sweep_duration_ms = cfg.get<int>        ("timing.sweep_duration_ms",  p.sweep_duration_ms);
    p.loop_period_ms    = cfg.get<int>        ("timing.loop_period_ms",     p.loop_period_ms);
    p.test_speed_rpm    = cfg.get<float>      ("behavior.test_speed_rpm",   p.test_speed_rpm);
    p.test_return_rpm   = cfg.get<float>      ("behavior.test_return_rpm",  p.test_return_rpm);
    p.sweep_rpm         = cfg.get<float>      ("behavior.sweep_rpm",        p.sweep_rpm);
    p.limit_test_rpm    = cfg.get<float>      ("behavior.limit_test_rpm",   p.limit_test_rpm);
    p.combined_passes   = cfg.get<float>      ("behavior.combined_passes",  p.combined_passes);
    p.health_jsonl_path = cfg.get<std::string>("behavior.health_jsonl_path",p.health_jsonl_path);
    spdlog::info("[hw_config] Loaded config: {}", path);
    return p;
}
```

- [ ] **Step 2.2: 更新 DeviceFixture 构造函数**

在 `struct DeviceFixture {` 成员列表中，将现有 `DeviceFixture()` 构造函数替换为：

首先在成员列表最前面**增加** `p` 成员：
```cpp
struct DeviceFixture {
    HwParams p;                                            // ← 新增，必须是第一个成员
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    // ... 其余成员不变
```

然后将构造函数体**整体替换**为：
```cpp
    DeviceFixture() : p(load_hw_test_config()) {
        using namespace robot;
        can_bus    = std::make_shared<driver::LinuxCanSocket>(p.can_iface);
        walk_group = std::make_shared<device::WalkMotorGroup>(can_bus, p.motor_id_base, p.comm_timeout_ms);
        imu_serial = std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu        = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial = std::make_shared<driver::LibSerialPort>(p.bms_port, hal::UartConfig{p.bms_baud});
        bms        = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        front_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.front_limit_line);
        rear_gpio  = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.rear_limit_line);
        front_sw   = std::make_shared<device::LimitSwitch>(front_gpio, device::LimitSide::FRONT);
        rear_sw    = std::make_shared<device::LimitSwitch>(rear_gpio, device::LimitSide::REAR);
    }
```

- [ ] **Step 2.3: 更新 FullSystemFixture 构造函数中的 motion_cfg**

在 `FullSystemFixture(bool pid_enabled)` 构造函数中，将 `motion_cfg` 相关行替换为（使用 `p.` 而非 `k`）：
```cpp
        service::MotionService::Config motion_cfg;
        motion_cfg.clean_speed_rpm   = p.test_speed_rpm;    // 原 kTestSpeedRpm
        motion_cfg.return_speed_rpm  = p.test_return_rpm;   // 原 kTestReturnRpm
        motion_cfg.brush_rpm         = 0;
        motion_cfg.return_brush_rpm  = 0;
        motion_cfg.edge_reverse_rpm  = 0.0f;
        motion_cfg.heading_pid_en    = pid_enabled;
```

- [ ] **Step 2.4: 更新 FullSystemFixture::init() 中的距离传感器部分**

在 `init()` 方法中，将距离传感器 Modbus 初始化的三行替换为（用 `p.` 替代旧常量）：
```cpp
            dist_modbus = std::make_shared<robot::driver::LibModbusMaster>(
                p.dist_port, robot::hal::ModbusConfig{p.dist_baud, 'N', 8, 1});
            if (dist_modbus->open()) {
                robot::device::DistanceSensorConfig dist_cfg;
                dist_cfg.slave_id      = p.dist_slave_id;
                dist_cfg.channel_count = p.dist_channel_count;
```

- [ ] **Step 2.5: 更新 hw_config.h 顶部注释**

将文件头 `@brief 硬件测试公共 Fixture` 下方的接线注释更新为：
```
 * 硬件接线（与 config/config.json 对齐，可通过 hw_test_config.json 覆盖）：
 *   CAN      : can0（默认），行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1（默认），WIT Motion，9600 baud
 *   BMS      : /dev/ttyS8（默认），嘉佰达通用协议 V4，9600 baud
 *   距离传感器: /dev/ttyS9（默认），RS485 Modbus RTU，9600 baud
 *   GPIO     : gpiochip5 line0=前限位，line1=后限位（默认）
```

- [ ] **Step 2.6: 构建验证**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target hw_tests 2>&1 | tail -30
```
Expected: `[100%] Built target hw_tests`（或仅出现 `hw::kXxx` 未定义的错误，说明 constexpr 已删除但引用处尚未更新 — 这是预期的，Task 3 会修）

- [ ] **Step 2.7: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/hw_config.h
git commit -m "refactor(test): replace hw_config.h constexpr with runtime HwParams struct

- Add HwParams struct with default values matching previous constexpr
- Add load_hw_test_config() loading from hw_test_config.json via ConfigService
- Update DeviceFixture ctor to use p.xxx members
- Update FullSystemFixture ctor and init() for dist sensor port params
- Add config_service.h include

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 3: 更新引用 hw::kXxx 的测试文件

**Files:**
- Modify: `test/integration/hardware/walk_motor_group_hw_test.cc`
- Modify: `test/integration/hardware/limit_switch_hw_test.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Modify: `test/integration/hardware/clean_cycle_hw_test.cc`
- Modify: `test/integration/hardware/distance_sensor_hw_test.cc`

### 3a. walk_motor_group_hw_test.cc

- [ ] **Step 3a.1: 在文件 #include 区块末尾添加 file-level HwParams（在 `using namespace robot;` 之前）**

在文件中找到 `using namespace robot;` 这行（通常在 includes 之后），在其**上方**插入：
```cpp
static const hw::HwParams kp = hw::load_hw_test_config();
```

- [ ] **Step 3a.2: 全局替换 hw::kXxx 引用**

在 `walk_motor_group_hw_test.cc` 中执行以下替换（用编辑器或 sed）：
```
hw::kCanIface        → kp.can_iface
hw::kMotorIdBase     → kp.motor_id_base
hw::kCommTimeoutMs   → kp.comm_timeout_ms
hw::kTestSpeedRpm    → kp.test_speed_rpm
hw::kOnlineTimeoutMs → kp.online_timeout_ms
```

验证：`grep -n "hw::k" walk_motor_group_hw_test.cc` 应无输出。

### 3b. limit_switch_hw_test.cc

- [ ] **Step 3b.1: 在文件 include 区末、第一个 TEST_CASE 前插入**

```cpp
static const hw::HwParams kp = hw::load_hw_test_config();
```

- [ ] **Step 3b.2: 替换全部 hw::kXxx 引用**

```
hw::kGpioChip    → kp.gpio_chip
hw::kFrontLine   → kp.front_limit_line
hw::kRearLine    → kp.rear_limit_line
```

验证：`grep -n "hw::k" limit_switch_hw_test.cc` 应无输出。

### 3c. system_hw_test.cc

system_hw_test.cc 访问 `FullSystemFixture f`，可直接用 `f.p.xxx`。

- [ ] **Step 3c.1: 替换全部 hw::kXxx 引用**

```
hw::kHealthJsonlPath  → f.p.health_jsonl_path
hw::kTestSpeedRpm     → f.p.test_speed_rpm
hw::kLimitTimeoutSec  → f.p.limit_timeout_sec
hw::kCombinedPasses   → f.p.combined_passes
```

注意：`f.p` 是 `FullSystemFixture` 继承自 `DeviceFixture` 的 public 成员 `HwParams p`。

验证：`grep -n "hw::k" system_hw_test.cc` 应无输出。

### 3d. clean_cycle_hw_test.cc

- [ ] **Step 3d.1: 替换 hw::kLimitTimeoutSec**

该文件使用 `hw::FullSystemFixture fx`，替换：
```
hw::kLimitTimeoutSec → fx.p.limit_timeout_sec
```

验证：`grep -n "hw::k" clean_cycle_hw_test.cc` 应无输出。

### 3e. distance_sensor_hw_test.cc

- [ ] **Step 3e.1: 在文件 include 区末插入 file-level HwParams**

```cpp
static const hw::HwParams kp = hw::load_hw_test_config();
```

- [ ] **Step 3e.2: 替换 hw::kDistSensorXxx 引用**

```
hw::kDistSensorBaud         → kp.dist_baud
hw::kDistSensorPort         → kp.dist_port
hw::kDistSensorSlaveId      → kp.dist_slave_id
hw::kDistSensorChannelCount → kp.dist_channel_count
```

验证：`grep -n "hw::k" distance_sensor_hw_test.cc` 应无输出。

- [ ] **Step 3f: 构建全部 hw_tests 验证**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target hw_tests 2>&1 | tail -20
```
Expected: `Built target hw_tests` 零错误零警告。

- [ ] **Step 3g: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/walk_motor_group_hw_test.cc \
        test/integration/hardware/limit_switch_hw_test.cc \
        test/integration/hardware/system_hw_test.cc \
        test/integration/hardware/clean_cycle_hw_test.cc \
        test/integration/hardware/distance_sensor_hw_test.cc
git commit -m "refactor(test): update hw test files to use HwParams instead of constexpr

Replace hw::kXxx references with runtime-loaded HwParams members.
Affected: walk_motor_group, limit_switch, system, clean_cycle, distance_sensor hw tests.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 4: 重构 full_sweep_hw_test.cc

**Files:**
- Modify: `test/integration/hardware/full_sweep_hw_test.cc`

该文件当前**不** include hw_config.h，有自己的独立 constexpr 常量块（约第 49–74 行）和 `wait_for`/`safe_stop` 辅助函数。

- [ ] **Step 4.1: 在 `#include <thread>` 之后插入 hw_config.h include**

在文件第一段 `#include` 列表末尾（紧接 `#include <thread>`）插入：
```cpp
#include "hw_config.h"
```

- [ ] **Step 4.2: 删除本地 constexpr 常量块（第 45–75 行区域）**

删除整个区块：
```cpp
// ═══════════════════════════════════════════════════════════════════════════
//  硬件配置常量（根据目标机接线修改）
// ═══════════════════════════════════════════════════════════════════════════

static constexpr char kCanIface[] = "can0";
static constexpr uint8_t kMotorIdBase = 1u;
static constexpr uint16_t kCommTimeoutMs = 500u;
static constexpr char kImuPort[] = "/dev/ttyS1";
static constexpr int kImuBaudrate = 9600;
static constexpr char kGpioChip[] = "gpiochip5";
static constexpr unsigned kFrontLine = 0u;
static constexpr unsigned kRearLine = 1u;
static constexpr float kSweepRpm = 20.0f;
static constexpr float kLimitTestRpm = 10.0f;
static constexpr int kLoopPeriodMs = 50;
static constexpr int kSweepDurationMs = 5000;
static constexpr int kSweepLoops = kSweepDurationMs / kLoopPeriodMs;
static constexpr int kLimitWaitMs = 10000;
static constexpr int kOnlineCheckAt = 19;
```

- [ ] **Step 4.3: 在删除区块的位置插入 file-level HwParams**

```cpp
static const hw::HwParams kp = hw::load_hw_test_config();
/// kSweepLoops 由 sweep_duration_ms / loop_period_ms 计算得出
static int sweep_loops() { return kp.sweep_duration_ms / kp.loop_period_ms; }
```

- [ ] **Step 4.4: 更新辅助函数 safe_stop 中的 kLoopPeriodMs**

将 `safe_stop` 函数体中的 `kLoopPeriodMs` 替换为 `kp.loop_period_ms`：
```cpp
static void safe_stop(device::WalkMotorGroup& group) {
    group.set_speeds(0.0f, 0.0f, 0.0f, 0.0f);
    group.enable_heading_control(false);
    for (int i = 0; i < 5; ++i) {
        group.update(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));  // ← 改
    }
    group.disable_all();
}
```

- [ ] **Step 4.5: 替换四个 TEST_CASE 内所有 kXxx 引用**

全局替换（注意这些是 static 局部常量，没有 `hw::` 前缀）：
```
kCanIface      → kp.can_iface
kMotorIdBase   → kp.motor_id_base
kCommTimeoutMs → kp.comm_timeout_ms
kImuPort       → kp.imu_port
kImuBaudrate   → kp.imu_baud
kGpioChip      → kp.gpio_chip
kFrontLine     → kp.front_limit_line
kRearLine      → kp.rear_limit_line
kSweepRpm      → kp.sweep_rpm
kLimitTestRpm  → kp.limit_test_rpm
kLoopPeriodMs  → kp.loop_period_ms
kSweepDurationMs → kp.sweep_duration_ms
kSweepLoops    → sweep_loops()
kLimitWaitMs   → (kp.limit_timeout_sec * 1000)
kOnlineCheckAt → 19
```

注意：`kOnlineCheckAt` 是第 19 次 update 后检查，这是循环计数不是硬件参数，保持硬编码 `19`。

- [ ] **Step 4.6: 更新文件头注释中的运行方法**

将文件顶部注释的运行方法行更新（只更改 `[hw_limit]` 系列）：
```
 *   ./hw_tests "[hw_sweep][limit_stop]"       # FSM 全链路限位急停（手动触发）
```
删除旧的 `[hw_limit][manual_front]` 和 `[hw_limit][manual_rear]` 行。

- [ ] **Step 4.7: 更改 [hw_limit] 测试标签**

找到并修改两个限位测试用例：

```cpp
// 原：
TEST_CASE("[hw_limit][manual_front] 手动触发前限位急停链路", "[hw_limit][manual_front]") {
// 改为：
TEST_CASE("[hw_sweep][limit_stop] 手动触发前限位急停链路（FSM 全链路）", "[hw_sweep][limit_stop]") {
```

```cpp
// 原：
TEST_CASE("[hw_limit][manual_rear] 手动触发后限位急停链路", "[hw_limit][manual_rear]") {
// 改为：
TEST_CASE("[hw_sweep][limit_stop] 手动触发后限位急停链路（FSM 全链路）", "[hw_sweep][limit_stop][rear]") {
```

- [ ] **Step 4.8: 构建验证**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target hw_tests 2>&1 | tail -20
```
Expected: 零错误。

- [ ] **Step 4.9: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/full_sweep_hw_test.cc
git commit -m "refactor(test): full_sweep_hw_test.cc use hw_config.h HwParams, fix limit tags

- Remove local duplicate constexpr block, include hw_config.h
- Add file-level kp = load_hw_test_config()
- Replace [hw_limit][manual_front/rear] tags with [hw_sweep][limit_stop]

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 5: 修复 bms_hw_test.cc 和 imu_hw_test.cc

**Files:**
- Modify: `test/integration/hardware/bms_hw_test.cc`
- Modify: `test/integration/hardware/imu_hw_test.cc`

### 5a. bms_hw_test.cc

- [ ] **Step 5a.1: 在现有 include 区末插入 hw_config.h 和 file-level HwParams**

在最后一个 `#include` 行之后插入：
```cpp
#include "hw_config.h"

static const hw::HwParams kp = hw::load_hw_test_config();
```

- [ ] **Step 5a.2: 替换 TEST_CASE 内硬编码端口**

将 `[integration][bms]` TEST_CASE 内的：
```cpp
auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS8", cfg);
```
替换为：
```cpp
auto serial = std::make_shared<driver::LibSerialPort>(kp.bms_port, cfg);
```

同时将 UartConfig 的 `baudrate = 9600` 改为：
```cpp
cfg.baudrate = kp.bms_baud;
```

- [ ] **Step 5a.3: 更改测试标签**

```cpp
// 原：
TEST_CASE("BMS 集成测试 - ttyS8 串口连接与数据读取", "[integration][bms]") {
// 改为：
TEST_CASE("BMS 集成测试 - ttyS8 串口连接与数据读取", "[hw_bms]") {
```

- [ ] **Step 5a.4: 更新文件头注释中的运行命令**

将顶部注释中的运行示例由：
```
 *   ./unit_tests "[integration][bms]" # 只跑硬件集成测试（需接 ttyS8 BMS）
```
改为：
```
 *   ./hw_tests "[hw_bms]"             # 只跑硬件集成测试（需接 ttyS8 BMS，默认）
```

### 5b. imu_hw_test.cc

- [ ] **Step 5b.1: 在现有 include 区末插入 hw_config.h 和 file-level HwParams**

```cpp
#include "hw_config.h"

static const hw::HwParams kp = hw::load_hw_test_config();
```

- [ ] **Step 5b.2: 替换硬编码串口路径**

在两个 TEST_CASE 内将：
```cpp
auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS1", cfg);
```
替换为：
```cpp
auto serial = std::make_shared<driver::LibSerialPort>(kp.imu_port, cfg);
```

将 `cfg.baudrate = 9600;` 替换为：
```cpp
cfg.baudrate = kp.imu_baud;
```

- [ ] **Step 5b.3: 更改测试标签**

```cpp
// 原：
TEST_CASE("集成测试 - /dev/ttyS1 读取真实 IMU 欧拉角", "[integration][imu]") {
// 改为：
TEST_CASE("集成测试 - IMU 读取真实欧拉角", "[hw_imu]") {
```

```cpp
// 原：
TEST_CASE("集成测试 - /dev/ttyS1 连续读取 60 秒并打印数据", "[integration][imu][long]") {
// 改为：
TEST_CASE("集成测试 - IMU 连续读取 60 秒并打印数据", "[hw_imu][long]") {
```

- [ ] **Step 5b.4: 更新文件头注释中的运行命令**

```
 *   ./hw_tests "[hw_imu]"      # 读取真实 IMU 欧拉角
 *   ./hw_tests "[hw_imu][long]" # 连续读取 60 秒
```

- [ ] **Step 5c: 构建验证**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target hw_tests 2>&1 | tail -10
```
Expected: 零错误。

- [ ] **Step 5d: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/bms_hw_test.cc \
        test/integration/hardware/imu_hw_test.cc
git commit -m "refactor(test): bms/imu hw tests use HwParams ports and fix tags

- [integration][bms] → [hw_bms], port from kp.bms_port
- [integration][imu] → [hw_imu], port from kp.imu_port
- Update file header run commands

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 6: 提取 lockfree_queue_test.cc + 删除 test_base.cc

**Files:**
- Create: `test/driver/lockfree_queue_test.cc`
- Delete: `test/test_base.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 6.1: 创建 lockfree_queue_test.cc**

创建文件 `test/driver/lockfree_queue_test.cc`，内容如下（从 test_base.cc 提取 `[driver][nonlock]` TEST_CASE）：

```cpp
/**
 * @file lockfree_queue_test.cc
 * @brief boost::lockfree::spsc_queue 无锁单生产者单消费者队列单元测试
 *
 * 测试分组：[driver][nonlock]
 *
 * 运行方法：
 *   ./unit_tests "[driver][nonlock]"
 */
#include <boost/lockfree/spsc_queue.hpp>
#include <cassert>
#include <catch2/catch.hpp>

TEST_CASE("无锁队列", "[driver][nonlock]") {
    boost::lockfree::spsc_queue<int, boost::lockfree::capacity<1024>> q;

    assert(q.empty());

    for (int i = 0; i < 100; ++i) {
        assert(q.push(i));
    }

    int v;
    for (int i = 0; i < 100; ++i) {
        assert(q.pop(v));
        assert(v == i);
    }

    assert(q.empty());
}
```

- [ ] **Step 6.2: 删除 test_base.cc**

```bash
rm /home/tronlong/pv_cleaning_robot/test/test_base.cc
```

- [ ] **Step 6.3: 更新 test/CMakeLists.txt 中的 unit_tests 源列表**

在 `add_executable(unit_tests` 的源列表中：

将：
```cmake
  test_base.cc
```
替换为：
```cmake
  driver/lockfree_queue_test.cc
```

同时在 `# 驱动层` 注释行下方的 driver 测试列表中也加入该文件（确保分组一致）：
```cmake
  # 驱动层（纯软件路径，无需真实硬件即可运行）
  driver/pi_mutex_test.cc
  driver/libgpiod_pin_test.cc
  driver/linux_can_socket_test.cc
  driver/libserialport_test.cc
  driver/libmodbus_test.cc
  driver/lockfree_queue_test.cc    ← 新增（原 test_base.cc 的 nonlock 测试）
```

如果 `test_base.cc` 在 CMakeLists 中只出现一次，只需替换那一处。

- [ ] **Step 6.4: 构建 unit_tests 验证**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target unit_tests 2>&1 | tail -20
```
Expected: `Built target unit_tests` 零错误。

- [ ] **Step 6.5: 在主机上运行 unit_tests 验证无锁队列测试通过**

```bash
cd /home/tronlong/pv_cleaning_robot/build
./unit_tests "[driver][nonlock]" -v 2>&1
```
Expected:
```
===============================================================================
All tests passed (1 assertion in 1 test case)
```

- [ ] **Step 6.6: 确认 test_base.cc 的硬件 TEST_CASEs 已在 driver_hw_test.cc 中覆盖**

验证 driver_hw_test.cc 包含同名 driver 测试（这些测试在 hw_tests 中仍然存在）：
```bash
grep -c "TEST_CASE" test/integration/hardware/driver_hw_test.cc
```
Expected: 输出 ≥ 4（can/gpio/modbus/serial 各一个）

- [ ] **Step 6.7: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/driver/lockfree_queue_test.cc test/CMakeLists.txt
git rm test/test_base.cc
git commit -m "refactor(test): extract lockfree_queue_test.cc, remove test_base.cc

- Create test/driver/lockfree_queue_test.cc with [driver][nonlock] test
- Remove test_base.cc (driver tests duplicated by driver_hw_test.cc,
  bms/walk_motor tests superseded by hw_tests files)
- Update test/CMakeLists.txt: test_base.cc → lockfree_queue_test.cc

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 7: 修复 driver_hw_test.cc 陈旧注释

**Files:**
- Modify: `test/integration/hardware/driver_hw_test.cc`

- [ ] **Step 7.1: 修正文件头 @FilePath 注释**

找到文件头的：
```cpp
 * @FilePath: /pv_cleaning_robot/test/test_base.cc
```
替换为：
```cpp
 * @FilePath: /pv_cleaning_robot/test/integration/hardware/driver_hw_test.cc
```

- [ ] **Step 7.2: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/driver_hw_test.cc
git commit -m "fix(test): correct stale @FilePath comment in driver_hw_test.cc

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 8: 全量构建验证

- [ ] **Step 8.1: 全量构建两个 binary**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . 2>&1 | tail -30
```
Expected: `Built target unit_tests` 和 `Built target hw_tests`，零错误。

- [ ] **Step 8.2: 在主机上运行 unit_tests（无硬件）**

```bash
cd /home/tronlong/pv_cleaning_robot/build
./unit_tests 2>&1 | tail -10
```
Expected:
```
===============================================================================
All tests passed (N assertions in M test cases)
```
（unit_tests 不依赖真实硬件，应全部通过）

- [ ] **Step 8.3: 验证 [driver][nonlock] 在 unit_tests 可用**

```bash
./unit_tests "[driver][nonlock]" -v 2>&1 | tail -5
```
Expected: `All tests passed`

- [ ] **Step 8.4: 验证 hw_tests 标签清单正确**

```bash
cd /home/tronlong/pv_cleaning_robot/build
./hw_tests --list-tests 2>&1 | grep -E "\[hw_bms\]|\[hw_imu\]|\[hw_sweep\]|\[integration\]" | head -30
```
Expected:
- 出现 `[hw_bms]` 和 `[hw_imu]`
- 不出现任何 `[integration][bms]` 或 `[integration][imu]`

- [ ] **Step 8.5: 目标机验证（可选，需在 RK3576 上执行）**

```bash
# 将 hw_tests 和 hw_test_config.json 拷贝到目标机 ~/pv_cleaning_robot/bin/
# 在目标机上：
cd ~/pv_cleaning_robot/bin
./hw_tests "[hw_bms]" -v   # 验证 BMS 测试标签正确
# 修改 hw_test_config.json 中 test_speed_rpm 为 5.0 后不重编译再运行
# 验证 [hw_walk][fwd_no_pid] 以 5 RPM 运行
```

- [ ] **Step 8.6: 最终提交（如有未 commit 的更改）**

```bash
cd /home/tronlong/pv_cleaning_robot
git status
# 确认无未追踪的修改文件
```

---

## 验证 Checklist

| 验证项 | 方法 | 预期 |
|--------|------|------|
| unit_tests 全通过 | `./unit_tests` | All tests passed |
| [driver][nonlock] 在 unit_tests | `./unit_tests "[driver][nonlock]"` | 1 test passed |
| hw_tests 无 [integration][bms] | `./hw_tests --list-tests` | 无此标签 |
| hw_tests 有 [hw_bms] | `./hw_tests --list-tests` | 出现此标签 |
| hw_tests 无 [integration][imu] | `./hw_tests --list-tests` | 无此标签 |
| 无配置文件时有 warning | `HW_TEST_CONFIG=/nonexist ./hw_tests "[hw_bms]"` | 打印 warning |
| 修改 JSON 后无需重编译 | 修改 test_speed_rpm → 5.0，重运行 | 新速度生效 |
