# HW Test Config Externalization & Test Audit Design

**Date:** 2026-04-16  
**Scope:** `test/integration/hardware/`, `test/test_base.cc`, `CMakeLists.txt`  
**Goal:** 将 hw_tests 所有硬件参数从编译期 constexpr 移到运行时 JSON 配置文件；整理测试文件的标签体系、二进制归属和重复测试。

---

## 问题背景

### 1. 硬件参数硬编码
`hw_config.h` 内有 15+ 个 `constexpr` 常量（串口设备路径、CAN 接口名、GPIO 行号、超时时长、测速 RPM 等）。每次在不同目标机或不同接线方案下测试，需重新编译。`full_sweep_hw_test.cc` 更是另起一套独立常量，与 `hw_config.h` 完全重复。

### 2. 测试标签体系混乱
- `bms_hw_test.cc`、`imu_hw_test.cc` 使用 `[integration][bms]`、`[integration][imu]` 标签，与 `unit_tests` 中的 `[integration][*]` 标签命名空间冲突。
- `bms_hw_test.cc`、`imu_hw_test.cc` 还包含 `[protocol][bms]`、`[device][bms]` 等纯内存测试，与 `unit_tests` 中对应文件重复。

### 3. 二进制归属错误
- `test_base.cc`（含真实 CAN/GPIO 硬件访问）编译进 `unit_tests`；`unit_tests` 应为纯软件测试。

### 4. 陈旧注释
- `driver_hw_test.cc` 文件头 `@FilePath` 指向 `test_base.cc`（历史复制痕迹）。

---

## 设计方案

### §1 配置文件结构

新建 `test/integration/hardware/hw_test_config.json`（随源码提交，作为默认参考配置）：

```json
{
  "hardware": {
    "can_iface":           "can0",
    "motor_id_base":       1,
    "imu_port":            "/dev/ttyS3",
    "imu_baud":            115200,
    "bms_port":            "/dev/ttyS8",
    "bms_baud":            9600,
    "dist_port":           "/dev/ttyS9",
    "dist_baud":           9600,
    "dist_slave_id":       1,
    "dist_channel_count":  4,
    "gpio_chip":           "/dev/gpiochip0",
    "front_limit_line":    5,
    "rear_limit_line":     6
  },
  "timing": {
    "limit_timeout_sec":   60,
    "online_timeout_ms":   600,
    "comm_timeout_ms":     100,
    "sweep_duration_ms":   5000,
    "loop_period_ms":      50
  },
  "behavior": {
    "test_speed_rpm":      10.0,
    "test_return_rpm":     10.0,
    "sweep_rpm":           20.0,
    "limit_test_rpm":      10.0,
    "combined_passes":     50.0,
    "health_jsonl_path":   "/tmp/health_hw.jsonl"
  }
}
```

**查找顺序（优先级从高到低）：**
1. 环境变量 `HW_TEST_CONFIG` 指定的路径
2. 当前工作目录 `hw_test_config.json`
3. 可执行文件同级目录 `hw_test_config.json`
4. 找不到时使用内嵌默认值（打印 `[warning]` 提示）

### §2 hw_config.h 重构

#### 2.1 删除所有 constexpr 参数

现有代码：
```cpp
constexpr const char* kCanIface        = "can0";
constexpr int         kMotorIdBase     = 1;
constexpr int         kLimitTimeoutSec = 60;
// ... 15+ 个
```

改为 `HwParams` 结构体成员变量（运行时赋值）：
```cpp
struct HwParams {
    std::string can_iface          = "can0";
    int         motor_id_base      = 1;
    std::string imu_port           = "/dev/ttyS3";
    int         imu_baud           = 115200;
    // ... 所有参数带内嵌默认值
};
```

#### 2.2 ConfigService 加载

在 `DeviceFixture::init()` 开头插入加载逻辑，**复用现有** `robot::service::ConfigService`：

```cpp
#include "pv_cleaning_robot/service/config_service.h"

// init() 内：
robot::service::ConfigService cfg = load_hw_test_config();
HwParams p;
p.can_iface          = cfg.get<std::string>("hardware.can_iface",        p.can_iface);
p.motor_id_base      = cfg.get<int>        ("hardware.motor_id_base",    p.motor_id_base);
p.limit_timeout_sec  = cfg.get<int>        ("timing.limit_timeout_sec",  p.limit_timeout_sec);
p.test_speed_rpm     = cfg.get<float>      ("behavior.test_speed_rpm",   p.test_speed_rpm);
// ...
```

`load_hw_test_config()` 为文件头内联 helper，按优先级查找配置文件路径。

#### 2.3 full_sweep_hw_test.cc

- 删除文件内的独立 constexpr 常量块（约 15 行）
- 改为 `#include "hw_config.h"` + 使用 `HwParams`
- `[hw_limit]` 测试用例不删除，改为 `[hw_sweep][limit_stop]` 以与 `limit_switch_hw_test.cc` 区分（后者测设备层，前者测 FSM 驱动的全链路）

### §3 测试标签整理

> **说明**：`bms_hw_test.cc` 和 `imu_hw_test.cc` 的文件头注释中提到了 `[protocol]`/`[device]` 标签，但这些仅是说明如何在 `unit_tests` 中运行对应的 *独立* 测试文件，本文件内并不存在这些 TEST_CASE。

| 文件 | 原标签 | 新标签 | 原因 |
|------|--------|--------|------|
| `bms_hw_test.cc` | `[integration][bms]` | `[hw_bms]` | 避免与 unit_tests `[integration]` 混淆 |
| `imu_hw_test.cc` | `[integration][imu]`、`[integration][imu][long]` | `[hw_imu]`、`[hw_imu][long]` | 同上 |
| `full_sweep_hw_test.cc` | `[hw_limit][manual_front/rear]` | `[hw_sweep][limit_stop]` | 区分纯设备层测试与 FSM 全链路 |
| `driver_hw_test.cc` | 文件头 `@FilePath: test_base.cc` | 修正为正确路径 | 陈旧注释 |
| `test_base.cc` | `[driver][can/gpio/modbus/serial]` | **删除该 TEST_CASE** | 与 `driver_hw_test.cc` 中同名测试完全重复（同 binary 后 name collision） |
| `test_base.cc` | `[integration][bms]`（BMS循环测试-ttyS8） | **删除该 TEST_CASE** | 与 `bms_hw_test.cc` 中的 BMS 集成测试重复 |
| `test_base.cc` | `[integration][walk_motor_test]` | **删除该 TEST_CASE** | 功能已被 `walk_motor_group_hw_test.cc` 覆盖 |
| `test_base.cc` | `[driver][nonlock]`（无锁队列纯软件测试） | 提取为独立文件 `test/driver/lockfree_queue_test.cc` | 唯一有价值的保留内容，应与 driver 单元测试放在一起 |

> `test_base.cc` 删完后整个文件废弃，重命名/提取后删除源文件。

### §4 二进制归属整理

`test_base.cc` 删除重复测试后，剩余唯一的 `[driver][nonlock]` 无锁队列测试提取为 `test/driver/lockfree_queue_test.cc`，**继续留在 unit_tests**（不移入 hw_tests）。原 `test_base.cc` 从 `unit_tests` 源列表移除并删除。

### §5 测试分层约定（整改后）

```
unit_tests 二进制（无硬件依赖，CI 可运行）
├── test/protocol/         → [protocol][*]     纯协议编解码（纯内存）
├── test/device/           → [device][*]       mock 设备层
└── test/integration/      → [integration][*]  mock 系统集成

hw_tests 二进制（需目标机 RK3576）
├── test_base.cc           → [driver][can/gpio]  驱动基础
├── driver_hw_test.cc      → [hw_driver]          CAN/GPIO/Serial
├── imu_hw_test.cc         → [hw_imu]             IMU 集成
├── bms_hw_test.cc         → [hw_bms]             BMS 集成
├── dist_sensor_hw_test.cc → [hw_dist]            距离传感器
├── walk_motor_*_hw_test.cc→ [hw_walk]            行走电机
├── full_sweep_hw_test.cc  → [hw_sweep]           完整扫描 FSM
└── system_hw_test.cc      → [hw_system]          系统级集成
```

---

## 实现范围（Out of Scope）

- 不修改 `config/config.json`（生产配置）
- 不修改 `unit_tests` 内的任何测试逻辑，只改二进制归属
- 不新增测试用例（测试内容审计独立任务）
- `imu_hw_test.cc` 中 IMU 物理常量（重力分量阈值等，来自传感器规格书）保持 `constexpr`，不放入配置文件

---

## 验证标准

1. `cmake --build . && ./unit_tests` 全通过（无硬件）
2. 目标机上 `./hw_tests` 无配置文件时使用内嵌默认值正常运行
3. 修改 `hw_test_config.json` 中 `test_speed_rpm` 为 `5.0` 后无需重新编译，`[hw_walk]` 测试以 5 RPM 运行
4. `./unit_tests "[protocol][bms]"` 只来自 `bms_protocol_test.cc`，不出现重复测试名
5. `./hw_tests "[hw_bms]"` 只运行真实硬件 BMS 测试

---

## 文件变更清单

| 文件 | 变更类型 |
|------|----------|
| `test/integration/hardware/hw_test_config.json` | **新建** |
| `test/integration/hardware/hw_config.h` | 重构（删 constexpr，加 HwParams + ConfigService 加载） |
| `test/integration/hardware/full_sweep_hw_test.cc` | 删除重复常量，include hw_config.h，改标签 |
| `test/integration/hardware/bms_hw_test.cc` | 改标签 `[integration][bms]` → `[hw_bms]` |
| `test/integration/hardware/imu_hw_test.cc` | 改标签 `[integration][imu]` → `[hw_imu]`、`[hw_imu][long]` |
| `test/integration/hardware/driver_hw_test.cc` | 修正文件头注释 |
| `test/test_base.cc` | 删除全部重复 TEST_CASEs（driver can/gpio/modbus/serial + BMS循环 + walk_motor），提取 `[driver][nonlock]` 为独立文件后删除 |
| `test/driver/lockfree_queue_test.cc` | **新建**（从 test_base.cc 提取无锁队列测试，留在 unit_tests） |
| `config/config.json` | `distance_sensor.port` `/dev/ttyS4` → `/dev/ttyS9` ✅ 已修复 |
| `test/integration/hardware/distance_sensor_hw_test.cc` | 注释 `/dev/ttyS4` → `/dev/ttyS9` ✅ 已修复 |
| `test/CMakeLists.txt` | 用 `test/driver/lockfree_queue_test.cc` 替换 `test_base.cc`（unit_tests 源列表） |
