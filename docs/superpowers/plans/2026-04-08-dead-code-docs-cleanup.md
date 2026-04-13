# Dead Code Removal & Documentation Update Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除废弃代码文件、修复 JSONL 字段命名语义歧义、迁移转发头文件、全面更新 API_REFERENCE.md 和 README.md 使其与当前实现保持一致。

**Architecture:** 纯重构 + 文档更新，不改动业务逻辑。修改涉及：`health_service.cc`（字段重命名）、`limit_switch.h` / `libgpiod_pin.h`（include 路径迁移）、`doc/API_REFERENCE.md`（多处勘误）、`README.md`（补充内容）。

**Tech Stack:** C++17, CMake, nlohmann/json, spdlog, Catch2, Git

---

## 背景与问题清单

| # | 问题 | 影响 | 文件 |
|---|------|------|------|
| 1 | `doc/API_REFERENCE.md.bak` 备份文件遗留 | 目录污染 | `doc/` |
| 2 | `walk` JSONL key `"current"` 实为 `torque_a`（Q轴相电流），与 BMS `"current"`（电池包电流）同名导致语义混淆 | 数据分析误判 | `health_service.cc` |
| 3 | `include/pv_cleaning_robot/driver/pi_mutex.h` 是转发头，两处使用者实际用 `hal::PiMutex`，转发头已无意义 | 头文件冗余 | `limit_switch.h`, `libgpiod_pin.h`, `driver/pi_mutex.h` |
| 4 | `doc/API_REFERENCE.md` BMS `current_a` 注释"正=放电，负=充电"错误（实际 BMS1 正=充电） | 接口文档错误 | `doc/API_REFERENCE.md:557` |
| 5 | API 文档 HealthService 表格仍写"current"字段名 | 文档与实现不符 | `doc/API_REFERENCE.md:1202` |
| 6 | API 文档 HealthService 段落缺少 `"ts"` 时间戳字段说明和 JSONL 载荷示例 | 文档缺失 | `doc/API_REFERENCE.md:1207-1215` |
| 7 | API 文档配置示例串口/GPIO 与 `config.json` 实际值不符（IMU 9600≠921600，BMS ttyS8≠ttyS4，GPIO gpiochip5≠gpiochip0，缺 `use_irq`） | 配置指导错误 | `doc/API_REFERENCE.md:1596-1645` |
| 8 | API 文档 BMS2 协议没有标注"未接线"备注 | 使用者困惑 | `doc/API_REFERENCE.md` BMS 协议层节 |
| 9 | API 文档 Section 10.7 同步语义表格提到"WalkMotor 遗留仍是 std::mutex"，实际 WalkMotor 已仅保留类型定义 | 文档过时 | `doc/API_REFERENCE.md:1549` |
| 10 | API 文档 Section 16 冗余分析提到"set_walk_speed 未考虑 LB/RB 反向 ⏳" 已在 P0 修复中处理 | 待确认/关闭 | `doc/API_REFERENCE.md:2292` |
| 11 | `README.md` 缺少硬件测试二进制、硬件配置要求、gpio.use_irq 说明 | 新开发者信息缺失 | `README.md` |

---

## Task 1: 删除废弃备份文件

**Files:**
- Delete: `doc/API_REFERENCE.md.bak`

- [ ] **Step 1: 删除备份文件**

```bash
cd /home/tronlong/pv_cleaning_robot
rm doc/API_REFERENCE.md.bak
```

- [ ] **Step 2: 验证删除**

```bash
ls doc/API_REFERENCE.md.bak 2>&1  # 应输出 No such file or directory
```

- [ ] **Step 3: Commit**

```bash
git add doc/API_REFERENCE.md.bak
git commit -m "chore: 删除 API_REFERENCE.md.bak 备份残留

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 2: 修复 JSONL walk 电流字段命名

**背景：**
`health_service.cc` 中，行走电机的 `torque_a`（Q轴转矩相电流，量程 ±33A）被写入 JSON key `"current"`，与 BMS 的 `"current"`（电池包总电流）同名。不同物理量用同名字段，导致数据分析时无法区分两种电流。

**修改范围：**
- 修改 `pv_cleaning_robot/service/health_service.cc`：
  - 构造函数 HEALTH 模式 JSON 模板：`{"current", 0.0f}` → `{"torque_a", 0.0f}`
  - 构造函数 DIAGNOSTICS 模式 JSON 模板（wdummy 中）：`{"current",0.0f}` → `{"torque_a",0.0f}`
  - DIAGNOSTICS 模式 `build_payload()` 中：`j_["walk"][kWn[i]]["current"] = wd.torque_a` → `j_["walk"][kWn[i]]["torque_a"] = wd.torque_a`
  - HEALTH 模式 `build_payload()` 中：`j_["walk"]["current"] = avg_torque` → `j_["walk"]["torque_a"] = avg_torque`

**Files:**
- Modify: `pv_cleaning_robot/service/health_service.cc`

- [ ] **Step 1: 修改 HEALTH 模式构造函数模板**

在 `health_service.cc` 构造函数中找到 HEALTH 模式 JSON 初始化块，将 `walk` 节点中的 `"current"` 键改为 `"torque_a"`：

旧代码（约第 47 行）：
```cpp
j_ = {{"ts",   ""},
      {"walk",    {{"rpm", 0.0f}, {"current", 0.0f}, {"fault", false}, {"temp", 0.0f}}},
```

新代码：
```cpp
j_ = {{"ts",   ""},
      {"walk",    {{"rpm", 0.0f}, {"torque_a", 0.0f}, {"fault", false}, {"temp", 0.0f}}},
```

- [ ] **Step 2: 修改 DIAGNOSTICS 模式构造函数模板**

找到 DIAGNOSTICS 模式 wdummy 定义（约第 54 行）：

旧代码：
```cpp
nlohmann::json wdummy = {{"rpm",0.0f},{"target",0.0f},{"current",0.0f},
                          {"can_err",0},{"fault",false},{"fault_code",0},{"online",false}};
```

新代码：
```cpp
nlohmann::json wdummy = {{"rpm",0.0f},{"target",0.0f},{"torque_a",0.0f},
                          {"can_err",0},{"fault",false},{"fault_code",0},{"online",false}};
```

- [ ] **Step 3: 修改 DIAGNOSTICS build_payload() 写入行**

找到 DIAGNOSTICS 模式下每轮写入块（约第 113 行）：

旧代码：
```cpp
j_["walk"][kWn[i]]["current"]    = wd.torque_a;
```

新代码：
```cpp
j_["walk"][kWn[i]]["torque_a"]   = wd.torque_a;
```

- [ ] **Step 4: 修改 HEALTH build_payload() 写入行**

找到 HEALTH 模式均值写入块（约第 170 行左右）：

旧代码：
```cpp
j_["walk"]["current"] = avg_torque;
```

新代码：
```cpp
j_["walk"]["torque_a"] = avg_torque;
```

- [ ] **Step 5: 验证编译通过**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . --target pv_cleaning_robot 2>&1 | tail -20
```

Expected: 无编译错误

- [ ] **Step 6: 在测试代码中同步更新字段名引用**

搜索测试代码中是否有硬编码 `"current"` 用于 walk 电流的 JSON 断言：

```bash
grep -rn '"current"' /home/tronlong/pv_cleaning_robot/test/ --include="*.cc" | grep -i "walk\|wheel\|torque"
```

若有，将对应测试文件中相关 JSON key 从 `"current"` 改为 `"torque_a"`。

- [ ] **Step 7: Commit**

```bash
git add pv_cleaning_robot/service/health_service.cc
git commit -m "fix: 重命名 JSONL 行走电机电流字段 current → torque_a

walk.*.current 存储的是 CAN 状态帧解码的 Q轴转矩相电流（±33A），
与 BMS 电池包总电流（bms.current）物理量不同，同名导致分析误判。

- HEALTH 模式：walk.current → walk.torque_a
- DIAGNOSTICS 模式：walk.lt/rt/lb/rb.current → walk.*.torque_a
- JSON 模板键同步更新，无功能逻辑变更

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 3: 迁移 driver/pi_mutex.h 转发头

**背景：**
`include/pv_cleaning_robot/driver/pi_mutex.h` 是一个转发头，将 `robot::driver::PiMutex` 定义为 `robot::hal::PiMutex` 的别名。检查发现，所有引用该转发头的文件（`limit_switch.h`、`libgpiod_pin.h`）实际使用的均是 `hal::PiMutex` 命名空间，无代码依赖 `driver::PiMutex` 别名。可安全迁移为直接 include `hal/pi_mutex.h`，消除冗余层。

**Files:**
- Modify: `include/pv_cleaning_robot/device/limit_switch.h` (line 17)
- Modify: `include/pv_cleaning_robot/driver/libgpiod_pin.h` (line 19)
- Delete: `include/pv_cleaning_robot/driver/pi_mutex.h`

- [ ] **Step 1: 更新 limit_switch.h 的 include**

旧代码（第 17 行）：
```cpp
#include "pv_cleaning_robot/driver/pi_mutex.h"
```

新代码：
```cpp
#include "pv_cleaning_robot/hal/pi_mutex.h"
```

- [ ] **Step 2: 更新 libgpiod_pin.h 的 include**

旧代码（第 19 行）：
```cpp
#include "pv_cleaning_robot/driver/pi_mutex.h"
```

新代码：
```cpp
#include "pv_cleaning_robot/hal/pi_mutex.h"
```

- [ ] **Step 3: 删除转发头文件**

```bash
rm /home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/driver/pi_mutex.h
```

- [ ] **Step 4: 验证没有其他文件引用该转发头**

```bash
grep -rn "driver/pi_mutex" /home/tronlong/pv_cleaning_robot/ \
  --include="*.cc" --include="*.h" | grep -v "build/"
```

Expected: 无输出（所有引用已迁移）

- [ ] **Step 5: 验证编译通过**

```bash
cd /home/tronlong/pv_cleaning_robot/build
cmake --build . 2>&1 | tail -20
```

Expected: 无编译错误

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/device/limit_switch.h \
        include/pv_cleaning_robot/driver/libgpiod_pin.h
git rm include/pv_cleaning_robot/driver/pi_mutex.h
git commit -m "refactor: 移除 driver/pi_mutex.h 转发头，直接 include hal/pi_mutex.h

两处使用者（limit_switch.h、libgpiod_pin.h）均使用 hal::PiMutex
命名空间，driver::PiMutex 别名从未被引用，转发头已无价值。

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 4: 修正 API_REFERENCE.md 的多处错误

**Files:**
- Modify: `doc/API_REFERENCE.md`

### 4A: 修正 BMS current_a 注释符号方向

- [ ] **Step 1: 修正 BMS BatteryData 注释（第 557 行）**

旧代码：
```cpp
float    current_a;        // 电流（正=放电，负=充电）
```

新代码：
```cpp
float    current_a;        // 电流（A），正=充电，负=放电（BMS1 嘉佰达 UART 协议；BMS2 Modbus 符号相反：放电=正）
```

### 4B: 修正 HealthService JSONL 字段描述

- [ ] **Step 2: 更新 HealthService 模式对比表（第 1202 行附近）**

旧代码：
```
| `Mode::HEALTH` | 精简 | 4轮均值 rpm/current/fault + temp | 生产/低带宽 | 1 Hz |
| `Mode::DIAGNOSTICS` | 完整 | 每轮独立 lt/rt/lb/rb + ctrl_frames/ctrl_err | 开发/调试 | 可配置 |
```

新代码：
```
| `Mode::HEALTH` | 精简 | 4轮均值 rpm/**torque_a**/fault + temp | 生产/低带宽 | 1 Hz |
| `Mode::DIAGNOSTICS` | 完整 | 每轮独立 lt/rt/lb/rb(rpm/**torque_a**/fault) + ctrl_frames/ctrl_err | 开发/调试 | 可配置 |
```

- [ ] **Step 3: 在 HealthService 段落中补充 JSONL 载荷示例和 ts 字段说明**

在第 1211 行（`配置键：diagnostics.local_path...`）下方，添加：

```markdown
**JSONL 载荷格式（每行一条 JSON 记录）**：

HEALTH 模式（精简）示例：
```json
{"ts":"2026-04-07T10:00:00Z","walk":{"rpm":-28.5,"torque_a":1.2,"fault":false,"temp":0.0},"brush":{"running":true,"fault":false},"battery":{"soc":82.5,"voltage":48.2,"charging":false,"alarm":false},"imu":{"pitch":2.1,"roll":0.3,"valid":true},"gps":{"lat":0.0,"lon":0.0,"fix":0,"valid":false}}
```

DIAGNOSTICS 模式（完整）每轮示例：
```json
{"ts":"2026-04-07T10:00:00Z","walk":{"lt":{"rpm":-28.5,"target":-30.0,"torque_a":1.2,"can_err":0,"fault":false,"fault_code":0,"online":true},"rt":{...},"lb":{...},"rb":{...},"temp":0.0,"ctrl_frames":1240,"ctrl_err":0},"bms":{"soc":82.5,"voltage":48.2,"current":-2.5,...}}
```

> **重要区别**：`walk.*.torque_a` 是 CAN 状态帧解码的 Q轴转矩相电流（±33 A），
> 反映电机转矩大小，**不是**直流母线电流，**不可**与 `bms.current`（电池包总电流，A）
> 直接求和比较。前者是相电流，后者是 DC 母线电流，物理量不同。
> `bms.current`：正=充电，负=放电；`walk.*.torque_a`：符号表示转矩方向。
```

- [ ] **Step 4: 删除陈旧的 DiagnosticsCollector 废弃注意（第 1214-1215 行）**

删除以下两行（`DiagnosticsCollector` 类已彻底不存在）：
```
> **注意**：`DiagnosticsCollector` 已废弃，其功能完全合并到 `HealthService::Mode::DIAGNOSTICS`。
> 头文件保留为迁移存根，新代码应直接使用 `HealthService`。
```

### 4C: 添加 BMS2 协议"未接线"备注

- [ ] **Step 5: 在 Protocol 层 BMS2 描述处添加备注**

在 `Bms2Protocol` 类说明所在段落末尾（Protocol 层节）追加：

```markdown
> **硬件说明**：`Bms2Protocol`（Modbus 寄存器协议，嘉佰达第二代）目前**未在生产接线中使用**。
> 当前实际接入的是 `BmsProtocol`（UART 自定义帧，BMS1）。
> BMS2 电流符号与 BMS1 **相反**：放电=正，充电=负（参见 `raw_to_current = (raw - 30000) * 0.1`）。
> 如需切换，修改 `bms.cc` 驱动层调用 `Bms2Protocol::decode_basic_info()` 替代现有解码逻辑。
```

### 4D: 修正 Section 10.7 同步语义表格

- [ ] **Step 6: 更新第 1549 行 WalkMotor 描述**

旧代码：
```
| Device | WalkMotorGroup / ImuDevice / LimitSwitch / GPS/BMS/Brush | PiMutex + mutex + atomic | ✅（主体） | `WalkMotor`（遗留）仍是 `std::mutex`，建议仅测试保留不进入生产路径 |
```

新代码：
```
| Device | WalkMotorGroup / ImuDevice / LimitSwitch / GPS/BMS/Brush | PiMutex + mutex + atomic | ✅ | `WalkMotor` 已重构为纯数据类型定义（`walk_motor_types.h`），不含硬件操作，无同步需求 |
```

### 4E: 修正配置文件示例

- [ ] **Step 7: 替换 Section 11 配置示例为与 config.json 实际值一致的版本**

找到 Section 11 的 JSON 代码块（第 1584-1645 行），将其中串口、GPIO 和配置字段替换为以下正确值：

```json
{
  "logging": {
    "log_dir": "/data/pv_robot/logs",
    "level": "info",
    "console": true
  },
  "can": {
    "interface": "can0",
    "walk_motor": { "motor_id": 1, "comm_timeout_ms": 200 }
  },
  "serial": {
    "imu":   { "port": "/dev/ttyS1", "baudrate": 9600 },
    "gps":   { "port": "/dev/ttyS2", "baudrate": 9600 },
    "brush": { "port": "/dev/ttyS3", "baudrate": 9600, "slave_id": 1 },
    "bms":   { "port": "/dev/ttyS8", "baudrate": 9600 }
  },
  "gpio": {
    "front_limit": { "chip": "gpiochip5", "line": 0 },
    "rear_limit":  { "chip": "gpiochip5", "line": 1 },
    "use_irq": false
  },
  "network": {
    "transport_mode": "mqtt_only",
    "mqtt": {
      "broker_uri": "ssl://tb.example.com:8883",
      "client_id": "pv_robot_001",
      "tls_enabled": true,
      "ca_cert_path": "/etc/pv_robot/certs/ca.crt",
      "keep_alive_s": 60,
      "qos": 1
    },
    "lorawan": {
      "port": "/dev/ttyS5",
      "baudrate": 115200,
      "dev_eui": "",
      "app_key": "",
      "report_interval_s": 300
    }
  },
  "storage": {
    "cache_path": "/data/pv_robot/telemetry_cache.jsonl"
  },
  "system": {
    "hw_watchdog": "/dev/watchdog"
  },
  "scheduler": {
    "windows": [
      { "hour": 8,  "minute": 0  },
      { "hour": 14, "minute": 30 }
    ]
  },
  "robot": {
    "track_length_m":   1000.0,
    "passes":           1.0,
    "clean_speed_rpm":  300.0,
    "return_speed_rpm": 300.0,
    "return_brush_rpm": 1000,
    "brush_rpm":        1000,
    "edge_reverse_rpm": 0.0,
    "heading_pid_en":   true,
    "battery_full_soc": 95.0,
    "battery_low_soc":  15.0,
    "wheel_circ_m":     0.3
  },
  "diagnostics": {
    "mode":                "development",
    "publish_interval_ms": 1000,
    "cloud_upload":        true,
    "local_log":           true,
    "local_path":          "/data/pv_robot/logs/telemetry.jsonl"
  },
  "device": {
    "fw_version": "1.0.0",
    "hw_version": "1.0"
  }
}
```

在配置说明文字中添加 `gpio.use_irq` 字段说明（在 `gpio` 节描述处插入）：

```markdown
- `gpio.use_irq`：`false` = 轮询模式，每 1ms 调用 `gpiod_line_get_value()` 检测边沿。
  RK3576 的 `gpiochip5` 不支持 `GPIO_GET_LINEEVENT_IOCTL`，**必须设为 false**。
  `true` = libgpiod 中断事件模式，适用于支持 IRQ 的平台。
```

### 4F: 关闭 Section 16 已修复条目

- [ ] **Step 8: 更新 Section 16 优化分析表（第 2292 行附近）**

旧代码：
```
| 16.15 | set_walk_speed(rpm) 调用 set_speeds(rpm,rpm,rpm,rpm) 未考虑 LB/RB 安装反向 | 🟡 | ⏳ 待修复（CleanTask 死代码路径）|
```

新代码：
```
| 16.15 | set_walk_speed(rpm) 调用 set_speeds(rpm,rpm,rpm,rpm) 未考虑 LB/RB 安装反向 | 🟢 | ✅ 已修复（commit 6cb831c0，测试断言已按轮修正；CleanTask 中此路径保留但不进入生产）|
```

- [ ] **Step 9: Commit**

```bash
git add doc/API_REFERENCE.md
git commit -m "docs: 修正 API_REFERENCE.md 多处错误与过时描述

- BMS current_a 符号方向：正=放电→正=充电（BMS1 实际语义）
- BMS2 协议添加未接线备注及与 BMS1 符号差异说明
- HealthService JSONL 字段 current → torque_a（与 Task 2 代码变更同步）
- 补充完整 JSONL 载荷示例和 torque_a vs bms.current 区别说明
- 删除陈旧 DiagnosticsCollector 废弃注意（类已不存在）
- 配置示例与 config.json 实际值对齐（ttyS8/gpiochip5/9600）
- 补充 gpio.use_irq 字段说明
- 更新同步语义表格 WalkMotor 描述（已为纯类型定义）
- 标记 set_walk_speed LB/RB 问题为已修复

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 5: 更新 README.md

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 替换 README.md 为更完整的版本**

用以下内容完整替换 `README.md`：

```markdown
# PV 清扫机器人固件

RK3576 / PREEMPT\_RT Linux / aarch64，C++17。光伏板面清扫机器人固件——主轨道清扫、故障自检、云端遥测。

## 硬件要求

| 接口 | 设备 | 配置路径 |
|------|------|----------|
| CAN (`can0`) | 行走电机组 × 4（M1502E_111，motor_id 1–4） | `can.interface` |
| UART `/dev/ttyS1` | IMU（WIT Motion 9轴，9600 baud） | `serial.imu` |
| UART `/dev/ttyS2` | GPS（NMEA 0183，可选，暂无硬件） | `serial.gps` |
| UART `/dev/ttyS3` | 辊刷电机（Modbus RTU，slave_id=1） | `serial.brush` |
| UART `/dev/ttyS8` | BMS（嘉佰达 UART 自定义协议，BMS1） | `serial.bms` |
| GPIO `gpiochip5/0` | 前限位传感器（接近开关，NPN 常开） | `gpio.front_limit` |
| GPIO `gpiochip5/1` | 后限位传感器（接近开关，NPN 常开） | `gpio.rear_limit` |

> **注意**：RK3576 的 `gpiochip5` 不支持 GPIO 中断（`GPIO_GET_LINEEVENT_IOCTL` 返回 `ENODEV`），
> 必须在 `config.json` 中设置 `"gpio": { "use_irq": false }` 启用轮询模式。

## 构建前置条件

1. 安装 aarch64 交叉编译工具链（`aarch64-linux-gnu-gcc`）
2. 设置 RK3576 SDK 路径环境变量：

```bash
source .env.example        # 或 export RK3576_SDK_PATH=/your/sdk/path
```

3. 配置并编译：

```bash
cmake --preset rk3576-cross-linux
cmake --build --preset rk3576-build
```

构建产物：
- 主程序：`build/aarch64/bin/pv_cleaning_robot`
- 单元测试：`build/aarch64/bin/unit_tests`（可在 QEMU 或目标板运行，**不需要真实硬件**）
- 硬件集成测试：`build/aarch64/bin/hw_tests`（**必须在目标板上运行，需要真实硬件**）

## 运行测试

### 单元测试（Mock 硬件）

```bash
# 全部单元测试
./unit_tests

# 按标签筛选
./unit_tests "[protocol]"          # 协议层解码测试
./unit_tests "[device][bms]"       # BMS 设备层
./unit_tests "[integration]"       # 系统集成测试（mock 硬件）
./unit_tests "[integration][system][health]"  # HealthService JSONL 落盘
```

### 硬件集成测试（目标板 + 真实硬件）

> **安全须知**：确保机器人在停机位，清扫轨道前方无障碍物。测试电机速度限制为 30 RPM。

```bash
# 限位传感器回调链路（需手动触发传感器）
./hw_tests "[hw_limit][callback_front]"
./hw_tests "[hw_limit][callback_rear]"

# 行走电机组
./hw_tests "[hw_walk]"

# BMS 数据读取
./hw_tests "[hw_bms]"

# IMU 数据读取
./hw_tests "[hw_imu]"

# 全栈系统测试（推荐先运行）
./hw_tests "[hw_system][full_init]"
./hw_tests "[hw_system][health_real_data]"

# N=1 完整任务链（CleanFwd → CleanReturn → Charging），持续采集 JSONL 健康数据
# 运行前确认机器人在停机位，前后限位传感器均已接线
./hw_tests "[hw_system][n1_clean_cycle]"

# 完整循环测试（同上 + 全程持续采集，JSONL 文件不删除）
./hw_tests "[hw_system][combined]"

# 运行全部硬件测试
./hw_tests "[hw_system]"
```

## 配置文件

运行时配置：`/opt/robot/config/config.json`（生产）/ `config/config.json`（开发回退）。

**关键配置项**：

| 键 | 说明 |
|----|------|
| `gpio.use_irq` | `false`=轮询模式（**RK3576 gpiochip5 必须设为 false**）；`true`=中断模式（其他平台）|
| `network.transport_mode` | `"mqtt_only"` / `"lorawan_only"` / `"dual_parallel"` |
| `network.mqtt.tls_enabled` | 是否启用 TLS（生产环境应为 `true`） |
| `network.mqtt.ca_cert_path` | CA 证书路径 |
| `diagnostics.mode` | `"production"` → HEALTH 精简模式；`"development"` → DIAGNOSTICS 完整模式 |
| `diagnostics.local_path` | 本地 JSONL 落盘路径（空字符串禁用） |
| `robot.passes` | 清扫趟数（`0.5`=单程，`1.0`=往返一趟） |
| `robot.heading_pid_en` | 是否启用 IMU 航向 PID 差速补偿（需 IMU 已接线） |

## 参考文档

- `doc/API_REFERENCE.md` — 完整 API 接口文档（含协议格式、JSONL 载荷格式、配置说明）
- `doc/dev-guide/CONCURRENCY.md` — 并发契约与线程关闭顺序
- `docs/superpowers/specs/` — 架构优化设计规格（历史记录）
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: 更新 README.md 补充硬件要求、测试命令和配置说明

- 新增硬件接口对照表（CAN/UART/GPIO 与设备的映射）
- 补充 hw_tests 硬件集成测试的运行命令和安全须知
- 添加 RK3576 gpio.use_irq=false 的必要性说明
- 添加关键配置项说明表格
- 更新单元测试和集成测试命令示例

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## 验收标准

完成所有任务后，验证：

```bash
cd /home/tronlong/pv_cleaning_robot

# 1. 废弃备份文件不存在
ls doc/API_REFERENCE.md.bak 2>&1  # No such file or directory

# 2. driver/pi_mutex.h 转发头不存在
ls include/pv_cleaning_robot/driver/pi_mutex.h 2>&1  # No such file or directory

# 3. 无遗留 driver/pi_mutex 引用
grep -rn "driver/pi_mutex" include/ pv_cleaning_robot/ --include="*.h" --include="*.cc"  # 空输出

# 4. health_service 使用新字段名（walk 不再有 "current"）
grep '"current"' pv_cleaning_robot/service/health_service.cc  # 只应看到 bms 相关，无 walk 相关

# 5. 构建成功
cmake --build build/ 2>&1 | grep "error:" | grep -v "3rdparty" | head -10  # 无编译错误

# 6. git log 确认全部提交
git --no-pager log --oneline -6
```
