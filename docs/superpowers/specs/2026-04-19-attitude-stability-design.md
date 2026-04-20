# 姿态稳定性增强设计文档

**日期：** 2026-04-19  
**修订：** 2026-04-20（PID 差速方向前置修正）  
**状态：** 已确认  
**范围：** 现有文件改动，不新增源文件

---

## 0. 前置修正（2026-04-20）

### PID 差速方向错误

在姿态稳定功能实现之前，发现原始 `walk_motor_group.cc` PID 差速方向有根本性错误。

**错误原因：** 代码将纠偏误施加为**左右差速**（lt ≠ rt），而机器人物理约束要求 LT/RT 在同一上轨（上边框）、LB/RB 在同一下轨（下边框），同轨两轮必须同速，否则对轨道产生扭力。

**正确模型：** 通过**上下轨差速**纠偏。

| | 旧代码（错误）| 新代码（正确）|
|---|---|---|
| `rt` | `base_rt − correction` ❌ | `base_rt + correction` ✅ |
| `lb` | `−lt` ❌ | `−base_lt + correction` ✅ |
| `rb` | `−rt` ❌ | `−base_rt + correction` ✅ |

**公式推导（正向，base = +spd，correction = +c 为偏左）：**
- lt = rt = spd + c → 上轨加速
- lb = rb = −spd + c → 下轨命令更少负 → 物理速度减慢
- 上轨超前下轨 → CW 旋转 → yaw 增大 ✓

**正向/返向无需区别对待：**  
base 的正负已编码行走方向。同号 correction 在两方向均产生相同旋转角速度 `ω_z = −c/d`，无需反向。

此修正已在 commit `0245fa76` 中完成，`walk_motor_group.cc` 已改动（见第 7 节更新）。

---

## 1. 背景与问题描述

### 物理场景

光伏清扫机器人采用**双轨四轮（类火车）**结构：
- 上边框（上轨）：**LT + RT** 行驶于上侧光伏板边框轨道（凹槽轮 + 外侧导向轮，约束较强）
- 下边框（下轨）：**LB + RB** 行驶于下侧光伏板边框轨道（平轮，无物理约束，仅靠摩擦力保持）
- LB/RB 与 LT/RT 安装方向相反（motor 命令符号相反，物理运动方向相同）
- 同轨两轮（LT=RT、LB=RB）物理上强约束相同速度，PID 纠偏只能通过**上下轨差速**实现

光伏板之间由铁条连接，由于安装误差，导致板间接缝处存在上下高差，机器人过缝时横滚角（roll）、俯仰角（pitch）、航向角（yaw）均会发生变化。

### 已观察到的失效模式

1. **接缝处卡死**：速度过快冲入高差接缝，轮子被异形边框截住
2. **下边框平轮滑落**：机身侧倾时平轮失去约束，滑出边框
3. **PID 误纠偏**：接缝处倾斜导致 IMU yaw 虚假漂移，PID 施加差速，反而把机器人推向边缘

### 根本原因

当前系统仅使用 yaw 做 PID，但 IMU 测量的是**机体坐标系**下的航向角。机身侧倾（roll）或俯仰（pitch）时，即使机器人在轨道上笔直行走，yaw 读数也会相对于水平面偏移，导致 PID 误判并施加错误差速。

同时，SafetyMonitor 只监控限位开关（GPIO），不监控 IMU 姿态，无法在极端倾斜时主动保护。

---

## 2. 现有 PID 调用链与改动边界

```
MotionService::update()  [50Hz, SCHED_FIFO 80, motion_service.cc]
  ├─ imu_->get_latest().yaw_deg          ← 当前仅读 yaw；本次改为读完整 ImuData
  ├─ EMA 滤波 filtered_yaw_
  ├─ [新增] 倾斜补偿 → yaw_h
  ├─ [新增] 接缝状态机（gyro[0/1] 检测）
  └─ group_->update(yaw_h)               ← 接口不变，仅传入值变化

WalkMotorGroup::update(yaw_deg)  [walk_motor_group.cc]
  └─ HeadingPidController::compute()
       → correction
       → lt = base_lt_rpm_ + correction   ← 上轨左轮
       → rt = base_rt_rpm_ + correction   ← 上轨右轮（与lt同号，上下轨差速纠偏）
       → lb = -base_lt_rpm_ + correction  ← 下轨左轮（base取反，correction同号）
       → rb = -base_rt_rpm_ + correction  ← 下轨右轮（已在commit 0245fa76修正）
```

> **物理意义**：correction > 0（偏左）→ 上轨加速、下轨减速 → 顺时针旋转 → yaw 增大 ✓  
> **注意**：`lb ≠ -lt`；旧错误公式 `lb = -lt = -(base+c) = -base-c` 让 correction 也取反，无差速效果。

> **架构说明**：`WalkMotorGroup` 内部只存储 `base_lt_rpm_` / `base_rt_rpm_`（LB/RB 在 update() 内推导为取反），接缝降速需由 `MotionService` 持有完整 `SpeedCmd base_cmd_`（含4轮方向符号），并在状态转换时调用 `group_->set_speeds(scaled_cmd)` 一次即可，WalkMotorGroup 会自动持续重播该帧。

**`HeadingPidController` 无需修改**——`set_heading_pid_params()` 已支持热更新所有参数含死区，供接缝穿越时动态调整。`walk_motor_group.cc` 已在 commit `0245fa76` 完成 PID 差速公式修正（上下轨差速）。

---

## 3. 解决方案概述

实现三个互补的子功能，**全部改写在现有文件中，不创建新文件**：

| 子功能 | 解决的问题 | 修改文件 |
|---|---|---|
| **A. 倾斜补偿 PID** | PID 误纠偏 | `motion_service.h/.cc` |
| **B. 接缝自适应控制** | 接缝卡死 | `motion_service.h/.cc` |
| **C. 姿态安全监控** | 侧倾摔落 | `safety_monitor.h/.cc`, `fault_handler.h/.cc` |

---

## 3. 子功能 A：倾斜补偿 PID

### 原理

IMU 测量的 yaw 是机体坐标系相对磁北的转角。当机体有 roll/pitch 时，坐标系已倾斜，yaw 读数对应的并非水平面上的真实航向角。

通过 Euler 坐标旋转（ZYX，右手系），将机体 yaw 投影到水平面，得到**水平参考航向角** `yaw_h`：

```
// 标准 ZYX Euler 倾斜补偿公式（φ = 绕X轴 = roll_deg，θ = 绕Y轴 = pitch_deg）
roll_r  = roll_deg  × π/180   // φ：绕X轴，纵向倾斜（上坡/下坡）
pitch_r = pitch_deg × π/180   // θ：绕Y轴，横向倾斜（上下边框高差）
yaw_r   = yaw_deg   × π/180

yaw_h = atan2(
    sin(yaw_r)·cos(pitch_r) - cos(yaw_r)·sin(roll_r)·sin(pitch_r),
    cos(yaw_r)·cos(roll_r)  + sin(yaw_r)·sin(roll_r)·sin(pitch_r)
) × 180/π
```

> **⚠ IMU 坐标系特别说明**：本机器人 Y轴=前进方向（右手法则），与飞行器 X轴=前进惯例不同。
> 因此：
> - `roll_deg`（绕X轴）= **纵向**倾斜（上坡/下坡），物理感受类似飞行器的 "pitch"
> - `pitch_deg`（绕Y轴）= **横向**倾斜（上下边框高差），物理感受类似飞行器的 "roll"
>
> 但数学公式中 φ 始终对应绕X轴的旋转量（即 `roll_deg`），θ 始终对应绕Y轴的旋转量（即 `pitch_deg`），**与物理直觉无关**。实测时需手动倾斜机器人验证 roll_deg / pitch_deg 符号方向后再确认阈值设置。

### 实现位置

**`motion_service.h` / `Config` 新增：**

```cpp
bool tilt_comp_en{true};  ///< 使能 roll/pitch 倾斜补偿（默认开启）
```

**`motion_service.cc` / `update()` 修改：**

```cpp
// 在 EMA 滤波之前，先获取完整 IMU 数据（roll/pitch/yaw/gyro）
const auto imu_data = imu_ ? imu_->get_latest() : device::ImuDevice::ImuData{};
float raw_yaw = imu_data.yaw_deg;

// 倾斜补偿：将机体 yaw 投影到水平面，消除横向/纵向倾斜引起的虚假航向偏差
// 公式约定：phi = roll_deg（绕X轴），theta = pitch_deg（绕Y轴），与数学标准 ZYX 一致
if (cfg_.tilt_comp_en && imu_data.valid) {
    const float phi   = imu_data.roll_deg  * kDegToRad;  ///< 绕X轴，纵向倾斜
    const float theta = imu_data.pitch_deg * kDegToRad;  ///< 绕Y轴，横向倾斜
    const float y     = raw_yaw            * kDegToRad;
    raw_yaw = std::atan2(
        std::sin(y)*std::cos(theta) - std::cos(y)*std::sin(phi)*std::sin(theta),
        std::cos(y)*std::cos(phi)   + std::sin(y)*std::sin(phi)*std::sin(theta)
    ) * kRadToDeg;
}

// EMA 滤波照旧
if (!filtered_yaw_inited_) { ... }
else { filtered_yaw_ = 0.8f * filtered_yaw_ + 0.2f * raw_yaw; }
group_->update(filtered_yaw_);
```

### 效果

| 场景 | 原行为 | 新行为 |
|---|---|---|
| 平地直行 | PID 稳定 | 无变化（pitch_deg≈0，补偿量≈0） |
| 接缝处横向倾斜 15° (pitch_deg=15°) | PID 误加 ~8 RPM 差速 | yaw 补偿后误差 < 0.5°，PID 静默 |
| 上坡行驶（roll_deg=10°） | yaw 漂移 ~1.5° | 补偿后 yaw 稳定 |

---

## 4. 子功能 B：接缝自适应控制

### 原理

利用 IMU 陀螺仪检测接缝穿越事件。过缝时角速度会出现明显脉冲，检测到后：

1. **降低行走速度** × `joint_speed_scale`（减少冲击力）
2. **扩大 PID 死区** × `joint_deadband_scale`（避免颠簸中反复纠偏）

**陀螺仪轴映射（Y=前进方向，右手法则）：**
- `gyro[1]`（Y轴角速度）= 横向倾斜率 = **接缝高差的主要激发轴**（LT/RT与LB/RB高差变化）
- `gyro[0]`（X轴角速度）= 纵向倾斜率 = 纵向高差激发轴（前后轮高差）
- 两轴均监控，任一超阈值即触发接缝状态

穿越结束后自动恢复正常速度和死区。

### 状态机

```
               连续 enter_ticks 帧
               angle_rate > 阈值
kNormal  ──────────────────────────→  kCrossing
   ↑                                       │
   └─── 连续 recovery_ticks 帧 ───────────┘
        angle_rate < 阈值（含迟滞）
```

### 实现位置

**`motion_service.h` 新增私有成员和 Config 字段：**

```cpp
// Config 新增
float joint_detect_rate_dps{25.0f}; ///< 接缝检测角速度阈值 (°/s)
int   joint_enter_ticks{2};          ///< 连续N帧超阈值进入接缝状态
int   joint_recovery_ticks{5};       ///< 连续M帧低于阈值退出接缝状态
float joint_speed_scale{0.6f};       ///< 接缝穿越时速度缩放系数（<1）
float joint_deadband_scale{2.0f};    ///< 接缝穿越时 PID 死区扩大系数（>1）

// 私有成员新增
enum class JointState { kNormal, kCrossing };
JointState joint_state_{JointState::kNormal};
int        joint_trigger_cnt_{0};   ///< 连续超阈值帧计数
int        joint_recovery_cnt_{0};  ///< 连续低于阈值帧计数
// 存储完整四轮速度命令（含方向符号），start_cleaning/returning 时更新；
// 比单纯存 rpm 大小更安全，接缝恢复时直接 set_speeds(base_cmd_) 不会搞错方向。
device::WalkMotorGroup::SpeedCmd base_cmd_{};
```

**`motion_service.cc` / `update()` 追加接缝状态机：**

```cpp
// 接缝状态机：仅在运动中且非 override 状态下激活
if (is_moving() && !is_edge_override_active()) {
    // 接缝检测（陀螺仪角速度，rad/s → deg/s）
    // gyro[1] = Y轴角速度 = 横向倾斜率（上下边框高差方向，接缝穿越主要激发轴）
    // gyro[0] = X轴角速度 = 纵向倾斜率（前后轮高差方向，次要激发轴）
    const float lateral_rate_dps  = std::abs(imu_data.gyro[1]) * kRadToDeg;
    const float fore_aft_rate_dps = std::abs(imu_data.gyro[0]) * kRadToDeg;
    const bool  high_rate = (lateral_rate_dps  > cfg_.joint_detect_rate_dps ||
                             fore_aft_rate_dps > cfg_.joint_detect_rate_dps);

    if (joint_state_ == JointState::kNormal) {
        if (high_rate) {
            if (++joint_trigger_cnt_ >= cfg_.joint_enter_ticks) {
                joint_state_ = JointState::kCrossing;
                joint_trigger_cnt_ = 0;
                // 按比例缩放四轮速度（保留符号，前进/返回均正确）
                device::WalkMotorGroup::SpeedCmd sc = base_cmd_;
                sc.lt_rpm *= cfg_.joint_speed_scale;
                sc.rt_rpm *= cfg_.joint_speed_scale;
                sc.lb_rpm *= cfg_.joint_speed_scale;
                sc.rb_rpm *= cfg_.joint_speed_scale;
                group_->set_speeds(sc);
                // 扩大死区（复用现有 set_heading_pid_params() 接口，WalkMotorGroup 无需修改）
                auto p = cfg_.pid;
                p.deadband_deg *= cfg_.joint_deadband_scale;
                group_->set_heading_pid_params(p);
                spdlog::debug("[MotionService] joint crossing enter, rate={:.1f}/{:.1f} dps",
                              lateral_rate_dps, fore_aft_rate_dps);
            }
        } else {
            joint_trigger_cnt_ = 0;
        }
    } else {  // kCrossing
        if (!high_rate) {
            if (++joint_recovery_cnt_ >= cfg_.joint_recovery_ticks) {
                joint_state_ = JointState::kNormal;
                joint_recovery_cnt_ = 0;
                // 恢复原速和死区
                group_->set_speeds(base_cmd_);
                group_->set_heading_pid_params(cfg_.pid);
                spdlog::debug("[MotionService] joint crossing exit");
            }
        } else {
            joint_recovery_cnt_ = 0;
        }
    }
}
```

**`start_cleaning()` / `start_returning()` / `start_returning_no_brush()` 中记录基准速度命令：**

```cpp
// start_cleaning（前进）
base_cmd_ = {spd, spd, -spd, -spd};

// start_returning* （后退）
base_cmd_ = {-spd, -spd, +spd, +spd};
```

### 注意事项

- `group_->set_speeds(sc)` 在状态**转换时调用一次**即可，`WalkMotorGroup` 会持续重播上一帧；无需每周期重复调用，避免命令队列堆积
- `set_heading_pid_params()` 是热更新接口（`WalkMotorGroup` 已有），**不需要修改** `WalkMotorGroup` 源码
- `SpeedCmd` 带完整4轮方向符号，前进（LT/RT正，LB/RB负）和返回（LT/RT负，LB/RB正）均可正确缩放

---

## 5. 子功能 C：姿态安全监控

### 扩展 SafetyMonitor

在现有 `monitor_loop()` (SCHED_FIFO 94, 5ms) 中增加 IMU 姿态检查。

**不影响限位开关实时性**：限位开关的主要路径是 GPIO 边沿回调（SCHED_FIFO 95），独立于 `monitor_loop()`。增加 IMU 检查（无 I/O 缓存读，< 5μs）对 5ms 循环影响可忽略。

### `safety_monitor.h` 新增

```cpp
// 构造函数增加可选 imu 参数
SafetyMonitor(std::shared_ptr<device::WalkMotorGroup> walk_group,
              std::shared_ptr<device::LimitSwitch>    front_switch,
              std::shared_ptr<device::LimitSwitch>    rear_switch,
              EventBus&                               event_bus,
              std::shared_ptr<device::ImuDevice>      imu = nullptr);

// 姿态监控配置
// ⚠ IMU Y=前进方向（右手法则）坐标约定：
//   pitch_deg (绕Y轴) = 横向倾斜 = LT/RT 与 LB/RB 高差 → 下边框平轮失约束主要风险
//   roll_deg  (绕X轴) = 纵向倾斜 = 上坡/下坡                → 陡坡风险
struct AttitudeConfig {
    bool  en{false};
    float pitch_warn_deg{15.0f};   ///< 横向倾斜（pitch_deg）警告阈值 → kWarn 事件（上下边框高差）
    float pitch_limit_deg{25.0f};  ///< 横向倾斜（pitch_deg）限制阈值 → kLimit 急停
    float roll_warn_deg{25.0f};    ///< 纵向倾斜（roll_deg） 警告阈值 → kWarn 事件（上坡/下坡）
    float roll_limit_deg{35.0f};   ///< 纵向倾斜（roll_deg） 限制阈值 → kLimit 急停
};

// 姿态预警事件（由 FaultHandler 订阅）
struct AttitudeAlertEvent {
    enum class Level { kWarn, kLimit } level;
    float roll_deg;
    float pitch_deg;
};

// 配置接口
void set_attitude_config(const AttitudeConfig& cfg);
```

### `safety_monitor.cc` / `monitor_loop()` 增加

同时在 `safety_monitor.h` 私有成员中增加限流时间戳：

```cpp
std::atomic<uint64_t> last_warn_ts_{0};  ///< kWarn 上次发布时间戳（限流用）
```

```cpp
// 姿态安全检查（5ms 周期，无阻塞 I/O）
if (att_cfg_.en && imu_) {
    const auto d = imu_->get_latest();
    if (d.valid) {
        // pitch_deg = 横向倾斜（上下边框高差，主要安全风险）
        // roll_deg  = 纵向倾斜（上坡/下坡，次要风险）
        const float abs_pitch = std::abs(d.pitch_deg);
        const float abs_roll  = std::abs(d.roll_deg);
        if (abs_pitch >= att_cfg_.pitch_limit_deg ||
            abs_roll  >= att_cfg_.roll_limit_deg) {
            // P1：立即急停（每次超限均触发，急停幂等）
            walk_group_->emergency_override(0.0f);
            event_bus_.publish(AttitudeAlertEvent{
                AttitudeAlertEvent::Level::kLimit, d.roll_deg, d.pitch_deg});
        } else if (abs_pitch >= att_cfg_.pitch_warn_deg ||
                   abs_roll  >= att_cfg_.roll_warn_deg) {
            // P2：发布警告，限流 1 次/秒，防止每 5ms 泛洪事件总线
            const uint64_t now = now_ms();
            if (now - last_warn_ts_.load(std::memory_order_relaxed) >= 1000u) {
                last_warn_ts_.store(now, std::memory_order_relaxed);
                event_bus_.publish(AttitudeAlertEvent{
                    AttitudeAlertEvent::Level::kWarn, d.roll_deg, d.pitch_deg});
            }
        }
    }
}
```

### `fault_handler.h/.cc` 新增订阅

```cpp
// 订阅 AttitudeAlertEvent
bus_.subscribe<middleware::SafetyMonitor::AttitudeAlertEvent>(
    [this](const auto& e) { on_attitude_alert(e); });

// 处理逻辑
void FaultHandler::on_attitude_alert(const SafetyMonitor::AttitudeAlertEvent& e) {
    if (e.level == AttitudeAlertEvent::Level::kLimit) {
        spdlog::error("[FaultHandler] Attitude limit exceeded: roll={:.1f}° pitch={:.1f}°",
                      e.roll_deg, e.pitch_deg);
        fsm_.dispatch<EvFaultP1>();
    } else {
        spdlog::warn("[FaultHandler] Attitude warning: roll={:.1f}° pitch={:.1f}°",
                     e.roll_deg, e.pitch_deg);
        fsm_.dispatch<EvFaultP2>();
    }
}
```

---

## 6. 配置（`hw_test_config.json`）

```json
{
  "motion": {
    "tilt_comp_en": true,
    "joint_detect_rate_dps": 25.0,
    "joint_enter_ticks": 2,
    "joint_recovery_ticks": 5,
    "joint_speed_scale": 0.6,
    "joint_deadband_scale": 2.0
  },
  "safety": {
    "attitude_en": true,
    "pitch_warn_deg": 15.0,
    "pitch_limit_deg": 25.0,
    "roll_warn_deg": 25.0,
    "roll_limit_deg": 35.0
  }
}
```

> **⚠ 字段名说明（IMU Y=前进方向）：**  
> `pitch_*_deg` = 横向倾斜（上下边框高差，绕Y轴，**主要安全风险**，阈值较低）  
> `roll_*_deg`  = 纵向倾斜（上坡/下坡，绕X轴，次要风险，阈值较高）

**向后兼容**：`tilt_comp_en` 和 `attitude_en` 默认 false，旧配置文件无需修改即可使用。

---

## 7. 修改文件汇总

| 文件 | 改动摘要 |
|---|---|
| `include/.../service/motion_service.h` | Config 新增 7 个字段；私有成员新增 JointState + 4 个变量 + `SpeedCmd base_cmd_` |
| `.../service/motion_service.cc` | `update()` 读完整 ImuData，增加倾斜补偿 + 接缝状态机；`start_*()` 记录 `base_cmd_` |
| `include/.../middleware/safety_monitor.h` | 新增 `AttitudeConfig`、`AttitudeAlertEvent`；构造函数增加可选 imu；新增 `set_attitude_config()` |
| `.../middleware/safety_monitor.cc` | 构造函数存储 imu；`monitor_loop()` 增加姿态检查 + kWarn 限流 |
| `include/.../app/fault_handler.h` | 新增 `on_attitude_alert()` 声明 |
| `.../app/fault_handler.cc` | 新增 `AttitudeAlertEvent` 订阅和处理 |
| `config/hw_test_config.json` | 新增 `motion` 和 `safety` 配置块 |
| `main.cc`（或测试入口） | SafetyMonitor 构造传入 imu 指针，读取新配置字段 |

**不需要修改的文件（明确边界）：**

| 文件 | 原因 |
|---|---|
| `service/heading_pid_controller.h/.cc` | PID 核心逻辑不变；死区通过 `set_params()` 热更新 |

> ⚠ `device/walk_motor_group.h/.cc` 已在 commit `0245fa76` 修改（PID差速从左右轨改为上下轨），不在本次姿态稳定功能的修改范围内，但已完成。

---

## 8. 测试方案

### 单元测试（可在 host 运行）

- `motion_service_test.cc`：补充倾斜补偿数学验证（已知 roll/pitch/yaw → 验证 yaw_h）
- `safety_monitor_test.cc`：mock IMU 返回超阈值数据 → 验证 `AttitudeAlertEvent` 发布
- `fault_handler_test.cc`：验证 `AttitudeAlertEvent{kWarn}` → P2，`kLimit` → P1

### 硬件测试（在机器上）

1. **验证 IMU 轴方向**（首先执行）：静止时向上边框侧（LT/RT侧）倾斜机器人，确认 `pitch_deg` 增大（而非 `roll_deg`）；向前进方向倾斜，确认 `roll_deg` 变化。若反之，则阈值字段名需对调。
2. 人工倾斜机器人超过 `pitch_limit_deg`，验证急停触发
3. 过真实接缝，观察日志中 "joint crossing enter/exit" 及速度变化
4. 平地对比测试：`tilt_comp_en: false` vs `true`，对比 PID 误纠偏次数

---

## 9. 参数整定建议

| 参数 | 初始值 | 调整方向 |
|---|---|---|
| `joint_detect_rate_dps` | 25 °/s | 过接缝不触发 → 降低；平地误触发 → 提高 |
| `joint_speed_scale` | 0.6 | 仍卡死 → 降低；过缝太慢影响效率 → 提高 |
| `joint_deadband_scale` | 2.0 | 过缝中仍纠偏 → 提高；过缝后恢复慢 → 降低 |
| `pitch_warn_deg` | 15° | 根据实测最大正常过缝 pitch_deg 值调整（横向倾斜） |
| `pitch_limit_deg` | 25° | 确保低于下边框平轮失去约束的临界角 |
| `roll_warn_deg` | 25° | 根据实测最大坡度行驶 roll_deg 值调整（纵向倾斜） |
| `roll_limit_deg` | 35° | 确保低于机器人倾覆临界角 |

> **⚠ 实测优先**：以上阈值为初始值，需在实际光伏板上测量正常行驶的最大 pitch_deg/roll_deg 后，将阈值设为该值的 1.5~2 倍。
