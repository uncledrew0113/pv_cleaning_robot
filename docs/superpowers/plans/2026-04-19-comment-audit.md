# Comment Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix all incorrect, missing, or garbled comments across every header and implementation file in the pv_cleaning_robot project.

**Architecture:** Six-layer project (HAL→Driver→Protocol→Device→Middleware/Service→App). Edits are pure comment-only changes (no logic changes), so no tests need to be written. Each task covers one layer/group of files, ends with a build to confirm no syntax errors were introduced.

**Tech Stack:** C++17, CMake, spdlog, Boost.SML, libgpiod, libmodbus, libserialport.

---

## Issue Inventory

The following 14 confirmed issues were found during the audit:

| # | File | Type | Description |
|---|------|------|-------------|
| 1 | `protocol/bms_protocol.h` | WRONG | Usage example code block: `if (parser.frame_complete()` line is missing before the orphaned `&&` |
| 2 | `device/brush_motor.h` | WRONG | Class comment says "周期更新（50ms）" — actual period is 500ms (bms_exec thread) |
| 3 | `service/scheduler_service.h` | WRONG | `tick()` doxygen says "应由 1Hz 循环调用" — main.cc calls it at 100ms (10Hz) |
| 4 | `device/imu_device.h` | MISSING | `ImuData` struct fields `roll_deg`/`pitch_deg`/`yaw_deg`/`timestamp_us`/`valid` have no inline comments |
| 5 | `middleware/lorawan_transport.h` | MISSING | `subscribe()` does not note that only a single downlink callback is supported (second call silently overwrites first) |
| 6 | `middleware/thread_executor.h` | MISSING | `IRunnable` interface and `Config` struct have no usage example |
| 7 | `service/nav_service.h` | MISSING | Constructor parameter `wheel_circumference_m` has no comment explaining its role or typical value |
| 8 | `service/heading_pid_controller.h` | MISSING | Private static methods `norm_angle()` and `clamp()` have no comments |
| 9 | `app/fault_handler.h` | WRONG | Destructor comment and `start_listening()` Doxygen are merged onto one line (formatting/merge error) |
| 10 | `service/motion_service.h` | MISSING | `start_returning_no_brush()` is missing a note that it is the P1 fault path |
| 11 | `service/fault_service.h` | MISSING | FaultEvent::Level enum P3 description is only in class-level comment, not in enum value comment |
| 12 | `service/motion_service.cc` | WRONG | Lines 190-192: garbled Chinese text — "阶塞" → "阻塞", "住用" → "占用", "周期平候" → "周期变化" |
| 13 | `app/fault_handler.cc` | MISSING | `on_fault()` P1 branch comment does not explain that FSM will also call `start_returning_no_brush()` after dispatch |
| 14 | `driver/libgpiod_pin.h` | MISSING | `@Description:` field in file header block is empty — no description of what the file does |

---

## Task 1: Protocol Layer — bms_protocol.h usage example

**Files:**
- Modify: `include/pv_cleaning_robot/protocol/bms_protocol.h`

Find the broken Doxygen usage example (around line 95–115 in the `@brief` block for `BmsProtocol`). The `if (parser.frame_complete()` line is missing; there is an orphaned `&&` on its own line.

- [ ] **Step 1: Open the file and locate the broken block**

The broken block currently looks like this (lines ~100–108):

```cpp
/// @code
/// for (uint8_t b : rx_bytes) {
///     parser.push_byte(b);
///             && parser.get_cmd() == 0x03) {
///         auto info = BmsProtocol::decode_basic_info(...);
```

- [ ] **Step 2: Fix the broken code example**

Replace the orphaned `&&` line with the complete `if` statement:

```cpp
/// @code
/// for (uint8_t b : rx_bytes) {
///     parser.push_byte(b);
///     if (parser.frame_complete() && parser.get_cmd() == 0x03) {
///         auto info = BmsProtocol::decode_basic_info(...);
```

- [ ] **Step 3: Build to verify no syntax error**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

Expected: build succeeds (no new errors vs. baseline).

- [ ] **Step 4: Commit**

```bash
cd /home/tronlong/pv_cleaning_robot
git add include/pv_cleaning_robot/protocol/bms_protocol.h
git commit -m "docs: fix broken BmsProtocol usage example in bms_protocol.h

The if (parser.frame_complete() line was missing from the Doxygen
code example, leaving an orphaned && that confused readers.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 2: Device Layer — brush_motor.h & imu_device.h

**Files:**
- Modify: `include/pv_cleaning_robot/device/brush_motor.h`
- Modify: `include/pv_cleaning_robot/device/imu_device.h`

### 2a — brush_motor.h: Fix "50ms" → "500ms"

The class-level comment currently says `// ── 周期更新（50ms）`. The actual period is 500ms (bms_exec thread in main.cc).

- [ ] **Step 1: Find and fix the period comment**

Replace:
```cpp
// ── 周期更新（50ms）
```
with:
```cpp
// ── 周期更新（500ms，由 bms_exec 线程 SCHED_OTHER 调用）
```

### 2b — imu_device.h: Add inline comments to ImuData fields

The `ImuData` struct currently has no inline field comments. Add comments matching the canonical definitions in `imu_protocol.h`:

- [ ] **Step 2: Add field comments to ImuData**

Find the `ImuData` struct in `imu_device.h`. It currently looks like:
```cpp
struct ImuData {
    float roll_deg{0};
    float pitch_deg{0};
    float yaw_deg{0};
    uint64_t timestamp_us{0};
    bool valid{false};
};
```

Change to:
```cpp
struct ImuData {
    float    roll_deg{0};        ///< 横滚角（度，-180~+180）
    float    pitch_deg{0};       ///< 俯仰角（度，-90~+90）
    float    yaw_deg{0};         ///< 航向角（度，-180~+180；顺时针为正）
    uint64_t timestamp_us{0};    ///< 设备本地时间戳（微秒，取自 get_latest() 调用时的系统单调时钟）
    bool     valid{false};       ///< 数据有效标志：read_loop 至少解析出 1 帧后置 true；超过 1s 无帧则清除
};
```

- [ ] **Step 3: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 4: Commit**

```bash
git add include/pv_cleaning_robot/device/brush_motor.h \
        include/pv_cleaning_robot/device/imu_device.h
git commit -m "docs: fix brush_motor period (50ms→500ms), add ImuData field comments

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 3: Service Layer — scheduler_service.h, nav_service.h, heading_pid_controller.h, motion_service.h, fault_service.h

**Files:**
- Modify: `include/pv_cleaning_robot/service/scheduler_service.h`
- Modify: `include/pv_cleaning_robot/service/nav_service.h`
- Modify: `include/pv_cleaning_robot/service/heading_pid_controller.h`
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `include/pv_cleaning_robot/service/fault_service.h`

### 3a — scheduler_service.h: Fix "1Hz" → actual call rate

`tick()` doxygen currently says "应由 1Hz 循环调用". main.cc calls it in a 100ms sleep loop (10Hz).

- [ ] **Step 1: Fix tick() frequency comment**

Replace:
```cpp
/// 应由 1Hz 循环调用
void tick();
```
with:
```cpp
/// 应由主循环定期调用（main.cc 以 100ms 间隔调用，即 ~10Hz）。
/// 内部计时器精度取决于调用间隔；建议调用间隔 ≤ 1000ms。
void tick();
```

### 3b — nav_service.h: Add comment for wheel_circumference_m

Find the constructor:
```cpp
explicit NavService(std::shared_ptr<device::WalkMotorGroup> group,
                    float wheel_circumference_m);
```
The parameter `wheel_circumference_m` has no comment. Add:
```cpp
/// @param group              行走电机组，用于读取各轮速度反馈
/// @param wheel_circumference_m 车轮周长（米），用于里程积分；
///        典型值：0.628f（直径 200mm 橡胶轮，π × 0.2）
```

### 3c — heading_pid_controller.h: Add comments to private static methods

Find the private section:
```cpp
static float norm_angle(float deg);
static float clamp(float v, float lo, float hi);
```
Replace with:
```cpp
/// 将角度规范化到 (-180, +180] 区间，处理 0°/360° 跨越边界的误差计算
static float norm_angle(float deg);
/// 通用限幅：将 v 限制在 [lo, hi] 范围内
static float clamp(float v, float lo, float hi);
```

### 3d — motion_service.h: Add P1 fault path note to start_returning_no_brush()

Find:
```cpp
bool start_returning_no_brush();
```
Replace with:
```cpp
/// @brief P1 故障路径：先停滚刷，再以返回速度倒退回停机位。
///
/// 由 RobotFsm::dispatch<EvFaultP1>() 调用；与 start_returning() 的区别是
/// 滚刷不反向运行（刷电机立即停止），适用于需要避免滚刷二次损伤的故障场景。
bool start_returning_no_brush();
```

### 3e — fault_service.h: Add P3 comment to enum value

Find the `Level` enum inside `FaultEvent`:
```cpp
enum class Level { P0, P1, P2, P3 };
```
Replace with:
```cpp
enum class Level {
    P0,  ///< 严重故障：立即急停，FSM → Fault 状态，等待人工复位
    P1,  ///< 一般故障：停滚刷，安全返回停机位，FSM → Returning
    P2,  ///< 告警：降速继续，告警上报（EventBus），不转换 FSM 状态
    P3   ///< 提示：仅记录日志，不影响运行
};
```

- [ ] **Step 2: Apply all 5 changes above**

- [ ] **Step 3: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 4: Commit**

```bash
git add include/pv_cleaning_robot/service/scheduler_service.h \
        include/pv_cleaning_robot/service/nav_service.h \
        include/pv_cleaning_robot/service/heading_pid_controller.h \
        include/pv_cleaning_robot/service/motion_service.h \
        include/pv_cleaning_robot/service/fault_service.h
git commit -m "docs: improve service-layer comments (tick freq, nav params, PID helpers, P1/P3 descriptions)

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 4: Middleware Layer — lorawan_transport.h, thread_executor.h

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/lorawan_transport.h`
- Modify: `include/pv_cleaning_robot/middleware/thread_executor.h`

### 4a — lorawan_transport.h: Document single-slot subscribe limitation

Find the `subscribe()` declaration. Add a note that only one downlink callback is supported:

Current:
```cpp
void subscribe(const std::string& topic, MessageCallback cb) override;
```
Replace with:
```cpp
/// 注册下行数据回调。
/// @note LoRaWAN 单信道限制：仅支持一个下行回调槽（downlink_cb_），
///       重复调用将覆盖上一个回调。如需多路分发，请在回调内部自行路由。
void subscribe(const std::string& topic, MessageCallback cb) override;
```

### 4b — thread_executor.h: Add usage example to IRunnable and Config

Find the `IRunnable` interface and `Config` struct. Add a usage comment block above or inside them:

Before `struct IRunnable`:
```cpp
/// @brief 可执行任务接口。
///
/// 使用方式（推荐用 RunnableAdapter 包装 lambda）：
/// @code
/// // 方式1：实现接口
/// struct MyTask : IRunnable {
///     void run() override { /* 业务逻辑 */ }
/// };
///
/// // 方式2：lambda 包装（零额外堆分配）
/// auto exec = ThreadExecutor(ThreadExecutor::Config{
///     .name = "my_worker",
///     .period_ms = 50,
///     .sched_policy = SCHED_FIFO,
///     .priority = 80,
/// });
/// exec.set_runnable(std::make_shared<RunnableAdapter>([]{ /* ... */ }));
/// exec.start();
/// @endcode
struct IRunnable {
```

In `Config` struct, add field comments if they are missing:
```cpp
struct Config {
    std::string name;          ///< 线程名称（最长 15 字符，用于 pthread_setname_np）
    int period_ms{50};         ///< 周期（毫秒）；0 = 无延迟连续运行
    int sched_policy{SCHED_OTHER}; ///< 调度策略：SCHED_OTHER / SCHED_FIFO / SCHED_RR
    int priority{0};           ///< RT 优先级（SCHED_FIFO/RR 时有效，范围 1–99）
    int cpu_affinity{-1};      ///< CPU 绑定掩码（-1 = 不绑定）
};
```

- [ ] **Step 1: Apply both changes above**

- [ ] **Step 2: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add include/pv_cleaning_robot/middleware/lorawan_transport.h \
        include/pv_cleaning_robot/middleware/thread_executor.h
git commit -m "docs: document LoRaWAN single-slot subscribe, add ThreadExecutor usage example

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 5: App Layer — fault_handler.h

**Files:**
- Modify: `include/pv_cleaning_robot/app/fault_handler.h`

The destructor comment and `start_listening()` Doxygen are merged onto one line (formatting/merge error). Currently:

```cpp
~FaultHandler();  ///< 析构时自动取消 EventBus 订阅，防止回调悬空指针/// 注册 EventBus 监听（在
                  ///< EventBus 上订阅 FaultEvent）
void start_listening();
```

- [ ] **Step 1: Fix the split/merged comment**

Replace the above with:
```cpp
/// 析构时自动取消 EventBus 订阅，防止回调访问悬空指针
~FaultHandler();

/// 注册 EventBus 监听（订阅 FaultEvent），必须在 EventBus 和 MotionService 就绪后调用
void start_listening();
```

- [ ] **Step 2: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add include/pv_cleaning_robot/app/fault_handler.h
git commit -m "docs: fix merged destructor+start_listening comment in fault_handler.h

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 6: Driver Layer — libgpiod_pin.h

**Files:**
- Modify: `include/pv_cleaning_robot/driver/libgpiod_pin.h`

The `@Description:` field in the file header block is empty.

- [ ] **Step 1: Fill in the @Description field**

The current header block has:
```cpp
 * @Description:
```

Replace with:
```cpp
 * @Description: libgpiod v1.6 GPIO 引脚驱动实现。
 *               支持输入（边沿检测）和输出两种模式。
 *               边沿检测优先使用硬件 IRQ，不可用时自动回退至 1ms 软件轮询。
 *               支持软件消抖（可配置延迟），RT 提权（SCHED_FIFO），CPU 绑定。
```

- [ ] **Step 2: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add include/pv_cleaning_robot/driver/libgpiod_pin.h
git commit -m "docs: fill @Description in libgpiod_pin.h file header

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 7: Implementation files — motion_service.cc & fault_handler.cc

**Files:**
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `pv_cleaning_robot/app/fault_handler.cc`

### 7a — motion_service.cc: Fix garbled comment (lines 190–192)

Current garbled text:
```cpp
// 原因：Modbus RTU 读取寄存器需 5~10ms阶塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
// 中将住用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期平候
```

Replace with corrected text:
```cpp
// 原因：Modbus RTU 读取寄存器需 5~10ms 阻塞 I/O，放在 walk_ctrl(FIFO 80, 20ms)
// 中将占用 25%~50% 控制周期时间预算。BrushMotor 状态 50~500ms 周期变化，
// 移至低优先级 bms_exec 线程可完全消除对运动控制周期的干扰。
```

### 7b — fault_handler.cc: Add note about P1 FSM dispatch sequence

Current P1 branch comment:
```cpp
case Level::P1:
    // 停止清扫，启动返回
    motion_->stop_cleaning();
    motion_->start_returning();
    dispatch_fn_(evt);
    break;
```

Replace with:
```cpp
case Level::P1:
    // P1 故障：先停滚刷，再启动返程，最后通知 FSM
    // 注意：dispatch_fn_ 将触发 RobotFsm::dispatch<EvFaultP1>()，
    // FSM 内部会进一步调用 motion_->start_returning_no_brush()（停刷+返回）。
    // 此处 stop_cleaning() + start_returning() 是 P1 的首次运动状态变更，
    // FSM 的 start_returning_no_brush() 会再次确认停刷状态，两者不冲突。
    motion_->stop_cleaning();
    motion_->start_returning();
    dispatch_fn_(evt);
    break;
```

- [ ] **Step 1: Apply both .cc changes**

- [ ] **Step 2: Build to verify**

```bash
cd /home/tronlong/pv_cleaning_robot && cmake --build build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add pv_cleaning_robot/service/motion_service.cc \
        pv_cleaning_robot/app/fault_handler.cc
git commit -m "docs: fix garbled motion_service.cc comment, clarify P1 fault handler sequence

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Self-Review Checklist

**Spec coverage:**
- [x] Issue 1 (bms_protocol.h broken example) → Task 1
- [x] Issue 2 (brush_motor 50ms) → Task 2a
- [x] Issue 3 (scheduler 1Hz) → Task 3a
- [x] Issue 4 (ImuData fields) → Task 2b
- [x] Issue 5 (lorawan subscribe) → Task 4a
- [x] Issue 6 (ThreadExecutor usage) → Task 4b
- [x] Issue 7 (nav_service wheel_circumference_m) → Task 3b
- [x] Issue 8 (norm_angle/clamp comments) → Task 3c
- [x] Issue 9 (fault_handler.h merged comment) → Task 5
- [x] Issue 10 (start_returning_no_brush note) → Task 3d
- [x] Issue 11 (P3 enum comment) → Task 3e
- [x] Issue 12 (motion_service.cc garbled) → Task 7a
- [x] Issue 13 (fault_handler.cc P1 comment) → Task 7b
- [x] Issue 14 (libgpiod_pin.h empty @Description) → Task 6

All 14 issues are covered.
