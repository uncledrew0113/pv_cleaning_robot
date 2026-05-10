# Hardware Shutdown And Test Exit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 补齐设备 `close()/析构` 生命周期，收紧执行器停机顺序，完善 `main()` 手动关停链路，并为硬件测试增加统一的可捕获退出清理逻辑。

**Architecture:** 采用方案 2，不引入全局 shutdown 框架。设备层按“执行器”和“采集器”两类收口；主程序保留手动关闭并补齐缺项；硬件测试通过统一退出守卫和夹具 `shutdown()` 走幂等清理路径。

**Tech Stack:** C++17、现有 device/service/app/middleware 模块、Catch2 硬件测试、POSIX `signal`

---

## File Map

**Create:**
- `test/integration/hardware/hw_exit_guard.h`
- `test/integration/hardware/hw_exit_guard.cc`

**Modify:**
- `include/pv_cleaning_robot/device/brush_motor.h`
- `pv_cleaning_robot/device/brush_motor.cc`
- `include/pv_cleaning_robot/device/distance_sensor.h`
- `pv_cleaning_robot/device/distance_sensor.cc`
- `include/pv_cleaning_robot/device/walk_motor_group.h`
- `pv_cleaning_robot/device/walk_motor_group.cc`
- `pv_cleaning_robot/main.cc`
- `test/integration/hardware/hw_config.h`
- `test/integration/hardware/system_hw_test.cc`

**Build verification only:**
- `cmake --build build --target pv_cleaning_robot -j4`
- `cmake --build build --target unit_tests -j4`
- `cmake --build build --target hw_tests -j4`

## Task 1: 补齐 BrushMotor 生命周期

**Files:**
- Modify: `include/pv_cleaning_robot/device/brush_motor.h`
- Modify: `pv_cleaning_robot/device/brush_motor.cc`
- Build: `cmake --build build --target pv_cleaning_robot unit_tests -j4`

- [ ] **Step 1: 在头文件声明析构与 close 接口**

在 `BrushMotor` 类中新增：

```cpp
~BrushMotor();
void close();
```

要求：

- `close()` 公开可调用
- 保持现有 API 不变

- [ ] **Step 2: 实现析构兜底**

在 `brush_motor.cc` 中增加：

```cpp
BrushMotor::~BrushMotor() {
    close();
}
```

- [ ] **Step 3: 实现 close() 的严格语义**

实现规则：

```cpp
void BrushMotor::close() {
    std::lock_guard<hal::PiMutex> guard(mtx_);
    if (!serial_ || !serial_->is_open()) {
        active_control_ = false;
        keepalive_required_ = false;
        target_rpm_ = 0;
        target_torque_nm_ = 0.0f;
        diag_.target_rpm = 0;
        diag_.target_torque_nm = 0.0f;
        update_running_locked();
        return;
    }

    char cmd[kCmdCap];
    size_t len = 0;
    if (control_mode_ == ControlMode::TORQUE) {
        len = protocol::encode_set_torque(axis_, 0.0f, cmd, sizeof(cmd));
    } else {
        len = protocol::encode_set_velocity(axis_, 0.0f, cmd, sizeof(cmd));
    }
    (void)write_ascii_locked(cmd, len);

    target_rpm_ = 0;
    target_torque_nm_ = 0.0f;
    diag_.target_rpm = 0;
    diag_.target_torque_nm = 0.0f;
    active_control_ = false;
    keepalive_required_ = false;
    update_running_locked();

    serial_->close();
}
```

约束：

- 不调用 `enter_idle()`
- `stop()` 语义要保留，但 `close()` 不再通过再次套调公开 `stop()` 来避免重复锁
- 即使发零速命令失败，也继续关闭串口

- [ ] **Step 4: 编译验证**

Run:

```bash
cmake --build build --target pv_cleaning_robot unit_tests -j4
```

Expected:

- 编译通过
- 不执行测试程序

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/device/brush_motor.h pv_cleaning_robot/device/brush_motor.cc
git commit -m "feat: add brush motor close lifecycle"
```

## Task 2: 补齐 DistanceSensor 生命周期并收紧 WalkMotorGroup::close

**Files:**
- Modify: `include/pv_cleaning_robot/device/distance_sensor.h`
- Modify: `pv_cleaning_robot/device/distance_sensor.cc`
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Build: `cmake --build build --target pv_cleaning_robot unit_tests -j4`

- [ ] **Step 1: 为 DistanceSensor 增加析构与 close 声明**

在头文件增加：

```cpp
~DistanceSensor();
void close();
```

- [ ] **Step 2: 实现 DistanceSensor 资源关闭**

在源文件增加：

```cpp
DistanceSensor::~DistanceSensor() {
    close();
}

void DistanceSensor::close() {
    if (modbus_) {
        modbus_->close();
    }
}
```

约束：

- 不增加任何停机语义
- 保持幂等

- [ ] **Step 3: 更新 WalkMotorGroup 头文件注释**

把 `close()` 注释明确成“先安全停机，再停接收线程，再关闭 CAN”。

- [ ] **Step 4: 收紧 WalkMotorGroup::close()**

在 `walk_motor_group.cc` 中按以下顺序实现：

```cpp
void WalkMotorGroup::close() {
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        pid_ctrl_.enable(false);
    }

    if (can_->is_open()) {
        (void)set_speed_uniform(0.0f);
        (void)disable_all();
    }

    running_.store(false);
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (can_->is_open())
        can_->close();
    last_update_time_ = {};
}
```

约束：

- 关停时不抛异常
- 保持幂等
- 停车/失能失败不阻止线程和总线关闭

- [ ] **Step 5: 编译验证**

Run:

```bash
cmake --build build --target pv_cleaning_robot unit_tests -j4
```

Expected:

- 编译通过
- 不执行测试程序

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/device/distance_sensor.h pv_cleaning_robot/device/distance_sensor.cc include/pv_cleaning_robot/device/walk_motor_group.h pv_cleaning_robot/device/walk_motor_group.cc
git commit -m "feat: tighten device shutdown semantics"
```

## Task 3: 补齐 main() 显式关闭顺序

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Build: `cmake --build build --target pv_cleaning_robot -j4`

- [ ] **Step 1: 保持现有信号处理与主循环退出模型**

不要改：

```cpp
static std::atomic<bool> g_running{true};
static void signal_handler(int /*sig*/) {
    g_running.store(false);
}
```

本任务只补齐退出顺序，不重构退出模型。

- [ ] **Step 2: 按设计顺序补全显式关闭**

把退出段改成：

```cpp
log->info("[Main] 收到退出信号，正在关闭...");
walk_exec.stop();
nav_exec.stop();
bms_exec.stop();
cloud_exec.stop();
safety_monitor.stop();
watchdog.stop();

motion->emergency_stop();
brush_motor->close();
walk_group->close();

imu->close();
gps->close();
bms->close();
left_switch->close();
right_switch->close();

net_mgr->disconnect();
data_cache->close();
log->info("[Main] 正常退出");
```

约束：

- 先停线程，再停执行器，再关采集设备
- 不删除现有 `motion->emergency_stop()`
- 不引入新的全局管理器

- [ ] **Step 3: 编译验证**

Run:

```bash
cmake --build build --target pv_cleaning_robot -j4
```

Expected:

- 编译通过

- [ ] **Step 4: Commit**

```bash
git add pv_cleaning_robot/main.cc
git commit -m "feat: complete main shutdown sequence"
```

## Task 4: 为硬件测试增加统一退出守卫

**Files:**
- Create: `test/integration/hardware/hw_exit_guard.h`
- Create: `test/integration/hardware/hw_exit_guard.cc`
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Build: `cmake --build build --target hw_tests -j4`

- [ ] **Step 1: 新增硬件测试退出守卫接口**

在 `hw_exit_guard.h` 中定义最小接口：

```cpp
namespace hw {

struct IGracefulShutdown {
    virtual ~IGracefulShutdown() = default;
    virtual void shutdown() = 0;
};

class HwExitGuard {
public:
    static HwExitGuard& instance();

    void install();
    void set_active(IGracefulShutdown* active);
    void clear_active(IGracefulShutdown* active);
    bool exit_requested() const;
    void request_exit();

private:
    std::atomic<bool> exit_requested_{false};
    std::atomic<IGracefulShutdown*> active_{nullptr};
};

}  // namespace hw
```

- [ ] **Step 2: 实现信号处理只置标志**

在 `hw_exit_guard.cc` 中实现：

```cpp
namespace {
void hw_signal_handler(int) {
    hw::HwExitGuard::instance().request_exit();
}
}

void HwExitGuard::install() {
    std::signal(SIGINT, hw_signal_handler);
    std::signal(SIGTERM, hw_signal_handler);
}
```

约束：

- 信号处理函数中不直接做设备 I/O
- 只置退出标志

- [ ] **Step 3: 让 FullSystemFixture 实现统一 shutdown()**

在 `hw_config.h` 中：

- 让 `FullSystemFixture` 继承 `IGracefulShutdown`
- 新增 `shutdown()` 方法
- 析构函数只调用 `shutdown()`

目标顺序：

```cpp
void shutdown() override {
    stop_loops_();
    if (safety)
        safety->stop();
    if (motion)
        motion->emergency_stop();
    if (brush)
        brush->close();
    if (walk_group)
        walk_group->close();
    if (gps)
        gps->close();
    if (imu)
        imu->close();
    if (bms)
        bms->close();
    if (left_sw)
        left_sw->close();
    if (right_sw)
        right_sw->close();
    if (watchdog)
        watchdog->stop();
}
```

要求：

- 幂等
- 允许部分初始化失败时安全调用

- [ ] **Step 4: 在硬件测试主循环接入退出标志**

在 `run_combined_system_test(...)` 和 `[hw_system][pid_combined]` 的轮询路径中：

- 测试开始时 `HwExitGuard::instance().install()`
- 注册当前活动 fixture
- 在 `poll_once()` / `wait_transition()` 循环中检测：

```cpp
if (hw::HwExitGuard::instance().exit_requested()) {
    f.shutdown();
    FAIL("hardware test interrupted by signal");
}
```

并在退出前清理 active fixture 注册。

- [ ] **Step 5: 编译验证**

Run:

```bash
cmake --build build --target hw_tests -j4
```

Expected:

- 编译通过
- 不执行任何硬件测试程序

- [ ] **Step 6: Commit**

```bash
git add test/integration/hardware/hw_exit_guard.h test/integration/hardware/hw_exit_guard.cc test/integration/hardware/hw_config.h test/integration/hardware/system_hw_test.cc
git commit -m "feat: add graceful exit for hardware tests"
```

## Task 5: 最终全量编译验证与收尾

**Files:**
- Verify only

- [ ] **Step 1: 编译主程序**

Run:

```bash
cmake --build build --target pv_cleaning_robot -j4
```

Expected:

- PASS

- [ ] **Step 2: 编译单元测试目标**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- PASS

- [ ] **Step 3: 编译硬件测试目标**

Run:

```bash
cmake --build build --target hw_tests -j4
```

Expected:

- PASS

- [ ] **Step 4: 记录未执行项**

在最终说明中明确：

- 交叉编译环境下未执行测试程序
- `SIGKILL` / 断电不在优雅退出保证范围内

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: harden hardware shutdown and test exit paths"
```

## Self-Review

- Spec coverage:
  - `BrushMotor close()/析构` -> Task 1
  - `DistanceSensor close()/析构` -> Task 2
  - `WalkMotorGroup close 收紧` -> Task 2
  - `main()` 手动关闭补齐 -> Task 3
  - 硬件测试统一退出守卫 -> Task 4
  - 只做编译验证 -> Task 5
- Placeholder scan:
  - 无 `TODO/TBD`
  - 每个任务都给出具体文件和命令
- Type consistency:
  - `BrushMotor::close()`
  - `DistanceSensor::close()`
  - `FullSystemFixture::shutdown()`
  - `hw::IGracefulShutdown`
  - `hw::HwExitGuard`

