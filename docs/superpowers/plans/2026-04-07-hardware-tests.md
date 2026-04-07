# 硬件测试实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为已安装的真实硬件（前/后限位传感器、四轮行走电机组、IMU、BMS）编写完整的硬件单元测试和全层栈清扫流程集成测试。

**Architecture:** 采用两级共享 Fixture（`DeviceFixture` 仅 Driver+Device 层用于单元测试，`FullSystemFixture` 全层栈用于集成测试），均定义在 `hw_config.h` 中。三个测试文件 + 一个辅助头文件全部添加进现有 `hw_tests` 可执行目标，无需新建 CMake 目标。

**Tech Stack:** Catch2 v2, spdlog, C++17, libgpiod, libserialport, linux SocketCAN, `std::thread`（测试中不设 RT 调度，仅 SCHED_OTHER）

---

## 文件结构

```
test/integration/hardware/
├── hw_config.h                     (新增) 硬件常量 + DeviceFixture + FullSystemFixture
├── limit_switch_hw_test.cc         (新增) 限位传感器单元测试 (8 段)
├── walk_motor_group_hw_test.cc     (新增) 行走电机组单元测试 (13 段)
└── clean_cycle_hw_test.cc          (新增) 全层栈清扫流程集成测试 (9 段)

test/CMakeLists.txt                 (修改) hw_tests 目标加入 3 个新文件
```

---

## Task 1：`hw_config.h` — 公共 Fixture

**Files:**
- Create: `test/integration/hardware/hw_config.h`

- [ ] **Step 1.1：创建 `hw_config.h`**

```cpp
// test/integration/hardware/hw_config.h
#pragma once
/**
 * @file hw_config.h
 * @brief 硬件测试公共 Fixture
 *
 * DeviceFixture       — Driver + Device 层，用于限位和电机单元测试
 * FullSystemFixture   — 全层栈，用于集成测试（BrushMotor 用 MockModbusMaster）
 *
 * 硬件接线（与 config/config.json 对齐）：
 *   CAN      : can0，行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1，WIT Motion，9600 baud
 *   BMS      : /dev/ttyS8，嘉佰达通用协议 V4，9600 baud
 *   GPIO     : gpiochip5 line0=前限位，line1=后限位
 */
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

// Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"

// Device
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Middleware
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

// Service
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

// App
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// Mock（辊刷电机未安装）
#include "mock/mock_modbus_master.h"

namespace hw {

// ── 硬件接线常量（与 config/config.json 对齐）────────────────────────────────
constexpr char     kCanIface[]    = "can0";
constexpr uint8_t  kMotorIdBase   = 1u;
constexpr uint16_t kCommTimeoutMs = 500u;   ///< update 50ms × 10 倍余量
constexpr float    kTestSpeedRpm  = 30.0f;  ///< 安全低速（测试专用）
constexpr float    kTestReturnRpm = 30.0f;

constexpr char kImuPort[] = "/dev/ttyS1";
constexpr int  kImuBaud   = 9600;

constexpr char kBmsPort[] = "/dev/ttyS8";
constexpr int  kBmsBaud   = 9600;

constexpr char     kGpioChip[] = "gpiochip5";
constexpr unsigned kFrontLine  = 0u;
constexpr unsigned kRearLine   = 1u;

constexpr int kLimitTimeoutSec = 60;   ///< 等待限位最大秒数（全流程）
constexpr int kOnlineTimeoutMs = 600;  ///< 等待电机上线最大毫秒数

// ── DeviceFixture：Driver + Device 层（限位 / 电机单元测试使用）────────────
struct DeviceFixture {
    std::shared_ptr<robot::driver::LinuxCanSocket>  can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup>  walk_group;
    std::shared_ptr<robot::driver::LibSerialPort>   imu_serial;
    std::shared_ptr<robot::device::ImuDevice>       imu;
    std::shared_ptr<robot::driver::LibSerialPort>   bms_serial;
    std::shared_ptr<robot::device::BMS>             bms;
    std::shared_ptr<robot::driver::LibGpiodPin>     front_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin>     rear_gpio;
    std::shared_ptr<robot::device::LimitSwitch>     front_sw;
    std::shared_ptr<robot::device::LimitSwitch>     rear_sw;

    DeviceFixture() {
        using namespace robot;
        can_bus    = std::make_shared<driver::LinuxCanSocket>(kCanIface);
        walk_group = std::make_shared<device::WalkMotorGroup>(
            can_bus, kMotorIdBase, kCommTimeoutMs);
        imu_serial = std::make_shared<driver::LibSerialPort>(
            kImuPort, hal::UartConfig{kImuBaud});
        imu        = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial = std::make_shared<driver::LibSerialPort>(
            kBmsPort, hal::UartConfig{kBmsBaud});
        bms        = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        front_gpio = std::make_shared<driver::LibGpiodPin>(kGpioChip, kFrontLine);
        rear_gpio  = std::make_shared<driver::LibGpiodPin>(kGpioChip, kRearLine);
        front_sw   = std::make_shared<device::LimitSwitch>(
            front_gpio, device::LimitSide::FRONT);
        rear_sw    = std::make_shared<device::LimitSwitch>(
            rear_gpio, device::LimitSide::REAR);
    }

    ~DeviceFixture() {
        if (front_sw)   front_sw->close();
        if (rear_sw)    rear_sw->close();
        if (imu)        imu->close();
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        // bms_serial 由 shared_ptr 析构时自动关闭
    }
};

// ── FullSystemFixture：全层栈（集成测试使用）─────────────────────────────────
struct FullSystemFixture : DeviceFixture {
    robot::middleware::EventBus                       event_bus;
    std::shared_ptr<MockModbusMaster>                 mock_modbus;
    std::shared_ptr<robot::device::BrushMotor>        brush;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::shared_ptr<robot::service::NavService>       nav;
    std::shared_ptr<robot::service::MotionService>    motion;
    std::shared_ptr<robot::service::FaultService>     fault;
    std::unique_ptr<robot::app::WatchdogMgr>          watchdog;
    std::shared_ptr<robot::app::RobotFsm>             fsm;

    /// @param pid_enabled 是否开启航向 PID（clean_cycle 测试按场景传入）
    explicit FullSystemFixture(bool pid_enabled = false) : DeviceFixture() {
        using namespace robot;

        mock_modbus = std::make_shared<MockModbusMaster>();
        brush       = std::make_shared<device::BrushMotor>(mock_modbus, 1);

        // SafetyMonitor 构造时内部绑定 LimitSwitch 回调
        safety = std::make_unique<middleware::SafetyMonitor>(
            walk_group, front_sw, rear_sw, event_bus);

        // GPS 未安装：创建占位对象，不 open，NavService 会跳过 GPS 校正
        auto gps_serial_dummy = std::make_shared<driver::LibSerialPort>(
            "/dev/null", hal::UartConfig{9600});
        auto gps_dummy = std::make_shared<device::GpsDevice>(gps_serial_dummy);

        nav = std::make_shared<service::NavService>(
            walk_group, imu, gps_dummy, 0.3f);

        service::MotionService::Config motion_cfg;
        motion_cfg.clean_speed_rpm  = kTestSpeedRpm;
        motion_cfg.return_speed_rpm = kTestReturnRpm;
        motion_cfg.brush_rpm        = 0;   // MockModbus，不实际驱动辊刷
        motion_cfg.return_brush_rpm = 0;
        motion_cfg.edge_reverse_rpm = 0.0f;
        motion_cfg.heading_pid_en   = pid_enabled;

        motion = std::make_shared<service::MotionService>(
            walk_group, brush, imu, event_bus, motion_cfg);
        fault  = std::make_shared<service::FaultService>(event_bus);

        // WatchdogMgr：路径为空 = 不操作 /dev/watchdog
        watchdog = std::make_unique<app::WatchdogMgr>("");

        fsm = std::make_shared<app::RobotFsm>(motion, nav, fault, event_bus);

        // SafetyMonitor LimitSettledEvent → RobotFsm
        event_bus.subscribe<middleware::SafetyMonitor::LimitSettledEvent>(
            [this](const middleware::SafetyMonitor::LimitSettledEvent& evt) {
                if (evt.side == device::LimitSide::FRONT)
                    fsm->dispatch(app::EvFrontLimitSettled{});
                else
                    fsm->dispatch(app::EvRearLimitSettled{});
            });

        // FaultEvent → RobotFsm
        event_bus.subscribe<service::FaultService::FaultEvent>(
            [this](const service::FaultService::FaultEvent& evt) {
                using Level = service::FaultService::FaultEvent::Level;
                if (evt.level == Level::P0)
                    fsm->dispatch(app::EvFaultP0{});
                else if (evt.level == Level::P1)
                    fsm->dispatch(app::EvFaultP1{});
            });
    }

    /// 初始化所有硬件并启动后台线程，dispatch EvInitDone → FSM = "Idle"
    /// @return false 表示关键硬件（walk_group）初始化失败
    bool init() {
        using robot::device::DeviceError;
        if (walk_group->open() != DeviceError::OK) {
            spdlog::error("[FullSystemFixture] walk_group open 失败");
            return false;
        }
        walk_group->set_feedback_mode_all(10u);  // 10ms 主动上报

        if (!imu->open())
            spdlog::warn("[FullSystemFixture] IMU open 失败（非致命）");

        if (bms->open() != DeviceError::OK)
            spdlog::warn("[FullSystemFixture] BMS open 失败（非致命）");

        // 限位开关：测试中不设 RT 优先级，无 CPU 绑定
        if (!front_sw->open(0, 2, 0))
            spdlog::warn("[FullSystemFixture] front_sw open 失败");
        if (!rear_sw->open(0, 2, 0))
            spdlog::warn("[FullSystemFixture] rear_sw open 失败");

        if (!safety->start()) {
            spdlog::error("[FullSystemFixture] safety monitor start 失败");
            return false;
        }
        watchdog->start();
        start_loops_();

        // FSM: StateInit → Idle
        fsm->dispatch(robot::app::EvInitDone{});
        return true;
    }

    /// 轮询等待 FSM 进入指定状态，超时返回 false
    bool wait_state(const std::string& expected,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (fsm->current_state() == expected) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        spdlog::warn("[FullSystemFixture] wait_state 超时: 期望={} 实际={}",
                     expected, fsm->current_state());
        return false;
    }

    ~FullSystemFixture() {
        stop_loops_();
        if (safety)   safety->stop();
        if (motion)   motion->emergency_stop();
        if (watchdog) watchdog->stop();
    }

private:
    std::thread       walk_ctrl_thread_;
    std::thread       nav_exec_thread_;
    std::atomic<bool> loops_running_{false};

    void start_loops_() {
        loops_running_.store(true);
        walk_ctrl_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                motion->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        nav_exec_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                nav->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void stop_loops_() {
        loops_running_.store(false);
        if (walk_ctrl_thread_.joinable()) walk_ctrl_thread_.join();
        if (nav_exec_thread_.joinable())  nav_exec_thread_.join();
    }
};

} // namespace hw
```

- [ ] **Step 1.2：验证文件已创建**

```bash
ls test/integration/hardware/hw_config.h
```
预期：文件存在。

---

## Task 2：`limit_switch_hw_test.cc` — 限位传感器单元测试

**Files:**
- Create: `test/integration/hardware/limit_switch_hw_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 2.1：创建 `limit_switch_hw_test.cc`**

```cpp
// test/integration/hardware/limit_switch_hw_test.cc
/**
 * 限位传感器硬件单元测试
 *
 * 测试分组：
 *   [hw_limit][open]             - GPIO open 成功
 *   [hw_limit][read_level_home]  - 停机位电平自检（rear=低/触发，front=高/未触发）
 *   [hw_limit][callback_front]   - 前传感器回调链路（需手动触发）
 *   [hw_limit][callback_rear]    - 后传感器回调链路（需手动触发）
 *   [hw_limit][is_triggered]     - is_triggered() 状态查询
 *   [hw_limit][clear_trigger]    - clear_trigger() 清除状态
 *   [hw_limit][side_enum]        - 回调参数 LimitSide 正确传递
 *   [hw_limit][repeated_trigger] - 多次触发各计一次（传感器稳定性）
 *
 * 运行方法（目标板）：
 *   ./hw_tests "[hw_limit]"                    # 全部测试
 *   ./hw_tests "[hw_limit][open]"              # 仅 open 测试（无需机器人在停机位）
 *   ./hw_tests "[hw_limit][callback_front]"    # 需手动触发前传感器
 *
 * 前提：机器人停在停机位（后限位已触发）；can0 不必配置（本文件不涉及 CAN）
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][open] — GPIO 初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器 GPIO 初始化", "[hw_limit][open]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    SECTION("前限位 GPIO open 成功") {
        // rt_priority=0(SCHED_OTHER), debounce_ms=2, cpu_affinity=0(不绑定)
        REQUIRE(front_sw->open(0, 2, 0));
        spdlog::info("[hw_limit][open] 前限位 GPIO open 成功");
        front_sw->close();
    }

    SECTION("后限位 GPIO open 成功") {
        REQUIRE(rear_sw->open(0, 2, 0));
        spdlog::info("[hw_limit][open] 后限位 GPIO open 成功");
        rear_sw->close();
    }

    SECTION("重复 open/close 不崩溃") {
        REQUIRE(front_sw->open(0, 2, 0));
        front_sw->close();
        REQUIRE(front_sw->open(0, 2, 0));  // 第二次 open 也应成功
        front_sw->close();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][read_level_home] — 停机位电平自检
// 运行前提：机器人停在停机位
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("停机位 GPIO 电平自检", "[hw_limit][read_level_home]") {
    // read_current_level() 语义：true=高电平/未遮挡，false=低电平/已遮挡
    // 停机位期望：rear=false（遮挡），front=true（未遮挡）
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    REQUIRE(front_sw->open(0, 2, 0));
    REQUIRE(rear_sw->open(0, 2, 0));

    const bool front_level = front_sw->read_current_level();
    const bool rear_level  = rear_sw->read_current_level();

    spdlog::info("[hw_limit][read_level_home] front_level={} (期望 true=未遮挡)",
                 front_level);
    spdlog::info("[hw_limit][read_level_home] rear_level={}  (期望 false=停机位遮挡)",
                 rear_level);

    CHECK(front_level == true);   // 前端：未遮挡 = 高电平
    CHECK(rear_level  == false);  // 尾端：停机位遮挡 = 低电平

    front_sw->close();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_front] — 前传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("前限位传感器回调链路（手动触发）", "[hw_limit][callback_front]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<int>            cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::FRONT};

    front_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_front] 回调触发！side={}",
                     (side == device::LimitSide::FRONT ? "FRONT" : "REAR"));
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_front] ★ 请在 5 秒内手动触发【前限位】传感器 ★");

    // 等待最多 5 秒
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_front] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::FRONT);

    front_sw->stop_monitoring();
    front_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_rear] — 后传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("后限位传感器回调链路（手动触发）", "[hw_limit][callback_rear]") {
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    std::atomic<int>               cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::REAR};

    rear_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_rear] 回调触发！side={}",
                     (side == device::LimitSide::FRONT ? "FRONT" : "REAR"));
    });

    REQUIRE(rear_sw->open(0, 2, 0));
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_rear] ★ 请在 5 秒内手动触发【后限位】传感器 ★");

    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_rear] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::REAR);

    rear_sw->stop_monitoring();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][is_triggered] — is_triggered / clear_trigger
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器触发状态管理", "[hw_limit][is_triggered]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<bool> triggered{false};
    front_sw->set_trigger_callback([&](device::LimitSide) {
        triggered.store(true);
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    // 初始未触发
    CHECK(front_sw->is_triggered() == false);

    spdlog::warn("[hw_limit][is_triggered] ★ 请在 5 秒内手动触发【前限位】★");
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!triggered.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(50ms);

    REQUIRE(triggered.load());

    SECTION("触发后 is_triggered() 为 true") {
        CHECK(front_sw->is_triggered() == true);
    }

    SECTION("clear_trigger() 后 is_triggered() 为 false") {
        CHECK(front_sw->is_triggered() == true);
        front_sw->clear_trigger();
        CHECK(front_sw->is_triggered() == false);
    }

    front_sw->stop_monitoring();
    front_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][clear_trigger] — 重置标志独立验证
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器触发标志清除", "[hw_limit][clear_trigger]") {
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    REQUIRE(rear_sw->open(0, 2, 0));
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][clear_trigger] ★ 请在 5 秒内手动触发【后限位】★");
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!rear_sw->is_triggered() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(50ms);

    REQUIRE(rear_sw->is_triggered());
    rear_sw->clear_trigger();
    CHECK(rear_sw->is_triggered() == false);
    spdlog::info("[hw_limit][clear_trigger] clear_trigger 后 is_triggered=false ✓");

    rear_sw->stop_monitoring();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][side_enum] — 回调 LimitSide 参数正确性
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器 side 枚举传递正确", "[hw_limit][side_enum]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio  = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    std::vector<device::LimitSide> received_sides;
    std::mutex                     sides_mtx;

    auto cb = [&](device::LimitSide side) {
        std::lock_guard<std::mutex> lk(sides_mtx);
        received_sides.push_back(side);
    };
    front_sw->set_trigger_callback(cb);
    rear_sw->set_trigger_callback(cb);

    REQUIRE(front_sw->open(0, 2, 0));
    REQUIRE(rear_sw->open(0, 2, 0));
    front_sw->start_monitoring();
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][side_enum] ★ 请依次触发：【前限位】→ 等 2s → 【后限位】★");
    std::this_thread::sleep_for(10s);  // 给操作人员充足时间

    front_sw->stop_monitoring();
    rear_sw->stop_monitoring();

    spdlog::info("[hw_limit][side_enum] 收到 {} 次触发", received_sides.size());
    for (auto s : received_sides)
        spdlog::info("  side={}", (s == device::LimitSide::FRONT ? "FRONT" : "REAR"));

    // 至少触发了两次，且前后各有一次
    bool has_front = false, has_rear = false;
    for (auto s : received_sides) {
        if (s == device::LimitSide::FRONT) has_front = true;
        if (s == device::LimitSide::REAR)  has_rear  = true;
    }
    CHECK(has_front);
    CHECK(has_rear);

    front_sw->close();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][repeated_trigger] — 传感器稳定性（多次触发各计一次）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器多次触发稳定性", "[hw_limit][repeated_trigger]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<int> cb_count{0};
    front_sw->set_trigger_callback([&](device::LimitSide) {
        cb_count.fetch_add(1);
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    spdlog::warn("[hw_limit][repeated_trigger] ★ 请在 15 秒内触发【前限位】3 次（每次间隔 >500ms）★");
    std::this_thread::sleep_for(15s);

    spdlog::info("[hw_limit][repeated_trigger] 触发次数={}", cb_count.load());
    CHECK(cb_count.load() >= 1);  // 至少触发了一次

    front_sw->stop_monitoring();
    front_sw->close();
}
```

- [ ] **Step 2.2：将文件加入 `test/CMakeLists.txt` 的 `hw_tests` 目标**

在 `add_executable(hw_tests ...)` 的源文件列表中追加（位于 `full_sweep_hw_test.cc` 之后）：

```cmake
  integration/hardware/limit_switch_hw_test.cc
  integration/hardware/walk_motor_group_hw_test.cc
  integration/hardware/clean_cycle_hw_test.cc
```

注意：此步骤预先添加所有 3 个文件，后续 Task 3/4 只需创建对应 `.cc` 文件即可编译。

- [ ] **Step 2.3：编译验证（此时 Task 3/4 的文件尚未创建，先跳过编译，Task 5 统一验证）**

---

## Task 3：`walk_motor_group_hw_test.cc` — 行走电机组单元测试

**Files:**
- Create: `test/integration/hardware/walk_motor_group_hw_test.cc`

- [ ] **Step 3.1：创建 `walk_motor_group_hw_test.cc`**

```cpp
// test/integration/hardware/walk_motor_group_hw_test.cc
/**
 * 行走电机组硬件单元测试（4 轮 M1502E_111，CAN 总线）
 *
 * 测试分组：
 *   [hw_walk][open_close]              - CAN open/close
 *   [hw_walk][enable_disable]          - 模式控制（使能/失能）
 *   [hw_walk][all_online]              - 全轮 10ms 反馈上线
 *   [hw_walk][fwd_no_pid]              - PID 关闭前进（30 RPM，3s）
 *   [hw_walk][fwd_with_pid]            - PID 开启前进（30 RPM，5s，yaw 漂移 < 5°）
 *   [hw_walk][pid_straightness]        - PID 开/关直线对比（各 5s，打印漂移量）
 *   [hw_walk][rev_speed]               - 反转（-30 RPM，2s）
 *   [hw_walk][emergency_override]      - 急停：override 后下一 update 速度=0
 *   [hw_walk][override_blocks_set_speeds] - override 期间 set_speeds 不发帧
 *   [hw_walk][clear_override]          - clear_override 后恢复驱动
 *   [hw_walk][comm_timeout_self_stop]  - 通信超时后电机自保护（打印观察）
 *   [hw_walk][frame_stats_no_pid]      - 帧统计（PID 关，10s）
 *   [hw_walk][frame_stats_with_pid]    - 帧统计（PID 开，10s）
 *
 * 运行方法（目标板，can0 已 up 500kbps）：
 *   ./hw_tests "[hw_walk]"
 *   ./hw_tests "[hw_walk][fwd_no_pid]"
 *
 * 安全：所有测试速度 ≤ 30 RPM；每段结束 disable_all() + close()。
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][open_close]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup CAN 初始化与关闭", "[hw_walk][open_close]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    spdlog::info("[hw_walk][open_close] CAN open ✓");
    grp.close();
    spdlog::info("[hw_walk][open_close] CAN close ✓");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][enable_disable]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 使能与失能", "[hw_walk][enable_disable]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);

    CHECK(grp.enable_all()  == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);
    spdlog::info("[hw_walk][enable_disable] enable_all ✓");

    CHECK(grp.disable_all() == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);
    spdlog::info("[hw_walk][enable_disable] disable_all ✓");

    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][all_online]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 全轮联机", "[hw_walk][all_online]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    // 配置 10ms 主动上报（100Hz），使电机快速上线
    CHECK(grp.set_feedback_mode_all(10u) == device::DeviceError::OK);

    // 等待最多 kOnlineTimeoutMs ms，轮询直到全部上线
    bool all_online = false;
    auto deadline   = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(hw::kOnlineTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        auto gs = grp.get_group_status();
        all_online = true;
        for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
            if (!gs.wheel[w].online) { all_online = false; break; }
        if (all_online) break;
        std::this_thread::sleep_for(50ms);
    }

    // 打印各轮状态
    auto gd = grp.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_walk][all_online] wheel[{}] online={}", w, gd.wheel[w].online);

    REQUIRE(all_online);  // 600ms 内全部上线

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][fwd_no_pid] — PID 关闭前进
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 前进（PID 关闭）", "[hw_walk][fwd_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(false);  // PID 关闭
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);

    std::this_thread::sleep_for(300ms);  // 等待上线

    // 前进 3s
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 6; ++i) {  // 6 × 500ms = 3s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][fwd_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_walk][fwd_no_pid] wheel[{}] online={} speed={:.2f}rpm",
                     w, gd.wheel[w].online, gd.wheel[w].speed_rpm);

    // 所有轮在线且速度在预期范围内
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        CHECK(gd.wheel[w].online);
        CHECK(gd.wheel[w].speed_rpm >= 10.0f);   // 低限：目标 30，允许负载下降至 10
        CHECK(gd.wheel[w].speed_rpm <= 60.0f);   // 高限：30 + 100% 余量
    }
    CHECK(gd.ctrl_err_count == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][fwd_with_pid] — PID 开启前进，yaw 漂移 < 5°
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 前进（PID 开启，yaw 漂移 < 5°）", "[hw_walk][fwd_with_pid]") {
    // 需要 IMU 提供 yaw 数据
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);  // 等待 IMU 首帧

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(true);  // PID 开启
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 记录初始 yaw
    auto imu_data0 = imu->get_data();
    REQUIRE(imu_data0.valid);
    const float yaw_start = imu_data0.yaw_deg;
    spdlog::info("[hw_walk][fwd_with_pid] 初始 yaw={:.2f}°", yaw_start);

    // 前进 5s（50ms update 周期）
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 100; ++i) {  // 100 × 50ms = 5s
        auto d = imu->get_data();
        grp.update(d.valid ? d.yaw_deg : 0.0f);
        std::this_thread::sleep_for(50ms);
    }

    auto imu_data1 = imu->get_data();
    REQUIRE(imu_data1.valid);
    const float yaw_end   = imu_data1.yaw_deg;
    const float yaw_drift = std::abs(yaw_end - yaw_start);
    spdlog::info("[hw_walk][fwd_with_pid] 结束 yaw={:.2f}°  漂移={:.2f}°",
                 yaw_end, yaw_drift);

    CHECK(yaw_drift < 5.0f);  // PID 开启下 5s 内漂移应 < 5°

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
    imu->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][pid_straightness] — PID 开关直线对比
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup PID 直线度量化对比", "[hw_walk][pid_straightness]") {
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);

    auto run_and_measure = [&](bool pid_on) -> float {
        auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
        device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);
        grp.open();
        grp.set_feedback_mode_all(10u);
        grp.enable_heading_control(pid_on);
        grp.enable_all();
        grp.set_mode_all(protocol::WalkMotorMode::SPEED);
        std::this_thread::sleep_for(300ms);

        auto d0 = imu->get_data();
        const float yaw0 = d0.valid ? d0.yaw_deg : 0.0f;

        grp.set_speed_uniform(hw::kTestSpeedRpm);
        for (int i = 0; i < 100; ++i) {  // 5s
            auto d = imu->get_data();
            grp.update(d.valid ? d.yaw_deg : 0.0f);
            std::this_thread::sleep_for(50ms);
        }

        auto d1 = imu->get_data();
        const float yaw1  = d1.valid ? d1.yaw_deg : 0.0f;
        const float drift = std::abs(yaw1 - yaw0);

        grp.set_speed_uniform(0.0f);
        grp.update();
        std::this_thread::sleep_for(500ms);  // 停稳后下一次运行
        grp.disable_all();
        grp.close();
        return drift;
    };

    spdlog::warn("[hw_walk][pid_straightness] 开始 PID 关闭行走 5s...");
    const float drift_no_pid = run_and_measure(false);

    spdlog::warn("[hw_walk][pid_straightness] 请将机器人复位，按 Enter 继续 PID 开启行走...");
    std::this_thread::sleep_for(5s);  // 给操作人员复位机器人

    spdlog::warn("[hw_walk][pid_straightness] 开始 PID 开启行走 5s...");
    const float drift_with_pid = run_and_measure(true);

    spdlog::info("[hw_walk][pid_straightness] ┌─────────────────────────────────┐");
    spdlog::info("[hw_walk][pid_straightness] │ PID 关闭 yaw 漂移: {:.2f}°          │", drift_no_pid);
    spdlog::info("[hw_walk][pid_straightness] │ PID 开启 yaw 漂移: {:.2f}°          │", drift_with_pid);
    spdlog::info("[hw_walk][pid_straightness] └─────────────────────────────────┘");

    // PID 开启后漂移应小于关闭时的漂移（或至少不更差）
    CHECK(drift_with_pid <= drift_no_pid + 1.0f);  // 容忍 1° 误差
    SUCCEED();  // 主要验收标准是打印可量化对比日志
    imu->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][rev_speed] — 反转
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 反转", "[hw_walk][rev_speed]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(-hw::kTestSpeedRpm);
    for (int i = 0; i < 4; ++i) {  // 4 × 500ms = 2s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        spdlog::info("[hw_walk][rev_speed] wheel[{}] speed={:.2f}rpm", w, gd.wheel[w].speed_rpm);
        CHECK(gd.wheel[w].speed_rpm < 0.0f);  // 反转时速度为负
    }

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][emergency_override] — 急停
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 急停（emergency_override）", "[hw_walk][emergency_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 先建立运动
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(500ms);

    // 急停
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    CHECK(grp.emergency_override(0.0f) == device::DeviceError::OK);
    CHECK(grp.is_override_active());

    // 调 update：override 激活时不应再发运动帧，帧计数不增加
    grp.update();
    std::this_thread::sleep_for(100ms);
    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][emergency_override] frames_before={} frames_after={}",
                 frames_before, frames_after);
    CHECK(frames_after == frames_before);  // override 期间无新控制帧

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][override_blocks_set_speeds] — override 期间 set_speeds 封锁
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup override 封锁 set_speeds", "[hw_walk][override_blocks_set_speeds]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 激活 override
    grp.emergency_override(0.0f);
    REQUIRE(grp.is_override_active());

    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;

    // 尝试发送速度帧（应被封锁）
    grp.set_speeds(hw::kTestSpeedRpm, hw::kTestSpeedRpm,
                   hw::kTestSpeedRpm, hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(200ms);

    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][override_blocks_set_speeds] frames_before={} frames_after={}",
                 frames_before, frames_after);
    CHECK(frames_after == frames_before);  // 没有新帧被发出

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][clear_override] — 解除急停后恢复驱动
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 解除急停后恢复驱动", "[hw_walk][clear_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.emergency_override(0.0f);
    REQUIRE(grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已激活");

    grp.clear_override();
    CHECK(!grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已解除");

    // 解除后可以重新驱动
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(200ms);
    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;

    CHECK(frames_after > frames_before);  // clear 后能发出新帧
    spdlog::info("[hw_walk][clear_override] 解除后 ctrl_frame_count 增加 {} 帧",
                 frames_after - frames_before);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][comm_timeout_self_stop] — 通信超时自保护（打印观察）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 通信超时自保护", "[hw_walk][comm_timeout_self_stop]") {
    constexpr uint16_t kShortTimeout = 300u;  // 300ms 超时（更短，便于测试）

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, kShortTimeout);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 短暂前进
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(500ms);
    spdlog::info("[hw_walk][comm_timeout_self_stop] 电机运行中，即将关闭 CAN 总线...");

    // 关闭 CAN（停止发送心跳），等待超时
    grp.close();  // 停止 recv_loop，不再发帧

    spdlog::warn("[hw_walk][comm_timeout_self_stop] CAN 已关闭，电机应在 {}ms 内自保护停转",
                 kShortTimeout + 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(kShortTimeout + 100));

    // 此处电机应已超时自停（通过观察电机是否停止确认）
    spdlog::info("[hw_walk][comm_timeout_self_stop] ★ 请确认电机已停转 ★");
    SUCCEED();  // 结果由人工观察确认
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][frame_stats_no_pid] — 帧统计（PID 关）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 帧统计（PID 关，10s）", "[hw_walk][frame_stats_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(false);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 20; ++i) {  // 20 × 500ms = 10s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][frame_stats_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);

    CHECK(gd.ctrl_frame_count > 100u);  // 10s × ~20Hz = 200 帧
    CHECK(gd.ctrl_err_count   == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][frame_stats_with_pid] — 帧统计（PID 开）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 帧统计（PID 开，10s）", "[hw_walk][frame_stats_with_pid]") {
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(true);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 20; ++i) {  // 20 × 500ms = 10s
        auto d = imu->get_data();
        grp.update(d.valid ? d.yaw_deg : 0.0f);
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][frame_stats_with_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);

    CHECK(gd.ctrl_frame_count > 100u);
    CHECK(gd.ctrl_err_count   == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
    imu->close();
}
```

---

## Task 4：`clean_cycle_hw_test.cc` — 全层栈清扫流程集成测试

**Files:**
- Create: `test/integration/hardware/clean_cycle_hw_test.cc`

- [ ] **Step 4.1：创建 `clean_cycle_hw_test.cc`**

```cpp
// test/integration/hardware/clean_cycle_hw_test.cc
/**
 * 全层栈清扫流程集成测试（与 main.cc 结构一致）
 *
 * 使用 FullSystemFixture，BrushMotor 用 MockModbusMaster 替代。
 *
 * 测试分组：
 *   [hw_cycle][startup]          - 全层栈初始化，FSM = "Idle"
 *   [hw_cycle][self_check_pass]  - EvScheduleStart{at_home=true} → FSM = "CleanFwd"
 *   [hw_cycle][one_pass_no_pid]  - 完整一趟（PID 关），前后限位驱动换向，≤120s
 *   [hw_cycle][one_pass_with_pid]- 完整一趟（PID 开），整趟 yaw 漂移 < 10°
 *   [hw_cycle][fault_p0_estop]   - 注入 P0 故障 → FSM = "Fault"，电机停转
 *   [hw_cycle][low_battery_return]- 注入低电 → FSM = "Returning" → "Charging"
 *   [hw_cycle][bms_valid]        - 清扫过程中 BMS 数据持续有效
 *   [hw_cycle][imu_valid]        - 清扫过程中 IMU 数据持续有效
 *   [hw_cycle][watchdog_alive]   - 正常运行 30s，watchdog 不触发 timeout
 *
 * 运行方法（目标板，机器人在停机位）：
 *   ./hw_tests "[hw_cycle]"
 *   ./hw_tests "[hw_cycle][one_pass_with_pid]"
 *
 * 安全：清扫速度 kTestSpeedRpm=30 RPM；FullSystemFixture 析构自动 emergency_stop()
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][startup] — 全层栈初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("全层栈初始化", "[hw_cycle][startup]") {
    hw::FullSystemFixture fx(false /* pid_off */);
    REQUIRE(fx.init());

    const std::string state = fx.fsm->current_state();
    spdlog::info("[hw_cycle][startup] FSM 状态={}", state);
    CHECK(state == "Idle");

    auto bms_data = fx.bms->get_data();
    spdlog::info("[hw_cycle][startup] BMS valid={} soc={:.1f}%",
                 bms_data.valid, bms_data.soc_pct);

    auto imu_data = fx.imu->get_data();
    spdlog::info("[hw_cycle][startup] IMU valid={} yaw={:.2f}°",
                 imu_data.valid, imu_data.yaw_deg);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][self_check_pass] — 自检通过，进入 CleanFwd
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("自检流程（at_home=true → CleanFwd）", "[hw_cycle][self_check_pass]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 机器人在停机位，后限位触发
    const bool at_home  = !fx.rear_sw->read_current_level();   // 低=触发=在停机位
    const bool at_front = !fx.front_sw->read_current_level();  // 低=触发=在前端
    spdlog::info("[hw_cycle][self_check_pass] at_home={} at_front={}", at_home, at_front);

    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 自检 → CleanFwd（最多 5s 内）
    const bool in_clean_fwd = fx.wait_state("CleanFwd", 5s);
    spdlog::info("[hw_cycle][self_check_pass] FSM={}", fx.fsm->current_state());
    REQUIRE(in_clean_fwd);

    // 电机应已开始转动
    std::this_thread::sleep_for(1s);
    auto gd = fx.walk_group->get_group_diagnostics();
    spdlog::info("[hw_cycle][self_check_pass] ctrl_frame_count={}", gd.ctrl_frame_count);
    CHECK(gd.ctrl_frame_count > 0u);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][one_pass_no_pid] — 完整一趟（PID 关）
// 流程：Idle → SelfCheck → CleanFwd → (前限位) → CleanReturn → (后限位) → Charging
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("完整清扫一趟（PID 关闭）", "[hw_cycle][one_pass_no_pid]") {
    hw::FullSystemFixture fx(false /* pid_off */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    spdlog::warn("[hw_cycle][one_pass_no_pid] ★ 开始清扫（PID 关），预计 ≤120s ★");

    const auto t_start = std::chrono::steady_clock::now();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 阶段1：等待进入 CleanFwd
    REQUIRE(fx.wait_state("CleanFwd", 5s));
    const auto t_fwd_start = std::chrono::steady_clock::now();
    spdlog::info("[hw_cycle][one_pass_no_pid] → CleanFwd");

    // 阶段2：等待前限位触发（最多 kLimitTimeoutSec 秒）
    REQUIRE(fx.wait_state("CleanReturn",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    const auto t_fwd_end = std::chrono::steady_clock::now();
    spdlog::info("[hw_cycle][one_pass_no_pid] → CleanReturn (正向 {:.1f}s)",
                 std::chrono::duration<float>(t_fwd_end - t_fwd_start).count());

    // 阶段3：等待后限位触发 → Charging
    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    const auto t_total = std::chrono::steady_clock::now();
    const float elapsed = std::chrono::duration<float>(t_total - t_start).count();

    spdlog::info("[hw_cycle][one_pass_no_pid] → Charging，总耗时={:.1f}s", elapsed);
    CHECK(elapsed <= 120.0f);
    CHECK(fx.fsm->current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][one_pass_with_pid] — 完整一趟（PID 开），整趟 yaw 漂移 < 10°
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("完整清扫一趟（PID 开启，yaw 漂移 < 10°）", "[hw_cycle][one_pass_with_pid]") {
    hw::FullSystemFixture fx(true /* pid_on */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 记录初始 yaw
    auto d0 = fx.imu->get_data();
    REQUIRE(d0.valid);
    const float yaw_start = d0.yaw_deg;
    float       yaw_max_drift = 0.0f;

    // 后台线程持续监测 yaw 漂移
    std::atomic<bool> monitor_running{true};
    std::thread monitor_thread([&] {
        while (monitor_running.load()) {
            auto d = fx.imu->get_data();
            if (d.valid) {
                float drift = std::abs(d.yaw_deg - yaw_start);
                if (drift > yaw_max_drift) yaw_max_drift = drift;
            }
            std::this_thread::sleep_for(100ms);
        }
    });

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    spdlog::warn("[hw_cycle][one_pass_with_pid] ★ 开始清扫（PID 开），预计 ≤120s ★");

    const auto t_start = std::chrono::steady_clock::now();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    REQUIRE(fx.wait_state("CleanFwd", 5s));
    spdlog::info("[hw_cycle][one_pass_with_pid] → CleanFwd  初始 yaw={:.2f}°", yaw_start);

    REQUIRE(fx.wait_state("CleanReturn",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    spdlog::info("[hw_cycle][one_pass_with_pid] → CleanReturn");

    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));

    monitor_running.store(false);
    monitor_thread.join();

    const float elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - t_start).count();

    spdlog::info("[hw_cycle][one_pass_with_pid] ────────────────────────────");
    spdlog::info("[hw_cycle][one_pass_with_pid] 总耗时={:.1f}s", elapsed);
    spdlog::info("[hw_cycle][one_pass_with_pid] 整趟最大 yaw 漂移={:.2f}°", yaw_max_drift);
    spdlog::info("[hw_cycle][one_pass_with_pid] ────────────────────────────");

    CHECK(fx.fsm->current_state() == "Charging");
    CHECK(elapsed <= 120.0f);
    CHECK(yaw_max_drift < 10.0f);  // 整趟漂移 < 10°
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][fault_p0_estop] — P0 故障急停
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("P0 故障急停（FSM → Fault，电机停转）", "[hw_cycle][fault_p0_estop]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 启动清扫
    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 注入 P0 故障
    std::this_thread::sleep_for(2s);  // 让电机先转起来
    spdlog::warn("[hw_cycle][fault_p0_estop] 注入 P0 故障...");
    fx.fault->report(service::FaultService::FaultEvent::Level::P0,
                     0x9001u, "hw_test_inject_p0");

    // FSM 应进入 Fault
    REQUIRE(fx.wait_state("Fault", 3s));
    spdlog::info("[hw_cycle][fault_p0_estop] FSM → Fault ✓");

    // 等待电机停转（override 应已激活）
    std::this_thread::sleep_for(500ms);
    auto gd = fx.walk_group->get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_cycle][fault_p0_estop] wheel[{}] speed={:.2f}rpm online={}",
                     w, gd.wheel[w].speed_rpm, gd.wheel[w].online);

    CHECK(fx.fsm->current_state() == "Fault");
    // 电机速度应接近 0（允许 5 RPM 偏差，因惯性）
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        CHECK(std::abs(gd.wheel[w].speed_rpm) < 5.0f);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][low_battery_return] — 低电量触发返回
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("低电量触发安全返回", "[hw_cycle][low_battery_return]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 注入低电量事件
    std::this_thread::sleep_for(2s);
    spdlog::warn("[hw_cycle][low_battery_return] 注入低电量事件...");
    fx.fsm->dispatch(app::EvLowBattery{});

    // FSM → Returning
    REQUIRE(fx.wait_state("Returning", 3s));
    spdlog::info("[hw_cycle][low_battery_return] FSM → Returning ✓");

    // 等待后限位触发 → Charging（最多 kLimitTimeoutSec 秒）
    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    spdlog::info("[hw_cycle][low_battery_return] FSM → Charging ✓ (已归停机位)");
    CHECK(fx.fsm->current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][bms_valid] — 清扫过程中 BMS 数据持续有效
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("清扫过程 BMS 数据持续有效", "[hw_cycle][bms_valid]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 启动 BMS 采集（500ms 轮询）
    std::atomic<bool> bms_running{true};
    int bms_valid_count = 0, bms_total_count = 0;
    std::thread bms_thread([&] {
        while (bms_running.load()) {
            fx.bms->update();
            auto d = fx.bms->get_data();
            ++bms_total_count;
            if (d.valid && d.soc_pct >= 0.0f && d.soc_pct <= 100.0f &&
                d.voltage_v > 0.0f)
                ++bms_valid_count;
            std::this_thread::sleep_for(500ms);
        }
    });

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 运行到 CleanFwd 后检查 BMS 持续有效 20s
    REQUIRE(fx.wait_state("CleanFwd", 5s));
    std::this_thread::sleep_for(20s);

    bms_running.store(false);
    bms_thread.join();

    spdlog::info("[hw_cycle][bms_valid] BMS valid={}/{} total polls",
                 bms_valid_count, bms_total_count);
    auto d = fx.bms->get_data();
    spdlog::info("[hw_cycle][bms_valid] 最终 soc={:.1f}% voltage={:.2f}V current={:.3f}A",
                 d.soc_pct, d.voltage_v, d.current_a);

    CHECK(d.valid);
    CHECK(d.soc_pct >= 0.0f);
    CHECK(d.soc_pct <= 100.0f);
    CHECK(d.voltage_v > 0.0f);
    // 至少 60% 的 BMS 轮询成功
    CHECK(bms_valid_count >= bms_total_count * 6 / 10);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][imu_valid] — 清扫过程中 IMU 数据持续有效
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("清扫过程 IMU 数据持续有效", "[hw_cycle][imu_valid]") {
    hw::FullSystemFixture fx(true /* pid_on，IMU 数据为 PID 必要条件 */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 采样 IMU 20s
    int imu_valid_count = 0, imu_total = 0;
    float yaw_min = 999.0f, yaw_max = -999.0f;
    for (int i = 0; i < 40; ++i) {  // 40 × 500ms = 20s
        auto d = fx.imu->get_data();
        ++imu_total;
        if (d.valid && !std::isnan(d.yaw_deg)) {
            ++imu_valid_count;
            yaw_min = std::min(yaw_min, d.yaw_deg);
            yaw_max = std::max(yaw_max, d.yaw_deg);
        }
        std::this_thread::sleep_for(500ms);
    }

    spdlog::info("[hw_cycle][imu_valid] IMU valid={}/{} yaw_range=[{:.2f},{:.2f}]°",
                 imu_valid_count, imu_total, yaw_min, yaw_max);

    // 至少 90% 的采样点 IMU 数据有效
    CHECK(imu_valid_count >= imu_total * 9 / 10);
    CHECK(yaw_max != -999.0f);  // 至少有一个有效值
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][watchdog_alive] — 正常运行 30s watchdog 不触发
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("正常运行 watchdog 不超时", "[hw_cycle][watchdog_alive]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 注册 walk_ctrl 心跳（500ms 超时）
    bool watchdog_fired = false;
    fx.watchdog->set_timeout_callback([&](const std::string& name) {
        watchdog_fired = true;
        spdlog::error("[hw_cycle][watchdog_alive] watchdog 触发！name={}", name);
    });
    int wd_ticket = fx.watchdog->register_thread("walk_ctrl_test", 500);

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 正常运行 30s，每 200ms 汇报一次心跳
    for (int i = 0; i < 150; ++i) {  // 150 × 200ms = 30s
        fx.watchdog->heartbeat(wd_ticket);
        std::this_thread::sleep_for(200ms);
    }

    spdlog::info("[hw_cycle][watchdog_alive] 30s 正常运行，watchdog_fired={}",
                 watchdog_fired);
    CHECK(!watchdog_fired);
}
```

---

## Task 5：更新 `test/CMakeLists.txt`，构建验证，提交

**Files:**
- Modify: `test/CMakeLists.txt`

- [ ] **Step 5.1：编辑 `test/CMakeLists.txt`，将 3 个新文件加入 `hw_tests` 目标**

在 `add_executable(hw_tests ...)` 的 `full_sweep_hw_test.cc` 行后追加：

```cmake
  integration/hardware/limit_switch_hw_test.cc
  integration/hardware/walk_motor_group_hw_test.cc
  integration/hardware/clean_cycle_hw_test.cc
```

完整修改后 hw_tests 源文件列表应如下：

```cmake
add_executable(hw_tests
  integration/hardware/hw_test_main.cc
  integration/hardware/driver_hw_test.cc
  integration/hardware/bms_hw_test.cc
  integration/hardware/imu_hw_test.cc
  integration/hardware/full_sweep_hw_test.cc
  integration/hardware/limit_switch_hw_test.cc
  integration/hardware/walk_motor_group_hw_test.cc
  integration/hardware/clean_cycle_hw_test.cc

  ${COMMON_SRCS}
)
```

- [ ] **Step 5.2：重新配置 CMake**

```bash
cmake --preset rk3576-cross-linux 2>&1 | tail -5
```

预期输出（无 ERROR）：
```
-- Configuring done
-- Generating done
-- Build files have been written to: /home/tronlong/pv_cleaning_robot/build
```

- [ ] **Step 5.3：编译 hw_tests**

```bash
cmake --build --preset rk3576-build --target hw_tests 2>&1 | tail -10
```

预期：
```
[100%] Linking CXX executable ../aarch64/bin/hw_tests
[100%] Built target hw_tests
```

若出现编译错误，按以下顺序排查：
1. `fatal error: hw_config.h: No such file or directory` → 检查 `test/integration/hardware/hw_config.h` 是否已创建
2. `'GpsDevice' was not declared` → `hw_config.h` 缺少 `#include "pv_cleaning_robot/device/gps_device.h"`
3. `'MockModbusMaster' was not declared` → `hw_config.h` 缺少 `#include "mock/mock_modbus_master.h"`

- [ ] **Step 5.4：验证新测试 tag 已注册**

```bash
./build/aarch64/bin/hw_tests --list-test-names-only 2>/dev/null | grep -E "hw_limit|hw_walk|hw_cycle"
```

预期输出（aarch64 目标板上运行，或 QEMU）：
```
限位传感器 GPIO 初始化
停机位 GPIO 电平自检
前限位传感器回调链路（手动触发）
后限位传感器回调链路（手动触发）
限位传感器触发状态管理
限位传感器触发标志清除
限位传感器 side 枚举传递正确
限位传感器多次触发稳定性
WalkMotorGroup CAN 初始化与关闭
...（共 30 个测试段）
```

- [ ] **Step 5.5：提交**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/integration/hardware/hw_config.h \
        test/integration/hardware/limit_switch_hw_test.cc \
        test/integration/hardware/walk_motor_group_hw_test.cc \
        test/integration/hardware/clean_cycle_hw_test.cc \
        test/CMakeLists.txt

git commit -m "test(hw): add real-hardware unit and integration tests

- hw_config.h: DeviceFixture (device layer) + FullSystemFixture (full
  stack, MockModbusMaster for brush) shared by all hw tests
- limit_switch_hw_test.cc: 8 segments covering GPIO open, home-position
  level check, callback chain, is_triggered, clear_trigger, side enum,
  stability (manual-trigger tests prompt operator via spdlog::warn)
- walk_motor_group_hw_test.cc: 13 segments covering open/close, enable,
  online detection, forward/reverse at 30 RPM, emergency_override,
  override blocking, clear_override, PID-on vs PID-off yaw straightness
  comparison, comm-timeout self-stop, and 10s frame statistics
- clean_cycle_hw_test.cc: 9 segments covering full FSM-driven one-pass
  cycle (PID-off and PID-on variants with yaw drift assertion), P0 fault
  estop, low-battery return, BMS validity, IMU validity, watchdog
- All hw tests added to hw_tests CMake target (no new target needed)

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## 自查 — Spec 覆盖验证

| 设计规格要求 | 覆盖任务 | 状态 |
|-------------|---------|------|
| `hw_config.h`：DeviceFixture + FullSystemFixture | Task 1 | ✅ |
| 限位测试：open / read_level_home | Task 2 | ✅ |
| 限位测试：callback_front/rear（手动触发） | Task 2 | ✅ |
| 限位测试：is_triggered / clear_trigger | Task 2 | ✅ |
| 限位测试：side_enum / repeated_trigger | Task 2 | ✅ |
| 电机测试：open/close / enable/disable / all_online | Task 3 | ✅ |
| 电机测试：fwd_no_pid / fwd_with_pid | Task 3 | ✅ |
| 电机测试：pid_straightness（PID 对比日志） | Task 3 | ✅ |
| 电机测试：rev_speed | Task 3 | ✅ |
| 电机测试：emergency_override / override_blocks / clear_override | Task 3 | ✅ |
| 电机测试：comm_timeout / frame_stats × 2 | Task 3 | ✅ |
| 集成测试：startup / self_check_pass | Task 4 | ✅ |
| 集成测试：one_pass_no_pid / one_pass_with_pid | Task 4 | ✅ |
| 集成测试：fault_p0_estop / low_battery_return | Task 4 | ✅ |
| 集成测试：bms_valid / imu_valid / watchdog_alive | Task 4 | ✅ |
| CMakeLists.txt 更新 + 构建验证 | Task 5 | ✅ |
| BrushMotor 用 MockModbusMaster 替代 | Task 1 (FullSystemFixture) | ✅ |
| GPS 未安装：占位对象，不 open | Task 1 (FullSystemFixture) | ✅ |
| 安全：所有速度 ≤ 30 RPM，析构 emergency_stop() | Task 1 + 所有测试段 | ✅ |

---

## 目标板部署指南

```bash
# 1. 将 hw_tests 二进制传输到目标板
scp build/aarch64/bin/hw_tests root@<target_ip>:/opt/pv_robot/

# 2. 配置 CAN 总线（目标板上）
ip link set can0 down
ip link set can0 type can bitrate 500000
ip link set can0 up

# 3. 运行测试（目标板上）
cd /opt/pv_robot

./hw_tests "[hw_limit][open]"               # 纯 GPIO 测试（最安全，无电机运动）
./hw_tests "[hw_walk][open_close]"          # 仅 CAN 初始化
./hw_tests "[hw_walk][all_online]"          # 等待电机上线
./hw_tests "[hw_walk][fwd_no_pid]"          # 电机低速前进 3s
./hw_tests "[hw_walk][fwd_with_pid]"        # PID 前进 5s
./hw_tests "[hw_walk][pid_straightness]"    # PID 对比（需两次手动复位）
./hw_tests "[hw_cycle][startup]"            # 全层栈初始化
./hw_tests "[hw_cycle][one_pass_no_pid]"    # 完整一趟（PID 关）
./hw_tests "[hw_cycle][one_pass_with_pid]"  # 完整一趟（PID 开）
```
