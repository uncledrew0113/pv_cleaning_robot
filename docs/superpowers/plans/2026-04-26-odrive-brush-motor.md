# ODrive Brush Motor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Modbus-based brush motor with an ODrive 3.6 UART ASCII implementation that supports speed mode, torque mode, diagnostics readback, watchdog keepalive, and fault-only idle release.

**Architecture:** Keep `BrushMotor` as the device role but rewrite it around `ISerialPort` and ODrive ASCII commands. Update service-layer call sites to ODrive-native control semantics, preserve health telemetry shape, and add a serial mock plus hardware coverage for UART/ODrive behavior.

**Tech Stack:** C++17, Catch2, `ISerialPort` / `LibSerialPort`, ODrive UART ASCII protocol, existing motion/fault/health services

---

## File Structure

- Modify: `include/pv_cleaning_robot/device/brush_motor.h`
  - Replace Modbus-specific constructor/API with ODrive UART API and add new mode/idle/watchdog state.
- Modify: `pv_cleaning_robot/device/brush_motor.cc`
  - Implement ASCII command send/read, parsing, unit conversion, watchdog behavior, and idle release.
- Modify: `pv_cleaning_robot/main.cc`
  - Construct brush motor from `LibSerialPort` and new brush config fields.
- Modify: `config/config.json`
  - Replace obsolete brush `slave_id` config with ODrive UART fields.
- Create: `test/mock/mock_serial_port.h`
  - Serial mock that records ASCII writes and serves scripted ASCII responses.
- Replace/Modify: `test/device/brush_motor_test.cc`
  - Rewrite device tests around ODrive UART semantics.
- Modify: `test/service/motion_service_test.cc`
  - Update expectations from Modbus register writes to ODrive-native brush API behavior.
- Modify: `test/mock/mock_brush_motor.h`
  - Remove `start()` mock path; add mode/torque/idle tracking.
- Modify: `test/app/fault_handler_test.cc`
  - Verify fault paths invoke fault-only idle release rather than normal stop where required.
- Modify: `test/app/robot_fsm_test.cc`
  - Update any brush setup that still assumes `start()`.
- Modify: `test/integration/task_chain_test.cc`
  - Update mocked brush construction and any start-semantics assertions.
- Modify: `test/integration/system_integration_test.cc`
  - Update mocked brush construction and control assertions.
- Modify: `test/integration/hardware/hw_config.h`
  - Add ODrive brush UART config and replace Modbus brush mock fixture with serial mock fixture where full-system hardware tests need a brush placeholder.
- Modify: `test/integration/hardware/hw_test_config.json`
  - Add ODrive brush UART config defaults.
- Create: `test/integration/hardware/odrive_brush_hw_test.cc`
  - Add target-machine UART/ODrive hardware tests.

## Task 1: Define the ODrive `BrushMotor` API Surface

**Files:**
- Modify: `include/pv_cleaning_robot/device/brush_motor.h`
- Modify: `test/device/brush_motor_test.cc`

- [ ] **Step 1: Write a failing compilation test for the new constructor and API**

Replace the old test fixture block at the top of `test/device/brush_motor_test.cc` with this minimal compile-target fixture:

```cpp
#include <catch2/catch.hpp>

#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/brush_motor.h"

using robot::device::BrushMotor;
using robot::device::DeviceError;

struct BrushMotorFixture {
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    BrushMotor motor;

    BrushMotorFixture()
        : motor(serial, 0, 8192.0f, true, 0.5f) {
        serial->open_result = true;
        motor.open();
    }
};

TEST_CASE("BrushMotor: supports ODrive constructor and mode API", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.set_mode_speed() == DeviceError::OK);
    REQUIRE(f.motor.set_mode_torque() == DeviceError::OK);
}
```

This should fail to compile before the header is updated.

- [ ] **Step 2: Build `unit_tests` to confirm the API does not exist yet**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile failure showing that `BrushMotor` does not accept `(serial, axis, counts_per_rev, watchdog_enabled, watchdog_timeout_s)` and lacks `set_mode_speed()` / `set_mode_torque()`.

- [ ] **Step 3: Update `BrushMotor` header to the new API**

Replace the public API in `include/pv_cleaning_robot/device/brush_motor.h` with this structure:

```cpp
#pragma once
#include <memory>
#include <string>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::device {

class BrushMotor {
   public:
    enum class ControlMode : uint8_t { SPEED = 0, TORQUE = 1 };

    struct Status {
        int actual_rpm{0};
        float current_a{0.0f};
        bool running{false};
        bool fault{false};
        uint32_t fault_code{0};
    };

    struct Diagnostics : Status {
        float temperature_c{0.0f};
        float bus_voltage_v{0.0f};
        int target_rpm{0};
        float target_torque_nm{0.0f};
        uint32_t stall_count{0};
        uint32_t comm_error_count{0};
    };

    BrushMotor(std::shared_ptr<hal::ISerialPort> serial,
               uint8_t axis,
               float counts_per_rev,
               bool watchdog_enabled,
               float watchdog_timeout_s);

    bool open();

    DeviceError set_mode_speed();
    DeviceError set_mode_torque();
    DeviceError set_rpm(int rpm);
    DeviceError set_torque(float torque_nm);
    DeviceError stop();
    DeviceError enter_idle();
    DeviceError clear_fault();

    Status get_status() const;
    Diagnostics get_diagnostics() const;
    void update();

   private:
    std::shared_ptr<hal::ISerialPort> serial_;
    uint8_t axis_;
    float counts_per_rev_;
    bool watchdog_enabled_;
    float watchdog_timeout_s_;
    mutable hal::PiMutex mtx_;
    Diagnostics diag_{};
    ControlMode control_mode_{ControlMode::SPEED};
    bool active_control_{false};
    bool keepalive_required_{false};
    int target_rpm_{0};
    float target_torque_nm_{0.0f};
};

}  // namespace robot::device
```

Remove the Modbus register constants and `start()` declaration entirely.

- [ ] **Step 4: Add a stub implementation signature so the code compiles**

In `pv_cleaning_robot/device/brush_motor.cc`, replace the constructor and method signatures with stubs:

```cpp
BrushMotor::BrushMotor(std::shared_ptr<hal::ISerialPort> serial,
                       uint8_t axis,
                       float counts_per_rev,
                       bool watchdog_enabled,
                       float watchdog_timeout_s)
    : serial_(std::move(serial))
    , axis_(axis)
    , counts_per_rev_(counts_per_rev)
    , watchdog_enabled_(watchdog_enabled)
    , watchdog_timeout_s_(watchdog_timeout_s) {}

bool BrushMotor::open() { return serial_ && (serial_->is_open() || serial_->open()); }
DeviceError BrushMotor::set_mode_speed() { return DeviceError::OK; }
DeviceError BrushMotor::set_mode_torque() { return DeviceError::OK; }
DeviceError BrushMotor::set_rpm(int) { return DeviceError::OK; }
DeviceError BrushMotor::set_torque(float) { return DeviceError::OK; }
DeviceError BrushMotor::stop() { return DeviceError::OK; }
DeviceError BrushMotor::enter_idle() { return DeviceError::OK; }
DeviceError BrushMotor::clear_fault() { return DeviceError::OK; }
BrushMotor::Status BrushMotor::get_status() const { return static_cast<Status>(diag_); }
BrushMotor::Diagnostics BrushMotor::get_diagnostics() const { return diag_; }
void BrushMotor::update() {}
```

This task only stabilizes the new type surface.

- [ ] **Step 5: Build `unit_tests` again to verify the API surface compiles**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: successful compile after the API surface is updated.

- [ ] **Step 6: Commit the new `BrushMotor` API surface**

Run:

```bash
git add include/pv_cleaning_robot/device/brush_motor.h \
        pv_cleaning_robot/device/brush_motor.cc \
        test/device/brush_motor_test.cc
git commit -m "refactor: define odrive brush motor api"
```

## Task 2: Add a UART Serial Mock and Rebuild Device Tests Around ODrive ASCII

**Files:**
- Create: `test/mock/mock_serial_port.h`
- Modify: `test/device/brush_motor_test.cc`
- Modify: `pv_cleaning_robot/device/brush_motor.cc`

- [ ] **Step 1: Create the UART mock**

Create `test/mock/mock_serial_port.h`:

```cpp
#pragma once
#include <deque>
#include <string>
#include <vector>

#include "pv_cleaning_robot/hal/i_serial_port.h"

struct MockSerialPort : robot::hal::ISerialPort {
    bool open_result{true};
    bool opened{false};
    int write_result{-1};
    int bytes_available_result{0};
    robot::hal::UartResult last_error{robot::hal::UartResult::OK};
    std::vector<std::string> writes;
    std::deque<std::string> read_chunks;

    bool open() override {
        opened = open_result;
        last_error = open_result ? robot::hal::UartResult::OK : robot::hal::UartResult::DISCONNECTED;
        return open_result;
    }
    void close() override { opened = false; }
    bool is_open() const override { return opened; }

    int write(const uint8_t* buf, size_t len, int) override {
        writes.emplace_back(reinterpret_cast<const char*>(buf), len);
        if (write_result >= 0) {
            last_error = robot::hal::UartResult::OK;
            return write_result == 0 ? static_cast<int>(len) : write_result;
        }
        last_error = robot::hal::UartResult::SYS_ERROR;
        return -1;
    }

    int read(uint8_t* buf, size_t max_len, int) override {
        if (read_chunks.empty()) {
            last_error = robot::hal::UartResult::TIMEOUT;
            return 0;
        }
        std::string chunk = read_chunks.front();
        read_chunks.pop_front();
        const size_t n = std::min(max_len, chunk.size());
        std::memcpy(buf, chunk.data(), n);
        last_error = robot::hal::UartResult::OK;
        return static_cast<int>(n);
    }

    bool flush_input() override { return true; }
    bool flush_output() override { return true; }
    int bytes_available() override { return bytes_available_result; }
    robot::hal::UartResult get_last_error() const override { return last_error; }
};
```

- [ ] **Step 2: Rewrite brush-motor unit tests to ODrive semantics**

Replace `test/device/brush_motor_test.cc` with ODrive-oriented tests like:

```cpp
TEST_CASE("BrushMotor: set_rpm() immediately sends ODrive velocity command", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.set_mode_speed() == DeviceError::OK);
    REQUIRE(f.motor.set_rpm(1200) == DeviceError::OK);
    REQUIRE_FALSE(f.serial->writes.empty());
    REQUIRE(f.serial->writes.back().find("v 0 ") == 0);
}

TEST_CASE("BrushMotor: stop() after speed mode sends zero velocity and disables keepalive",
          "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.set_mode_speed() == DeviceError::OK);
    REQUIRE(f.motor.set_rpm(1200) == DeviceError::OK);
    f.serial->writes.clear();
    REQUIRE(f.motor.stop() == DeviceError::OK);
    REQUIRE(f.serial->writes.back() == "v 0 0 0\\n");
}

TEST_CASE("BrushMotor: set_torque() sends ODrive current command", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.set_mode_torque() == DeviceError::OK);
    REQUIRE(f.motor.set_torque(1.5f) == DeviceError::OK);
    REQUIRE(f.serial->writes.back().find("c 0 1.5") == 0);
}

TEST_CASE("BrushMotor: enter_idle() stops keepalive", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.enter_idle() == DeviceError::OK);
    f.serial->writes.clear();
    f.motor.update();
    REQUIRE(f.serial->writes.empty());
}
```

Also add readback tests using scripted responses:

```cpp
TEST_CASE("BrushMotor: update() parses f response into actual RPM", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.serial->read_chunks = {"0.0 8192.0\\n", "24.0\\n", "5.0\\n", "36.5\\n", "0\\n", "0\\n", "0\\n"};
    f.motor.update();
    auto st = f.motor.get_status();
    REQUIRE(st.actual_rpm == 60);
}
```

- [ ] **Step 3: Build `unit_tests` to verify the new tests fail for the stub implementation**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile succeeds, but the new brush-motor tests would fail on a runnable environment because the implementation still returns stub values and does not emit ASCII commands.

- [ ] **Step 4: Implement the core ASCII helper methods**

In `pv_cleaning_robot/device/brush_motor.cc`, add local helpers:

```cpp
static float rpm_to_counts_per_sec(int rpm, float counts_per_rev) {
    return (static_cast<float>(rpm) * counts_per_rev) / 60.0f;
}

static int counts_per_sec_to_rpm(float cps, float counts_per_rev) {
    return static_cast<int>(std::lround((cps * 60.0f) / counts_per_rev));
}
```

Add a private helper pattern in the `.cc` file for sending one ASCII line:

```cpp
static DeviceError write_line(robot::hal::ISerialPort& serial, const std::string& line) {
    return serial.write(reinterpret_cast<const uint8_t*>(line.data()), line.size()) ==
                   static_cast<int>(line.size())
               ? DeviceError::OK
               : DeviceError::COMM_TIMEOUT;
}
```

Then implement:

- `set_mode_speed()` to update cached mode only
- `set_mode_torque()` to update cached mode only
- `set_rpm()` to send `v axis velocity 0\n`, cache `target_rpm_`, set `active_control_` and `keepalive_required_`
- `set_torque()` to send `c axis torque\n`, cache `target_torque_nm_`, set `active_control_` and `keepalive_required_`
- `stop()` to send zero command for current mode and clear `keepalive_required_`
- `clear_fault()` to send `sc\n`

- [ ] **Step 5: Implement `update()` readback parsing and watchdog rules**

Extend `pv_cleaning_robot/device/brush_motor.cc` so `update()`:

1. acquires the serial lock
2. reads `f axis`
3. reads voltage/current/temperature/error fields
4. updates diagnostics cache
5. sends `u axis\n` only when:
   - watchdog is enabled
   - active control is still required
   - no control command was already sent in the same cycle

Use simple parsing with `std::stringstream`:

```cpp
std::stringstream ss(resp);
float pos = 0.0f;
float vel = 0.0f;
ss >> pos >> vel;
```

Aggregate fault fields into one `uint32_t fault_code` by OR-ing them together.

- [ ] **Step 6: Implement `enter_idle()`**

Add `enter_idle()` behavior:

```cpp
DeviceError BrushMotor::enter_idle() {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    keepalive_required_ = false;
    active_control_ = false;
    target_rpm_ = 0;
    target_torque_nm_ = 0.0f;
    return write_ascii_locked("w axis" + std::to_string(axis_) + ".requested_state 1\n");
}
```

If the runtime write path is later found incompatible, adjust implementation to the documented fallback behavior. For the plan, the initial implementation should still provide one explicit idle-transition attempt.

- [ ] **Step 7: Build `unit_tests` again**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: successful compile of `unit_tests` after the ODrive implementation is in place.

- [ ] **Step 8: Commit the ODrive device implementation and tests**

Run:

```bash
git add test/mock/mock_serial_port.h \
        test/device/brush_motor_test.cc \
        pv_cleaning_robot/device/brush_motor.cc
git commit -m "feat: implement odrive brush motor device"
```

## Task 3: Switch Main Wiring and Brush Config to UART ODrive

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `config/config.json`

- [ ] **Step 1: Write the config update**

In `config/config.json`, replace the brush block:

```json
"brush": { "port": "/dev/ttyS3", "baudrate": 9600, "slave_id": 1 }
```

with:

```json
"brush": {
  "port": "/dev/ttyS3",
  "baudrate": 115200,
  "axis": 0,
  "counts_per_rev": 8192,
  "watchdog_enabled": true,
  "watchdog_timeout_s": 0.5
}
```

- [ ] **Step 2: Replace the brush driver construction in `main.cc`**

Replace:

```cpp
    auto brush_modbus = std::make_shared<robot::driver::LibModbusMaster>(
        cfg.get<std::string>("serial.brush.port", "/dev/ttyS3"),
        robot::hal::ModbusConfig{cfg.get<int>("serial.brush.baudrate", 115200)});
    ...
    auto brush_motor = std::make_shared<robot::device::BrushMotor>(
        brush_modbus, cfg.get<int>("serial.brush.slave_id", 1));
```

with:

```cpp
    auto brush_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.brush.port", "/dev/ttyS3"),
        robot::hal::UartConfig{cfg.get<int>("serial.brush.baudrate", 115200)});
    ...
    auto brush_motor = std::make_shared<robot::device::BrushMotor>(
        brush_serial,
        cfg.get<uint8_t>("serial.brush.axis", 0u),
        cfg.get<float>("serial.brush.counts_per_rev", 8192.0f),
        cfg.get<bool>("serial.brush.watchdog_enabled", true),
        cfg.get<float>("serial.brush.watchdog_timeout_s", 0.5f));
```

- [ ] **Step 3: Build the main binary**

Run:

```bash
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected: successful compile of the main executable with UART brush construction.

- [ ] **Step 4: Commit the ODrive wiring**

Run:

```bash
git add pv_cleaning_robot/main.cc config/config.json
git commit -m "feat: wire brush motor to odrive uart"
```

## Task 4: Update `MotionService` and Brush Mocks to ODrive-Native Semantics

**Files:**
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `test/mock/mock_brush_motor.h`
- Modify: `test/service/motion_service_test.cc`

- [ ] **Step 1: Rewrite the brush mock**

Update `test/mock/mock_brush_motor.h` to remove `start()` and add the new methods:

```cpp
    robot::device::DeviceError set_mode_speed_result{robot::device::DeviceError::OK};
    robot::device::DeviceError set_mode_torque_result{robot::device::DeviceError::OK};
    robot::device::DeviceError set_torque_result{robot::device::DeviceError::OK};
    robot::device::DeviceError enter_idle_result{robot::device::DeviceError::OK};
    int set_mode_speed_count{0};
    int set_mode_torque_count{0};
    int set_torque_count{0};
    int enter_idle_count{0};
```

and methods:

```cpp
    robot::device::DeviceError set_mode_speed() { ++set_mode_speed_count; return set_mode_speed_result; }
    robot::device::DeviceError set_mode_torque() { ++set_mode_torque_count; return set_mode_torque_result; }
    robot::device::DeviceError set_torque(float torque) {
        ++set_torque_count;
        diag_result.target_torque_nm = torque;
        return set_torque_result;
    }
    robot::device::DeviceError enter_idle() { ++enter_idle_count; return enter_idle_result; }
```

- [ ] **Step 2: Update `MotionService` implementation**

In `pv_cleaning_robot/service/motion_service.cc`, replace brush calls:

```cpp
    brush_->set_rpm(cfg_.brush_rpm);
    brush_->start();
```

with:

```cpp
    if (brush_->set_mode_speed() != device::DeviceError::OK)
        return false;
    if (brush_->set_rpm(cfg_.brush_rpm) != device::DeviceError::OK)
        return false;
```

Replace:

```cpp
    brush_->set_rpm(-static_cast<float>(cfg_.return_brush_rpm));
    brush_->start();
```

with:

```cpp
    if (brush_->set_mode_speed() != device::DeviceError::OK)
        return false;
    if (brush_->set_rpm(-cfg_.return_brush_rpm) != device::DeviceError::OK)
        return false;
```

Keep `stop()` for normal stopping paths.

- [ ] **Step 3: Rewrite service tests**

Update `test/service/motion_service_test.cc` to use `MockSerialPort` + real `BrushMotor`, or convert to `MockBrushMotor` if that is simpler. At minimum, replace assertions like:

```cpp
TEST_CASE("MotionService: start_cleaning() 写 BrushMotor REG_ENABLE=1", "[service][motion]") { ... }
```

with:

```cpp
TEST_CASE("MotionService: start_cleaning() issues brush speed-mode command", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    REQUIRE_FALSE(f.serial->writes.empty());
    bool found_velocity = false;
    for (const auto& w : f.serial->writes)
        if (w.find("v 0 ") == 0)
            found_velocity = true;
    REQUIRE(found_velocity);
}
```

Also add:

```cpp
TEST_CASE("MotionService: stop_cleaning() sends brush stop command", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    f.serial->writes.clear();
    f.motion.stop_cleaning();
    REQUIRE_FALSE(f.serial->writes.empty());
    REQUIRE(f.serial->writes.back() == "v 0 0 0\n");
}
```

- [ ] **Step 4: Build `unit_tests`**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: successful compile after service-layer updates.

- [ ] **Step 5: Commit the service-layer migration**

Run:

```bash
git add pv_cleaning_robot/service/motion_service.cc \
        include/pv_cleaning_robot/service/motion_service.h \
        test/mock/mock_brush_motor.h \
        test/service/motion_service_test.cc
git commit -m "refactor: update motion service for odrive brush control"
```

## Task 5: Update Fault Paths and Integration Fixtures

**Files:**
- Modify: `test/app/fault_handler_test.cc`
- Modify: `test/app/robot_fsm_test.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/integration/system_integration_test.cc`
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/hardware/hw_test_config.json`

- [ ] **Step 1: Add a fault-path assertion for idle release**

In `test/app/fault_handler_test.cc`, add a mock-brush-based assertion:

```cpp
TEST_CASE("FaultHandler: severe fault releases brush into idle control state", "[app][fault_handler]") {
    MockBrushMotor brush;
    brush.status_result.running = true;
    brush.enter_idle();
    REQUIRE(brush.enter_idle_count >= 1);
}
```

If the production `FaultHandler` currently only uses `MotionService`, update the test after the production path is changed to verify the severe-fault flow reaches `enter_idle()`.

- [ ] **Step 2: Replace remaining Modbus-based brush fixtures in integration tests**

In `test/integration/task_chain_test.cc` and `test/integration/system_integration_test.cc`, replace:

```cpp
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(modbus, 1)};
```

with:

```cpp
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{
        std::make_shared<BrushMotor>(brush_serial, 0u, 8192.0f, true, 0.5f)};
```

Set:

```cpp
    brush_serial->open_result = true;
    brush_serial->write_result = 0;
```

and prime `read_chunks` as needed for `update()` consumers.

- [ ] **Step 3: Replace hardware full-system brush placeholder**

In `test/integration/hardware/hw_config.h`, replace:

```cpp
    std::shared_ptr<MockModbusMaster> mock_modbus;
```

with:

```cpp
    std::shared_ptr<MockSerialPort> mock_brush_serial;
```

and replace brush construction:

```cpp
        mock_brush_serial = std::make_shared<MockSerialPort>();
        mock_brush_serial->open_result = true;
        mock_brush_serial->write_result = 0;
        brush = std::make_shared<device::BrushMotor>(
            mock_brush_serial, p.brush_axis, p.brush_counts_per_rev, p.brush_watchdog_enabled, p.brush_watchdog_timeout_s);
```

Add these fields to `HwParams`:

```cpp
    std::string brush_port = "/dev/ttyS3";
    int brush_baud = 115200;
    uint8_t brush_axis = 0u;
    float brush_counts_per_rev = 8192.0f;
    bool brush_watchdog_enabled = true;
    float brush_watchdog_timeout_s = 0.5f;
```

and load them from `hw_test_config.json`.

- [ ] **Step 4: Update hardware config defaults**

In `test/integration/hardware/hw_test_config.json`, add:

```json
    "brush_port": "/dev/ttyS3",
    "brush_baud": 115200,
    "brush_axis": 0,
    "brush_counts_per_rev": 8192,
    "brush_watchdog_enabled": true,
    "brush_watchdog_timeout_s": 0.5,
```

- [ ] **Step 5: Build both `unit_tests` and `hw_tests`**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
cmake --build --preset rk3576-build --target hw_tests
```

Expected: successful compile of both test targets after all fixtures are moved to serial-based brush setup.

- [ ] **Step 6: Commit the fixture and fault-path migration**

Run:

```bash
git add test/app/fault_handler_test.cc \
        test/app/robot_fsm_test.cc \
        test/integration/task_chain_test.cc \
        test/integration/system_integration_test.cc \
        test/integration/hardware/hw_config.h \
        test/integration/hardware/hw_test_config.json
git commit -m "test: migrate brush fixtures to odrive serial"
```

## Task 6: Add Real ODrive UART Hardware Tests

**Files:**
- Create: `test/integration/hardware/odrive_brush_hw_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Create the hardware test file**

Create `test/integration/hardware/odrive_brush_hw_test.cc`:

```cpp
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

static const hw::HwParams kp = hw::load_hw_test_config();

TEST_CASE("ODrive brush UART connect and readback", "[hw_brush][connect]") {
    auto serial = std::make_shared<driver::LibSerialPort>(
        kp.brush_port, hal::UartConfig{kp.brush_baud});
    device::BrushMotor brush(
        serial, kp.brush_axis, kp.brush_counts_per_rev, kp.brush_watchdog_enabled, kp.brush_watchdog_timeout_s);

    REQUIRE(brush.open());
    brush.update();
    auto diag = brush.get_diagnostics();
    REQUIRE(diag.bus_voltage_v >= 0.0f);
}

TEST_CASE("ODrive brush speed command and watchdog keepalive", "[hw_brush][speed_watchdog]") {
    auto serial = std::make_shared<driver::LibSerialPort>(
        kp.brush_port, hal::UartConfig{kp.brush_baud});
    device::BrushMotor brush(
        serial, kp.brush_axis, kp.brush_counts_per_rev, kp.brush_watchdog_enabled, kp.brush_watchdog_timeout_s);

    REQUIRE(brush.open());
    REQUIRE(brush.set_mode_speed() == device::DeviceError::OK);
    REQUIRE(brush.set_rpm(300) == device::DeviceError::OK);
    for (int i = 0; i < 10; ++i) {
        brush.update();
        std::this_thread::sleep_for(100ms);
    }
    brush.stop();
}
```

- [ ] **Step 2: Register the hardware test target**

In `test/CMakeLists.txt`, add:

```cmake
  integration/hardware/odrive_brush_hw_test.cc
```

to the `hw_tests` sources list.

- [ ] **Step 3: Cross-compile `hw_tests`**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: successful compile including the new ODrive brush hardware test.

- [ ] **Step 4: Run the new brush hardware tests on the target**

Run on the target machine:

```bash
./hw_tests "[hw_brush][connect]"
./hw_tests "[hw_brush][speed_watchdog]"
```

Expected:

- connect test reads at least one valid diagnostic response
- speed watchdog test keeps the brush under control during the update loop and stops cleanly at the end

- [ ] **Step 5: Commit the hardware-test addition**

Run:

```bash
git add test/integration/hardware/odrive_brush_hw_test.cc \
        test/CMakeLists.txt
git commit -m "test: add odrive brush hardware coverage"
```

## Task 7: Final Verification

**Files:**
- Modify: none

- [ ] **Step 1: Rebuild all affected targets**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
cmake --build --preset rk3576-build --target hw_tests
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected: all three targets compile successfully.

- [ ] **Step 2: Run the brush-related unit-test subsets on a runnable environment**

Run on a host or target that can execute the test binary:

```bash
./unit_tests "[device][brush_motor]"
./unit_tests "[service][motion]"
```

Expected: both subsets pass with the new ODrive brush semantics.

- [ ] **Step 3: Run the new ODrive brush hardware subsets on target**

Run:

```bash
./hw_tests "[hw_brush]"
```

Expected: all ODrive brush hardware tests pass.

- [ ] **Step 4: Inspect the final diff and worktree state**

Run:

```bash
git diff --stat HEAD~6..HEAD
git status --short
```

Expected:

- diff stat limited to brush motor, motion/fault test, config, and hardware-test files from this plan
- only expected external untracked artifacts remain, if any

- [ ] **Step 5: Create the final integration commit if needed**

If earlier task commits were skipped, run:

```bash
git add include/pv_cleaning_robot/device/brush_motor.h \
        pv_cleaning_robot/device/brush_motor.cc \
        pv_cleaning_robot/main.cc \
        config/config.json \
        test/mock/mock_serial_port.h \
        test/mock/mock_brush_motor.h \
        test/device/brush_motor_test.cc \
        test/service/motion_service_test.cc \
        test/app/fault_handler_test.cc \
        test/app/robot_fsm_test.cc \
        test/integration/task_chain_test.cc \
        test/integration/system_integration_test.cc \
        test/integration/hardware/hw_config.h \
        test/integration/hardware/hw_test_config.json \
        test/integration/hardware/odrive_brush_hw_test.cc \
        test/CMakeLists.txt
git commit -m "feat: migrate brush motor to odrive uart"
```

Expected: one clean integration commit covering the ODrive brush migration.
