# Walk Motor Termination Init Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `WalkMotorGroup::open()` optionally send the existing `0x109` termination-resistor command for one configured `motor_id` immediately after CAN open, before any other motor initialization traffic.

**Architecture:** Keep the feature inside `WalkMotorGroup` rather than adding a caller-side startup step. Extend the constructor/configuration surface just enough to carry three new settings, then enforce the startup order inside `open()` and cover it with mock-based unit tests plus a small hardware regression test path.

**Tech Stack:** C++17, Catch2, SocketCAN abstraction (`ICanBus`), existing walk-motor CAN codec/config loader

---

## File Structure

- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
  - Extend `WalkMotorGroup` constructor and private state for termination-init config.
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
  - Validate config, build termination bitmap, send startup termination frames before `comm_timeout`.
- Modify: `pv_cleaning_robot/main.cc`
  - Read `can.walk_motor.termination_*` settings and pass them into `WalkMotorGroup`.
- Modify: `config/config.json`
  - Add default config values for termination init.
- Modify: `test/device/walk_motor_group_test.cc`
  - Add unit tests for frame ordering, retry count, target-motor mapping, disabled path, and invalid config.
- Modify: `test/integration/hardware/hw_config.h`
  - Load hardware-test equivalents of the new termination-init settings.
- Modify: `test/integration/hardware/hw_test_config.json`
  - Add default hardware-test config values for the new settings.
- Modify: `test/integration/hardware/walk_motor_group_hw_test.cc`
  - Add one startup regression test with termination init enabled.

## Task 1: Extend `WalkMotorGroup` Configuration Surface

**Files:**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `config/config.json`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Write the failing constructor/config compilation test**

Add this test near the existing `open()` tests in `test/device/walk_motor_group_test.cc`:

```cpp
TEST_CASE("设备层WalkMotorGroup - 构造支持终端电阻启动配置", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u, 200u, true, 3u, 2u);

    REQUIRE(group.open() == device::DeviceError::OK);
    group.close();
}
```

This should fail to compile before the constructor is extended.

- [ ] **Step 2: Build `unit_tests` to verify the new constructor does not exist yet**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile failure in `test/device/walk_motor_group_test.cc` indicating no matching `WalkMotorGroup` constructor with 6 arguments.

- [ ] **Step 3: Extend the header with the new constructor signature and state**

Update `include/pv_cleaning_robot/device/walk_motor_group.h`:

```cpp
explicit WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                        uint8_t id_base = 1u,
                        uint16_t comm_timeout_ms = 200u,
                        bool termination_init_enabled = true,
                        uint8_t termination_init_retry_count = 3u,
                        uint8_t termination_motor_id = 2u);
```

Add private members:

```cpp
    bool termination_init_enabled_{true};
    uint8_t termination_init_retry_count_{3u};
    uint8_t termination_motor_id_{2u};
```

Keep existing constructor defaults backward compatible so old call sites still compile unchanged until `main.cc` is updated.

- [ ] **Step 4: Wire the new constructor parameters in the implementation file**

Update the constructor definition in `pv_cleaning_robot/device/walk_motor_group.cc`:

```cpp
WalkMotorGroup::WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                               uint8_t id_base,
                               uint16_t comm_timeout_ms,
                               bool termination_init_enabled,
                               uint8_t termination_init_retry_count,
                               uint8_t termination_motor_id)
    : can_(std::move(can))
    , id_base_(id_base)
    , ctrl_id_((id_base <= 4u) ? protocol::kWalkMotorCtrlIdGroup1
                               : protocol::kWalkMotorCtrlIdGroup2)
    , comm_timeout_ms_(comm_timeout_ms)
    , termination_init_enabled_(termination_init_enabled)
    , termination_init_retry_count_(termination_init_retry_count)
    , termination_motor_id_(termination_motor_id)
    , codecs_{protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 0u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 1u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 2u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 3u))} {}
```

Do not add behavior yet. This task only makes the type surface compile.

- [ ] **Step 5: Read the new config in `main.cc` and pass it through**

Replace the current `WalkMotorGroup` construction in `pv_cleaning_robot/main.cc` with:

```cpp
    auto walk_group = std::make_shared<robot::device::WalkMotorGroup>(
        can_bus,
        cfg.get<uint8_t>("can.walk_motor.motor_id", 1u),
        cfg.get<uint16_t>("can.walk_motor.comm_timeout_ms", 200u),
        cfg.get<bool>("can.walk_motor.termination_init_enabled", true),
        cfg.get<uint8_t>("can.walk_motor.termination_init_retry_count", 3u),
        cfg.get<uint8_t>("can.walk_motor.termination_motor_id", 2u));
```

- [ ] **Step 6: Add default config values**

Update `config/config.json`:

```json
    "walk_motor": {
      "motor_id": 1,
      "comm_timeout_ms": 200,
      "termination_init_enabled": true,
      "termination_init_retry_count": 3,
      "termination_motor_id": 2
    }
```

- [ ] **Step 7: Build `unit_tests` to verify the surface compiles**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: successful compile of `unit_tests`.

- [ ] **Step 8: Commit the configuration-surface change**

Run:

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h \
        pv_cleaning_robot/device/walk_motor_group.cc \
        pv_cleaning_robot/main.cc \
        config/config.json \
        test/device/walk_motor_group_test.cc
git commit -m "feat: add walk motor termination init config"
```

## Task 2: Implement Startup Ordering and Validation

**Files:**
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Write the failing unit tests for ordering, retries, disabled mode, and invalid target**

Add these tests to `test/device/walk_motor_group_test.cc`:

```cpp
TEST_CASE("设备层WalkMotorGroup - open() 先发终端电阻初始化再发 comm_timeout",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u, 200u, true, 3u, 2u);

    REQUIRE(group.open() == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 7u);
    REQUIRE(bus->sent_frames[0].id == protocol::kWalkMotorTermResId);
    REQUIRE(bus->sent_frames[1].id == protocol::kWalkMotorTermResId);
    REQUIRE(bus->sent_frames[2].id == protocol::kWalkMotorTermResId);
    for (size_t i = 3; i < bus->sent_frames.size(); ++i)
        REQUIRE(bus->sent_frames[i].id == 0x10Au);

    const auto& term = bus->sent_frames[0];
    REQUIRE(term.data[0] == 0u);
    REQUIRE(term.data[1] == 1u);
    REQUIRE(term.data[2] == 0u);
    REQUIRE(term.data[3] == 0u);
    group.close();
}

TEST_CASE("设备层WalkMotorGroup - 禁用终端电阻初始化时 open() 不发送 0x109",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u, 200u, false, 3u, 2u);

    REQUIRE(group.open() == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 4u);
    for (const auto& frame : bus->sent_frames)
        REQUIRE(frame.id != protocol::kWalkMotorTermResId);
    group.close();
}

TEST_CASE("设备层WalkMotorGroup - 非法 termination_motor_id 使 open() 失败",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u, 200u, true, 3u, 5u);

    REQUIRE(group.open() == device::DeviceError::NOT_OPEN);
    REQUIRE(bus->sent_frames.empty());
}
```

Add one retry-clamp test:

```cpp
TEST_CASE("设备层WalkMotorGroup - retry_count=0 时按 1 次发送处理",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u, 0u, true, 0u, 2u);

    REQUIRE(group.open() == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 1u);
    REQUIRE(bus->sent_frames[0].id == protocol::kWalkMotorTermResId);
    group.close();
}
```

- [ ] **Step 2: Build `unit_tests` to verify the behavior tests fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile succeeds, but when the test binary is run on a runnable environment, the new cases fail because `open()` still sends only `comm_timeout` frames and does not validate `termination_motor_id`.

- [ ] **Step 3: Implement validation and startup termination send logic**

In `pv_cleaning_robot/device/walk_motor_group.cc`, add small local helpers near the top of the file:

```cpp
static bool motor_id_in_group(uint8_t id_base, uint8_t motor_id) {
    return motor_id >= id_base &&
           motor_id < static_cast<uint8_t>(id_base + device::WalkMotorGroup::kWheelCount);
}

static hal::CanFrame make_group_termination_frame(uint8_t motor_id) {
    std::array<bool, 8> enables{};
    enables[static_cast<std::size_t>(motor_id - 1u)] = true;
    return protocol::WalkMotorCanCodec::encode_set_termination_batch(enables);
}
```

Then update `WalkMotorGroup::open()` so the startup section becomes:

```cpp
    running_.store(true);
    recv_thread_ = std::thread(&WalkMotorGroup::recv_loop, this);

    if (termination_init_enabled_) {
        if (!motor_id_in_group(id_base_, termination_motor_id_)) {
            spdlog::error("[WalkMotorGroup] invalid termination_motor_id={} for group base={}",
                          termination_motor_id_,
                          id_base_);
            running_.store(false);
            if (recv_thread_.joinable())
                recv_thread_.join();
            can_->close();
            return DeviceError::NOT_OPEN;
        }

        const uint8_t retry_count =
            termination_init_retry_count_ == 0u ? 1u : termination_init_retry_count_;
        const auto frame = make_group_termination_frame(termination_motor_id_);
        bool any_success = false;
        spdlog::info("[WalkMotorGroup] init termination motor_id={} retries={}",
                     termination_motor_id_,
                     retry_count);
        for (uint8_t i = 0; i < retry_count; ++i) {
            if (can_->send(frame)) {
                any_success = true;
            } else {
                spdlog::warn("[WalkMotorGroup] termination init send failed attempt {}/{}",
                             static_cast<unsigned>(i + 1u),
                             static_cast<unsigned>(retry_count));
            }
        }
        if (!any_success) {
            spdlog::error("[WalkMotorGroup] termination init failed for motor_id={}, bus may rely on external termination",
                          termination_motor_id_);
        }
    } else {
        spdlog::info("[WalkMotorGroup] termination init disabled by config");
    }
```

Keep the existing `comm_timeout` loop after this block.

- [ ] **Step 4: Count termination-init sends in existing control-frame diagnostics**

Inside the retry loop, mirror the current `send_ctrl()` accounting:

```cpp
            std::lock_guard<hal::PiMutex> lk(mtx_);
            if (ok)
                ++ctrl_frame_count_;
            else
                ++ctrl_err_count_;
```

Do this inline in `open()` for each attempt so diagnostics stay consistent with existing send accounting.

- [ ] **Step 5: Add a partial-failure regression test**

Extend `MockCanBus` in `test/mock/mock_can_bus.h` with queued send results:

```cpp
    std::vector<bool> send_results{};
```

and update `send()` to use them:

```cpp
    bool send(const robot::hal::CanFrame& f) override {
        sent_frames.push_back(f);
        if (!send_results.empty()) {
            const bool result = send_results.front();
            send_results.erase(send_results.begin());
            return result;
        }
        return send_result;
    }
```

Then add this test in `test/device/walk_motor_group_test.cc`:

```cpp
TEST_CASE("设备层WalkMotorGroup - 终端电阻初始化前两次失败后重试成功仍继续 open()",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    bus->send_results = {false, false, true, true, true, true, true};
    device::WalkMotorGroup group(bus, 1u, 200u, true, 3u, 2u);

    REQUIRE(group.open() == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 7u);
    group.close();
}
```

- [ ] **Step 6: Build `unit_tests` again**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: successful compile of `unit_tests` with the new tests and implementation.

- [ ] **Step 7: Run the walk-motor unit-test subset on a runnable environment**

Run on a host or target that can execute `unit_tests`:

```bash
./unit_tests "[device][walk_motor_group]"
```

Expected: all `WalkMotorGroup` mock tests pass, including the new termination-init cases.

- [ ] **Step 8: Commit the startup-ordering implementation**

Run:

```bash
git add pv_cleaning_robot/device/walk_motor_group.cc \
        test/device/walk_motor_group_test.cc \
        test/mock/mock_can_bus.h
git commit -m "feat: initialize walk motor termination on open"
```

## Task 3: Add Hardware-Test Config and Startup Regression Coverage

**Files:**
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/hardware/hw_test_config.json`
- Modify: `test/integration/hardware/walk_motor_group_hw_test.cc`

- [ ] **Step 1: Write the hardware regression test**

Add this test near the startup-oriented hardware cases in `test/integration/hardware/walk_motor_group_hw_test.cc`:

```cpp
TEST_CASE("WalkMotorGroup 启动阶段终端电阻初始化后仍可正常进入控制流程",
          "[hw_walk][termination_init_startup]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    device::WalkMotorGroup grp(can,
                               kp.motor_id_base,
                               kp.comm_timeout_ms,
                               kp.termination_init_enabled,
                               kp.termination_init_retry_count,
                               kp.termination_motor_id);

    REQUIRE(grp.open() == device::DeviceError::OK);
    CHECK(grp.set_feedback_mode_all(10u) == device::DeviceError::OK);
    CHECK(grp.enable_all() == device::DeviceError::OK);
    CHECK(grp.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);

    grp.disable_all();
    grp.close();
}
```

- [ ] **Step 2: Extend hardware config loading**

Add fields to `hw::HwParams` in `test/integration/hardware/hw_config.h`:

```cpp
    bool termination_init_enabled = true;
    uint8_t termination_init_retry_count = 3u;
    uint8_t termination_motor_id = 2u;
```

Load them in `load_hw_test_config()`:

```cpp
        p.termination_init_enabled =
            cfg.get<bool>("hardware.termination_init_enabled", p.termination_init_enabled);
        p.termination_init_retry_count = static_cast<uint8_t>(cfg.get<int>(
            "hardware.termination_init_retry_count",
            static_cast<int>(p.termination_init_retry_count)));
        p.termination_motor_id = static_cast<uint8_t>(
            cfg.get<int>("hardware.termination_motor_id", static_cast<int>(p.termination_motor_id)));
```

Update the fixture construction path:

```cpp
        walk_group = std::make_shared<device::WalkMotorGroup>(can_bus,
                                                              p.motor_id_base,
                                                              p.comm_timeout_ms,
                                                              p.termination_init_enabled,
                                                              p.termination_init_retry_count,
                                                              p.termination_motor_id);
```

- [ ] **Step 3: Add default hardware config values**

Update `test/integration/hardware/hw_test_config.json` under `hardware`:

```json
  "hardware": {
    "can_iface": "can0",
    "motor_id_base": 1,
    "termination_init_enabled": true,
    "termination_init_retry_count": 3,
    "termination_motor_id": 2,
```

Keep the existing JSON shape unchanged aside from these additions.

- [ ] **Step 4: Cross-compile the hardware test target**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: successful compile of `hw_tests`.

- [ ] **Step 5: Run the dedicated hardware startup regression on the target board**

Run on the target machine:

```bash
./hw_tests "[hw_walk][termination_init_startup]"
```

Expected: PASS, proving that startup termination init does not block `set_feedback_mode_all()`, `enable_all()`, or `set_mode_all()`.

- [ ] **Step 6: Commit the hardware regression coverage**

Run:

```bash
git add test/integration/hardware/hw_config.h \
        test/integration/hardware/hw_test_config.json \
        test/integration/hardware/walk_motor_group_hw_test.cc
git commit -m "test: cover walk motor termination init startup"
```

## Task 4: Final Verification

**Files:**
- Modify: none
- Test: `test/device/walk_motor_group_test.cc`
- Test: `test/integration/hardware/walk_motor_group_hw_test.cc`

- [ ] **Step 1: Rebuild both relevant test targets**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
cmake --build --preset rk3576-build --target hw_tests
```

Expected: both targets compile successfully.

- [ ] **Step 2: Run the mock walk-motor subset on a runnable environment**

Run:

```bash
./unit_tests "[device][walk_motor_group]"
```

Expected: PASS for all `WalkMotorGroup` mock tests.

- [ ] **Step 3: Run the dedicated hardware startup regression on target**

Run:

```bash
./hw_tests "[hw_walk][termination_init_startup]"
```

Expected: PASS.

- [ ] **Step 4: Inspect the final diff before handoff**

Run:

```bash
git diff --stat HEAD~3..HEAD
git status --short
```

Expected:

- `git diff --stat` shows only the walk-motor config, implementation, and test files listed in this plan.
- `git status --short` is clean.

- [ ] **Step 5: Create the final integration commit if the work was done without intermediate commits**

If the earlier task commits were skipped, run:

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h \
        pv_cleaning_robot/device/walk_motor_group.cc \
        pv_cleaning_robot/main.cc \
        config/config.json \
        test/mock/mock_can_bus.h \
        test/device/walk_motor_group_test.cc \
        test/integration/hardware/hw_config.h \
        test/integration/hardware/hw_test_config.json \
        test/integration/hardware/walk_motor_group_hw_test.cc
git commit -m "feat: init walk motor termination during open"
```

Expected: a clean final commit containing only subproject A.
