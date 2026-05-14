# WalkMotorGroup Override Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Simplify `WalkMotorGroup` into a two-mode executor (`Normal` / `LatchedOverride`) while preserving the existing strong safety semantics of `emergency_override()` and moving override-clear reset handling into `MotionService`.

**Architecture:** Replace the normal command queue with a single "latest normal control slot", keep immediate synchronous emergency override transmission, and expose a monotonic `override_clear_generation()` signal so `MotionService` can reset `HeadingCorrector` exactly when a latched override is actually cleared. Preserve the public API shape for existing callers and keep startup convenience behavior inside `WalkMotorGroup`.

**Tech Stack:** C++17, Catch2 unit tests, existing CAN codec/device abstractions, `spdlog`, pthread-based RT receive thread.

---

## File Map

**Primary files**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
  - remove queue-oriented state
  - add two-mode control state and clear-generation accessor
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
  - replace queue semantics with latest-slot semantics
  - preserve immediate override send
  - preserve send ordering guarantee under `send_mtx_`
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
  - store last-seen override-clear generation
- Modify: `pv_cleaning_robot/service/motion_service.cc`
  - reset `HeadingCorrector` when clear-generation changes
  - skip correction output on the same cycle as reset

**Test files**
- Modify: `test/device/walk_motor_group_test.cc`
  - replace queue-shape assumptions with two-mode executor tests
  - add clear-generation tests
  - add ordering/lockout tests
- Modify: `test/service/motion_service_test.cc`
  - add override-clear generation driven corrector reset coverage
- Modify: `test/integration/hardware/walk_motor_group_hw_test.cc`
  - remove stale old embedded-heading-control API usage so the file matches current architecture

**Reference files to inspect during implementation**
- `include/pv_cleaning_robot/service/heading_corrector.h`
- `pv_cleaning_robot/service/heading_corrector.cc`
- `docs/superpowers/specs/2026-05-13-walk-motor-group-override-simplification-design.md`

---

### Task 1: Lock Down The New Public Contract In Tests And Header

**Files:**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Write the failing API/behavior tests for clear generation and latest-slot semantics**

```cpp
TEST_CASE("设备层WalkMotorGroup - clear_override 生效前 generation 不变", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    const auto gen0 = group.override_clear_generation();
    REQUIRE(group.emergency_override(0.0f) == device::DeviceError::OK);
    group.clear_override();

    REQUIRE(group.override_clear_generation() == gen0);
    group.close();
}

TEST_CASE("设备层WalkMotorGroup - clear_override 经过一次 update 后 generation 加一", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    const auto gen0 = group.override_clear_generation();
    REQUIRE(group.emergency_override(0.0f) == device::DeviceError::OK);
    group.clear_override();
    group.update();

    REQUIRE(group.override_clear_generation() == gen0 + 1u);
    group.close();
}

TEST_CASE("设备层WalkMotorGroup - 多次 set_speeds 后 update 仅重发最后一次 normal 命令", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    group.set_speeds(10.0f, 10.0f, -10.0f, -10.0f);
    group.set_speeds(20.0f, 20.0f, -20.0f, -20.0f);
    group.set_speeds(30.0f, 30.0f, -30.0f, -30.0f);
    bus->sent_frames.clear();

    group.update();

    REQUIRE(bus->sent_frames.size() == 1u);
    const auto& f = bus->sent_frames.back();
    REQUIRE(be16s(f.data[0], f.data[1]) == 3000);
    REQUIRE(be16s(f.data[2], f.data[3]) == 3000);
    REQUIRE(be16s(f.data[4], f.data[5]) == -3000);
    REQUIRE(be16s(f.data[6], f.data[7]) == -3000);
    group.close();
}
```

- [ ] **Step 2: Run the targeted device tests to verify they fail on the current implementation**

Run: `cmake --build --preset rk3576-build --target unit_tests && build/aarch64/bin/unit_tests "[device][walk_motor_group]"`
Expected: build may pass, but the binary is not runnable in this host environment (`Exec format error`); if run on target hardware/tooling, new tests fail because `override_clear_generation()` does not exist and queue semantics are still present.

- [ ] **Step 3: Add the new header API and state shape without removing old implementation yet**

```cpp
class WalkMotorGroup {
   public:
    enum class ControlMode : uint8_t {
        Normal = 0,
        LatchedOverride = 1,
    };

    uint32_t override_clear_generation() const;

   private:
    hal::CanFrame normal_ctrl_frame_{};
    std::array<float, 4> normal_target_values_{};
    bool has_normal_ctrl_frame_{false};

    hal::CanFrame override_frame_{};
    std::array<float, 4> override_target_values_{};

    ControlMode control_mode_{ControlMode::Normal};
    bool clear_override_pending_{false};
    uint32_t override_clear_generation_{0};

    // remove queue declarations in a later task
};
```

- [ ] **Step 4: Re-run the build to ensure the header and tests compile together**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 5: Commit the contract-first test/header step**

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h test/device/walk_motor_group_test.cc
git commit -m "test: lock down walk motor group override contract"
```

### Task 2: Replace The Queue With A Latest Normal Control Slot

**Files:**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Add a failing test that queue-specific behavior is no longer required and latest-slot behavior is authoritative**

```cpp
TEST_CASE("设备层WalkMotorGroup - set_currents 更新 normal 控制槽并由 update 重发", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    REQUIRE(group.set_currents(1.0f, -1.0f, 2.0f, -2.0f) == device::DeviceError::OK);
    bus->sent_frames.clear();

    group.update();

    REQUIRE(bus->sent_frames.size() == 1u);
    REQUIRE(bus->sent_frames.back().id == 0x032u);
    group.close();
}
```

- [ ] **Step 2: Remove queue fields and rewrite normal `set_*` APIs to update the normal slot directly**

```cpp
DeviceError WalkMotorGroup::set_speeds(float lt, float rt, float lb, float rb) {
    lt = clamp_rpm(lt);
    rt = clamp_rpm(rt);
    lb = clamp_rpm(lb);
    rb = clamp_rpm(rb);

    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
    normal_target_values_ = {lt, rt, lb, rb};
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = lt;
    diag_[1].target_value = rt;
    diag_[2].target_value = lb;
    diag_[3].target_value = rb;
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_currents(float lt, float rt, float lb, float rb) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ = protocol::WalkMotorCanCodec::encode_group_current(id_base_, lt, rt, lb, rb);
    normal_target_values_ = {lt, rt, lb, rb};
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = lt;
    diag_[1].target_value = rt;
    diag_[2].target_value = lb;
    diag_[3].target_value = rb;
    return DeviceError::OK;
}
```

- [ ] **Step 3: Remove queue draining and queue helper code from `update()` and the class**

```cpp
// delete from header:
// struct Cmd { ... };
// std::array<Cmd, kCmdQueueSize> cmd_buf_{};
// int cmd_head_{0};
// int cmd_tail_{0};
// hal::PiMutex cmd_mtx_;
// bool enqueue_cmd(const Cmd& cmd);

// delete from .cc:
// bool WalkMotorGroup::enqueue_cmd(...)
// queue-drain while(true) loop inside update()
```

- [ ] **Step 4: Re-run the focused build after queue removal**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 5: Commit the latest-slot refactor**

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h pv_cleaning_robot/device/walk_motor_group.cc test/device/walk_motor_group_test.cc
git commit -m "refactor: replace walk motor group queue with latest-slot control"
```

### Task 3: Rebuild The Override State Machine Around `Normal` / `LatchedOverride`

**Files:**
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Add failing tests for override lockout and ordering semantics**

```cpp
TEST_CASE("设备层WalkMotorGroup - emergency_override 后 update 不再发送 normal 帧", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    REQUIRE(group.set_speed_uniform(50.0f) == device::DeviceError::OK);
    REQUIRE(group.emergency_override(0.0f) == device::DeviceError::OK);
    bus->sent_frames.clear();

    group.update();

    REQUIRE(bus->sent_frames.empty());
    group.close();
}

TEST_CASE("设备层WalkMotorGroup - clear_override 生效后才允许恢复 normal 帧", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    REQUIRE(group.set_speed_uniform(50.0f) == device::DeviceError::OK);
    REQUIRE(group.emergency_override(0.0f) == device::DeviceError::OK);
    group.clear_override();
    bus->sent_frames.clear();

    group.update();

    REQUIRE(bus->sent_frames.size() == 1u);
    group.close();
}
```

- [ ] **Step 2: Rewrite `emergency_override()` to set latched mode before sending and to store override diagnostics**

```cpp
DeviceError WalkMotorGroup::emergency_override(float reverse_rpm) {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;

    float lt = 0.0f, rt = 0.0f, lb = 0.0f, rb = 0.0f;
    if (reverse_rpm > 0.0f) {
        const float rpm = clamp_rpm(reverse_rpm);
        lt = -rpm;
        rt = -rpm;
        lb = +rpm;
        rb = +rpm;
    }

    const auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        control_mode_ = ControlMode::LatchedOverride;
        clear_override_pending_ = false;
        override_frame_ = frame;
        override_target_values_ = {lt, rt, lb, rb};
    }

    std::lock_guard<hal::PiMutex> send_lk(send_mtx_);
    const bool ok = can_->send(frame);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        diag_[0].target_value = lt;
        diag_[1].target_value = rt;
        diag_[2].target_value = lb;
        diag_[3].target_value = rb;
        if (ok) ++ctrl_frame_count_;
        else ++ctrl_err_count_;
    }
    return ok ? DeviceError::OK : DeviceError::COMM_TIMEOUT;
}
```

- [ ] **Step 3: Rewrite `clear_override()` and `update()` around the two-mode state machine and lock-internal final send check**

```cpp
void WalkMotorGroup::clear_override() {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    clear_override_pending_ = true;
}

void WalkMotorGroup::update() {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return;

    auto now = std::chrono::steady_clock::now();
    // keep existing online refresh loop here

    hal::CanFrame candidate{};
    bool should_send_normal = false;

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (control_mode_ == ControlMode::LatchedOverride && clear_override_pending_) {
            clear_override_pending_ = false;
            control_mode_ = ControlMode::Normal;
            ++override_clear_generation_;
        }

        if (control_mode_ == ControlMode::Normal && has_normal_ctrl_frame_) {
            candidate = normal_ctrl_frame_;
            should_send_normal = true;
        }
    }

    if (!should_send_normal)
        return;

    std::lock_guard<hal::PiMutex> send_lk(send_mtx_);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (control_mode_ != ControlMode::Normal || !has_normal_ctrl_frame_)
            return;
        candidate = normal_ctrl_frame_;
    }

    const bool ok = can_->send(candidate);
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (ok) ++ctrl_frame_count_;
    else ++ctrl_err_count_;
}
```

- [ ] **Step 4: Add the generation accessor implementation and remove stale queue/override comments**

```cpp
uint32_t WalkMotorGroup::override_clear_generation() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return override_clear_generation_;
}
```

- [ ] **Step 5: Re-run the build after the override state-machine rewrite**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 6: Commit the override state-machine refactor**

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h pv_cleaning_robot/device/walk_motor_group.cc test/device/walk_motor_group_test.cc
git commit -m "refactor: simplify walk motor group override state machine"
```

### Task 4: Move Clear-Apply Reset Handling Into MotionService

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `test/service/motion_service_test.cc`

- [ ] **Step 1: Write the failing `MotionService` tests for clear-generation driven reset behavior**

```cpp
TEST_CASE("MotionService clear_override 生效后重置 HeadingCorrector 并当拍不输出纠偏", "[service][motion]") {
    MotionFixture f;
    f.cfg.heading_pid_en = true;
    MotionService motion(f.group, f.brush, nullptr, f.bus, f.cfg);

    REQUIRE(motion.start_cleaning());
    motion.emergency_stop();
    f.group->clear_override();
    f.can->sent_frames.clear();

    motion.update();

    REQUIRE(f.can->sent_frames.empty());
}
```

- [ ] **Step 2: Extend `MotionService` state to remember override-clear generation**

```cpp
class MotionService {
   private:
    uint32_t last_override_clear_generation_{0};
};
```

- [ ] **Step 3: Update constructor/start paths to initialize generation tracking and add clear-apply reset logic to `update()`**

```cpp
MotionService::MotionService(...)
    : group_(std::move(group))
    , brush_(std::move(brush))
    , imu_(std::move(imu))
    , bus_(bus)
    , cfg_(cfg)
    , heading_corrector_(cfg.pid) {
    if (group_) {
        last_override_clear_generation_ = group_->override_clear_generation();
    }
}

void MotionService::update() {
    bool reset_due_to_override_clear = false;

    group_->update();
    const auto clear_gen = group_->override_clear_generation();
    if (clear_gen != last_override_clear_generation_) {
        last_override_clear_generation_ = clear_gen;
        heading_corrector_.reset();
        reset_due_to_override_clear = true;
    }

    if (reset_due_to_override_clear) {
        return;
    }

    // existing IMU read / filtering / correction path continues here
}
```

- [ ] **Step 4: Reorder `MotionService::update()` carefully so `group_->update()` happens before generation comparison and correction output is skipped on the reset cycle**

```cpp
void MotionService::update() {
    // 1. read IMU and update filters if desired
    // 2. call group_->update() so clear requests can actually apply
    // 3. compare override_clear_generation()
    // 4. if changed: reset corrector and return early
    // 5. otherwise compute correction and send latest base command
}
```

- [ ] **Step 5: Re-run the build after integrating MotionService reset ownership**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 6: Commit the motion-layer reset handoff**

```bash
git add include/pv_cleaning_robot/service/motion_service.h pv_cleaning_robot/service/motion_service.cc test/service/motion_service_test.cc
git commit -m "refactor: move override clear reset handling into motion service"
```

### Task 5: Clean Up Drifted Tests And Comments To Match The New Architecture

**Files:**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Modify: `test/integration/hardware/walk_motor_group_hw_test.cc`

- [ ] **Step 1: Remove stale comments that still refer to queue dispatching or embedded heading correction in `WalkMotorGroup`**

```cpp
/// 周期心跳：在 `Normal` 模式重发当前正常控制帧；
/// 在 `LatchedOverride` 模式下屏蔽 normal 帧恢复，直到 `clear_override()` 被 `update()` 应用。
void update();

/// 解除紧急覆盖请求；真正解除发生在下一次 `update()` 中。
void clear_override();
```

- [ ] **Step 2: Update hardware tests so they no longer reference removed old embedded-heading APIs**

```cpp
// remove patterns like:
// grp.enable_heading_control(true);
// grp.update(d.valid ? d.yaw_deg : 0.0f);

// replace with:
grp.set_speed_uniform(kp.test_speed_rpm);
for (int i = 0; i < 100; ++i) {
    grp.update();
    std::this_thread::sleep_for(50ms);
}
```

- [ ] **Step 3: Keep hardware test focus on executor semantics only**

```cpp
TEST_CASE("WalkMotorGroup 解除急停后恢复驱动", "[hw_walk][clear_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    REQUIRE(grp.enable_all() == device::DeviceError::OK);
    REQUIRE(grp.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);

    REQUIRE(grp.emergency_override(0.0f) == device::DeviceError::OK);
    grp.clear_override();
    grp.update();

    REQUIRE(grp.set_speed_uniform(kp.test_speed_rpm) == device::DeviceError::OK);
    grp.update();
}
```

- [ ] **Step 4: Re-run the build after cleanup**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 5: Commit the cleanup step**

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h pv_cleaning_robot/device/walk_motor_group.cc test/integration/hardware/walk_motor_group_hw_test.cc
git commit -m "test: align walk motor group docs and hardware tests with simplified design"
```

### Task 6: Final Verification And Review Pass

**Files:**
- Verify only: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Verify only: `pv_cleaning_robot/device/walk_motor_group.cc`
- Verify only: `include/pv_cleaning_robot/service/motion_service.h`
- Verify only: `pv_cleaning_robot/service/motion_service.cc`
- Verify only: `test/device/walk_motor_group_test.cc`
- Verify only: `test/service/motion_service_test.cc`

- [ ] **Step 1: Build the full unit-test target**

Run: `cmake --build --preset rk3576-build --target unit_tests`
Expected: PASS

- [ ] **Step 2: If a runnable target environment is available, execute the focused test groups**

Run: `./unit_tests "[device][walk_motor_group]"`
Expected: PASS

Run: `./unit_tests "[service][motion]"`
Expected: PASS

- [ ] **Step 3: Verify no stale API references remain**

Run: `rg -n "enable_heading_control|update\(.*yaw|cmd_buf_|enqueue_cmd|pending_clear_override_" include pv_cleaning_robot test`
Expected: no matches for removed queue state or removed old embedded heading-control APIs in active code paths.

- [ ] **Step 4: Review git diff for scope control**

Run: `git diff --stat HEAD~5..HEAD`
Expected: only `WalkMotorGroup`, `MotionService`, and directly related tests/comments changed.

- [ ] **Step 5: Commit the final verification checkpoint**

```bash
git add include/pv_cleaning_robot/device/walk_motor_group.h pv_cleaning_robot/device/walk_motor_group.cc include/pv_cleaning_robot/service/motion_service.h pv_cleaning_robot/service/motion_service.cc test/device/walk_motor_group_test.cc test/service/motion_service_test.cc test/integration/hardware/walk_motor_group_hw_test.cc
git commit -m "refactor: simplify walk motor group override flow"
```
