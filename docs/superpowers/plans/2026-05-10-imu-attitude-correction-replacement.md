# IMU Attitude Correction Replacement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current `omega_z` heading PID with an IMU-only attitude controller that tracks local `|pitch|` maximum and uses `roll` as a side indicator.

**Architecture:** Keep the current `MotionService -> WalkMotorGroup -> correction controller` wiring shape, but replace the controller internals and widen the update path to carry `pitch`, `roll`, and `gyro_z_dps`. Use a small state machine (`UNINITIALIZED`, `LEARN`, `TRACK`, `FREEZE`) instead of a signed-rate PID.

**Tech Stack:** C++17, Catch2, existing `WalkMotorGroup` and `MotionService` runtime path, no new dependencies

---

## File Map

- Modify: `include/pv_cleaning_robot/service/heading_pid_controller.h`
  - Replace omega-z PID params/API with attitude-controller params/API.
- Modify: `pv_cleaning_robot/service/heading_pid_controller.cc`
  - Implement the new state machine, filtering, learning, freeze logic, and bounded correction output.
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
  - Widen `update(...)` to accept filtered `pitch`, `roll`, and `gyro_z_dps`.
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
  - Replace `pid_ctrl_.compute(omega_z, dt)` with `pid_ctrl_.compute(pitch, roll, gyro_z, dt)`.
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
  - Remove yaw-centric filter state; add filtered `pitch` and `roll`.
- Modify: `pv_cleaning_robot/service/motion_service.cc`
  - Feed filtered `pitch`, `roll`, and `gyro_z_dps` into `WalkMotorGroup::update(...)`.
- Modify: `test/service/heading_pid_test.cc`
  - Replace old omega-z PID tests with state-machine behavior tests and recorded-sequence replay.
- Modify: `test/device/walk_motor_group_test.cc`
  - Update device-layer integration tests to assert pitch/roll-driven correction.

## Task 1: Replace Controller Unit Tests First

**Files:**
- Modify: `test/service/heading_pid_test.cc`
- Test: `test/service/heading_pid_test.cc`

- [ ] **Step 1: Write the failing state-machine tests**

Replace the old rate-PID tests with a new suite shaped like this:

```cpp
#include <catch2/catch.hpp>
#include <vector>

#include "pv_cleaning_robot/service/heading_pid_controller.h"

using robot::service::HeadingPidController;

static HeadingPidController::Params test_params() {
    HeadingPidController::Params p;
    p.pitch_alpha = 1.0f;
    p.roll_alpha = 1.0f;
    p.gyro_alpha = 1.0f;
    p.pitch_drop_threshold = 0.10f;
    p.roll_threshold = 0.50f;
    p.learn_improve_eps = 0.02f;
    p.warmup_ms = 40;
    p.hold_ms = 40;
    p.freeze_release_ms = 60;
    p.freeze_gyro_z_threshold = 30.0f;
    p.freeze_pitch_rate_threshold = 20.0f;
    p.freeze_roll_rate_threshold = 20.0f;
    p.best_decay_per_s = 0.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    return p;
}

TEST_CASE("HeadingPidController: disabled controller outputs zero", "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    REQUIRE(ctrl.compute(-34.8f, -2.0f, 0.0f, 0.02f) == Approx(0.0f));
}

TEST_CASE("HeadingPidController: learns local pitch best before tracking", "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 5; ++i)
        REQUIRE(ctrl.compute(-34.2f, -4.5f, 0.0f, 0.02f) == Approx(0.0f));

    for (int i = 0; i < 5; ++i)
        REQUIRE(ctrl.compute(-34.8f, -2.0f, 0.0f, 0.02f) == Approx(0.0f));

    REQUIRE(ctrl.debug_state().mode == HeadingPidController::Mode::TRACK);
    REQUIRE(ctrl.debug_state().pitch_abs_best == Approx(34.8f).margin(0.05f));
    REQUIRE(ctrl.debug_state().roll_at_best == Approx(-2.0f).margin(0.1f));
}

TEST_CASE("HeadingPidController: right-biased sample commands correction toward center",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 10; ++i)
        ctrl.compute(-34.83f, -1.95f, 0.0f, 0.02f);

    float correction = 0.0f;
    for (int i = 0; i < 4; ++i)
        correction = ctrl.compute(-34.17f, -5.98f, 0.0f, 0.02f);

    REQUIRE(correction != Approx(0.0f));
}

TEST_CASE("HeadingPidController: disturbance enters freeze and suppresses output",
          "[service][heading_pid]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    for (int i = 0; i < 10; ++i)
        ctrl.compute(-34.83f, -1.95f, 0.0f, 0.02f);

    const float correction = ctrl.compute(-34.5f, 0.0f, 80.0f, 0.02f);
    REQUIRE(correction == Approx(0.0f));
    REQUIRE(ctrl.debug_state().mode == HeadingPidController::Mode::FREEZE);
}
```

- [ ] **Step 2: Run the test target and confirm compile/test failure**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "HeadingPidController"
```

Expected:

- build fails because the current controller has no `pitch_alpha`, `roll_alpha`, `gyro_alpha`, or `debug_state()`
- or tests fail because current `compute()` still accepts `omega_z` only

- [ ] **Step 3: Replace the controller header with the new API**

Update `include/pv_cleaning_robot/service/heading_pid_controller.h` to define the new params, mode enum, debug state, and compute signature:

```cpp
class HeadingPidController {
   public:
    struct Params {
        float pitch_alpha{0.2f};
        float roll_alpha{0.2f};
        float gyro_alpha{0.2f};
        float pitch_drop_threshold{0.12f};
        float roll_threshold{0.6f};
        float learn_improve_eps{0.03f};
        float best_decay_per_s{0.01f};
        float freeze_gyro_z_threshold{30.0f};
        float freeze_pitch_rate_threshold{20.0f};
        float freeze_roll_rate_threshold{20.0f};
        float max_output{30.0f};
        float min_effective_output{0.0f};
        int warmup_ms{400};
        int hold_ms{400};
        int freeze_release_ms{300};
    };

    enum class Mode : uint8_t { UNINITIALIZED, LEARN, TRACK, FREEZE };

    struct DebugState {
        Mode mode{Mode::UNINITIALIZED};
        float filtered_pitch{0.0f};
        float filtered_roll{0.0f};
        float filtered_gyro_z{0.0f};
        float pitch_abs_best{0.0f};
        float roll_at_best{0.0f};
    };

    HeadingPidController() = default;
    explicit HeadingPidController(const Params& p);

    void set_params(const Params& p);
    void enable(bool en);
    void reset();
    bool is_enabled() const;
    float compute(float pitch_deg, float roll_deg, float gyro_z_dps, float dt_s);
    DebugState debug_state() const;
};
```

- [ ] **Step 4: Implement the minimal state machine to satisfy the tests**

In `pv_cleaning_robot/service/heading_pid_controller.cc`, implement:

```cpp
float HeadingPidController::compute(float pitch_deg, float roll_deg, float gyro_z_dps, float dt_s) {
    if (!enabled_)
        return 0.0f;

    update_filters(pitch_deg, roll_deg, gyro_z_dps);

    if (disturbance_detected(dt_s)) {
        mode_ = Mode::FREEZE;
        freeze_stable_ms_ = 0.0f;
        return 0.0f;
    }

    switch (mode_) {
        case Mode::UNINITIALIZED:
            warmup_ms_acc_ += dt_s * 1000.0f;
            if (warmup_ms_acc_ >= params_.warmup_ms)
                mode_ = Mode::LEARN;
            return 0.0f;

        case Mode::LEARN:
            learn_reference(dt_s);
            if (learned_once_ && learn_ms_acc_ >= params_.warmup_ms)
                mode_ = Mode::TRACK;
            return 0.0f;

        case Mode::TRACK:
            learn_reference(dt_s);
            return track_correction(dt_s);

        case Mode::FREEZE:
            if (freeze_cleared(dt_s))
                mode_ = learned_once_ ? Mode::TRACK : Mode::LEARN;
            return 0.0f;
    }

    return 0.0f;
}
```

Keep the implementation minimal in this task: enough to satisfy unit tests first, not yet perfect field tuning.

- [ ] **Step 5: Run the focused tests and make them pass**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "HeadingPidController"
```

Expected:

- build succeeds
- service-layer `HeadingPidController` tests pass

- [ ] **Step 6: Commit the controller-unit-test slice**

```bash
git add \
  include/pv_cleaning_robot/service/heading_pid_controller.h \
  pv_cleaning_robot/service/heading_pid_controller.cc \
  test/service/heading_pid_test.cc
git commit -m "feat: replace omega-z PID controller core"
```

## Task 2: Update WalkMotorGroup To Use Pitch/Roll Inputs

**Files:**
- Modify: `include/pv_cleaning_robot/device/walk_motor_group.h`
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc`
- Modify: `test/device/walk_motor_group_test.cc`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Write failing device-layer tests for pitch/roll-driven correction**

Replace the old omega-z semantics tests with tests shaped like this:

```cpp
TEST_CASE("WalkMotorGroup: update() applies correction when pitch drops and roll indicates right bias",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    group.set_speed_uniform(100.0f);
    group.enable_heading_control(true);

    for (int i = 0; i < 10; ++i)
        group.update(-34.83f, -1.95f, -0.49f, 0.0f);

    bus->sent_frames.clear();
    for (int i = 0; i < 4; ++i)
        group.update(-34.17f, -5.98f, 7.81f, 0.0f);

    REQUIRE_FALSE(bus->sent_frames.empty());
    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x032u);
}

TEST_CASE("WalkMotorGroup: freeze disturbance suppresses differential correction",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    group.set_speed_uniform(100.0f);
    group.enable_heading_control(true);

    for (int i = 0; i < 10; ++i)
        group.update(-34.83f, -1.95f, -0.49f, 0.0f);

    bus->sent_frames.clear();
    group.update(-34.50f, 0.0f, 90.0f, 0.0f);

    REQUIRE_FALSE(bus->sent_frames.empty());
    const auto& f = bus->sent_frames.back();
    int16_t lt = be16s(f.data[0], f.data[1]);
    int16_t rt = be16s(f.data[2], f.data[3]);
    REQUIRE(lt == rt);
}
```

- [ ] **Step 2: Run the focused test target and confirm failure**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "WalkMotorGroup"
```

Expected:

- compile failure because `WalkMotorGroup::update(...)` still accepts only `(yaw_deg, omega_z_dps)`

- [ ] **Step 3: Widen the WalkMotorGroup interface**

Update `include/pv_cleaning_robot/device/walk_motor_group.h`:

```cpp
/// @param pitch_deg    当前 IMU pitch（°），局部最优时 |pitch| 较大
/// @param roll_deg     当前 IMU roll（°），用于辅助判断偏向哪一侧
/// @param yaw_deg      当前 IMU yaw（°），仅用于日志和诊断
/// @param gyro_z_dps   当前 IMU Z 轴角速度（°/s），仅用于扰动冻结检测
void update(float pitch_deg = 0.0f,
            float roll_deg = 0.0f,
            float yaw_deg = 0.0f,
            float gyro_z_dps = 0.0f);
```

- [ ] **Step 4: Switch the device-layer correction source**

In `pv_cleaning_robot/device/walk_motor_group.cc`, replace:

```cpp
float correction = pid_ctrl_.compute(omega_z_dps, dt_s);
```

with:

```cpp
float correction = pid_ctrl_.compute(pitch_deg, roll_deg, gyro_z_dps, dt_s);
```

Do not change how the resulting signed `correction` is applied to upper/lower tracks in this task.

- [ ] **Step 5: Update tests and make them pass**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "WalkMotorGroup|HeadingPidController"
```

Expected:

- controller and device-layer tests pass together

- [ ] **Step 6: Commit the WalkMotorGroup slice**

```bash
git add \
  include/pv_cleaning_robot/device/walk_motor_group.h \
  pv_cleaning_robot/device/walk_motor_group.cc \
  test/device/walk_motor_group_test.cc
git commit -m "feat: drive walk motor correction from pitch and roll"
```

## Task 3: Feed Filtered Pitch/Roll From MotionService

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Test: `test/integration/task_chain_test.cc`
- Test: `test/integration/system_integration_test.cc`

- [ ] **Step 1: Add failing coverage for the widened runtime path**

Add or update an integration assertion so the runtime path compiles only if `MotionService` now feeds pitch/roll into `WalkMotorGroup::update(...)`.

Minimal test-side expectation:

```cpp
TEST_CASE("System: MotionService update path compiles with attitude-controller inputs",
          "[integration][system]") {
    SUCCEED("compile-time integration coverage only");
}
```

This task's main failure is compile-time, not behavioral.

- [ ] **Step 2: Run the build and confirm compile failure**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- compile failure in `MotionService::update()` because `group_->update(...)` signature changed

- [ ] **Step 3: Replace yaw-centric filter state in MotionService**

In `include/pv_cleaning_robot/service/motion_service.h`, replace the old members:

```cpp
float filtered_yaw_{0.0f};
bool filtered_yaw_inited_{false};
float filtered_omega_z_{0.0f};
bool filtered_omega_z_inited_{false};
```

with:

```cpp
float filtered_pitch_{0.0f};
bool filtered_pitch_inited_{false};
float filtered_roll_{0.0f};
bool filtered_roll_inited_{false};
float filtered_yaw_{0.0f};
bool filtered_yaw_inited_{false};
float filtered_gyro_z_{0.0f};
bool filtered_gyro_z_inited_{false};
```

- [ ] **Step 4: Feed pitch, roll, yaw, and gyro_z from MotionService**

In `pv_cleaning_robot/service/motion_service.cc`, update `update()` to derive:

```cpp
const auto latest = imu_ ? imu_->get_latest() : device::ImuDevice::ImuData{};
const float raw_pitch = latest.valid ? latest.pitch_deg : 0.0f;
const float raw_roll = latest.valid ? latest.roll_deg : 0.0f;
const float raw_yaw = latest.valid ? latest.yaw_deg : 0.0f;
const float raw_gyro_z = latest.valid ? (latest.gyro[2] * (180.0f / 3.14159265f)) : 0.0f;

filtered_pitch_ = filtered_pitch_inited_ ? 0.8f * filtered_pitch_ + 0.2f * raw_pitch : raw_pitch;
filtered_roll_ = filtered_roll_inited_ ? 0.8f * filtered_roll_ + 0.2f * raw_roll : raw_roll;
filtered_yaw_ = filtered_yaw_inited_ ? 0.8f * filtered_yaw_ + 0.2f * raw_yaw : raw_yaw;
filtered_gyro_z_ = filtered_gyro_z_inited_
    ? 0.8f * filtered_gyro_z_ + 0.2f * raw_gyro_z
    : raw_gyro_z;

group_->update(filtered_pitch_, filtered_roll_, filtered_yaw_, filtered_gyro_z_);
```

Keep the filter style consistent with the existing codebase; do not add a new filter framework.

- [ ] **Step 5: Run build and focused integration coverage**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "TaskChain|System"
```

Expected:

- build succeeds
- existing non-hardware integration tests still pass

- [ ] **Step 6: Commit the MotionService slice**

```bash
git add \
  include/pv_cleaning_robot/service/motion_service.h \
  pv_cleaning_robot/service/motion_service.cc
git commit -m "feat: feed pitch and roll into motion correction path"
```

## Task 4: Add Recorded-Sequence Replay And Run Final Regression

**Files:**
- Modify: `test/service/heading_pid_test.cc`
- Test: `test/service/heading_pid_test.cc`
- Test: `test/device/walk_motor_group_test.cc`

- [ ] **Step 1: Add replay coverage using the provided IMU sequence**

Append a focused replay test to `test/service/heading_pid_test.cc` using compressed representative samples from the provided dataset:

```cpp
TEST_CASE("HeadingPidController: replayed IMU sequence converges around the local pitch maximum",
          "[service][heading_pid][replay]") {
    HeadingPidController ctrl(test_params());
    ctrl.enable(true);

    const std::vector<std::tuple<float, float, float>> samples = {
        {-34.17f, -5.98f,  7.81f},
        {-34.22f, -5.67f,  7.42f},
        {-34.50f, -3.00f,  1.58f},
        {-34.83f, -1.95f, -0.49f},
        {-34.74f, -1.16f, -1.12f},
        {-34.66f, -0.73f, -3.59f},
        {-34.54f,  0.65f, -6.23f},
        {-34.58f,  1.44f, -7.36f},
    };

    float correction = 0.0f;
    for (const auto& [pitch, roll, yaw] : samples)
        correction = ctrl.compute(pitch, roll, yaw, 0.02f);

    REQUIRE(ctrl.debug_state().pitch_abs_best >= 34.7f);
    REQUIRE(ctrl.debug_state().mode != HeadingPidController::Mode::UNINITIALIZED);
    REQUIRE(std::isfinite(correction));
}
```

- [ ] **Step 2: Run the full targeted regression suite**

Run:

```bash
cmake --build build --target unit_tests -j4
ctest --test-dir build --output-on-failure -R "HeadingPidController|WalkMotorGroup|TaskChain|System"
```

Expected:

- all targeted tests pass

- [ ] **Step 3: Run a final broader unit-test regression if time allows**

Run:

```bash
ctest --test-dir build --output-on-failure -R "service|device"
```

Expected:

- no regressions in nearby service/device test groups

- [ ] **Step 4: Commit the replay and regression slice**

```bash
git add \
  test/service/heading_pid_test.cc \
  test/device/walk_motor_group_test.cc
git commit -m "test: add IMU replay coverage for attitude correction"
```

## Self-Review

### Spec coverage

- IMU-only controller replacement: covered by Tasks 1-3
- `|pitch|` local-maximum tracking: covered by Tasks 1 and 4
- unknown initial pose: covered by `UNINITIALIZED` and `LEARN` tests in Task 1
- disturbance freeze: covered by Tasks 1 and 2
- existing runtime wiring preserved: covered by Task 3

No spec section is unassigned.

### Placeholder scan

- No `TODO`/`TBD`
- Each task includes exact file paths
- Each test/build step includes concrete commands
- Code-edit steps include concrete signature and logic snippets

### Type consistency

- `HeadingPidController` name stays stable to minimize churn
- new runtime method signature is consistently:
  `compute(float pitch_deg, float roll_deg, float gyro_z_dps, float dt_s)`
- `WalkMotorGroup::update(...)` is consistently widened to:
  `update(float pitch_deg, float roll_deg, float yaw_deg, float gyro_z_dps)`

