# HeadingCorrector Boundary Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all heading-correction-related computation out of `MotionService` and into `HeadingCorrector` while preserving the accepted `MotionService -> HeadingCorrector -> WalkMotorGroup` architecture and `clear_override -> reset()` semantics.

**Architecture:** `MotionService` becomes a thin orchestration layer that passes raw IMU and wheel feedback plus the base speed command into `HeadingCorrector`. `HeadingCorrector` becomes the single owner of correction filtering, internal state, mode transitions, reference management, and final wheel-target generation. `WalkMotorGroup` remains unchanged as the actuator layer.

**Tech Stack:** C++17, Catch2 unit tests, existing `WalkMotorGroup` and `MotionService` test fixtures, CMake preset `rk3576-build`

---

## File Structure

- Modify: `include/pv_cleaning_robot/service/heading_corrector.h`
  - Expand the controller-owned state and simplify `Input` to raw observations.
- Modify: `pv_cleaning_robot/service/heading_corrector.cc`
  - Move the remaining control-pipeline computation in here.
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
  - Remove controller-local filter members and keep only orchestration state.
- Modify: `pv_cleaning_robot/service/motion_service.cc`
  - Stop filtering IMU locally; pass raw inputs into `HeadingCorrector`.
- Modify: `test/service/heading_pid_test.cc`
  - Update unit tests to validate the new full-boundary controller behavior.
- Modify: `test/service/motion_service_test.cc`
  - Update orchestration tests so they assert dispatch/reset behavior, not controller internals.

## Task 1: Redesign `HeadingCorrector::Input` Around Raw Inputs

**Files:**
- Modify: `include/pv_cleaning_robot/service/heading_corrector.h`
- Test: `test/service/heading_pid_test.cc`

- [ ] **Step 1: Write the failing controller-interface tests**

Add tests that express the new contract: raw IMU in, final wheel targets out, internal filtering owned by the controller.

```cpp
TEST_CASE("HeadingCorrector: raw samples are accepted directly", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.raw_yaw_deg = -8.0f;
    input.raw_gyro_z_dps = 0.5f;
    input.dt_s = 0.02f;

    const auto out = ctrl.compute(input);
    REQUIRE_FALSE(out.has_speed_command);
}

TEST_CASE("HeadingCorrector: wheel feedback remains part of raw input", "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.8f;
    input.raw_roll_deg = -2.0f;
    input.raw_yaw_deg = -8.0f;
    input.raw_gyro_z_dps = 0.5f;
    input.dt_s = 0.02f;
    input.wheel_feedback.valid = true;
    input.wheel_feedback.rpm = {25.0f, 24.0f, -25.0f, -24.0f};

    REQUIRE(ctrl.compute(input).correction_rpm == Approx(0.0f));
}
```

- [ ] **Step 2: Run the focused controller tests and confirm they fail**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: compile failure in `heading_pid_test.cc` because `HeadingCorrector::Input` does not yet contain `raw_pitch_deg`, `raw_roll_deg`, `raw_yaw_deg`, or `raw_gyro_z_dps`.

- [ ] **Step 3: Change the controller input contract in the header**

Update `HeadingCorrector::Input` so the controller consumes raw observations and owns all correction-related preprocessing.

```cpp
struct Input {
    float raw_pitch_deg{0.0f};
    float raw_roll_deg{0.0f};
    float raw_yaw_deg{0.0f};
    float raw_gyro_z_dps{0.0f};
    float dt_s{0.0f};
    WheelFeedback wheel_feedback{};
    bool has_base_command{false};
    SpeedCommand base_command{};
};
```

Also add controller-owned yaw filter state in the private section:

```cpp
float filtered_yaw_{0.0f};
bool yaw_initialized_{false};
```

- [ ] **Step 4: Re-run the controller build target**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: compile now advances into `heading_corrector.cc` / `motion_service.cc` failures because implementation still uses the old field names.

- [ ] **Step 5: Commit the interface-only change**

```bash
git add include/pv_cleaning_robot/service/heading_corrector.h test/service/heading_pid_test.cc
git commit -m "refactor: switch heading corrector input to raw observations"
```

## Task 2: Move Controller-Local Filtering and Preprocessing Into `HeadingCorrector`

**Files:**
- Modify: `pv_cleaning_robot/service/heading_corrector.cc`
- Modify: `include/pv_cleaning_robot/service/heading_corrector.h`
- Test: `test/service/heading_pid_test.cc`

- [ ] **Step 1: Add a failing test for controller-owned filtering state**

Extend `heading_pid_test.cc` to assert that the controller debug state reflects filtered raw input without help from `MotionService`.

```cpp
TEST_CASE("HeadingCorrector: debug state reflects controller-owned filtering",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -35.0f;
    input.raw_roll_deg = -1.5f;
    input.raw_yaw_deg = -7.0f;
    input.raw_gyro_z_dps = 1.0f;
    input.dt_s = 0.02f;

    ctrl.compute(input);
    const auto state = ctrl.debug_state();
    REQUIRE(state.filtered_pitch == Approx(-35.0f));
    REQUIRE(state.filtered_roll == Approx(-1.5f));
    REQUIRE(state.filtered_gyro_z == Approx(1.0f));
}
```

- [ ] **Step 2: Run the tests and confirm implementation mismatch**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: compile or test failure because `heading_corrector.cc` still reads `input.pitch_deg`, `input.roll_deg`, and `input.gyro_z_dps`.

- [ ] **Step 3: Update `HeadingCorrector` implementation to consume raw fields and own all filter initialization**

Change `compute()` and `update_filters()` to use raw fields:

```cpp
update_filters(input.raw_pitch_deg, input.raw_roll_deg, input.raw_gyro_z_dps);
filtered_yaw_ = yaw_initialized_
    ? low_pass(filtered_yaw_, input.raw_yaw_deg, params_.gyro_alpha)
    : input.raw_yaw_deg;
yaw_initialized_ = true;
```

Reset yaw state in `reset()`:

```cpp
filtered_yaw_ = 0.0f;
yaw_initialized_ = false;
```

Keep all filter initialization inside the controller; do not add any helper back to `MotionService`.

- [ ] **Step 4: Run the controller tests again**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: controller tests compile and the new filtering assertions pass, while `motion_service.cc` still fails to compile against the changed input contract.

- [ ] **Step 5: Commit the controller-local filtering migration**

```bash
git add include/pv_cleaning_robot/service/heading_corrector.h \
        pv_cleaning_robot/service/heading_corrector.cc \
        test/service/heading_pid_test.cc
git commit -m "refactor: move heading correction preprocessing into controller"
```

## Task 3: Slim `MotionService` Down to Orchestration

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Test: `test/service/motion_service_test.cc`

- [ ] **Step 1: Add failing orchestration tests**

Add tests that express the new boundary: `MotionService` should resume base speed after `clear_override`, but should not own filtered IMU state.

```cpp
TEST_CASE("MotionService clear_override 生效后恢复基础速度并保留 reset 语义",
          "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    f.motion.update();

    REQUIRE(f.group->emergency_override(0.0f) == robot::device::DeviceError::OK);
    f.group->clear_override();

    f.motion.update();
    REQUIRE(f.can->sent_frames.empty());

    f.motion.update();
    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, 210.0f, 210.0f, -210.0f, -210.0f)));
}
```

- [ ] **Step 2: Run the motion tests and capture failures**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: compile failure in `motion_service.cc` because it still populates `HeadingCorrector::Input` using the old names and still owns filtered members.

- [ ] **Step 3: Remove controller-local filter members from `MotionService`**

Delete these members from `motion_service.h`:

```cpp
float filtered_pitch_{0.0f};
bool filtered_pitch_inited_{false};
float filtered_roll_{0.0f};
bool filtered_roll_inited_{false};
float filtered_yaw_{0.0f};
bool filtered_yaw_inited_{false};
float filtered_omega_z_{0.0f};
bool filtered_omega_z_inited_{false};
```

Update `MotionService::update()` so it only forwards raw observations:

```cpp
HeadingCorrector::Input input;
input.raw_pitch_deg = imu_data.pitch_deg;
input.raw_roll_deg = imu_data.roll_deg;
input.raw_yaw_deg = imu_data.yaw_deg;
input.raw_gyro_z_dps = imu_data.gyro[2] * (180.0f / 3.14159265f);
input.dt_s = 0.02f;
input.has_base_command = true;
input.base_command = to_corrector_command(base_speed_cmd_);
input.wheel_feedback.valid = true;
input.wheel_feedback.rpm = {status.wheel[0].speed_rpm,
                            status.wheel[1].speed_rpm,
                            status.wheel[2].speed_rpm,
                            status.wheel[3].speed_rpm};
input.wheel_feedback.current = {diagnostics.wheel[0].torque_a,
                                diagnostics.wheel[1].torque_a,
                                diagnostics.wheel[2].torque_a,
                                diagnostics.wheel[3].torque_a};
```

Leave `clear_override` handling in `MotionService` unchanged:

```cpp
if (clear_generation != last_override_clear_generation_) {
    last_override_clear_generation_ = clear_generation;
    heading_corrector_.reset();
    if (walk_command_active_) {
        group_->set_speeds(base_speed_cmd_);
    }
}
```

- [ ] **Step 4: Run the full unit-test target**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: successful build of `unit_tests`.

- [ ] **Step 5: Commit the orchestration-only `MotionService` change**

```bash
git add include/pv_cleaning_robot/service/motion_service.h \
        pv_cleaning_robot/service/motion_service.cc \
        test/service/motion_service_test.cc
git commit -m "refactor: make motion service orchestration-only for heading correction"
```

## Task 4: Tighten Controller and Motion Tests Around the New Boundary

**Files:**
- Modify: `test/service/heading_pid_test.cc`
- Modify: `test/service/motion_service_test.cc`

- [ ] **Step 1: Add a failing controller test for base-command output ownership**

```cpp
TEST_CASE("HeadingCorrector: final wheel target generation stays inside controller",
          "[service][heading_pid]") {
    HeadingCorrector ctrl(test_params());
    ctrl.enable(true);

    HeadingCorrector::Input input;
    input.raw_pitch_deg = -34.2f;
    input.raw_roll_deg = -5.5f;
    input.raw_yaw_deg = -8.0f;
    input.raw_gyro_z_dps = 0.0f;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.base_command = {100.0f, 100.0f, -100.0f, -100.0f};

    for (int i = 0; i < 12; ++i) {
        ctrl.compute(input);
    }

    const auto out = ctrl.compute(input);
    REQUIRE(out.has_speed_command);
    REQUIRE(std::isfinite(out.speed_command.lt_rpm));
}
```

- [ ] **Step 2: Add a failing motion test that avoids asserting controller internals**

```cpp
TEST_CASE("MotionService update emits corrected group command without owning filter state",
          "[service][motion]") {
    MotionFixture f;
    f.cfg.heading_pid_en = true;
    MotionService motion(f.group, f.brush, nullptr, f.bus, f.cfg);

    REQUIRE(motion.start_cleaning());
    motion.update();
    REQUIRE_FALSE(f.can->sent_frames.empty());
}
```

- [ ] **Step 3: Run the unit-test build and fix any test assumptions that still depend on old boundaries**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: either green or a small number of failing tests that still assume `MotionService` owns filter state or controller preprocessing.

- [ ] **Step 4: Make test assertions boundary-correct**

Adjust tests so:

- `heading_pid_test.cc` verifies algorithm behavior and debug state
- `motion_service_test.cc` verifies orchestration, base-command resumption, and CAN emission

Use assertions like:

```cpp
REQUIRE(output.has_speed_command);
REQUIRE(output.speed_command.lt_rpm != Approx(input.base_command.lt_rpm));
```

and avoid asserting private preprocessing details from `MotionService`.

- [ ] **Step 5: Re-run the unit-test build**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: successful build of `unit_tests`.

- [ ] **Step 6: Commit the test-boundary cleanup**

```bash
git add test/service/heading_pid_test.cc test/service/motion_service_test.cc
git commit -m "test: align heading correction tests with controller boundary"
```

## Task 5: Final Verification

**Files:**
- Modify: none expected

- [ ] **Step 1: Build unit tests**

Run: `cmake --build --preset rk3576-build --target unit_tests`

Expected: `Built target unit_tests`

- [ ] **Step 2: Build hardware tests**

Run: `cmake --build --preset rk3576-build --target hw_tests`

Expected: `Built target hw_tests`

- [ ] **Step 3: Inspect for leftover controller-local filter state in `MotionService`**

Run: `rg -n "filtered_pitch_|filtered_roll_|filtered_yaw_|filtered_omega_z_|pitch_inited_|roll_inited_|yaw_inited_|omega_z_inited_" include/pv_cleaning_robot/service/motion_service.h pv_cleaning_robot/service/motion_service.cc`

Expected: no matches

- [ ] **Step 4: Inspect for raw-input usage inside `HeadingCorrector`**

Run: `rg -n "raw_pitch_deg|raw_roll_deg|raw_yaw_deg|raw_gyro_z_dps" include/pv_cleaning_robot/service/heading_corrector.h pv_cleaning_robot/service/heading_corrector.cc`

Expected: matches in both header and implementation

- [ ] **Step 5: Commit final verification if any small cleanup was needed**

```bash
git add include/pv_cleaning_robot/service/heading_corrector.h \
        pv_cleaning_robot/service/heading_corrector.cc \
        include/pv_cleaning_robot/service/motion_service.h \
        pv_cleaning_robot/service/motion_service.cc \
        test/service/heading_pid_test.cc \
        test/service/motion_service_test.cc
git commit -m "chore: finalize heading corrector boundary migration"
```

## Self-Review

- Spec coverage: the plan covers the agreed goal of moving all heading-correction-related computation into `HeadingCorrector`, keeping `MotionService` orchestration-only, and preserving `clear_override -> reset()` behavior.
- Placeholder scan: no `TODO`, `TBD`, or “similar to previous task” placeholders remain.
- Type consistency: the plan consistently uses `HeadingCorrector::Input` raw fields (`raw_pitch_deg`, `raw_roll_deg`, `raw_yaw_deg`, `raw_gyro_z_dps`) and keeps `base_command`, `wheel_feedback`, and `reset()` naming aligned across tasks.
