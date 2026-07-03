# Attitude Recovery Overall Timeout Implementation Plan

> **For agentic workers:** Use `superpowers:executing-plans` to implement this plan task by task. This repository task requires user confirmation before each implementation step.

**Goal:** Make attitude-limit recovery a bounded strategy attempt. `lower_attitude_center()` uses one overall timeout, relies on existing attitude-limit and main-limit callbacks for emergency stopping, and returns non-fatal strategy outcomes for single-attempt non-completion. Repeated-trigger escalation remains owned by `ErrorManager`.

**Confirmed constraints:**

- Keep changes minimal and surgical.
- Do not add a main-limit query port.
- Do not add a new hard-failure outcome or direct failed-recovery path for this feature.
- Hardware command or prepare failures during recovery must stop/log/wait until the attempt timeout, then return a timeout-style result. Later `ErrorManager::update()` diagnostics or repeated attitude-limit triggers handle escalation.
- Single-side `AttitudeLimit` while centering is a measurement signal, not a new error.
- Both-side `AttitudeLimitBoth` remains a hard event and can later lead to `FaultStopped`.
- Main-limit callback and attitude-limit callback remain responsible for emergency stop during recovery.
- Recovery timeout wait duration must not count into the 10 second repeated-attitude-limit window; the repeat window starts when recovery finishes.

---

## Files

- `include/pv_cleaning_robot/service/attitude_limit_service.h`
- `pv_cleaning_robot/service/attitude_limit_service.cc`
- `pv_cleaning_robot/service/motion_service.cc`
- `include/pv_cleaning_robot/app/recovery_executor.h`
- `pv_cleaning_robot/app/recovery_executor.cc`
- `pv_cleaning_robot/app/robot_application.cc`
- `test/service/attitude_limit_service_test.cc`
- `test/service/motion_service_test.cc`
- `test/app/recovery_executor_test.cc`
- `test/app/error_manager_test.cc`
- `test/integration/hardware/system_hw_common.h`
- `test/integration/hardware/system_hw_main_flow_tests.cc`

---

## Task 1: Define Non-Fatal Attitude Center Result

**Step 1: Update service header**

Modify only `include/pv_cleaning_robot/service/attitude_limit_service.h`.

Change `CenterConfig` from per-step timeouts to one overall timeout:

```cpp
struct CenterConfig {
    float lower_rpm{5.0f};
    int stable_samples_required{2};
    std::chrono::milliseconds overall_timeout{std::chrono::seconds(30)};
    std::chrono::milliseconds tick{std::chrono::milliseconds(20)};
};
```

Add:

```cpp
enum class CenterOutcome {
    Completed,
    TimedOut,
    InterruptedBySafetyOverride,
};

struct CenterResult {
    CenterOutcome outcome{CenterOutcome::TimedOut};
};
```

Change `lower_attitude_center()` overloads to return `CenterResult`.

Do not modify `MotionPorts` in this step. In particular, do not add a main-limit interruption port.

**Verify:** build is expected to fail until the implementation and tests are updated.

---

## Task 2: Implement Overall-Timeout Centering

**Step 1: Add center-state bookkeeping**

Modify only `pv_cleaning_robot/service/attitude_limit_service.cc`.

Inside `AttitudeLimitService`, use existing callback flow to detect expected attitude-limit stops while centering:

- `handle_trigger()` still calls `motion_ports_.emergency_stop()`.
- If `centering_active_` and exactly one side is active, record the side as the latest centering trigger and suppress the pending `AttitudeLimit` event.
- If both sides are active, keep recording `AttitudeLimitBoth`.

No new main-limit detection is added here.

**Verify:** compile may still fail until the full return type transition is complete.

**Step 2: Replace per-stage timeouts with one deadline**

In `lower_attitude_center(const CenterConfig&)`:

- Compute one `deadline = now + config.overall_timeout`.
- Every wait loop checks only the overall deadline.
- Remove use of `search_timeout`, `release_timeout`, and `opposite_timeout`.
- On timeout, issue final stop, finish centering, return `{TimedOut}`.

**Verify:** attitude service tests should compile after test updates.

**Step 3: Implement the three start cases**

Use the existing lower-wheel direction convention and mirror logic:

- Neither side active: move default direction until the default-side attitude proximity triggers. Record time `a`; callback has stopped motion. Clear only this expected attitude stop, reverse direction, then wait until the opposite-side proximity triggers. Record `b`; callback has stopped motion. Clear only this expected attitude stop, then move default direction for `(b - a) / 2`.
- Side A active: move toward side B until A becomes inactive. Record `a`; continue toward B until B proximity triggers. Record `b`; callback has stopped motion. Clear only this expected attitude stop, then move toward A for `(b - a) / 2`.
- Side B active: mirror the Side A case.

Important safety rule: do not call a "clear override" helper unless the observed stop was the expected single-side attitude trigger. If a main-limit callback created the override, no expected attitude edge is recorded, no further motion command is issued, and the function waits until the overall timeout.

**Verify:** add focused tests in Task 3 before claiming behavior.

**Step 4: Hardware capability failure path**

If `prepare_center_motion`, lower-wheel command, or stop command reports failure:

- Log the failure.
- Stop motion if possible.
- Wait until the same overall timeout expires.
- Finish centering.
- Return `{TimedOut}`.

This is intentionally not a direct recovery failure.

**Verify:** focused unit test covers lower-wheel command failure returning `TimedOut`.

---

## Task 3: Update and Extend AttitudeLimitService Tests

Modify only `test/service/attitude_limit_service_test.cc`.

**Step 1: Update existing tests**

- Change config construction from six fields to four fields.
- Change `REQUIRE(ok)` to checking `CenterOutcome::Completed`.
- Keep existing single-side suppression and both-side hard-event checks.

**Step 2: Add timeout test**

When the expected attitude transitions never arrive:

- result is `TimedOut`;
- the call lasts until approximately the configured overall timeout;
- final lower-wheel command is stop or stop port is called.

**Step 3: Add command-failure timeout test**

When lower-wheel command returns false:

- result is `TimedOut`;
- the function waits until the overall timeout;
- stop path is attempted.

**Step 4: Add expected attitude-trigger resume test**

Simulate a centering-time single-side trigger through the switch callback and verify:

- no pending single-side error is submitted;
- the expected trigger lets centering continue;
- result can reach `Completed`.

**Verify:**

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests "[service][attitude_limit]"
```

---

## Task 4: Stop Auto-Clearing Overrides in Center Motion Command

Modify only `pv_cleaning_robot/service/motion_service.cc` and related focused tests if required.

**Step 1: Remove automatic override clearing from `command_lower_wheels_for_attitude_center()`**

This function must not clear a main-limit override by itself. The centering algorithm clears/re-prepares only after it has identified an expected single-side attitude trigger.

**Step 2: Add or adjust focused motion test**

Verify a center lower-wheel command does not clear override state implicitly.

**Verify:**

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests "[service][motion]"
```

---

## Task 5: Make Reverse Recovery Timeout-Style

Modify `pv_cleaning_robot/service/motion_service.cc` and `test/service/motion_service_test.cc`.

**Step 1: Reverse setup failure**

If `reverse_for_recovery()` cannot start motion:

- log/stop;
- wait until the configured duration expires;
- return true.

**Step 2: Reverse interrupted by main limit**

Do not add a new main-limit query. Existing main-limit callback emergency-stops and latches override. `reverse_for_recovery()` waits until its duration expires, then returns true.

**Verify:**

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests "[service][motion]"
```

---

## Task 6: Keep RecoveryExecutor Non-Fatal

Modify `include/pv_cleaning_robot/app/recovery_executor.h`, `pv_cleaning_robot/app/recovery_executor.cc`, and `test/app/recovery_executor_test.cc`.

**Step 1: Update lower-attitude-center port result**

Use an app-local strategy result or the service result type, following existing dependency direction. Allowed outcomes:

- `Completed`
- `TimedOut`
- `InterruptedBySafetyOverride`

**Step 2: Preserve sequencing**

- `RecoverAttitudeCenter` returns ok for all center outcomes.
- `RecoverAttitudeCenterThenReverse` still runs reverse after a center timeout or safety override interruption.
- GPS pause/resume behavior is preserved.

**Verify:**

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests "[app][recovery_executor]"
```

---

## Task 7: Preserve ErrorManager Timing

Modify `test/app/error_manager_test.cc`; only modify production code if the test exposes a real mismatch.

**Step 1: Add repeat-window test**

Add a test proving that the 10 second repeated-attitude-limit window starts from recovery finish timestamp, not from original trigger timestamp and not including recovery timeout wait duration.

**Verify:**

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests "[app][error_manager]"
```

---

## Task 8: Align Hardware Integration Helpers

Modify `test/integration/hardware/system_hw_common.h` and `test/integration/hardware/system_hw_main_flow_tests.cc`.

**Step 1: Replace helper phase timeouts**

Use one overall timeout for standalone hardware helper centering logic.

**Step 2: Rely on callbacks for safety**

Hardware helper recovery does not add a new main-limit query. If a main-limit callback stops motion and latches override, helper logic must not clear it unless an expected single-side attitude trigger is observed.

**Step 3: Make single-attempt timeout non-fatal**

Hardware helper tests may log timeout, but should not fail the test solely because one recovery strategy did not reach center. Repeated-trigger policy remains the escalation mechanism.

**Verify:** focused hardware integration tests cannot be executed here unless real hardware is available; at minimum build the affected test targets if present.

---

## Final Verification

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576/aarch64/bin/unit_tests
```

Also run any available hardware integration build target. Real hardware execution remains manual.
