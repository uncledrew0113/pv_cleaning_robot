# UDS Gyro Correction Compare Design

## Background

The current hardware PID test path uses UDS yaw as the heading error source and maps the
PID correction into wheel RPM commands. Field tests show that the robot can become
mechanically skewed even when UDS yaw appears to improve. The likely causes include
different upper/lower wheel traction, groove-wheel edge contact, and limited correction
authority at normal working speed.

The first phase will validate alternative correction strategies in hardware tests only.
It will not change the production default behavior of `MotionService` or `HeadingCorrector`.

## Goals

- Split the oversized `test/integration/hardware/system_hw_test.cc` into smaller hardware
  test modules without changing test behavior.
- Add a configurable UDS yaw plus IMU Z-axis gyro fusion algorithm for test comparison.
- Add hardware comparison tests that keep the same mission execution flow as
  `[hw_system][pid_combined]`.
- Make correction and fusion parameters configurable through
  `test/integration/hardware/hw_test_config.json`, so tuning does not require recompilation.

## Non-Goals

- Do not change the production default correction behavior.
- Do not use odometry as a control input for the new comparison strategies.
- Do not replace the existing `[hw_system][pid_combined]` test.
- Do not add a separate manual `set_speeds()` driving loop for these comparison tests.

## Hardware Test Split

`system_hw_test.cc` will be split into focused files compiled into the same `hw_tests`
executable:

- `system_hw_fixture.h/.cc`
  - `SystemHwFixture`
  - motion config construction
  - full-system initialization
  - safety bridge setup
  - graceful shutdown
- `system_hw_metrics.h/.cc`
  - JSONL metrics helpers
  - PID and correction comparison sample records
  - segment and final summaries
- `system_hw_motion_tests.cc`
  - existing full-system mission-chain tests such as `full_init`, `combined`, and
    `pid_combined`
- `system_hw_attitude_tests.cc`
  - existing attitude tests such as `lower_uds_zero` and `lower_attitude_center`
  - new correction comparison tests

The split should preserve existing test tags and behavior. CMake will include the new
source files in `hw_tests`.

## UDS Gyro Fusion

Add a small reusable class for test-side yaw post-processing:

```text
UdsGyroYawFusion
```

Input:

- UDS yaw angle in degrees
- UDS validity, confidence, and age
- IMU `gyro_z` in radians per second
- `dt_s`

Output:

- fused angle in degrees
- validity
- `gyro_z_dps`
- innovation in degrees
- Kalman gain/debug values

Sign convention:

- clockwise robot attitude offset: angle < 0 and gyro_z < 0
- counterclockwise robot attitude offset: angle > 0 and gyro_z > 0

The fusion model is a 1D Kalman filter with angle and gyro bias:

```text
state:
  angle_deg
  gyro_bias_dps

predict:
  angle_deg += (gyro_z_dps - gyro_bias_dps) * dt_s
  gyro_bias_dps unchanged

measure:
  z = uds_yaw_deg
```

UDS measurements correct gyro drift. Gyro prediction bridges short UDS latency or missing
samples. If UDS is unavailable for longer than `max_gyro_only_ms`, the fused output becomes
invalid and correction returns to the base command.

## Correction Strategy Runner

The comparison tests will keep the same high-level mission execution logic as
`[hw_system][pid_combined]`:

- initialize `SystemHwFixture`
- start the safety bridge
- start the configured mission from the active dock
- let limit switches settle and advance segments
- run watchdog, health, brush, motion, and navigation updates as in the existing chain
- finish when the controller returns to `Idle`

The new tests only change the correction algorithm variant used during the mission. They
do not use odometry for control decisions. Odometry may still be logged as an observation.

## Strategy Matrix

Add these hardware test tags:

- `[hw_system][corr_raw_slow_all]`
- `[hw_system][corr_raw_slow_lower_only]`
- `[hw_system][corr_raw_slow_top_decel_only]`
- `[hw_system][corr_fused_fast_all]`
- `[hw_system][corr_fused_fast_lower_only]`
- `[hw_system][corr_fused_slow_all]`
- `[hw_system][corr_fused_slow_lower_only]`
- `[hw_system][corr_fused_slow_top_decel_only]`

Strategy dimensions:

- `raw`: use raw UDS yaw angle
- `fused`: use UDS plus gyro fused angle
- `fast`: keep the normal test working speed
- `slow`: when absolute angle exceeds the configured threshold, reduce base speed to
  `slow_base_rpm`
- `all`: upper and lower tracks both participate in correction
- `lower_only`: upper wheels stay at base speed, lower wheels receive correction
- `top_decel_only`: both tracks participate, but upper wheels can only reduce absolute speed,
  never increase it

The correction sign follows the current project convention:

```text
error = -angle
correction = PID(error)
```

Wheel command generation must preserve the current direction conventions:

- toward A: upper wheels are positive, lower wheels are negative
- toward B: upper wheels are negative, lower wheels are positive

Commands must be clamped to configured limits and must not cross zero unless a specific
test explicitly allows reversal. The first phase will not allow reversal.

## Configuration

Add a `correction_compare` section to
`test/integration/hardware/hw_test_config.json`:

```json
{
  "correction_compare": {
    "slow_base_rpm": 15.0,
    "yaw_slow_threshold_deg": 1.0,
    "max_output": 10.0,
    "min_effective_output": 1.0,
    "kp": 5.0,
    "ki": 0.0,
    "kd": 0.0,
    "integral_limit": 1.0,
    "fusion": {
      "process_noise_angle": 0.05,
      "process_noise_bias": 0.001,
      "measurement_noise_uds": 0.5,
      "initial_angle_variance": 1.0,
      "initial_bias_variance": 1.0,
      "max_gyro_only_ms": 300
    }
  }
}
```

`hw_config.h` will load this section into a typed configuration structure. Every comparison
test uses the same config values; only the strategy enum changes.

## Metrics

Each comparison test writes JSONL metrics with enough data to compare strategies:

- strategy name
- angle source: raw or fused
- raw UDS yaw
- fused yaw
- UDS validity, confidence, and age
- gyro Z in degrees per second
- fusion innovation and Kalman debug values
- base RPM
- correction RPM
- four wheel target RPM
- four wheel actual RPM
- segment duration
- final summary

Odometry may be included as observation-only fields, but it must not affect the correction
algorithm in this phase.

## Validation

Implementation validation should include:

- unit tests for the fusion filter prediction, UDS update, gyro-only timeout, and sign convention
- unit tests for wheel command generation for all strategies and both travel directions
- cross-build of `unit_tests`
- cross-build of `hw_tests`
- hardware execution of selected comparison tests on the target board

The first useful hardware comparison should include:

```bash
./hw_tests "[hw_system][pid_combined]"
./hw_tests "[hw_system][corr_raw_slow_top_decel_only]"
./hw_tests "[hw_system][corr_fused_slow_top_decel_only]"
```

These give a baseline, the best raw-angle candidate, and the best fused-angle candidate.
