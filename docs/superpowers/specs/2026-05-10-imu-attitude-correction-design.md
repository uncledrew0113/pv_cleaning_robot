# IMU Attitude Correction Replacement Design

Date: 2026-05-10

## Goal

Replace the current `omega_z -> 0` heading-rate PID with an IMU-only attitude-correction controller that matches the robot's real installation geometry on PV panels.

The new controller must:

- use IMU data only
- treat local `|pitch|` maximum as the "best aligned" signal
- tolerate unknown initial pose at the start of a run
- avoid reacting to vibration, motor stop shock, seams, bridge connectors, and local panel warp as if they were true heading error
- preserve the existing runtime wiring shape as much as possible

This design is intentionally narrow. It replaces the correction logic, not the FSM, motion API, or CAN frame model.

## Background And Geometry

The robot runs only left-right along PV panel laying direction.

Confirmed IMU installation:

- `Y` axis points along robot running direction
- `X` axis points along PV panel up-down direction
- `Z` axis is approximately normal to panel surface

Current protocol parsing keeps standard Euler field mapping:

- `roll` = rotation about `X`
- `pitch` = rotation about `Y`
- `yaw` = rotation about `Z`

Under this installation:

- when the robot is well aligned on the panel track, `|pitch|` is locally large
- when heading deviates to either side, `|pitch|` decreases
- `roll` changes more strongly than `pitch` and acts as a useful side-indicator near the local optimum

Observed sample data supports this:

- right-biased segment: `pitch ~= -34.17`, `roll ~= -5.98`, `yaw ~= 7.81`
- near-correct segment: `pitch ~= -34.83`, `roll ~= -1.95`, `yaw ~= -0.49`
- left-biased segment: `pitch ~= -34.58`, `roll ~= 1.44`, `yaw ~= -7.36`

So the control objective is not "drive `yaw` to zero". The local objective is "keep `|pitch|` close to its local maximum for the current panel region".

## Why Current Controller Is Not Kept

The current controller uses `gyro[2]` / `omega_z` as the main correction input.

This is a poor fit for the field constraints here because:

- vibration contaminates `omega_z`
- motor acceleration and emergency stop inject large transient angular-rate spikes
- seams and bridge connectors create high-frequency shocks that look like short rotations
- the useful geometric signal for alignment is slow-changing panel attitude projection, not instantaneous angular rate

For this task, `omega_z` will be removed from the main correction path. It may still be used as a disturbance/freeze signal if needed, but not as the control target.

## Chosen Approach

Use an IMU-only extremum-seeking controller:

- primary quality signal: `pitch_abs = |filtered_pitch|`
- direction helper: `filtered_roll`
- disturbance guard: filtered angular-rate / attitude-delta thresholds
- output: bounded differential RPM correction

This is not a PID around a signed error. It is a state machine around a local optimum.

## Controller State Machine

The new controller has four states.

### 1. `UNINITIALIZED`

The controller has just been enabled or reset.

Behavior:

- initialize filters
- do not correct
- do not assume current pose is correct
- wait until IMU signal becomes stable for a minimum warm-up window

Transition:

- stable for `warmup_ms` -> `LEARN`

### 2. `LEARN`

The controller builds local reference values without assuming the run started in a correct pose.

Tracked reference values:

- `pitch_abs_best`
- `roll_at_best`

Learning rules:

- compute `pitch_abs = |filtered_pitch|`
- update `pitch_abs_best` only when signal is stable and `pitch_abs` exceeds the old best by at least `learn_improve_eps`
- when `pitch_abs_best` updates, also store current `filtered_roll` into `roll_at_best`
- allow only slow decay of `pitch_abs_best` over long windows so that local warp or changing panel section can be followed gradually, but shock events cannot immediately drag the reference down

Transition:

- after at least one stable best sample and a minimum learn window -> `TRACK`
- disturbance detected -> `FREEZE`

### 3. `TRACK`

Normal operation.

Correction trigger is threshold-based rather than continuous:

- if `pitch_abs_best - pitch_abs <= pitch_drop_threshold`, output `0`
- if `pitch_abs_best - pitch_abs > pitch_drop_threshold` continuously for `hold_ms`, start correction

Correction direction:

- if `filtered_roll > roll_at_best + roll_threshold`, correct toward one side
- if `filtered_roll < roll_at_best - roll_threshold`, correct toward the other side
- if `filtered_roll` stays inside the deadband, output `0`

Correction magnitude:

- proportional to `pitch_abs_best - pitch_abs`
- bounded by `max_output`
- optionally with a low minimum effective output so small corrections are not lost in drivetrain deadzone

Reference maintenance during tracking:

- continue to refresh `pitch_abs_best` only in stable windows
- never let a single short disturbance re-anchor the best reference

Transition:

- disturbance detected -> `FREEZE`
- controller disabled/reset -> `UNINITIALIZED`

### 4. `FREEZE`

Used to survive seams, bridge connectors, sharp vibration, and stop/start shock.

Behavior:

- output `0`
- do not update `pitch_abs_best`
- do not update `roll_at_best`
- keep filter state running

Enter conditions:

- `|gyro_z_dps| > freeze_gyro_z_threshold`
- or total gyro magnitude too large
- or `|pitch_filt - prev_pitch_filt| / dt` too large
- or `|roll_filt - prev_roll_filt| / dt` too large

Exit conditions:

- all disturbance indicators remain below release thresholds for `freeze_release_ms`

## Signals And Filtering

The controller uses these IMU-derived inputs each update:

- `pitch_deg`
- `roll_deg`
- `gyro_z_dps`
- `dt_s`

Filtering:

- first-order low-pass on `pitch`
- first-order low-pass on `roll`
- optional first-order low-pass on `gyro_z_dps` only for freeze detection

The motion layer should stop low-pass filtering `yaw` for this correction path because `yaw` is no longer the control target.

## Output Semantics

Output remains a signed differential RPM correction added to the existing upper/lower track base speeds.

The drivetrain-side correction application stays unchanged:

- positive correction biases one side
- negative correction biases the other side

Only the correction source changes.

This minimizes risk in `WalkMotorGroup`.

## Interface Changes

The current `HeadingPidController` API is too tied to `omega_z`.

Replace it with an attitude-correction controller that keeps the same ownership location but changes the compute signature.

Recommended shape:

- rename class to something like `HeadingAttitudeController`
- params include:
  - filter gains or time constants
  - learn thresholds
  - tracking thresholds
  - freeze thresholds
  - output limit and minimum effective output
- `compute(...)` takes:
  - `pitch_deg`
  - `roll_deg`
  - `gyro_z_dps`
  - `dt_s`
- returns signed RPM correction

`WalkMotorGroup::update(...)` therefore must receive `pitch_deg` and `roll_deg` in addition to the existing signals.

`MotionService::update()` must pass filtered `pitch`, filtered `roll`, and `gyro_z_dps`.

## File-Level Change Plan

Primary implementation files:

- `include/pv_cleaning_robot/service/heading_pid_controller.h`
- `pv_cleaning_robot/service/heading_pid_controller.cc`
- `include/pv_cleaning_robot/device/walk_motor_group.h`
- `pv_cleaning_robot/device/walk_motor_group.cc`
- `include/pv_cleaning_robot/service/motion_service.h`
- `pv_cleaning_robot/service/motion_service.cc`

Primary test files:

- `test/service/heading_pid_test.cc`
- `test/device/walk_motor_group_test.cc`

Possible integration updates:

- `test/integration/task_chain_test.cc`
- `test/integration/system_integration_test.cc`

No FSM or app-layer behavior changes are intended.

## Initial Parameters

These values are starting points only and must be easy to tune from config.

- `pitch_alpha`: equivalent to `tau ~= 0.4s to 0.8s`
- `roll_alpha`: equivalent to `tau ~= 0.3s to 0.6s`
- `pitch_drop_threshold`: `0.10° to 0.15°`
- `roll_threshold`: `0.4° to 0.8°`
- `hold_ms`: `300ms to 500ms`
- `warmup_ms`: `300ms to 500ms`
- `learn_improve_eps`: `0.03° to 0.05°`
- `best_decay_per_s`: very small, enough to follow slow regional geometry drift only
- `freeze_gyro_z_threshold`: tune from field logs, intentionally conservative
- `freeze_release_ms`: `200ms to 500ms`
- `max_output`: keep existing conservative differential-speed limit
- `min_effective_output`: optional small floor to overcome drivetrain deadzone

## Failure Modes And Mitigations

### Failure: wrong local best captured during transient

Mitigation:

- learn only in stable windows
- require multiple consecutive stable samples before first lock
- freeze on strong transients

### Failure: local panel geometry changes across sections

Mitigation:

- allow very slow best-reference adaptation
- do not pin a global fixed `pitch_ref`

### Failure: correction chatters near optimum

Mitigation:

- threshold + hold logic
- roll deadband
- bounded output

### Failure: large disturbance mistaken for heading error

Mitigation:

- dedicated `FREEZE` state
- no reference update during freeze

## Validation

### Unit Tests

Add tests for:

- warm-up and learn transitions
- best-reference update only in stable windows
- correction stays zero inside pitch-drop deadband
- correction direction flips when roll crosses `roll_at_best`
- disturbance enters `FREEZE`
- `FREEZE` suppresses output and learning

### Recorded-Sequence Replay

Use the provided three-column IMU sequence:

- right-biased segment should command correction toward center
- near-correct segment should command little or no correction
- left-biased segment should command correction toward center

This replay is a required regression target because it reflects the real geometry assumption behind the new controller.

### Build/Test Scope

- build `unit_tests`
- run `unit_tests` for service and walk-motor-group coverage

Hardware validation is required later, but not part of the first code-change loop.

## Non-Goals

- do not redesign FSM behavior
- do not add new sensors
- do not depend on limit switches, wheel odometry, or cloud runtime for correction logic
- do not preserve the old `omega_z` PID as the main closed-loop controller
