# HeadingCorrector Boundary Design

## Goal

Clarify the responsibility split for heading correction so that:

- `MotionService` is an orchestration layer
- `HeadingCorrector` owns all heading-correction-related computation
- `WalkMotorGroup` remains an actuator layer

This design does **not** change the accepted high-level architecture:

- `MotionService` still owns a `HeadingCorrector`
- `WalkMotorGroup` still owns command delivery, heartbeat resend, and latched `emergency_override`
- `MotionService` still reacts to `override_clear_generation()` and triggers `heading_corrector.reset()`

The change is about **algorithm boundary**, not about moving control back into `WalkMotorGroup`.

## Problem

The current split is improved compared with the old design, but `MotionService` still contains part of the heading-correction pipeline:

- IMU EMA filtering
- filtered-value initialization
- correction invocation timing
- partial input shaping before `HeadingCorrector::compute()`

That leaves heading-correction behavior spread across both `MotionService` and `HeadingCorrector`.

As a result:

- algorithm behavior is harder to reason about
- control tuning is split across two modules
- tests are tempted to observe or reproduce internal behavior outside the controller
- future control changes will continue to leak into `MotionService`

## Target Boundary

### MotionService

`MotionService` should only:

- read raw IMU data
- read raw `WalkMotorGroup` feedback/diagnostics
- maintain the current base speed command for the task state
- decide whether heading correction is currently allowed
- pass raw inputs plus base command into `HeadingCorrector`
- send the resulting final wheel command to `WalkMotorGroup`
- call `heading_corrector.reset()` when `clear_override` has actually taken effect

`MotionService` should **not** perform heading-correction computation.

### HeadingCorrector

`HeadingCorrector` should own **all heading-correction-related computation**, including:

- IMU filtering
- filter initialization state
- timing-driven internal state updates
- stable-window logic
- `LEARN / TRACK / FREEZE` state transitions
- best/reference pose updates
- pitch/roll derived control terms
- wheel-skew feedforward
- final correction output
- final wheel target generation

This makes `HeadingCorrector` the single algorithm owner.

### WalkMotorGroup

`WalkMotorGroup` remains responsible for:

- applying final wheel targets
- resend/heartbeat behavior
- actuator diagnostics
- latched override behavior

It must not regain heading-correction logic.

## Proposed Data Flow

The corrected online path becomes:

1. `MotionService` samples raw IMU and walk feedback
2. `MotionService` builds a thin `HeadingCorrector::Input`
3. `HeadingCorrector` performs the entire correction pipeline internally
4. `HeadingCorrector` returns final wheel targets
5. `MotionService` forwards those targets to `WalkMotorGroup`

The `clear_override` recovery path becomes:

1. `WalkMotorGroup` increments `override_clear_generation()` when latched override is actually released
2. `MotionService` detects the generation change after `group_->update()`
3. `MotionService` calls `heading_corrector.reset()`
4. `MotionService` resumes normal base-command-driven correction on subsequent cycles

This preserves the already-accepted reset semantics while keeping reset ownership inside the controller.

## Interface Direction

`HeadingCorrector::Input` should move toward raw, source-level data rather than half-processed control data.

Expected input shape:

- raw `pitch / roll / yaw / gyro_z`
- raw wheel rpm/current feedback
- base wheel command
- `dt`
- minimal run-state flags if needed

Expected output shape:

- correction terms useful for debug
- final wheel command to apply

`HeadingCorrector` should hide:

- internal filtered values
- internal reference maintenance
- internal mode progression rules

Those may still be exposed through `debug_state()`, but only as read-only diagnostics.

## What Must Move Out of MotionService

The following current `MotionService` responsibilities should migrate into `HeadingCorrector`:

- `filtered_pitch_`
- `filtered_roll_`
- `filtered_yaw_`
- `filtered_omega_z_`
- all `*_inited_` flags for heading-correction filtering
- any future stable-window or freeze pre-processing
- any future feedforward pre-computation that belongs to correction behavior

After migration, `MotionService` should not contain controller-local filtered state.

## What Must Stay in MotionService

The following must remain outside the controller:

- task-level speed selection (`clean_speed`, `return_speed`)
- task direction/sign selection
- runtime config synchronization
- override lifecycle observation
- deciding whether correction is enabled at all
- actuator command submission to `WalkMotorGroup`

In short:

- `MotionService` decides **when** correction is used
- `HeadingCorrector` decides **how** correction is computed

## Migration Strategy

Use a two-step migration to avoid changing control behavior and boundaries at the same time.

### Step 1: Boundary Migration

Move filtering and input shaping into `HeadingCorrector` without changing the core control law.

Success criteria:

- behavior remains approximately the same
- `MotionService` loses controller-local filtered state
- tests compile against the new controller interface

### Step 2: Algorithm Completion

Once the boundary is clean, finish the controller logic inside `HeadingCorrector`:

- real stable-window reset/invalidations
- actual `FREEZE` entry/exit logic
- real wheel-skew feedforward usage
- local reference handling

This keeps architecture cleanup separate from control-law evolution.

## Risks

### Partial Migration

If only filter code moves while reference or mode logic stays in `MotionService`, the split becomes worse, not better.

### Mixed Time Semantics

All controller-internal time behavior must use the same `dt` contract. Avoid some logic using external cadence assumptions and other logic using internal timing.

### Reset Semantics Drift

The accepted `clear_override -> reset()` chain must remain intact. Only the trigger stays in `MotionService`; the reset behavior itself belongs to `HeadingCorrector`.

### Test Drift

Tests should avoid pushing `MotionService` to expose temporary algorithm internals just to make a hardware case easier to write. The controller boundary should drive tests, not the other way around.

## Testing Impact

Unit tests should increasingly target:

- `HeadingCorrector` for algorithm behavior
- `MotionService` for orchestration behavior
- `WalkMotorGroup` for actuator/override behavior

Hardware integration tests for heading correction should ultimately validate the real online path:

- raw IMU/feedback in
- `HeadingCorrector` computes
- final wheel targets out

They should not require `MotionService` to become a debugging transport for controller internals unless there is a separately justified diagnostics need.

## Recommendation

Adopt the full-boundary version:

- move **all heading-correction-related computation** into `HeadingCorrector`
- keep `MotionService` as an orchestration layer
- keep `WalkMotorGroup` as an actuator layer

This is the cleanest long-term boundary and the safest basis for later control-law improvements.
