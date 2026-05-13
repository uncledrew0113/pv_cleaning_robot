# WalkMotorGroup Override Simplification Design

Date: 2026-05-13

## Goal

Simplify `WalkMotorGroup` so its behavior is easier to reason about without changing the external behavior that the rest of the system depends on.

The specific goals are:

- keep `WalkMotorGroup` at the "executor + startup convenience" boundary
- preserve existing public control APIs used by upper layers
- preserve the current strong safety meaning of `emergency_override()`
- remove queue-driven complexity that is no longer justified by current usage
- preserve the old `pid_ctrl_.reset()` effect by moving that responsibility to `MotionService`
- prevent any normal control frame from being sent after an override frame has already been sent

This design is intentionally narrow. It simplifies the internal control-state model of `WalkMotorGroup`. It does not redesign the motion FSM, the CAN protocol, or the `HeadingCorrector` algorithm.

## Current Problems

The current `WalkMotorGroup` implementation mixes several concerns:

- CAN protocol send/receive
- startup hardware convenience (`termination`, `comm_timeout`, batch mode setup)
- periodic executor heartbeat
- emergency latched override state machine
- a queue-like command staging model

Some of that complexity is justified, especially the emergency override safety path. But some of it is residual complexity from the earlier architecture:

- the normal control queue is effectively used as a "latest command wins" buffer
- `pending_clear_override_` exists partly because `clear_override()` must bypass the queue
- comments and state shape still reflect earlier correction-controller coupling
- `last_update_time_` no longer contributes meaningful behavior

This makes the module harder to understand than its current functional role requires.

## Target Boundary

After simplification, `WalkMotorGroup` remains responsible for:

- batch encoding and sending 4-wheel control frames
- receiving and storing 4-wheel status feedback
- periodically re-sending the currently active normal control frame
- enforcing a latched emergency override state
- providing startup convenience operations such as `enable_all()`, `disable_all()`, `set_mode_all()`, termination initialization, and `comm_timeout` initialization

It is explicitly not responsible for:

- heading correction or IMU-based control
- generic command scheduling semantics
- non-latched one-shot preemption semantics

`WalkMotorGroup` will therefore be modeled as a small executor with two control modes, not a partial command dispatcher.

## Chosen Model

Use a two-mode state model inside `WalkMotorGroup`:

- `Normal`
- `LatchedOverride`

### `Normal`

Meaning:

- the module may periodically re-send the current normal control frame
- normal `set_*` APIs update the current normal control slot

### `LatchedOverride`

Meaning:

- `emergency_override()` has taken control
- a high-priority override frame has already been synchronously sent
- periodic normal control re-send must remain blocked
- the module stays latched until `clear_override()` has been requested and that request has been applied in `update()`

No third mode is introduced in this design.

## Public API Semantics

The external API remains compatible.

### APIs that remain unchanged

- `set_speeds(...)`
- `set_currents(...)`
- `set_open_loops(...)`
- `set_positions(...)`
- `set_speed_uniform(...)`
- `emergency_override(...)`
- `clear_override()`
- `update()`

### Preserved semantic meaning

`emergency_override()` keeps its strong existing meaning:

- synchronously send a stop or reverse frame immediately
- enter a latched override state
- block normal periodic control from resuming until explicit `clear_override()`

`clear_override()` keeps its delayed-apply meaning:

- it is a request, not an immediate transition
- the actual mode transition happens inside `update()`

## Internal State Simplification

Replace the normal command queue model with a single normal-control slot.

### Remove

- `Cmd`
- `cmd_buf_`
- `cmd_head_`
- `cmd_tail_`
- `cmd_mtx_`
- `enqueue_cmd()`
- queue draining logic inside `update()`
- `last_update_time_`

### Keep or reshape

- one mutex protecting diagnostic and control state
- one send mutex protecting final transmission ordering
- one explicit control mode field
- one delayed clear request flag
- one normal control slot
- one override control slot

### New internal shape

Recommended internal state:

- `control_mode_`
- `clear_override_pending_`
- `normal_ctrl_frame_`
- `normal_target_values_`
- `has_normal_ctrl_frame_`
- `override_frame_`
- `override_target_values_`
- `diag_`
- `last_fb_time_`
- `send_mtx_`
- `mtx_`

The design intent is that normal control state and override state are separate and explicit.

## Normal Control Semantics

All normal `set_*` APIs become "replace current normal control slot" operations.

Example:

- `set_speeds(...)` encodes one frame and stores it into `normal_ctrl_frame_`
- it also updates `normal_target_values_`
- it does not directly send unless a future design explicitly chooses to do so

Effectively this makes normal control semantics:

- latest command wins
- `update()` re-sends the latest valid normal frame in `Normal` mode

This matches current practical behavior better than the existing queue abstraction, since the queue is already drained into a single final `last_ctrl_frame_` before re-send.

## Override Semantics

### `emergency_override()`

Required behavior:

1. compute override frame
2. transition internal mode to `LatchedOverride`
3. synchronously send the override frame immediately
4. update diagnostics/target values to reflect the actual transmitted override command

This direct send is required to preserve current emergency responsiveness.

### `clear_override()`

Required behavior:

1. mark `clear_override_pending_ = true`
2. return immediately
3. do not directly switch back to `Normal`

### `update()` clear handling

At the beginning of `update()`:

1. refresh online state
2. if `clear_override_pending_` is set and current mode is `LatchedOverride`
   - clear the pending request
   - switch mode back to `Normal`
   - clear the active override slot if needed
   - record an observable "clear applied" event
3. continue with normal send path only if current mode is `Normal`

This preserves the current delayed-apply semantics of `clear_override()`.

## Transmission Ordering Guarantee

This is a mandatory design constraint.

The simplified implementation must preserve the following property:

- once an override frame has already been sent, `update()` must not send a normal control frame afterward unless the override has been explicitly cleared and that clear has been applied

To enforce this, the final "may I send a normal frame" decision must be checked while holding `send_mtx_`.

### Required send-side rule

In `update()`:

1. collect the candidate normal frame
2. lock `send_mtx_`
3. re-check the current `control_mode_`
4. send normal frame only if the mode is still `Normal`

In `emergency_override()`:

1. set mode to `LatchedOverride`
2. lock `send_mtx_`
3. send override frame

This prevents the race where `update()` decides to send a normal frame just before `emergency_override()` sends the override frame.

The guarantee is not "no normal frame was ever sent before emergency". That is impossible if a normal frame has already gone out. The guarantee is:

- no normal frame is sent after the override frame has already been transmitted

## Replacing The Old `pid_ctrl_.reset()` Effect

The old behavior tied `pid_ctrl_.reset()` to the moment when `clear_override` was actually applied, not merely requested.

That behavior must be preserved in the new architecture, but the reset responsibility must live outside `WalkMotorGroup`.

### New ownership

- `WalkMotorGroup` owns only executor mode transitions
- `MotionService` owns `HeadingCorrector` reset behavior

### Required mechanism

`WalkMotorGroup` must expose an observable "override clear applied" signal.

A generation counter is preferred over a boolean flag.

Recommended form:

- `uint32_t override_clear_generation() const`

Semantics:

- increment by 1 each time `clear_override()` is actually applied in `update()`
- monotonically increasing for the lifetime of the object

`MotionService` stores the last seen generation value.

On each update:

1. call `group_->update()`
2. read current `override_clear_generation()`
3. if it differs from the last seen value
   - call `heading_corrector_.reset()`
   - store the new generation
   - skip correction output for that same update cycle

This preserves the old safety meaning of `pid_ctrl_.reset()` without re-coupling the corrector back into the device layer.

## Diagnostics Semantics

`diag_[i].target_value` should represent the most recent command actually made authoritative for the wheel.

That means:

- when normal control slot is updated, target values may be updated to the current normal target
- when an override frame is synchronously sent, target values should be updated to the override target

The implementation should not leave diagnostics ambiguous between:

- normal desired target
- currently authoritative transmitted override target

If needed, that distinction can later be extended, but this design keeps the existing single-field telemetry model and makes it reflect the currently authoritative command.

## Testing Changes

### `WalkMotorGroup` tests

Add or update tests for:

- multiple consecutive `set_speeds()` calls before `update()` result in only the latest normal target being re-sent
- `emergency_override()` sends immediately
- `emergency_override()` blocks subsequent normal periodic sends
- `clear_override()` does not immediately unlock
- one `update()` after `clear_override()` request applies the unlock
- `override_clear_generation()` increments exactly when unlock is applied
- after override frame has been sent, `update()` does not send a normal frame until clear has been applied

### `MotionService` tests

Add or update tests for:

- `HeadingCorrector` reset when override-clear generation changes
- no correction output on the same cycle that reset is triggered by clear-apply
- next cycle resumes normal correction behavior

### Hardware tests

Refocus hardware tests away from old embedded heading-control semantics inside `WalkMotorGroup`.

`WalkMotorGroup` hardware coverage should focus on:

- open/close
- feedback online/offline
- batch command behavior
- emergency override latching
- clear-override recovery

IMU correction behavior belongs to `MotionService + HeadingCorrector` path, not `WalkMotorGroup`.

## Risks And Non-Goals

### Risks

- code that implicitly relied on queue semantics may now observe "latest slot wins" behavior more directly
- diagnostics semantics must be kept consistent during override transitions
- generation handling must be monotonic and thread-safe so `MotionService` cannot miss a clear-apply event

### Non-goals

- no redesign of `HeadingCorrector`
- no redesign of motion FSM behavior
- no introduction of one-shot override semantics
- no redesign of CAN protocol framing

## Recommendation

Implement the simplification as a focused refactor with no public API break for existing callers.

The key outcome should be that `WalkMotorGroup` becomes easy to explain:

- it stores the current normal command
- it periodically re-sends that command in `Normal`
- it immediately sends and latches an override in `LatchedOverride`
- it only returns to `Normal` when `clear_override()` has been requested and applied in `update()`
- `MotionService` resets `HeadingCorrector` when that unlock actually happens

This retains safety semantics while removing the current queue-oriented and historically-coupled complexity.
