# ODrive Brush Motor Design

## Summary

Replace the current Modbus-based brush motor implementation with an ODrive 3.6 UART ASCII implementation.
The brush motor remains exposed as `BrushMotor`, but its control semantics become ODrive-native:

- speed setpoint commands immediately drive the motor
- torque mode is supported as a first-class device capability
- runtime status and diagnostics are read back from ODrive through ASCII `f` and `r` commands
- robustness is improved with an explicit ODrive watchdog keepalive strategy

This spec covers only the brush motor migration. It does not change the walk motor CAN stack.

## Goals

- Move brush motor control from Modbus RTU to ODrive 3.6 UART ASCII.
- Keep brush telemetry available: speed, voltage, current, temperature, fault/error state.
- Support both speed mode and torque mode at the device layer.
- Use speed mode as the default business-control mode.
- Add a watchdog keepalive strategy based on ODrive `u motor`.
- Support a fault-only transition into an uncontrolled / idle state.

## Non-Goals

- No CAN-based ODrive integration in this project.
- No dual-stack Modbus/ODrive runtime selection in the same implementation.
- No redesign of `HealthService` brush telemetry JSON shape.
- No unrelated refactoring of FSM, navigation, or walk motor control.

## Existing Context

The current `BrushMotor` is a Modbus register wrapper with this behavior:

- `set_rpm()` writes a target RPM register
- `start()` enables the drive
- `stop()` disables the drive
- `update()` polls status registers

That control model does not match ODrive ASCII behavior. In ODrive UART ASCII:

- a speed command such as `v axis velocity ...` directly drives the axis
- a torque command such as `c axis torque` directly drives the axis
- `u axis` only refreshes the internal watchdog timer
- `f axis` returns current position and velocity
- `r property` reads numeric properties

Therefore the old `set_rpm() + start()` control pattern must be removed. The business layer should use ODrive-native semantics instead of preserving the old Modbus mental model.

## Configuration

Extend the existing brush serial config:

```json
"serial": {
  "brush": {
    "port": "/dev/ttyS3",
    "baudrate": 115200,
    "axis": 0,
    "counts_per_rev": 8192,
    "watchdog_enabled": true,
    "watchdog_timeout_s": 0.5
  }
}
```

Field meanings:

- `port`
  - UART device connected to ODrive
- `baudrate`
  - UART baudrate for ODrive ASCII communication
- `axis`
  - ODrive axis index used by the brush motor, `0` or `1`
- `counts_per_rev`
  - conversion parameter between business RPM and ODrive velocity units
- `watchdog_enabled`
  - whether keepalive should be managed by the brush motor device
- `watchdog_timeout_s`
  - configured watchdog timeout used by the device-level keepalive logic

The ODrive axis selection must be configurable, not hard-coded.

## Device Architecture

Keep the `BrushMotor` name and device role, but replace its internals completely.

The device should depend on a UART-style serial interface rather than `IModbusMaster`.
Internally it should:

- send ASCII control commands
- read ASCII responses
- parse numeric responses into cached diagnostics
- maintain control mode, target cache, and watchdog state

This keeps service-layer dependencies narrow while still allowing the device API to evolve where Modbus semantics no longer make sense.

## Device API

### Keep

- `open()`
- `stop()`
- `set_rpm(int rpm)`
- `clear_fault()`
- `update()`
- `get_status()`
- `get_diagnostics()`

### Add

- `set_mode_speed()`
- `set_mode_torque()`
- `set_torque(float torque_nm)`
- `enter_idle()`

### Remove

- `start()`

## Control Semantics

### Speed Mode

`set_mode_speed()` selects speed-mode behavior in the device cache and prepares subsequent control commands to use ODrive velocity control.

`set_rpm(rpm)`:

- converts RPM into ODrive velocity units using `counts_per_rev`
- immediately sends a `v axis velocity 0` command
- updates the cached target RPM
- marks the device as controlled and active

The motor should begin running immediately after `set_rpm()` if the command succeeds.

### Torque Mode

`set_mode_torque()` selects torque-mode behavior in the device cache.

`set_torque(torque_nm)`:

- immediately sends a `c axis torque_nm` command
- updates cached torque target state
- marks the device as controlled and active

Speed mode remains the default business-facing mode, but torque mode is a formal device capability for future scenarios.

### Normal Stop

`stop()` means controlled stop, not uncontrolled release.

Its semantics are:

1. send a zero-output command appropriate for the current active mode
   - speed mode: send zero velocity
   - torque mode: send zero torque
2. update cached target state to stopped
3. mark the device as no longer requiring watchdog keepalive

After a normal stop, the device should **not continue feeding the watchdog**.

### Fault-Only Uncontrolled Release

`enter_idle()` is separate from `stop()`.

It is intended for fault handling only, not normal business flow.

Its semantics are:

1. stop future watchdog keepalive immediately
2. clear controlled-target cache
3. attempt to move the axis into an uncontrolled / idle state
4. if the direct idle transition is unavailable for the configured firmware path, fall back to:
   - sending a zero-output command once
   - stopping keepalive
   - allowing watchdog timeout to remove active control

This gives the system two distinct stop behaviors:

- `stop()` = controlled stop
- `enter_idle()` = fault-only uncontrolled release

Only fault-handling paths should call `enter_idle()`.

## ODrive Command Mapping

### Control Commands

- speed command:
  - `v <axis> <velocity_counts_per_sec> 0`
- torque command:
  - `c <axis> <torque_nm>`
- watchdog keepalive:
  - `u <axis>`
- clear fault:
  - `sc`

### Readback Commands

- current position and velocity:
  - `f <axis>`
- voltage:
  - `r vbus_voltage`
- motor current:
  - `r axis<axis>.motor.current_control.Iq_measured`
- temperature:
  - `r axis<axis>.motor.fet_thermistor.temperature`
- error fields:
  - `r axis<axis>.motor.error`
  - `r axis<axis>.error`
  - `r axis<axis>.controller.error`

### Idle / Release Transition

The ASCII docs provided for this project do not clearly document one canonical "idle" shortcut command.
Therefore the implementation should model `enter_idle()` as a state transition abstraction rather than hard-coding a single undocumented command in the spec.

The implementation may use an ASCII writable state property path such as `w axis<axis>.requested_state ...` if that is confirmed by the runtime environment. If not, the fallback behavior is to stop issuing keepalive and allow watchdog expiry to end active control.

## Unit Conversion

Business-layer speed remains in RPM.
ODrive velocity control uses encoder-based velocity units.

The device must convert:

- `RPM -> counts/s` when sending speed commands
- `counts/s -> RPM` when parsing `f <axis>` readback

The conversion factor must come from `counts_per_rev` configuration, not from hard-coded constants.

## Status and Diagnostics Mapping

The external telemetry shape should remain as stable as possible.

Suggested mapping:

- `actual_rpm`
  - parsed from `f <axis>` velocity and converted to RPM
- `target_rpm`
  - cached from the most recent successful speed command
- `current_a`
  - from `Iq_measured`
- `bus_voltage_v`
  - from `vbus_voltage`
- `temperature_c`
  - from `fet_thermistor.temperature`
- `fault`
  - true if any relevant ODrive error field is non-zero
- `fault_code`
  - aggregated or reduced from axis/motor/controller error fields
- `running`
  - derived from control state, target state, and fault state rather than from a Modbus-style status bit

`HealthService` should continue exporting the same brush JSON fields:

- `running`
- `fault`
- `rpm`
- `target`
- `current`
- `voltage`
- `temp`
- `stalls`
- `comm_err`

Even though the source protocol changes, the telemetry shape should not be needlessly changed.

## Concurrency and Serial Access Model

All ODrive ASCII communication must be serialized.

There must be one internal serial mutex for:

- speed commands
- torque commands
- mode changes
- stop
- idle/release
- status polling
- watchdog keepalive

No two threads should write to the UART concurrently.

The current project structure already keeps brush updates on a low-priority periodic thread rather than in the high-rate walk-control loop. That pattern should remain.

## Watchdog Strategy

When `watchdog_enabled == false`:

- do not send `u axis`

When `watchdog_enabled == true`:

- `v ...` and `c ...` count as watchdog-refreshing control activity
- if a cycle already sent a control command, do not send an extra `u`
- if no control command was sent during the cycle, but the brush is still meant to remain actively controlled, `update()` sends `u axis`

This yields the intended behavior:

- changing speed/torque: command itself refreshes watchdog
- steady-state running: periodic `u axis` keeps control alive
- after `stop()`: no keepalive
- after `enter_idle()`: no keepalive

## Polling Strategy

Do not read every diagnostic field at the same frequency.

Suggested polling tiers inside `update()`:

- high frequency:
  - `f <axis>`
- medium frequency:
  - `r vbus_voltage`
  - `r axis<axis>.motor.current_control.Iq_measured`
- low frequency:
  - `r axis<axis>.motor.fet_thermistor.temperature`
  - `r axis<axis>.motor.error`
  - `r axis<axis>.error`
  - `r axis<axis>.controller.error`

This keeps UART load reasonable while preserving useful diagnostics.

## Error Handling

- single command send/read failure:
  - increment `comm_error_count`
  - leave previous cached diagnostics intact
- repeated failures:
  - continue preserving last good telemetry rather than zeroing fields
- ODrive error fields become non-zero:
  - set `fault = true`
  - update `fault_code`
- `clear_fault()`:
  - send `sc`
  - later `update()` validates whether the error fields have cleared

The implementation should not assume every readback path always succeeds. Cached state must remain usable across transient UART failures.

## Service-Layer Changes

### `MotionService`

Change brush control to ODrive-native usage:

- start cleaning:
  - `set_mode_speed()`
  - `set_rpm(brush_rpm)`
- stop cleaning:
  - `stop()`
- start returning with reverse brush:
  - `set_mode_speed()`
  - `set_rpm(-return_brush_rpm)`
- torque-specific scenarios:
  - may later call `set_mode_torque()` + `set_torque()`

`MotionService` should no longer call `start()`.

### `FaultHandler`

Fault paths that require brush release should call `enter_idle()`, not `stop()`.

This preserves the distinction between:

- normal stop for business flow
- uncontrolled release for fault handling

### `HealthService`

No structural telemetry redesign is needed.
It continues consuming cached `BrushMotor` status and diagnostics.

## Testing

### Unit Tests

Add or replace brush motor tests to cover:

- mode changes:
  - `set_mode_speed()`
  - `set_mode_torque()`
- speed command send and RPM conversion
- torque command send
- `stop()` behavior in speed and torque contexts
- `enter_idle()` behavior and watchdog disable
- `f` response parsing
- `r` response parsing for voltage/current/temperature/error
- watchdog logic:
  - control-command cycle does not send extra `u`
  - steady-state active cycle sends `u`
  - stopped state does not send `u`
- fault aggregation from ODrive error fields

### Service Tests

Update motion-service tests to match the new control semantics:

- no expectation of `start()`
- start-cleaning now expects mode selection + speed command
- stop-cleaning expects `stop()`
- reverse-return expects negative speed command

### Hardware Tests

Add target-machine ODrive UART integration tests that verify:

- UART connectivity to ODrive
- speed command successfully drives the axis
- `f` returns valid velocity data
- `r` returns voltage/current/temperature values
- watchdog keepalive maintains steady operation
- after stopping keepalive, watchdog expiry removes active control as expected

## Compatibility

This is not source-compatible with the old Modbus-oriented `BrushMotor::start()` semantics.

The compatibility strategy is:

- keep the `BrushMotor` device role and telemetry shape
- intentionally change the control API to match ODrive behavior
- update service-layer call sites accordingly

That is preferable to preserving a misleading API that no longer matches the hardware.

## Acceptance Criteria

- brush motor control uses ODrive 3.6 UART ASCII instead of Modbus
- default business flow uses speed mode
- torque mode is implemented as a formal device capability
- readback and diagnostics are preserved through `f` and `r`
- watchdog keepalive is implemented with explicit `u axis` management
- `stop()` performs normal controlled stop and does not keep feeding the watchdog
- `enter_idle()` exists for fault-only uncontrolled release
- fault-handling paths use `enter_idle()` rather than normal stop
- `HealthService` brush telemetry shape remains stable
