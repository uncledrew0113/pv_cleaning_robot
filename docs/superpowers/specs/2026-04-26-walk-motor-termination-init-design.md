# Walk Motor Termination Init Design

## Summary

Add a startup-time CAN bus termination initialization step to `WalkMotorGroup::open()`.
After the CAN interface is opened successfully, and before any other motor initialization traffic is sent, the device layer may send the existing protocol command that enables the termination resistor on one configured motor. This behavior is enabled by default but can be turned off for deployments where the bus already has external termination.

This spec covers only subproject A. The brush motor migration to ODrive 3.6 is out of scope and will be designed separately.

## Goals

- Ensure the termination-resistor command is sent immediately after CAN open and before other motor commands.
- Keep the behavior inside `WalkMotorGroup::open()` so callers do not need to remember ordering.
- Allow deployments to disable the behavior when the CAN bus already has physical termination.
- Allow the target motor to be configured by physical `motor_id`.
- Improve robustness by supporting repeated sends during initialization.

## Non-Goals

- No changes to the brush motor stack.
- No new application-level startup steps in `main.cc` or services.
- No automatic detection of bus topology or resistor state.
- No protocol-layer changes; the termination command already exists and will be reused.

## Existing Context

`WalkMotorGroup::open()` already performs startup-time device initialization:

1. Open the CAN bus
2. Set RX filters
3. Start the receive thread
4. Optionally write `comm_timeout_ms` to each motor

The protocol layer already supports termination configuration through CAN ID `0x109`, and `WalkMotorGroup` already exposes `set_terminations(...)` for manual use.

The new requirement is not "before every speed command". It is a one-time startup action: after CAN is opened, send the termination command first, then continue with other initialization and later runtime control traffic. One send is usually sufficient while power remains applied, but the startup path should support retrying the same command several times for robustness.

## Configuration

Add the following configuration fields under `can.walk_motor`:

```json
"can": {
  "interface": "can0",
  "walk_motor": {
    "motor_id": 1,
    "comm_timeout_ms": 200,
    "termination_init_enabled": true,
    "termination_init_retry_count": 3,
    "termination_motor_id": 2
  }
}
```

Field meanings:

- `termination_init_enabled`
  - Whether `WalkMotorGroup::open()` automatically sends the termination initialization command.
  - Default: `true`
- `termination_init_retry_count`
  - Number of times to send the termination initialization frame during startup.
  - `1` means send once.
  - Recommended default: `3`
- `termination_motor_id`
  - Physical motor ID whose internal termination resistor should be enabled.
  - Interpreted as a real bus `motor_id`, not as `LT/RT/LB/RB`.

## Design

### API Surface

Keep the behavior internal to `WalkMotorGroup::open()`.

`WalkMotorGroup` construction will gain configuration inputs for:

- whether startup-time termination init is enabled
- retry count
- target `motor_id`

No new startup call will be required from `main.cc`, `MotionService`, tests, or other callers.

### Startup Order

`WalkMotorGroup::open()` will execute in this order:

1. Open CAN
2. Set receive filters
3. Start receive thread
4. If `termination_init_enabled == true`, send the termination initialization frame for `termination_motor_id`, repeated `termination_init_retry_count` times
5. If `comm_timeout_ms > 0`, send the existing per-motor comm-timeout initialization frames
6. Return from `open()`

This preserves the requirement that the termination command is the first motor-related initialization traffic after CAN becomes available.

### Mapping `termination_motor_id` to the Existing Protocol

The protocol encoder already expects an 8-slot termination bitmap.

The implementation will:

- validate that `termination_motor_id` belongs to the current `WalkMotorGroup`
- build an 8-slot bitmap with only the target motor slot set to `true`
- reuse the existing termination command encoder to generate the `0x109` frame

For the current deployment, that means `termination_motor_id = 2` produces the existing desired command effect.

### Validation Rules

`termination_motor_id` must belong to the current 4-motor group controlled by the instance:

- for `id_base = 1`, valid IDs are `1..4`
- for `id_base = 5`, valid IDs are `5..8`

If `termination_init_enabled == true` and the configured `termination_motor_id` is outside the active group, this is a deterministic configuration error and `open()` should fail fast rather than silently doing nothing.

`termination_init_retry_count` should be treated as a positive count. If configuration parsing provides `0`, the implementation should clamp it to `1` rather than interpreting it as "disabled". The enable/disable switch is `termination_init_enabled`.

## Error Handling

Preserve current hard-failure boundaries for CAN availability:

- if CAN open fails, `open()` fails
- if filter setup fails, `open()` fails

For termination initialization:

- send the frame `termination_init_retry_count` times
- if one or more sends fail, continue retrying until the count is exhausted
- if all sends fail, log the failure and continue with the rest of `open()`
- do not fail `open()` solely because the termination init frame could not be transmitted

Rationale:

- CAN availability failures mean the device cannot function at all
- termination initialization is important for bus robustness, but a transient TX failure should not make startup more fragile than the current implementation
- the configuration error case is different and should fail fast because it is local, deterministic, and actionable

The existing `comm_timeout` behavior remains unchanged: write attempts are best-effort and do not cause `open()` to fail.

## Logging

Add startup logs for observability:

- `info` when termination initialization is enabled and attempted, including target `motor_id` and retry count
- `warn` for individual failed send attempts
- `error` if all termination-init sends fail, explicitly stating that bus reliability may depend on external termination
- `info` when termination initialization is disabled by configuration

## Testing

### Unit Tests

Add or update `WalkMotorGroup` tests to cover:

- when `termination_init_enabled = true`, `open()` sends the termination frame before `comm_timeout` frames
- when `termination_init_enabled = false`, no termination frame is sent
- retry count is honored
- the termination bitmap enables only the configured `termination_motor_id`
- invalid `termination_motor_id` for the active group causes `open()` to fail
- partial send failure followed by later success still allows `open()` to continue
- all termination-init sends failing still allows `open()` to return success, while recording the expected send behavior

These tests should use the existing mock CAN bus and inspect sent frame order and payload.

### Hardware Tests

Add a regression-oriented hardware test that confirms the normal startup path still works when termination initialization is present:

- `open()`
- `set_feedback_mode_all(...)`
- `enable_all()`
- `set_mode_all(...)`

This test only needs to verify that startup and normal control commands still work after the new init step. It does not need to read back resistor state unless a stable hardware readback path already exists.

## Compatibility

This change is backward compatible for most deployments:

- default behavior enables startup-time termination init
- deployments with external bus termination can explicitly disable it
- no caller-side sequence changes are required

The only new hard failure is misconfiguration where startup-time termination init is enabled but the configured `termination_motor_id` does not belong to the active `WalkMotorGroup`.

## Acceptance Criteria

- `WalkMotorGroup::open()` can automatically send the termination init frame before other motor initialization commands
- the behavior is enabled by default and can be disabled via config
- the target motor is configured by physical `motor_id`
- repeated sends are supported for robustness
- invalid target IDs are rejected deterministically
- existing caller code does not need to add manual initialization steps
