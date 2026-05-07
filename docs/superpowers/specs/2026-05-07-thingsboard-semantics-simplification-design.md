# ThingsBoard Semantics Simplification Design

Date: 2026-05-07

## Goal

Simplify the current ThingsBoard-facing runtime model so that cloud config, business logic, sensor interpretation, FSM state flow, telemetry, and tests all follow one minimal and internally consistent semantics.

This work is explicitly driven by four constraints:

- only integer `passes` are supported
- the parking side has only two valid values: `left` or `right`
- the charging side is always the parking side
- physical `front_limit` / `rear_limit` sensor names must stop acting as business truth

The result should remove stale or duplicated runtime concepts rather than layering new abstractions on top of them.

## Non-Goals

This design does not add any new product capability.

It explicitly excludes:

- half-pass or `passes = 0.5`
- dual-terminal parking policies
- an independently configurable charging side
- compatibility shims that preserve the old semantic model in parallel
- OTA, LoRaWAN, or unrelated runtime refactors

## Required Outcome

After this work:

- shared attributes, runtime state, RPC admission, telemetry, and tests must all use the same simplified business model
- the code must no longer rely on `at_home`, `at_front`, `parking_policy`, `charging_side`, `terminal_a`, or `terminal_b` as business-level truth
- `front_limit` and `rear_limit` may remain as hardware-facing names in low layers, but all business decisions must go through a parking-side-aware mapping
- the final code shape should be smaller and easier to reason about than the current implementation

## Simplified Business Model

### 1. Parking Side

The only business-side terminal configuration is:

- `parking_side = left`
- `parking_side = right`

`parking_side` determines all of the following:

- where the robot is allowed to start a new task
- where the robot must end a completed task
- where the robot returns for task-level return flow
- where the robot may reset after `Fault` or `Terminated`
- where charging occurs

No second configuration is needed for charging location.

### 2. Passes

`passes` must be a positive integer.

Task semantics:

- `passes = 1` means one full out-and-back cycle
- `passes = 2` means two full out-and-back cycles
- task completion always happens after returning to the configured parking side

Half-pass semantics do not exist in the simplified model.

### 3. Sensor Truth vs. Business Truth

The confirmed physical mapping is:

- `front_limit` is the left-side proximity sensor
- `rear_limit` is the right-side proximity sensor

These physical names are not valid business truth by themselves.

Business logic must instead operate on:

- `parking-side sensor`
- `far-end sensor`

derived from `parking_side`.

Required mapping:

- when `parking_side = left`
  - parking-side sensor = left sensor = `front_limit`
  - far-end sensor = right sensor = `rear_limit`

- when `parking_side = right`
  - parking-side sensor = right sensor = `rear_limit`
  - far-end sensor = left sensor = `front_limit`

This mapping must become the single source of truth for startup admission, task completion, return completion, and reset admission.

### 4. State Semantics

The runtime remains state-machine-based, but the meaning of states must be simplified.

Recommended state set:

- `Idle`
- `Charging`
- `CleanOut`
- `CleanBack`
- `Returning`
- `Paused`
- `Fault`
- `Terminated`

If the implementation keeps `CleanFwd` / `CleanReturn` names for code-churn reasons, their business meaning must still be:

- `CleanFwd` = moving away from the parking side toward the far end
- `CleanReturn` = moving back toward the parking side

`Charging` must become strict again:

- it only means the robot is stopped at the configured parking side after task completion or return completion
- it must not be used as a generic synonym for “task stopped” or “finished somewhere”

### 5. Start and End Rules

Task start:

- allowed only when the robot is currently at the configured parking side
- allowed only from `Idle` or `Charging`

Task end:

- a normal task may only complete after returning to the configured parking side
- completing at the far end is invalid in this model

Reset:

- allowed only from `Fault` or `Terminated`
- allowed only while the robot is at the configured parking side

## Runtime Architecture Changes

The runtime path should be simplified to:

`sensor facts -> parking-side mapping -> supervisor admission -> FSM transition -> telemetry/event output`

This keeps hardware interpretation separate from business admission, and keeps ThingsBoard-specific code out of business-state decisions.

## Module Design

### 1. `ThingsBoardConfigManager`

Reduce the runtime config model to:

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_side`
- `schedules`

Remove:

- `parking_policy`
- `charging_side`
- `TerminalSide`
- `ParkingPolicy`
- `ChargingSide`
- `allows_half_pass`
- `can_start_from_terminal`
- `supports_charging_at`

Validation rules:

- `passes` must be a positive integer
- `parking_side` must be `left` or `right`
- schedules continue to apply immediately to active scheduler state
- task parameters continue to flow through `pending -> active`

The config manager remains responsible for:

- parsing shared attributes
- validating supported fields
- persisting `config.json` and `config.pending.json`
- promoting pending config before the next task

It no longer encodes multi-terminal business semantics.

### 2. Sensor-to-Business Mapping

Introduce one narrow runtime mapping path that converts physical limit-switch readings into business position facts.

The rest of the codebase should read:

- `at_parking_side`
- `at_far_end`

and should not directly branch on `front_limit` vs. `rear_limit` for business decisions.

This mapping may live in a small helper or a thin service-level utility. The design requirement is semantic centralization, not a new heavyweight abstraction.

### 3. `RobotSupervisor`

Keep `RobotSupervisor`, but simplify it into a thin coordination layer.

Retained responsibilities:

- `start_task()`
- `pause_task()`
- `return_task()`
- `terminate_task()`
- `reset_task()`
- `snapshot()`

Removed or merged responsibilities:

- merge `start_scheduled_task()` and `start_manual_task()` into one start path
- remove `desired_cloud_period_ms()`
- remove any use of old `at_home` / `at_front` semantics

`start_task()` should only:

1. verify the robot is at the configured parking side
2. promote pending config if needed
3. dispatch the unified start event to the FSM

`snapshot()` remains the telemetry assembly point for:

- FSM state
- active and pending config
- command tracking state

The supervisor should not contain parallel terminal semantics or cloud-specific policy.

### 4. `RobotFsm`

Simplify task flow to a strict out-and-back model.

Required behavior:

- start from parking side
- move outward toward far end
- when far-end sensor settles, reverse into return-cleaning phase
- when parking-side sensor settles, either start the next pass or complete the task

Task counting rule:

- one completed `CleanOut + CleanBack` cycle equals one completed pass

Delete any behavior that allows:

- half-pass completion
- far-end completion
- completion semantics that depend on separate charging-side rules

`Returning` remains valid for:

- manual return
- low-battery return
- fault-triggered return

but return completion must also be defined by reaching the configured parking side.

### 5. `ThingsBoardControlPlane`

Keep it as the protocol adaptation layer only.

Responsibilities stay limited to:

- subscribing shared attributes
- registering RPC handlers
- publishing startup attributes
- publishing status events
- publishing command events
- publishing business telemetry

It must stop embedding old semantic assumptions in:

- RPC rejection reasons
- telemetry state interpretation
- config field names

RPC and telemetry must both reflect the simplified parking-side model.

### 6. `CloudService`

The current startup-order race was already partially corrected in `main.cc` by registering shared attributes and RPC handlers before `connect()`.

Remaining required change:

- propagate the incoming RPC `request_id` through the full command path so command events can be correlated with the original ThingsBoard request

This is a confirmed correctness bug in the current design because command telemetry currently loses request correlation even though the response topic still uses the request id.

## Key Business Flows

### 1. Start Task

Preconditions:

- current state is `Idle` or `Charging`
- robot is at the configured parking side
- pending config promotion succeeds if pending config exists

Flow:

- supervisor admits the command
- FSM starts a new integer-pass task from the parking side
- runtime enters `CleanOut`

### 2. Far-End Turnaround

Preconditions:

- state is outward cleaning
- the far-end sensor settles

Flow:

- FSM switches to return-cleaning direction
- no task completion occurs here

### 3. Return to Parking Side

Preconditions:

- state is return-cleaning or returning-home
- the parking-side sensor settles

Flow:

- if in return-cleaning and passes remain, begin next outward pass
- if in return-cleaning and no passes remain, complete task at the parking side
- if in `Returning`, complete the return flow at the parking side

### 4. Pause / Return / Terminate / Reset

- `pause` only from active cleaning states
- `return` only from active cleaning or paused states
- `terminate` only from active cleaning, paused, or returning
- `reset` only from `Fault` or `Terminated`, and only while at parking side

## Confirmed Problems To Fix

### 1. Physical Sensor Names Still Leak Into Business Decisions

Current code still uses `rear` as home-like truth and `front` as front-like truth in multiple runtime paths.

Given the confirmed hardware meaning, this becomes wrong whenever parking side is changed.

Required fix:

- centralize parking-side-aware sensor mapping
- remove direct business branching on physical `front` / `rear` meanings

### 2. Config Model and Runtime Model Are Split

Current code already introduced richer terminal semantics in config handling, but supervisor and FSM still use the old home/front model.

Required fix:

- collapse config semantics and runtime semantics into the same simplified parking-side model

### 3. `Charging` Is Still Overloaded

Current task-complete behavior includes flows that can end in `Charging` from the wrong side or for the wrong reason.

Required fix:

- only allow normal task completion at the parking side
- ensure `Charging` is reached only from semantically valid parking-side completion

### 4. RPC Command Events Lose `request_id`

Current command tracking stores empty request ids for accepted and completed commands.

Required fix:

- carry request id from MQTT topic parsing through command tracking and published command events

### 5. Duplicate Start Paths Preserve Old Semantics

`start_scheduled_task()` and `start_manual_task()` currently duplicate behavior and preserve old home/front assumptions.

Required fix:

- merge them into one `start_task()` path

### 6. Tests Encode Multiple Conflicting Truths

The current test suite mixes:

- old home/front semantics
- richer terminal semantics
- release restrictions that are about to be removed entirely

Required fix:

- rewrite ThingsBoard-related tests around the simplified left/right parking-side model only

## Test Design

### 1. Config Manager Tests

Rewrite to validate only:

- positive integer `passes`
- `parking_side = left/right`
- correct pending persistence
- correct active promotion
- schedule handling

Delete tests for:

- `parking_policy`
- `charging_side`
- half-pass rejection

### 2. Sensor Mapping Tests

Add or rewrite tests that verify:

- `front_limit` maps to left sensor
- `rear_limit` maps to right sensor
- `parking_side = left` selects left as parking-side sensor
- `parking_side = right` selects right as parking-side sensor

### 3. Supervisor Tests

Rewrite to validate:

- start allowed only at parking side
- pending config is promoted before start
- pause, return, terminate, and reset admission follow the simplified model
- there is only one task-start path

### 4. FSM Tests

Rewrite to validate:

- task starts into outward cleaning
- far-end sensor transitions into return cleaning
- parking-side sensor is the only legal normal completion point
- one completed out-and-back cycle equals one completed pass

### 5. ThingsBoard Control Plane and Cloud Tests

Rewrite to validate:

- shared attributes apply the simplified config model
- RPC acceptance and rejection reflect the new semantics
- command events preserve `request_id`
- telemetry does not expose removed semantic fields

### 6. Real or Mock Runtime Integration Tests

Keep a narrow high-value integration layer that validates:

- live shared-attribute update of `parking_side`
- live RPC-driven task control
- telemetry consistency with the simplified runtime state model

## Implementation Scope

Primary code-change targets are expected to include:

- `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/service/cloud_service.cc`
- `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- `pv_cleaning_robot/main.cc`

Primary test rewrites are expected to include:

- `test/service/thingsboard_config_manager_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/service/cloud_service_test.cc`
- `test/app/robot_supervisor_test.cc`
- `test/app/terminal_semantics_test.cc`
- `test/integration/thingsboard_runtime_mock_integration_test.cc`

## Deletion Scope

Delete, not deprecate:

- `parking_policy`
- `charging_side`
- multi-terminal helper enums and helper functions
- half-pass semantics
- duplicated task-start entry points
- telemetry fields or test assumptions that depend on removed semantics

This work should prefer removal over compatibility layers unless a concrete compiler or runtime dependency forces a narrower transition step.
