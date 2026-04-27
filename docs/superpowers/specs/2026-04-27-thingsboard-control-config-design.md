## ThingsBoard Control And Config Design

Date: 2026-04-27

### Problem

The project already has a usable MQTT/ThingsBoard transport skeleton:

- `CloudService` subscribes to RPC and shared attributes
- `ConfigService` can read and write `config.json`
- `RobotFsm`, `SchedulerService`, `FaultService`, and `HealthService` cover the core runtime loop

But the current implementation does not yet support the confirmed ThingsBoard business contract:

- RPC control for `start`, `stop`, `return`, `terminate`, and `reset`
- telemetry with task-aware reporting cadence and command execution visibility
- shared-attribute-driven task configuration with strict validation
- local persistence with backup and rollback
- client attributes for static device information

The current code also has business bugs that would make a direct ThingsBoard feature rollout unsafe.

### Scope

In scope:

- ThingsBoard RPC control semantics and execution model
- shared attribute schema, validation, persistence, and staged activation
- client attribute publishing for static device metadata
- telemetry model for command state, task state, and business-facing device state
- `RobotFsm` and runtime changes required to support the approved command semantics
- pre-implementation bug fixes that block a safe rollout

Out of scope:

- implementing real overcurrent detection logic in `FaultService`
- redesigning MQTT transport, LoRaWAN transport, or the general networking stack
- unrelated hardware protocol refactors
- OTA workflow changes

### Confirmed Product Decisions

- ThingsBoard is the source of truth for operational configuration.
- Shared attributes are persisted locally.
- Configuration updates require strict validation.
- Local configuration needs a main copy plus a backup copy for rollback.
- `schedule` updates take effect immediately for future scheduler scans.
- All other shared-attribute task settings take effect on the next task only.
- `overcurrent_protection` is accepted and persisted in this phase, but does not yet drive real fault behavior.
- RPC replies return command acceptance quickly; final execution status is exposed through telemetry.
- `return` means task-level return to the parking position. When the robot reaches home, the current task ends and does not continue remaining passes.
- `terminate` is a manual/testing-oriented intervention. It aborts the task, disables motion, and requires manual repair plus a later `reset`.
- After `terminate`, the device must be moved back to the parking position and then explicitly reset before returning to service.
- `stop` means pause. Only a later `start` RPC may resume it.
- `start` has two distinct semantics:
  - resume from `Paused` without requiring the robot to be at home
  - create a new task from `Idle` or `Charging` only when the robot is at home

### Current Blocking Findings

These bugs must be fixed before the ThingsBoard feature set is implemented:

1. `RobotFsm` can expose a string state that disagrees with its internal SML state.
2. Illegal or out-of-order FSM events can still mutate outward-visible runtime state.
3. Shared attribute updates are non-transactional and not validated as a whole.
4. The configuration system has no `active` / `pending` / `backup` model.
5. Current RPC handlers return success even when the requested business action is invalid.
6. The current `stop` path is incorrectly mapped onto a P1 fault-return path.
7. P1 fault handling can briefly reverse the brush despite the intended “stop-brush return” behavior.
8. `stop_cleaning()` disables motion entirely, so it cannot back the approved `Paused` semantics.
9. `DataCache` can lose consistency when full-queue eviction succeeds in memory but its journal ack append fails.
10. The current single `EvScheduleStart` model is not rich enough to represent both “new task start” and “paused task resume.”

### Architecture

Add a ThingsBoard business layer above the existing transport layer rather than continuing to expand `main.cc` lambdas.

Target runtime structure:

- `CloudService`
  - remains the ThingsBoard transport boundary
  - publishes telemetry and attributes
  - dispatches raw RPC and shared-attribute payloads inward

- `ThingsBoardConfigManager`
  - parses supported shared attributes
  - performs strict schema, range, and semantic validation
  - persists `active`, `pending`, and `backup` configuration state
  - applies immediate scheduler changes
  - stages next-task-only changes

- `CommandOrchestrator`
  - owns RPC business semantics
  - validates command admissibility against robot runtime state
  - drives FSM events and reset workflow steps
  - updates command tracking state

- `CommandTracker`
  - records the currently active command
  - records the last completed command
  - provides a telemetry-facing snapshot

- `TaskRuntimeContext`
  - stores task-scoped runtime information not represented well by FSM states alone
  - tracks task progress, active config version, stop reason, return reason, and task source

- `HealthService` plus business snapshot augmentation
  - `HealthService` continues producing device-health data
  - a higher-layer telemetry snapshot combines health data with business state

This keeps business control and state modeling out of `CloudService` and avoids turning `main.cc` into the permanent coordinator.

### State Machine Design

The FSM should be expanded conservatively. The goal is not to encode every cloud concept as a new state, but to represent the business states that materially change command semantics.

Target externally visible business states:

- `Idle`
- `SelfCheck`
- `CleanFwd`
- `CleanReturn`
- `Returning`
- `Paused`
- `Charging`
- `Fault`
- `Terminated`

State intent:

- `Paused`
  - entered by the `stop` RPC
  - task context is preserved
  - motion is held at zero speed without losing resumability
  - only a later `start` RPC may resume the task

- `Terminated`
  - entered by the `terminate` RPC
  - task context is discarded
  - motion is disabled
  - the robot may be left away from home
  - the robot must be manually repaired and moved home before `reset`

- `Returning`
  - reused for manual return, low-battery return, and P1 fault return
  - differentiated by `return_reason` in task context, not by separate FSM states

Reset is not modeled as a long-running FSM state. It should be a workflow outside the motion FSM that checks preconditions, performs self-check/reset steps, and only then transitions the FSM back to `Idle`.

### Task Runtime Context

Add a dedicated runtime context object alongside the FSM. Required fields:

- `task_id`
- `task_source` (`schedule`, `rpc_start`)
- `active_config_version`
- `target_half_passes`
- `completed_half_passes`
- `stop_reason`
- `return_reason`
- `last_transition_time`
- `has_active_task`

This avoids overloading FSM states with data that should be queryable and testable separately.

### RPC Semantics

RPC methods:

- `start`
- `stop`
- `return`
- `terminate`
- `reset`

Approved semantics:

- `start`
  - from `Paused`: resume the current task without requiring the robot to be at home
  - from `Idle` or `Charging`: create a new task only if `at_home == true`
  - from `Fault` or `Terminated`: reject

- `stop`
  - transitions into `Paused`
  - preserves task context
  - does not disable the drivetrain as a terminated/faulted stop would
  - cannot be auto-resumed by the scheduler

- `return`
  - transitions into `Returning`
  - uses task-level return semantics
  - when the robot reaches home, the current task ends immediately
  - any remaining passes are abandoned for this task

- `terminate`
  - transitions into `Terminated`
  - aborts the task
  - disables motion
  - is mainly for testing/manual intervention, not normal fault handling

- `reset`
  - valid only from `Fault` or `Terminated`
  - requires the robot to be back at home
  - runs the reset/self-check workflow
  - on success returns the runtime to `Idle`
  - on failure keeps the prior state and publishes a failure reason

RPC reply model:

- the immediate RPC response returns command acceptance, rejection reason, and a command id
- final command outcome is published through telemetry

### Shared Attribute Model

Supported shared attributes:

- `parking_side`
- `passes`
- `schedules`
- `low_battery_threshold`
- `temperature_threshold`
- `overcurrent_threshold`
- `task_timeout_sec`
- `walk_speed_percent`
- `brush_speed_percent`

Activation rules:

- `schedules`
  - applied immediately to scheduler state
  - affects future scheduler scans only

- all other fields
  - validated and persisted immediately
  - staged as `pending`
  - promoted to `active` only when the next task is created

Update behavior:

- only supported fields are accepted
- unknown fields are ignored and logged
- partial success is not allowed
- if any supported field fails validation, the whole update is rejected

### Configuration Persistence Model

Persist three logical configuration layers:

- `active`
  - the configuration currently used for scheduler-active behavior and new task creation

- `pending`
  - validated next-task configuration waiting to be promoted

- `backup`
  - the previous known-good persisted configuration snapshot

Recommended file layout:

- either a single persisted JSON file with explicit `active` / `pending` metadata plus a backup file
- or a small set of dedicated files such as:
  - `config.json`
  - `config.pending.json`
  - `config.backup.json`

Required behavior:

1. receive shared attributes
2. parse and normalize supported fields
3. validate the entire candidate update
4. copy current persisted good state to backup
5. persist the new `pending` and/or immediately effective scheduler state atomically
6. update in-memory state only after persistence succeeds
7. on any failure, keep the prior in-memory and on-disk state

Startup behavior:

- load the main persisted state first
- if main load fails, try backup automatically
- if backup load succeeds, publish a startup warning that backup was used
- if both fail, fall back to explicit defaults and surface a hard warning

### Validation Rules

Validation must be performed at three levels:

- schema validation
  - field presence where required
  - correct JSON types
  - correct array/object structure

- range validation
  - `passes` limited to the supported set: `0.5`, `1..10`, and `infinite`
  - speed percentages restricted to `0..100`
  - protection thresholds restricted to hardware-safe bounds
  - schedule hour/minute/weekday values restricted to valid ranges

- semantic validation
  - duplicate schedule entries rejected
  - timeout lower bounds must remain physically plausible
  - parking side must be a supported enum
  - cross-field contradictions rejected

Validation failure handling:

- reject the whole update
- keep existing config effective
- publish structured failure information to telemetry
- update a “last config error” business field

### Telemetry Design

Telemetry has two modes of emission:

- periodic telemetry
- event telemetry

Periodic cadence:

- active task: every 1 second
- no active task: every 5 minutes

Periodic payload should include:

- current health payload
- `device_state`
- `task_state`
- `target_half_passes`
- `completed_half_passes`
- `clean_count`
- `temperature`
- `active_config_version`
- `active_command`
- `last_command`

Event telemetry should be emitted for:

- RPC accepted/rejected
- command completed/failed
- shared attribute update accepted/rejected
- backup-config startup fallback
- reset workflow success/failure

### Client Attributes

Client attributes remain the place for static or near-static metadata:

- `software_version`
- `device_model`
- `hardware_version`
- `device_id`

Optional static metadata:

- `supported_rpc_methods`
- `config_schema_version`

Do not use client attributes to mirror mutable task configuration or command execution state.

### Command Tracking

Track both:

- the currently active command
- the last completed command

Recommended tracked fields:

- `command_id`
- `name`
- `phase`
- `accepted_at`
- `finished_at`
- `result`
- `reason`

Recommended phases:

- `accepted`
- `running`
- `succeeded`
- `failed`
- `rejected`

### Scheduler Behavior

The scheduler must be updated to match the confirmed business rules:

- schedule changes apply immediately to future scans
- a manually paused task is never auto-resumed by the scheduler
- a returned task is considered complete for that run when the robot reaches home
- a terminated task is never resumed

The current window-duration behavior should be rewritten so the scheduler has one clear job: decide when to issue a new scheduled task start request, not implicitly own task-stop semantics through a fragile minute-match implementation.

### Error Handling

Configuration errors:

- reject the update as a whole
- preserve current effective state
- emit event telemetry and record the failure

Command errors:

- reject illegal commands before they enter the execution path
- return structured rejection reasons

Runtime errors:

- continue to let `FaultService` own formal fault handling
- keep `Terminated` reserved for manual/testing intervention
- never allow reset failure to silently force a transition to `Idle`

### Testing And Verification

Required unit coverage:

- FSM illegal-event handling and outward-state consistency
- `Paused` and `Terminated` transitions
- command admissibility checks
- shared-attribute full-update validation and all-or-nothing rejection
- active/pending/backup persistence behavior
- `DataCache` consistency during queue eviction and ack persistence failure

Required integration coverage:

- scheduled start from home
- pause then RPC resume away from home
- return command ending the task at home without continuing remaining passes
- terminate, manual-home precondition, then reset back to idle
- startup fallback to backup config
- active 1-second versus idle 5-minute telemetry policy

### Implementation Order

1. Fix current blocking business bugs.
2. Introduce runtime state and command-tracking primitives.
3. Extend the FSM and motion-control interfaces for `Paused` and `Terminated`.
4. Introduce `ThingsBoardConfigManager` with active/pending/backup persistence.
5. Introduce `CommandOrchestrator` and replace direct RPC lambdas in `main.cc`.
6. Add business telemetry snapshotting and state-aware cadence control.
7. Add tests for all new runtime semantics and the pre-existing bugs fixed in step 1.

### Non-Goals

- implementing real overcurrent trip detection in this phase
- redesigning the transport layer itself
- broad cleanup of unrelated hardware code
