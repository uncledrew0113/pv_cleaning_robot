# FSM / Fault / Business Kernel Design

Date: 2026-05-24
Status: Draft approved for planning

## Goal

Refactor the task orchestration core so that:

- mission semantics are simple and hardware-aligned
- FSM stays small, stable, and predictable
- fault handling is explicit and action-oriented
- business state/fault codes become the single source of truth for runtime, tests, logs, and cloud reporting
- external field contracts stay compact and easy to maintain

Absolute design rule:

- overall implementation must stay simple, stable, and reliable
- implementation should avoid redundancy and keep code paths minimal
- SOLID principles must be preserved during restructuring
- existing code files may be modified, deleted, or renamed if that simplifies the design
- new code files should not be added for this work
- reducing the current number of code files is preferred when it improves clarity

This spec intentionally avoids speculative abstractions and does not try to redesign unrelated subsystems.

## Current Problems

The current codebase still has three structural issues in this area:

1. Task semantics are built around integer `passes`, which does not model:
   - single-dock round-trip tasks cleanly
   - dual-dock dock-to-dock tasks cleanly
   - test-only single-leg tasks cleanly

2. FSM states encode business direction directly:
   - `CleanFwd`
   - `CleanReturn`
   - `Returning`

   This makes direction, brush behavior, return target, and completion semantics too coupled to state names.

3. Fault severity and fault action are mixed together.
   `P1` especially needs action decisions based on remaining capability and current task context, not only on severity level.

Separately, the external field contracts had grown too wide. For this reason, compact outward contracts are part of the design, not a separate cleanup.

## Business Model

### Lane Mode

Two deployment modes are supported.

- `SingleDockLane`
  One side is the parking/charging dock. The opposite side is only the far terminal.

- `DualDockLane`
  Both sides are valid parking/charging docks.

### Mission Type

The task model is no longer based on "integer passes" as the primary runtime concept.

Supported mission types:

- `RoundTripClean`
  Formal single-dock business task.
  Start from dock, clean to far terminal, then return to dock.
  Task is complete only after returning to dock.

- `DockToDockClean`
  Formal dual-dock business task.
  Start from one dock and finish at the opposite dock.
  Reaching the opposite dock completes the task.

- `SingleLegTest`
  Test or manual-only task.
  Single-direction execution is allowed.
  This mode is not part of formal production business flow.

### Segment-Based Execution

FSM runtime is driven by `Mission -> SegmentSpec[]`.

A `SegmentSpec` is the smallest business execution unit. It is intentionally minimal and does not include motor-level parameters.

Minimal fields:

- `from_terminal`
- `to_terminal`
- `segment_mode`
  - `clean`
  - `return_no_brush`
- `completion_condition`

Examples:

- single-dock formal clean:
  - segment 1: `dock -> far`, `clean`
  - segment 2: `far -> dock`, `clean`

- dual-dock formal clean:
  - segment 1: `dock_a -> dock_b`, `clean`

- single-leg test:
  - segment 1: `start -> target`, `clean` or `return_no_brush`

### Why Passes Are No Longer the Core Model

`0.5` pass is not modeled as a primitive runtime concept anymore.

Instead:

- single-dock one-way execution is represented as `SingleLegTest`
- dual-dock one-time cleaning is represented as `DockToDockClean`

This keeps runtime semantics tied to hardware endpoints and actual task completion rules instead of arithmetic on pass counts.

## FSM Design

### State Set

FSM must be reduced to a small, explicit state set:

- `Idle`
- `SelfCheck`
- `ExecutingSegment`
- `SegmentBoundary`
- `FaultStopped`
- `Charging`

No dedicated degraded-running state is introduced.

### State Responsibilities

- `Idle`
  No active mission. Accepts scheduler start, RPC start, reset, configuration refresh.

- `SelfCheck`
  Validates whether the requested mission may start.
  It does not decide motor direction or mission geometry.

- `ExecutingSegment`
  Executes exactly one current `SegmentSpec`.

- `SegmentBoundary`
  Short decision state entered after segment completion.
  It decides whether to:
  - move to next segment
  - end task
  - enter charging
  - switch to a fault-disposal segment

- `FaultStopped`
  Hard stop / non-returnable stop state.

- `Charging`
  Reached valid charging dock and charging policy requires charging.

### Explicit Non-Goals

The FSM must not directly encode:

- "forward clean" versus "return clean" as separate states
- wheel direction tables
- brush direction tables
- degraded running behavior
- raw hardware fault details

### Main Transition Chain

Main path:

`Idle -> SelfCheck -> ExecutingSegment -> SegmentBoundary -> ExecutingSegment / Idle / Charging`

Abnormal path:

- `P0`:
  any running state -> `FaultStopped`

- `P1`:
  any running state -> inject fault-disposal segment if return is possible, else `FaultStopped`

- `P2`:
  no state transition required by default

## Segment Execution Resolution

### Design Rule

`SegmentSpec` stays business-level only.
It must not carry direct wheel direction, motor target rpm, or brush motor low-level output values.

### Resolver Rule

Segment business semantics are resolved inside the existing motion layer.
The resolver logic may stay private to current files; no standalone resolver file is required.

Inputs:

- `lane_mode`
- `from_terminal`
- `to_terminal`
- `segment_mode`

Outputs:

- walk direction profile
- brush enable/direction profile
- target motion profile needed by `MotionService`
- terminal completion target needed by completion checking

This keeps complexity out of the FSM while still allowing different docks and segments to map to different motor directions.

## Fault Chain Design

### Two-Level Fault Model

Fault handling is split into two layers:

- `RawFault`
  Produced by hardware, protocol, safety, or service layers

- `BusinessFaultDecision`
  Produced by a business-level fault arbiter embedded in the existing fault-handling path
  This is the only fault decision object consumed by FSM logic

`BusinessFaultDecision` must contain at least:

- severity level
- business fault code
- remaining mobility capability
- target terminal if return is required

### P0

Meaning:

- immediate safety stop required
- human safety or major equipment safety at risk
- motion is no longer safely controllable

Action:

- emergency stop immediately
- enter `FaultStopped`

No return attempt is allowed.

### P1

Meaning:

- task may not continue cleaning
- some controlled motion capability may still remain

Action decision is based on:

- concrete fault reason
- remaining mobility capability
- current mission context
- current segment context

P1 handling result:

- if return is still allowed:
  create one fault-disposal segment, normally `return_no_brush`
- if return is not allowed:
  enter `FaultStopped`

### P2

Meaning:

- non-fatal warning
- no mandatory task behavior change

Action:

- report only
- keep current mission/FSM path unchanged by default

Important rule:

If a fault actually requires changing task behavior, it must not remain `P2`.
It must be upgraded to `P1` or `P0`.

### No Degraded Running Mode

This design explicitly rejects a formal degraded-running business state.

Therefore:

- no "continue but slower" state
- no "continue without vision" business mode
- no "continue with partial cleaning capability" mode

Either:

- the fault is harmless enough to stay `P2` and only be reported
- or it is action-relevant and must become `P1`
- or it is safety-critical and must become `P0`

## Business Codes

Business codes are required.

They must be the single source of truth for:

- FSM-visible runtime semantics
- logs
- tests
- compact cloud reporting

Cloud-side code must not invent a parallel state vocabulary.

### BusinessStateCode

Represents the current business lifecycle state.

Initial target set:

- `IDLE`
- `SELF_CHECK`
- `EXECUTING_SEGMENT`
- `SEGMENT_BOUNDARY`
- `CHARGING`
- `FAULT_STOPPED`

### BusinessFaultCode

Represents business-level fault reasons.

Examples:

- `NONE`
- `STARTUP_POSITION_INVALID`
- `BMS_UNAVAILABLE`
- `BRUSH_FAILURE_RETURN_REQUIRED`
- `WALK_DRIVE_FAILURE_STOP_REQUIRED`
- `LIMIT_SIGNAL_CONFLICT`
- `PARKING_TARGET_UNREACHABLE`

Final code list should remain compact and action-oriented.
Do not expose raw driver error catalogs as business fault codes.

No standalone `BusinessActionCode` is part of the outward contract for this phase.
If action-level observability is needed later, it should only be added when `state + fault` is proven insufficient in production.

## Self-Check Rules

`SelfCheck` only determines whether the requested mission may start.

It must validate at least:

- lane mode and mission type compatibility
- required dock/terminal context
- BMS readiness according to environment policy
- mission geometry prerequisites
- start-position prerequisites for the selected mission type

Environment policy already agreed:

- production: BMS unavailable => startup forbidden
- test environment: BMS unavailable => startup may be allowed under test policy

## Task Completion Rules

### SingleDockLane

Formal business task completion requires returning to the dock.

Single-leg completion is allowed only in test/manual mode.

### DualDockLane

Dock-to-dock arrival completes the task.

No "half-pass" interpretation is needed.
This is one complete mission.

## External Contracts

### Runtime Config / Downlink

The compact runtime config contract for this phase is:

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_side`
- `min_battery_soc`
- `charge_stop_soc`
- `schedules`

Notes:

- `passes` remains for compatibility and scheduling input, but should not remain the FSM's internal truth
- `return_brush_rpm` is removed
- `start_battery_soc` and `charge_start_soc` are merged into `min_battery_soc`

### Periodic Business Telemetry

The compact periodic business telemetry contract is:

- `state`
- `fault`
- `cfg_ver`

Notes:

- `state` is the outward representation of current business lifecycle state
- `fault` is the current or last active compact business fault code value
- `cfg_ver` is optional but allowed as a compact configuration version field
- `device_state`, `task_state`, and similar duplicate vocabularies must not continue outward

### Status Events

Status events use a compact event contract:

- `event`
- `code`

### RPC Replies

RPC replies use a compact result contract:

- `code`

### Command Lifecycle Visibility

Command lifecycle tracking may remain local for debugging and control flow.
It is not part of periodic telemetry for this phase.

### Health And Diagnostics

Existing `HEALTH` and `DIAGNOSTICS` payload schemas are intentionally not changed by this design update.
Their compaction is explicitly out of scope for this phase.

## Testing Strategy

Verification must follow the business model, not the old state names.

Minimum required test matrix:

### Mission Semantics

- single-dock formal mission => two segments, final return to dock
- dual-dock formal mission => one segment, complete on opposite dock
- single-leg test mission => single segment completion allowed

### FSM

- normal transition chain from idle to completion
- segment boundary switching
- charging entry on eligible terminal

### Faults

- `P0` => immediate stop to `FaultStopped`
- `P1` returnable => fault-disposal segment is created and executed
- `P1` non-returnable => `FaultStopped`
- `P2` => report only, no task-state change

### Business Codes

- state code changes match actual FSM lifecycle
- fault code changes match business decisions

### External Contracts

- runtime config fields match the compact contract
- periodic business telemetry emits only `state`, `fault`, and `cfg_ver`
- status events emit only `event` and `code`
- RPC replies emit only `code`

### Environment Policy

- production startup blocked if BMS unavailable
- test startup policy behaves as configured

## Scope Boundaries

This work includes:

- mission model simplification
- FSM simplification
- P0/P1/P2 business handling semantics
- business state/fault code design
- compact telemetry truth alignment with kernel business codes

This work does not include:

- redesigning the entire HAL/driver stack
- redesigning scheduler semantics beyond mission input adaptation
- introducing a degraded-running framework
- changing existing `HEALTH` and `DIAGNOSTICS` field schemas in this phase
- broad refactors unrelated to mission execution and fault semantics
- adding new code files as part of this refactor

## Implementation Principles

The implementation plan derived from this spec must obey:

- smallest possible change set that achieves the new business truth
- FSM logic stays small and readable
- no speculative abstraction layers
- no duplicate state vocabularies between kernel and cloud
- business rules first, motor rules resolved separately
- prefer in-place simplification of existing files over creating new files
- deleting or renaming existing files is allowed if it reduces structural noise
- reducing total file count is preferred when responsibilities become clearer, not more coupled

Whenever there is a tradeoff, prefer:

- simpler runtime semantics
- fewer FSM states
- fewer outward fields
- clearer fault decisions
- more deterministic hardware behavior

## Verification Notes

The repository environment may use available `/usr/bin/qemu-aarch64*_static` or similar user-space emulation binaries during later validation work where that helps execute or inspect target-architecture artifacts.

This is a verification aid only.
It does not change the business design or the runtime architecture described above.
