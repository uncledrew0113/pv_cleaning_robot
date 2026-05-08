# PV Cleaning Robot Business Flow Audit Design

## 1. Goal

This spec defines the first remaining sub-project: audit the real business flow implemented by the main program against a newly confirmed simplified business model, identify deterministic defects and business-semantic risks, and produce a bounded input for the next simplification/refactor stage.

This phase does not implement fixes. It establishes:

- what the real runtime truth is
- which behaviors are correct
- which issues are deterministic bugs
- which issues are design debts that should drive later simplification

## 2. Scope

The audit source of truth is the current main-program runtime chain, in this priority order:

1. `pv_cleaning_robot/main.cc`
2. runtime modules wired by `main.cc`
3. unit/integration tests, only as validation and divergence detectors
4. existing docs and comments only as historical reference, not truth

The audit covers these runtime areas:

1. startup and configuration loading
2. power-on self-check and position interpretation
3. scheduled start and RPC start
4. cleaning round-trip main flow
5. stop and task-level return
6. reset / reboot behavior
7. fault reporting and safe-halt behavior
8. charging and battery-gate behavior
9. telemetry / shared attributes / RPC / command events
10. thread, event, cache, watchdog, and runtime support behavior

This phase audits the main program first. Tests must be updated later to match the corrected runtime truth, but tests do not define the truth.

## 2.1 User-Confirmed Simplified Business Baseline

The audit must treat the following user-confirmed rules as the target business model:

1. Configuration truth should be split into three classes:
   - fixed local configuration
     - hardware interfaces, GPIO wiring, device addresses, and other deployment-fixed settings
   - cloud-mutable immediate configuration
     - changes that should take effect without waiting for the next task
   - cloud-mutable next-task configuration
     - changes that should persist but only take effect when the next task starts
2. If both endpoint sensors are active at the same time:
   - report invalid position/state
   - do not allow a cleaning task to start
3. If neither endpoint sensor is active:
   - report invalid/off-endpoint state
   - do not start a cleaning task directly
   - attempt to return to the parking side
4. The RPC contract should be simplified to:
   - `start`: always start a new task, never mean resume
   - `stop`: terminate the current task and halt the robot
   - `return`: return to the parking side
   - `reset`: reboot the device
5. Pause/resume is not part of the target business model.
6. `terminate` is not part of the target RPC model; its remaining code paths should be treated as simplification candidates.
7. Low-battery return is not part of the target business model.
   - low battery should gate task start during self-check / pre-start checks
   - if battery is below configured start threshold, report state and do not start
8. Charging should be threshold-driven:
   - when the robot is at the parking side and battery is below configured charge-low threshold, enter `Charging`
   - when charging reaches configured charge-high threshold, enter `Idle`
   - if the robot returns to the parking side but battery is not below the charge-low threshold, stay `Idle`
   - if the robot is already in `Charging`, a `start` command may start a task only if battery is above the configured start threshold
9. Fault handling should be simplified as much as safely possible:
   - fault status must be reported
   - differentiated P0/P1/P2 task branches are simplification candidates unless they are required for basic safety
10. Direction-bound state names such as `CleanFwd` / `CleanReturn` are suspected simplification debt.
    The audit should determine whether they should be renamed to parking-side semantics in the follow-on phase.

## 3. Out of Scope

This phase does not:

- redesign unrelated product features
- add new robot capabilities
- redesign external platform contracts beyond what is required to describe defects
- perform broad code cleanup without audit justification
- treat README or design docs as authoritative if code differs

## 4. Audit Method

Each audited flow segment is reviewed through five lenses:

1. entry conditions
2. state progression
3. action execution
4. cloud/platform expression
5. module responsibility and boundary ownership

Each finding is classified as one of two types:

- `Deterministic defect`
  - current code can be shown to allow wrong behavior, reject valid behavior, lose context, race, or apply configuration at the wrong time
- `Semantic / responsibility risk`
  - current code may still work, but names, ownership, state meaning, or control boundaries are unclear enough to create recurring defects

## 5. Runtime Boundary Model

The current runtime chain is:

`ConfigService -> main wiring -> LimitSwitch/SafetyMonitor -> main event bridge -> RobotFsm -> RobotSupervisor -> Motion/Nav/Fault/Scheduler -> CloudService/ThingsBoard -> periodic reporting/cache`

Expected responsibility boundaries:

- `main`
  - dependency wiring
  - lifecycle start/stop
  - minimal physical-to-business bridge
- `SafetyMonitor`
  - physical limit trigger
  - immediate emergency stop
  - debounce and publish settled physical event
- `RobotFsm`
  - task state truth
  - pass progression
  - start/stop/return/fault/charging transitions
- `RobotSupervisor`
  - single task-admission and pre-start-gating authority
  - pending-to-active promotion before task start
  - snapshot aggregation for upper layers
- `ThingsBoardConfigManager`
  - shared-attribute validation
  - fixed vs immediate vs next-task config ownership boundaries
  - active/pending runtime config where next-task staging is still required
  - schedule synchronization
- `ThingsBoardControlPlane`
  - reduced cloud protocol mapping
  - RPC entry for `start/stop/return/reset`
  - telemetry/event publishing

Any finding that shows these boundaries are violated should be recorded, even if the current behavior still works.

## 6. Flow-by-Flow Audit Structure

The audit result will be organized by business flow, with module attribution inside each flow:

1. startup and configuration loading
2. power-on self-check and current position interpretation
3. scheduler-triggered start and RPC-triggered start
4. cleaning out-and-back main loop
5. stop and task-level return
6. reset / reboot behavior
7. fault reporting and safe-halt behavior
8. charging and battery-gate behavior
9. cloud reporting and command/state projection
10. thread, event, cache, watchdog, and runtime support layers

For each flow section, the audit will explicitly describe:

- intended behavior
- actual behavior in current code
- deterministic defects
- semantic / responsibility risks
- directly involved modules

## 7. Initial Findings to Validate During Audit

The current code review already indicates the following high-probability findings that the full audit must confirm and formalize:

### 7.1 Deterministic defects

1. Configuration truth is not yet cleanly split into fixed-local, cloud-immediate, and cloud-next-task ownership.
   The audit must confirm where current code still mixes these layers or lets multiple modules interpret the same configuration differently.

2. Abnormal endpoint states are handled too weakly.
   Cases like "both left and right active" or "neither endpoint active" are currently expected to require explicit status reporting and task-start denial.

3. Start admission is improved but still not a single absolute authority.
   The logic still spans `main`, `ThingsBoardControlPlane`, `RobotSupervisor`, and `RobotFsm`.

4. The current RPC/runtime control model is richer than the desired one.
   `pause/resume/terminate/low-battery-return` are likely simplification debt relative to the confirmed target contract.

5. Charging and battery gating are not yet expressed as one clean, threshold-driven rule set.

6. Cloud status is a multi-view projection, not a single atomic runtime object.
   RPC response, command event, telemetry, and snapshot do not inherently represent the same moment.

### 7.2 Semantic / responsibility risks

1. `main.cc` still carries too much runtime knowledge.
2. The bridge from physical facts to business facts is still too dispersed.
3. The old directional and branch-rich task model is likely more complex than the confirmed target model.
4. The runtime model remains mixed: event-driven plus direct calls plus periodic ticking.

These are not yet final findings; they are the expected focal points for the audit result.

## 8. Expected Deliverables

This audit phase should produce:

1. a main-program business-flow map
2. per-flow findings
3. a `must-fix defects` list
4. a `high-priority design debt` list
5. a bounded input for the next simplification phase:
   - what can be deleted safely
   - what must be fixed before simplification
   - what boundaries must be tightened first

## 9. Success Criteria

This audit is complete when:

1. every major runtime flow in the scope has been reviewed
2. each finding is classified as deterministic defect or semantic/risk debt
3. each finding has explicit module attribution
4. the result is actionable enough to drive the next code-simplification spec without reopening the scope question

## 10. Follow-On Phase

The next sub-project after this audit is not general refactoring.

It is a constrained simplification phase that uses this audit as input to:

- centralize configuration ownership and position-truth handling
- further thin `main`
- collapse task admission into one authority
- reduce the RPC contract to `start/stop/return/reset`
- remove pause/resume/low-battery-return complexity where not required
- simplify charging/battery gating into one coherent rule set
- delete obsolete compatibility paths and redundant tests safely
