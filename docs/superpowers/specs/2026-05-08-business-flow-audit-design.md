# PV Cleaning Robot Business Flow Audit Design

## 1. Goal

This spec defines the first remaining sub-project: audit the real business flow implemented by the main program, identify deterministic defects and business-semantic risks, and produce a bounded input for the next simplification/refactor stage.

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
5. pause and resume
6. task-level return
7. manual terminate and reset
8. P0/P1/P2 fault handling
9. low-battery return flow
10. telemetry / shared attributes / RPC / command events
11. thread, event, cache, watchdog, and runtime support behavior

This phase audits the main program first. Tests must be updated later to match the corrected runtime truth, but tests do not define the truth.

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
  - pause/return/fault/low-battery/terminate transitions
- `RobotSupervisor`
  - action admission
  - pending-to-active promotion before task start
  - snapshot aggregation for upper layers
- `ThingsBoardConfigManager`
  - shared-attribute validation
  - active/pending runtime config
  - schedule synchronization
- `ThingsBoardControlPlane`
  - cloud protocol mapping
  - RPC entry
  - telemetry/event publishing

Any finding that shows these boundaries are violated should be recorded, even if the current behavior still works.

## 6. Flow-by-Flow Audit Structure

The audit result will be organized by business flow, with module attribution inside each flow:

1. startup and configuration loading
2. power-on self-check and current position interpretation
3. scheduler-triggered start and RPC-triggered start
4. cleaning out-and-back main loop
5. pause and resume
6. task-level return
7. terminate and reset
8. P0/P1/P2 fault chain
9. low-battery return
10. cloud reporting and runtime support layers

For each flow section, the audit will explicitly describe:

- intended behavior
- actual behavior in current code
- deterministic defects
- semantic / responsibility risks
- directly involved modules

## 7. Initial Findings to Validate During Audit

The current code review already indicates the following high-probability findings that the full audit must confirm and formalize:

### 7.1 Deterministic defects

1. Position truth is not fully centralized.
   The project now uses `left/right + parking_side`, but some call sites still decide for themselves whether to interpret current position against active config or next-start config.

2. Abnormal endpoint states are handled too weakly.
   Cases like "both left and right active" or "neither endpoint active" are mainly warnings, not first-class invalid-position states.

3. Start admission is improved but still not a single absolute authority.
   The logic still spans `main`, `ThingsBoardControlPlane`, `RobotSupervisor`, and `RobotFsm`.

4. Return-source context is lost too early.
   Manual return, P1 return, and low-battery return converge into one outward runtime expression too early.

5. Cloud status is a multi-view projection, not a single atomic runtime object.
   RPC response, command event, telemetry, and snapshot do not inherently represent the same moment.

### 7.2 Semantic / responsibility risks

1. `main.cc` still carries too much runtime knowledge.
2. The bridge from physical facts to business facts is still too dispersed.
3. Resume/start/reset semantics are outwardly under-specified.
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

- centralize position-truth handling
- further thin `main`
- reduce duplicated admission logic
- preserve return/fault context where needed
- delete obsolete compatibility paths and redundant tests safely
