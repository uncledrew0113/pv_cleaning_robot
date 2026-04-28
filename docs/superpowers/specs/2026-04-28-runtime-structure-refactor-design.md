## Runtime Structure Refactor Design

Date: 2026-04-28

### Problem

The project now has a workable runtime and a mostly-complete ThingsBoard feature set, but the business structure has become hard to follow:

- `main.cc` still mixes bootstrap, scheduling, safety policy, telemetry cadence, and thread orchestration
- task-start and task-control rules are split across `main.cc`, `ThingsBoardRuntime`, `RobotFsm`, and `SchedulerService`
- `CleanTask` remains in the tree even though it is no longer the runtime truth
- the ThingsBoard integration layer has started to become a second coordinator instead of staying at the cloud boundary

The result is not “too many files” by itself. The deeper issue is that task orchestration lacks one clear owner.

### Goal

Improve code comprehension by introducing a single runtime orchestration layer, removing dead task-model code, and shrinking `main.cc` until it is mostly a bootstrap file.

### Scope

In scope:

- remove `CleanTask` from the main build and test graph
- add a single runtime orchestration class for task admission and safety policy
- move scheduling-start, RPC task control, low-battery handling, spin-free handling, and cloud cadence policy behind that orchestrator
- keep the existing FSM, motion, navigation, fault, health, and ThingsBoard config semantics intact while restructuring
- reduce the coordination burden currently spread across `main.cc` and `ThingsBoardRuntime`

Out of scope:

- redesigning `hal/`, `driver/`, `protocol/`, or `device/`
- changing the confirmed ThingsBoard RPC semantics
- redesigning MQTT/LoRaWAN transport
- replacing `RobotFsm` with a different state-machine library or model
- introducing a database, actor system, or generalized plugin architecture

### Current Structural Findings

1. `main.cc` still acts as a business coordinator rather than a pure bootstrap file.
2. `ThingsBoardRuntime` now owns cloud concerns plus a growing amount of runtime policy.
3. `SchedulerService` exposes a lifecycle-shaped API, but the real start admission logic still lives outside it.
4. `CleanTask` is no longer part of the actual runtime truth and should be removed instead of preserved.
5. Runtime policy is currently split across several modules, so changing task semantics requires touching multiple files.

### Target Architecture

The runtime should be organized into four layers:

- `core runtime`
  - `RobotFsm`
  - `MotionService`
  - `NavService`
  - `FaultService`
  - `HealthService`
  - these classes keep owning robot behavior and motion primitives

- `runtime orchestration`
  - new `RobotSupervisor`
  - owns task admission, task lifecycle decisions, low-battery policy, spin-free policy, and cloud-period policy
  - becomes the single business-level entry point for schedule- and RPC-triggered actions

- `thingsboard integration`
  - keep `ThingsBoardConfigManager`
  - keep `ThingsBoardRuntime` in phase 1, but reduce it to cloud-facing control and publishing
  - in phase 2, split it into control-plane and telemetry-publisher responsibilities

- `platform bootstrap`
  - `main.cc`
  - only constructs dependencies, wires callbacks, starts threads, runs the process loop, and shuts down

### Design Principles

- one runtime truth for task admission and safety policy
- one state machine truth for motion state
- one cloud-facing integration boundary for ThingsBoard
- no dead or misleading task abstractions kept “just in case”
- prefer focused files over large coordinators, but do not churn stable lower layers unnecessarily

### Phase 1 Design: Structure Consolidation

Phase 1 is intentionally conservative. It should improve structure without changing approved business behavior.

#### New Runtime Orchestrator

Add:

- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`

`RobotSupervisor` responsibilities in phase 1:

- start a new task from schedule or RPC start
- resume a paused task
- pause the task
- trigger task-level return
- trigger terminate
- trigger reset from `Fault` or `Terminated`
- run safety-policy checks from the main loop
- expose the desired cloud reporting period
- expose a stable runtime snapshot for higher-level publishers

`RobotSupervisor` is not a replacement for `RobotFsm`. It is the business orchestrator above the FSM.

#### Data Flow

Inputs:

- `SchedulerService` time hit
- ThingsBoard RPC requests
- main-loop safety signals such as low battery and spin-free detection

Flow:

- all of those inputs go into `RobotSupervisor`
- `RobotSupervisor` validates admissibility and decides the business action
- `RobotSupervisor` drives `RobotFsm` events when motion-state transitions are required
- `RobotSupervisor` uses `ThingsBoardConfigManager` when a pending config must be promoted during task creation

Outputs:

- `RobotSupervisor` -> `RobotFsm`
- `RobotSupervisor` -> `FaultService`
- `RobotSupervisor` -> telemetry-facing runtime snapshot
- `RobotSupervisor` -> desired cloud period for the background publisher thread

#### CleanTask Removal

Remove:

- `include/pv_cleaning_robot/app/clean_task.h`
- `pv_cleaning_robot/app/clean_task.cc`

Also remove:

- construction of `CleanTask` in `main.cc`
- source references from product and test build files
- any tests that exist only to protect `CleanTask`

The current runtime already uses `RobotFsm` plus half-pass counters as the real task model. Keeping `CleanTask` only makes the code harder to trust.

#### Scheduler Role Clarification

`SchedulerService` remains in phase 1, but its role is explicitly narrowed:

- it is a time trigger, not the owner of task semantics
- it should notify a callback when a schedule entry is hit
- the callback should delegate to `RobotSupervisor`
- `RobotSupervisor` decides whether the robot is at home, whether pending config promotion succeeds, and whether a task is actually created

#### Main File End State For Phase 1

After phase 1, `main.cc` should still:

- load config
- create hardware and services
- create `RobotSupervisor`
- wire schedule triggers, ThingsBoard runtime, and thread executors
- run the outer process loop

But it should no longer directly:

- perform schedule-start admission logic
- contain task-control business rules
- perform low-battery and spin-free business decisions inline
- decide cloud publish cadence inline
- construct dead task-model objects

### Phase 2 Design: Model Unification

Phase 2 starts after phase 1 compiles cleanly and behavior remains intact.

#### Split ThingsBoard Runtime

Replace the current all-in-one shape of `ThingsBoardRuntime` with:

- `ThingsBoardControlPlane`
  - shared attributes
  - RPC registration
  - command acceptance/rejection plumbing
  - delegation into `RobotSupervisor`

- `ThingsBoardTelemetryPublisher`
  - startup client attributes
  - event telemetry
  - periodic business telemetry

This prevents the current “cloud integration layer becomes another coordinator” problem.

#### Runtime Snapshot Unification

Phase 2 should give `RobotSupervisor` a dedicated runtime snapshot type that becomes the upstream truth for:

- task state
- device state mapping
- half-pass progress
- command activity visibility
- cadence policy inputs

This reduces business-state derivation inside the ThingsBoard publisher layer.

### Files To Keep Stable

The following should not be deeply reworked during this refactor unless a direct bug blocks progress:

- `hal/*`
- `driver/*`
- `protocol/*`
- `device/*`
- `MotionService`
- `NavService`
- `FaultService`
- `HealthService`
- `RobotFsm`
- `ThingsBoardConfigManager`

These are lower-layer building blocks. The refactor problem is in orchestration, not in the lower stack.

### Phase 1 File Plan

Create:

- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `test/app/robot_supervisor_test.cc`

Delete:

- `include/pv_cleaning_robot/app/clean_task.h`
- `pv_cleaning_robot/app/clean_task.cc`

Modify:

- `pv_cleaning_robot/main.cc`
- `include/pv_cleaning_robot/service/thingsboard_runtime.h`
- `pv_cleaning_robot/service/thingsboard_runtime.cc`
- `include/pv_cleaning_robot/service/scheduler_service.h`
- `pv_cleaning_robot/service/scheduler_service.cc`
- `test/CMakeLists.txt`
- integration tests that currently assume the old placement of runtime logic

### Risk Management

1. Do not move everything at once. Introduce `RobotSupervisor` first, then reroute inputs to it.
2. Remove `CleanTask` only after the main runtime path no longer references it.
3. Keep `RobotFsm` as the motion-state authority during the whole refactor.
4. Do not change approved RPC semantics while moving code.
5. Prefer compile-safe, reviewable slices over large tree-wide moves.

### Testing Strategy

Phase 1 needs focused tests for:

- `RobotSupervisor` schedule-start admission
- `RobotSupervisor` paused-task resume
- `RobotSupervisor` return / terminate / reset routing
- low-battery policy routing
- spin-free policy routing
- cadence selection for active vs idle runtime
- integration tests still exercising schedule start and ThingsBoard-triggered control

Existing `ThingsBoardRuntime` tests should be adjusted so they validate delegation behavior rather than duplicating runtime business logic.

### Success Criteria

Phase 1 is successful when:

- `CleanTask` is fully removed from the build graph
- `main.cc` no longer directly owns task admission or safety policy
- schedule start and RPC task control both route through `RobotSupervisor`
- low-battery, spin-free, and cloud-cadence policy route through `RobotSupervisor`
- the code still compiles cleanly in the current environment

Phase 2 is successful when:

- the cloud integration layer is split into control-plane and telemetry responsibilities
- runtime state has one stable snapshot owner
- task semantics no longer need to be reconstructed across multiple files

