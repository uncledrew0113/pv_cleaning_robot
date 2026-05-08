# Runtime Config And Code Simplification Design

Date: 2026-05-08

## Goal

This design defines a narrow refactor that makes runtime configuration easier to reason about, reduces duplicated code, and improves comments around ThingsBoard integration.

The priorities are:

1. keep the project stable
2. simplify configuration truth
3. reduce duplication and scattered runtime-config logic
4. improve readability, especially in ThingsBoard-related code

This design does not introduce a new framework or a large architecture rewrite.

## Scope

This work covers three areas:

1. runtime configuration file layout and ownership
2. simplification of duplicated or scattered implementation and test code related to runtime config and ThingsBoard
3. targeted code comments for ThingsBoard and runtime-config behavior

This design is intentionally focused. It does not redesign the FSM, MQTT stack, or the full `main.cc` structure beyond what is necessary to centralize config behavior.

## Configuration Model

The project will use three runtime-related files:

- `config/config.fixed.json`
- `config/config.runtime.json`
- `config/config.runtime.pending.json`

Their responsibilities are fixed:

### `config.fixed.json`

Contains non-cloud-mutable system configuration such as:

- `gpio.*`
- `serial.*`
- `can.*`
- `network.mqtt.*`
- `network.lorawan.*`
- `device.*`
- logging, watchdog, thread, and storage-path settings

ThingsBoard must never write this file.

### `config.runtime.json`

Contains the active runtime configuration used by the running system.

Runtime fields are:

- `passes`
- `parking_side`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `return_brush_rpm`
- `start_battery_soc`
- `charge_start_soc`
- `charge_stop_soc`
- `scheduler windows`

### `config.runtime.pending.json`

Contains pending runtime changes that only take effect for the next task.

Pending fields are:

- `passes`
- `parking_side`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `return_brush_rpm`
- `start_battery_soc`
- `charge_start_soc`
- `charge_stop_soc`

If there are no pending changes, this file should not exist.

## Effect Timing

The effect model is deliberately simple:

- `scheduler windows` take effect immediately
- every other runtime field takes effect on the next task start

### Immediate effect

When `scheduler windows` change through ThingsBoard shared attributes:

- `config.runtime.json` is updated immediately
- in-memory active runtime config is updated immediately
- `SchedulerService` windows are replaced immediately
- `config.runtime.pending.json` is untouched

### Next-task effect

All other runtime fields are written to pending only.

They do not change the current active runtime config while a task is already in progress.

Pending config is promoted only when a new task is started from one of these states:

- `Idle`
- `Stopped`
- `Charging`

Promotion order must be:

1. validate start conditions
2. if start is allowed and pending exists, promote pending to active
3. persist new active runtime config
4. remove pending file
5. enter the new task start path

This order prevents a failed start from consuming pending config.

## Ownership And Responsibilities

## `ConfigService`

`ConfigService` remains a light JSON file service.

It is responsible for:

- loading and saving config files
- replace-and-save behavior
- backup and fallback behavior for persisted runtime config if retained
- loading, saving, and clearing pending files
- subtree access helpers

It is not responsible for:

- deciding which fields are immediate or pending
- understanding business meaning of `parking_side`
- understanding start or charging rules

## `ThingsBoardConfigManager`

`ThingsBoardConfigManager` becomes the only runtime-config semantics owner.

It is responsible for:

- parsing shared attributes
- validating runtime fields
- classifying each field as immediate or next-task
- maintaining in-memory active and pending runtime config
- updating `SchedulerService`
- persisting active runtime config
- persisting and clearing pending runtime config
- promoting pending to active

Other modules should not re-implement any of this logic.

## `main.cc`

`main.cc` should only:

- load fixed config
- construct services and devices
- inject runtime-config dependencies

`main.cc` should not:

- classify runtime fields
- decide whether a shared attribute belongs in active or pending
- directly read scattered runtime values when a runtime-config object already owns them

## Read Rules

The codebase will follow these read rules:

- fixed system settings are read from fixed config during startup and dependency construction
- runtime business settings are read from `ThingsBoardConfigManager::active_config()`
- pending config is only read by config-management code and by the new-task promotion path

This keeps runtime config truth single-sourced.

## Simplification Targets

This refactor should explicitly reduce duplication in these areas:

1. scattered runtime-config path handling across main code and tests
2. repeated ThingsBoard shared-attribute key lists
3. repeated fixture code that constructs runtime config, pending paths, and manager objects
4. direct runtime-config reads in modules that should only consume active config
5. stale compatibility branches that are no longer needed after the split

This refactor should not add broad abstraction layers to solve one narrow problem.

## Comment Strategy

Comments should be added only where the behavior is not obvious.

Priority comment targets:

- `ThingsBoardConfigManager`
  - active vs pending split
  - why scheduler windows are immediate
  - when pending is promoted
- `ThingsBoardControlPlane`
  - shared-attribute handling contract
  - why runtime shared attributes do not change an active task
  - RPC-to-business mapping where non-obvious
- `CloudService`
  - ThingsBoard topic semantics and reject behavior
- `main.cc`
  - boundary between fixed config and runtime config
  - why `MotionService` direction depends on active parking side

Comments should explain why a rule exists, not restate code mechanically.

## Testing Strategy

Validation will be done in four layers:

### 1. Config tests

Verify:

- fixed/runtime/pending file loading
- pending absence behavior
- pending promotion behavior
- scheduler window immediate replacement behavior

### 2. App tests

Verify:

- pending promotion only on allowed new-task start states
- current task is unaffected by pending runtime updates
- natural `Charging -> Idle` transition does not auto-promote pending

### 3. ThingsBoard tests

Verify:

- scheduler windows update active immediately
- other runtime attributes update pending only
- reconnect preserves the same behavior
- runtime-mock path consumes pending only on the next new task

### 4. System regression

Run focused regression on:

- motion
- supervisor
- ThingsBoard config manager
- ThingsBoard control plane
- task chain
- system integration
- real shared-attribute tests
- real runtime-mock tests

## Implementation Phases

### Phase 1: File layout and fixture updates

- introduce the fixed/runtime/pending file split
- update config-oriented tests and fixtures

### Phase 2: Centralize runtime semantics in `ThingsBoardConfigManager`

- move immediate/pending classification fully into manager
- keep scheduler windows immediate
- move all other runtime fields to pending

### Phase 3: Main and call-site cleanup

- ensure runtime reads go through manager active config
- remove scattered runtime config interpretation from `main.cc` and fixtures

### Phase 4: Comment and duplication cleanup

- delete stale repeated code paths
- consolidate repeated helpers
- add focused explanatory comments

Each phase should finish with the smallest relevant build and test set before moving on.

## Non-Goals

This design does not include:

- a new config framework
- a full rewrite of `main.cc`
- a redesign of MQTT, CloudService, or the FSM
- broad unrelated refactors for style alone

## Recommendation

Implement the runtime config split with the existing `ConfigService + ThingsBoardConfigManager` structure, not with a new heavyweight store abstraction.

This is the smallest design that:

- separates fixed vs cloud-managed config
- makes runtime behavior easier to explain
- reduces duplication
- preserves system stability while simplifying the codebase
