# Runtime Semantics and Codebase Re-Audit Design

Date: 2026-04-28

## Goal

Produce a fresh audit of the current repository state, using the code as it exists now rather than inheriting older audit conclusions, and turn that audit into actionable remediation packages.

This audit is explicitly focused on:

- runtime correctness
- code comprehension and maintainability
- redundant or repeated implementation shapes worth converging
- test coverage gaps
- documentation, config-sample, and operations-doc drift

The audit must not stop at “the code is messy.” It must identify which current implementations are incorrect, which abstractions are stale or incomplete, and how to group the problems into medium-sized remediation phases.

## Scope

Primary audit scope:

- `main.cc`
- `include/pv_cleaning_robot/app/*`
- `include/pv_cleaning_robot/service/*`
- `include/pv_cleaning_robot/middleware/*`
- `pv_cleaning_robot/app/*`
- `pv_cleaning_robot/service/*`
- `pv_cleaning_robot/middleware/*`
- `test/app/*`
- `test/service/*`
- `test/integration/*`
- `README.md`
- `doc/*`
- `docs/*` where current runtime behavior, architecture, or operations are described
- `config/config.json`

Additional code outside those directories must still be included in conclusions when it is the real root cause of a finding. The audit is not allowed to artificially stop at a layer boundary.

## Business Baseline For This Audit

The current code must be judged against the product semantics confirmed during this review, not against older simplified assumptions.

### Terminal Model

Business-layer runtime and documentation should move toward neutral terminal language:

- `terminal_a`
- `terminal_b`

Physical sensor names such as `front_limit` and `rear_limit` may remain in device and driver layers, but they must not continue to act as the business truth for parking, charging, or task-start semantics.

### Parking And Charging Model

Parking and charging are separate concerns.

- `parking_policy`
  - `left_only`
  - `right_only`
  - `both`

- `charging_side`
  - `left`
  - `right`
  - `both`

This means:

- the system may allow both terminals as valid task endpoints while only one terminal supports charging
- stopping at a valid terminal does not automatically mean the robot is charging

### Task-Pass Semantics

- `passes = 0.5`
  - valid only when `parking_policy = both`
  - the robot may end at the opposite terminal
  - if that terminal supports charging, the runtime may enter `Charging`
  - if that terminal does not support charging, the runtime should be `Idle`

- `passes = 1/2/3...`
  - the task ends at the terminal where that task started
  - task completion does not force a return to a fixed global charging side
  - whether the final state is `Charging` or `Idle` depends on whether the current terminal supports charging

### State Semantics

- `Charging`
  - means the robot is parked at a valid terminal and charging conditions are satisfied
  - must not be used as a generic “task ended and the robot is stopped” state

- `Idle`
  - means the robot has no active task
  - may still be parked at a valid terminal that is not charge-capable

### Start Semantics

When `parking_policy = both`:

- a new task may start from either valid terminal
- the robot does not need to move to `charging_side` before a new task starts

## Audit Output Structure

The audit output is split into five formal finding groups.

1. Runtime correctness
2. Code comprehension and maintainability
3. Convergeable or optimizable implementation shapes
4. Test coverage gaps
5. Documentation and operations drift

Each finding must include:

- `ID`
- `Title`
- `Category`
- `Priority`
- `Files`
- `Summary`
- `Impact`
- `Root Cause Layer`
- `Recommended Action`
- `Remediation Package`

## Priority Rules

- `P0`
  - confirmed wrong runtime behavior
  - incorrect or incomplete business-state modeling
  - concurrency or lifecycle issues that can break source-of-truth assumptions
  - docs or configs that can directly mislead operations into unsafe behavior

- `P1`
  - important structural debt
  - incomplete abstraction boundaries
  - duplicated serialization or snapshot logic
  - high-value test and documentation remediation items

- `P2`
  - localized but real problems
  - limited-scope cleanup candidates

- `Observe`
  - record only
  - not worth immediate code-change planning

## Current Confirmed Findings

This section captures the major findings already confirmed during this audit pass.

### Runtime Correctness

#### RAUDIT-001
- **Title:** Start admission is still hard-wired around one “home” side
- **Category:** Runtime correctness
- **Priority:** P0
- **Files:** `pv_cleaning_robot/app/robot_supervisor.cc`, `pv_cleaning_robot/main.cc`
- **Summary:** `RobotSupervisor::start_scheduled_task()` still requires `at_home == true`, and the main wiring still derives `at_home` directly from the rear limit. This cannot implement the confirmed `parking_policy = both` behavior where a valid task may start from either terminal.
- **Impact:** Valid tasks will be wrongly rejected when the robot is parked at the opposite legal terminal.
- **Root Cause Layer:** `app` plus top-level runtime wiring
- **Recommended Action:** Replace the current `at_home/at_front` admission model with a terminal-aware parking-policy model.
- **Remediation Package:** `A`

#### RAUDIT-002
- **Title:** `Charging` is overloaded and no longer matches confirmed product semantics
- **Category:** Runtime correctness
- **Priority:** P0
- **Files:** `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/app/robot_fsm.cc`, `pv_cleaning_robot/app/robot_supervisor.cc`
- **Summary:** The runtime currently uses `Charging` to mean task complete, parked, and charge-capable, even though those are now distinct concepts. `passes=0.5` also currently ends in `Charging` unconditionally.
- **Impact:** Runtime state, telemetry, task admission, and external understanding of the robot state are all incorrect or ambiguous.
- **Root Cause Layer:** `app`
- **Recommended Action:** Keep charging as a strict semantic state and move terminal and charge-capability semantics into explicit runtime data.
- **Remediation Package:** `A`

#### RAUDIT-003
- **Title:** Integer-pass tasks cannot currently model “return to the start terminal”
- **Category:** Runtime correctness
- **Priority:** P0
- **Files:** `pv_cleaning_robot/app/robot_fsm.cc`
- **Summary:** `EvScheduleStart{at_front=true}` is implemented as a special “0.5 restart from the front” path rather than as a general “new task starts from terminal_b” semantic. The task model still assumes a fixed home/front world.
- **Impact:** The confirmed rule “integer-pass tasks return to the terminal where that task started” is not represented.
- **Root Cause Layer:** `app`
- **Recommended Action:** Add explicit start-terminal and current-terminal semantics to the task model.
- **Remediation Package:** `A`

#### RAUDIT-004
- **Title:** The runtime config model cannot represent `parking_policy` or `charging_side`
- **Category:** Runtime correctness
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/service/thingsboard_config_manager.h`, `pv_cleaning_robot/service/thingsboard_config_manager.cc`, `config/config.json`
- **Summary:** The current ThingsBoard runtime config only models passes, speeds, brush RPM, and schedules. The confirmed product semantics require parking-policy and charge-capability modeling, but there is no formal config representation for either.
- **Impact:** The system cannot even be configured correctly for the current product contract.
- **Root Cause Layer:** `service` config model
- **Recommended Action:** Extend the formal config model before further runtime corrections are attempted.
- **Remediation Package:** `A`

#### RAUDIT-005
- **Title:** Shared-attribute source-of-truth can be lost during startup
- **Category:** Runtime correctness
- **Priority:** P0
- **Files:** `pv_cleaning_robot/main.cc`, `pv_cleaning_robot/service/cloud_service.cc`, `test/service/cloud_service_test.cc`
- **Summary:** The network connects before the shared-attribute callback is registered. `CloudService` silently drops shared-attribute messages when no callback is installed yet, and the current test suite explicitly treats that loss as acceptable behavior.
- **Impact:** The robot may start with stale local config while ThingsBoard is supposed to be the operational source of truth.
- **Root Cause Layer:** `service` plus startup wiring
- **Recommended Action:** Ensure the callback is installed before connection or add an explicit post-connect attribute sync path; update tests accordingly.
- **Remediation Package:** `A`

#### RAUDIT-006
- **Title:** `Charging -> Idle` is modeled, but the runtime path that should drive it is incomplete
- **Category:** Runtime correctness
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/app/robot_fsm.cc`, `pv_cleaning_robot/main.cc`
- **Summary:** The FSM expects `EvChargeDone` to leave `Charging`, but the main runtime path does not clearly wire real charge-complete behavior into that event.
- **Impact:** The robot may remain in `Charging` indefinitely, distorting later task admission and telemetry.
- **Root Cause Layer:** `app` plus runtime wiring
- **Recommended Action:** Explicitly wire charge-complete detection or redesign how charging entry and exit are modeled.
- **Remediation Package:** `A`

#### RAUDIT-007
- **Title:** Runtime truth still leaks physical `front/rear/home` naming into business decisions
- **Category:** Runtime correctness
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/main.cc`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- **Summary:** Physical sensor orientation names are still directly used as business semantics for start admission and state flow, even though the product model now requires terminal-level abstractions.
- **Impact:** Parking and charging semantics will remain fragile and difficult to correct if business logic stays coupled to physical sensor names.
- **Root Cause Layer:** `app` and `service`
- **Recommended Action:** Keep physical names in device/driver layers only and introduce terminal semantics in business-facing layers.
- **Remediation Package:** `B`

#### RAUDIT-008
- **Title:** Runtime and telemetry snapshots do not expose terminal or charge-capability truth
- **Category:** Runtime correctness
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`, `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`, `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
- **Summary:** The snapshot model currently exposes only generic states and pass counts. It cannot tell the platform which terminal the robot is at or whether that terminal is charge-capable.
- **Impact:** The platform will continue to see incomplete or misleading state even after some runtime logic is corrected.
- **Root Cause Layer:** `app` plus `service`
- **Recommended Action:** Promote terminal and charging semantics into the formal runtime snapshot.
- **Remediation Package:** `C`

#### RAUDIT-009
- **Title:** `EventBus` silently truncates subscribers beyond 32 despite comments claiming truncation is observable
- **Category:** Runtime correctness
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/middleware/event_bus.h`
- **Summary:** The implementation copies only the first 32 callbacks during `publish()` and drops the rest without any warning or metric, even though comments say the truncation is recorded.
- **Impact:** Event consumers can disappear silently as the system grows.
- **Root Cause Layer:** `middleware`
- **Recommended Action:** Make truncation observable at minimum; ideally make the limit explicit and testable.
- **Remediation Package:** `A`

### Code Comprehension And Maintainability

#### RAUDIT-010
- **Title:** `RobotSupervisor` centralizes old logic but does not yet define an independent runtime model
- **Category:** Code comprehension and maintainability
- **Priority:** P1
- **Files:** `pv_cleaning_robot/app/robot_supervisor.cc`, `pv_cleaning_robot/app/robot_fsm.cc`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- **Summary:** The supervisor still derives most decisions from FSM string states and old `at_home/at_front` assumptions. It is a coordinator, not yet a clean runtime truth layer.
- **Impact:** New semantics are likely to be bolted on as more conditionals rather than integrated into a coherent task model.
- **Root Cause Layer:** `app`
- **Recommended Action:** Separate runtime task semantics from raw FSM outward state names.
- **Remediation Package:** `B`

#### RAUDIT-011
- **Title:** `SchedulerService` still sits in an unclear middle ground between “time trigger” and “lifecycle owner”
- **Category:** Code comprehension and maintainability
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/service/scheduler_service.h`, `pv_cleaning_robot/service/scheduler_service.cc`
- **Summary:** It was renamed toward time-trigger semantics, but it still exposes unused “window expiry” lifecycle hooks and keeps owner-shaped internal state.
- **Impact:** Readers cannot easily tell whether it is only a scheduler tick source or whether it still governs task windows.
- **Root Cause Layer:** `service`
- **Recommended Action:** Either reduce it to a pure trigger or finish the lifecycle contract explicitly.
- **Remediation Package:** `B`

#### RAUDIT-012
- **Title:** Business-facing code still uses physical direction vocabulary instead of terminal vocabulary
- **Category:** Code comprehension and maintainability
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/main.cc`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- **Summary:** Physical direction naming is still leaking into task semantics, telemetry assumptions, and task-start reasoning.
- **Impact:** It is hard for developers to reason about real business behavior independently of installation orientation.
- **Root Cause Layer:** `app` and `service`
- **Recommended Action:** Standardize business-facing terminology around `terminal_a` and `terminal_b`.
- **Remediation Package:** `B`

### Convergeable Or Optimizable Implementation Shapes

#### RAUDIT-013
- **Title:** `RobotRuntimeSnapshot` and `BusinessTelemetrySnapshot` are near-duplicate models
- **Category:** Convergeable implementations
- **Priority:** P1
- **Files:** `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`, `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`, `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
- **Summary:** Runtime truth is first built as one snapshot and then copied field-by-field into another snapshot type with almost the same shape.
- **Impact:** Every new runtime field must be maintained in two models and one mapping layer.
- **Root Cause Layer:** `app` plus telemetry-facing `service`
- **Recommended Action:** Collapse to a single business/runtime snapshot model or make the telemetry builder consume the runtime snapshot directly.
- **Remediation Package:** `C`

#### RAUDIT-014
- **Title:** `BusinessTelemetrySnapshot::to_json()` is a second JSON encoding path that no longer matches the primary runtime path
- **Category:** Convergeable implementations
- **Priority:** P1
- **Files:** `pv_cleaning_robot/service/business_telemetry_snapshot.cc`, `pv_cleaning_robot/service/business_payload_builder.cc`
- **Summary:** The product path now uses `BusinessPayloadBuilder`, but `BusinessTelemetrySnapshot::to_json()` still serializes the same domain independently through `nlohmann::json`.
- **Impact:** Payload semantics can drift across two implementations.
- **Root Cause Layer:** `service`
- **Recommended Action:** Keep only one formal business-telemetry encoding path.
- **Remediation Package:** `C`

#### RAUDIT-015
- **Title:** Business and event payload builders duplicate the same fixed JSON writer and command-phase mapping logic
- **Category:** Convergeable implementations
- **Priority:** P1
- **Files:** `pv_cleaning_robot/service/business_payload_builder.cc`, `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- **Summary:** Both builders embed almost identical fixed-buffer JSON writer logic plus duplicated `CommandPhase -> string` mapping.
- **Impact:** Low-level serialization fixes and semantic field changes must be repeated across separate files.
- **Root Cause Layer:** `service`
- **Recommended Action:** Extract shared fixed-buffer JSON utilities and shared command-field encoding helpers.
- **Remediation Package:** `C`

### Test Coverage Gaps

#### RAUDIT-016
- **Title:** The current test suite hard-codes “task end means Charging” across multiple layers
- **Category:** Test coverage gaps
- **Priority:** P0
- **Files:** `test/app/robot_fsm_test.cc`, `test/app/robot_supervisor_test.cc`, `test/integration/task_chain_test.cc`
- **Summary:** Many tests assert that `passes=0.5`, return-to-home completion, and normal task completion all end in `Charging`, regardless of parking and charging semantics.
- **Impact:** The suite currently protects outdated behavior and will actively resist correct semantic fixes.
- **Root Cause Layer:** tests reflecting stale runtime assumptions
- **Recommended Action:** Add terminal-aware state assertions first, then rewrite existing end-state expectations.
- **Remediation Package:** `D`

#### RAUDIT-017
- **Title:** No formal tests exist for terminal-aware parking, charging, or `passes=0.5` legality rules
- **Category:** Test coverage gaps
- **Priority:** P1
- **Files:** `test/app/robot_supervisor_test.cc`, `test/service/thingsboard_control_plane_test.cc`, `test/integration/system_integration_test.cc`
- **Summary:** The existing tests cover only the old `at_home/at_front/Charging` model, not the now-confirmed terminal and charging semantics.
- **Impact:** Future runtime fixes will have almost no meaningful regression net until new semantic tests are added.
- **Root Cause Layer:** tests
- **Recommended Action:** Add a terminal-semantics test matrix before major semantic refactors.
- **Remediation Package:** `D`

#### RAUDIT-018
- **Title:** The current cloud test suite explicitly encodes shared-attribute startup loss as expected behavior
- **Category:** Test coverage gaps
- **Priority:** P1
- **Files:** `test/service/cloud_service_test.cc`
- **Summary:** The test currently asserts that messages arriving before callback registration are ignored.
- **Impact:** It normalizes a startup loss path that is incompatible with ThingsBoard-as-source-of-truth behavior.
- **Root Cause Layer:** tests
- **Recommended Action:** Replace this expectation with tests that validate correct startup synchronization behavior.
- **Remediation Package:** `D`

### Documentation And Operations Drift

#### RAUDIT-019
- **Title:** `doc/thingsboard_config.md` still documents start semantics that are no longer valid
- **Category:** Documentation and operations drift
- **Priority:** P1
- **Files:** `doc/thingsboard_config.md`
- **Summary:** The document still says start does not need to check parking position, while the current confirmed semantics require terminal-aware admission and `passes=0.5` restrictions.
- **Impact:** It will mislead future implementation and integration work.
- **Root Cause Layer:** product-facing documentation
- **Recommended Action:** Rewrite the document against the current terminal/parking/charging model.
- **Remediation Package:** `E`

#### RAUDIT-020
- **Title:** `README` and `API_REFERENCE` are materially out of sync with current runtime structure and semantics
- **Category:** Documentation and operations drift
- **Priority:** P0
- **Files:** `README.md`, `doc/API_REFERENCE.md`
- **Summary:** The docs still contain stale scheduler API names, describe `CleanTask`, omit `Paused` and `Terminated`, and preserve the old simplified charging model.
- **Impact:** Developers, testers, and operators cannot trust the repo documentation as an accurate system reference.
- **Root Cause Layer:** maintained documentation
- **Recommended Action:** Treat API docs and README synchronization as a first-class remediation task, not an optional polish item.
- **Remediation Package:** `E`

#### RAUDIT-021
- **Title:** README claims OTA is excluded from product build, but the product build still globs OTA sources
- **Category:** Documentation and operations drift
- **Priority:** P1
- **Files:** `README.md`, `pv_cleaning_robot/CMakeLists.txt`
- **Summary:** Repository messaging says OTA is dormant and not part of the product build, but the build system still compiles middleware sources by glob.
- **Impact:** Contributors cannot tell whether OTA is merely unwired or truly excluded.
- **Root Cause Layer:** documentation plus build-system presentation
- **Recommended Action:** Make build behavior and documentation say the same thing.
- **Remediation Package:** `E`

## Remediation Packages

### Package A: Runtime Correctness And Business Model Repair

Priority: `P0`

Targets:

- `RAUDIT-001`
- `RAUDIT-002`
- `RAUDIT-003`
- `RAUDIT-004`
- `RAUDIT-005`
- `RAUDIT-006`
- `RAUDIT-009`

Purpose:

- repair the core runtime semantics first
- introduce formal parking and charging business data
- stop treating `Charging` as a generic stopped state
- ensure cloud configuration truth is not lost at startup

### Package B: Runtime Main-Path Convergence

Priority: `P1`

Targets:

- `RAUDIT-007`
- `RAUDIT-010`
- `RAUDIT-011`
- `RAUDIT-012`

Purpose:

- reduce the remaining semantic split across main runtime, supervisor, and FSM layers
- move business language from physical sensor orientation to terminal semantics

### Package C: Telemetry And Config Encoding Convergence

Priority: `P1`

Targets:

- `RAUDIT-008`
- `RAUDIT-013`
- `RAUDIT-014`
- `RAUDIT-015`

Purpose:

- align runtime truth, telemetry snapshots, and payload encoding
- remove duplicated snapshot and serialization logic
- extend the config model so telemetry and cloud control can represent the real product

### Package D: Test Reinforcement

Priority: `P1`

Targets:

- `RAUDIT-016`
- `RAUDIT-017`
- `RAUDIT-018`

Purpose:

- stop protecting stale semantics
- build a terminal-aware regression net before deeper runtime changes

### Package E: Documentation And Operations Synchronization

Priority: `P1`

Targets:

- `RAUDIT-019`
- `RAUDIT-020`
- `RAUDIT-021`

Purpose:

- restore trust in repo documentation, API references, and config guidance
- keep operations guidance aligned with the actual runtime model and build graph

## Recommended Execution Order

1. Package `A`
2. The subset of Package `D` needed to safely support Package `A`
3. Package `B`
4. Package `C`
5. Package `E`

Rationale:

- the primary issue is no longer just code shape; it is runtime semantics drift
- medium-scale convergence work is appropriate, but only after the runtime truth is corrected

## Non-Goals

This audit and remediation design does not:

- redesign device protocols
- redesign MQTT or LoRaWAN transport architecture
- refactor low-level hardware wrappers purely for style
- turn the runtime into a new generalized framework

The focus is the live product path and the clarity of the code and documentation that describe it.
