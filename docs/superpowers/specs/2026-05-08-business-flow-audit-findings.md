# PV Cleaning Robot Business Flow Audit Findings

## 1. Audit Scope

- Runtime truth source:
  - `pv_cleaning_robot/main.cc`
  - modules wired directly by `main.cc`
- Tests are used only as validation and divergence detectors.
- README, design docs, and historical comments are not treated as truth when code differs.
- This audit is evaluated against the user-confirmed simplified business model:
  - fixed local config vs. cloud-immediate config vs. cloud-next-task config
  - invalid dual-endpoint / off-endpoint states must be first-class runtime states
  - RPC contract should collapse to `start / stop / return / reset`
  - `start` always means new task, not resume
  - `stop` means terminate and halt, not pause
  - `reset` should reboot the device
  - low battery should gate start, not trigger mid-task return
  - charging should be threshold-driven at the parking side
  - project-wide goal is deletion and simplification while preserving basic stable behavior

## 2. Runtime Flow Map

`ConfigService -> main wiring -> LimitSwitch/SafetyMonitor -> main event bridge -> RobotFsm -> RobotSupervisor -> Motion/Nav/Fault/Scheduler -> CloudService/ThingsBoard -> periodic reporting/cache`

Observed high-level control path in current code:

1. `ConfigService` loads `config.json`, with optional `backup` and `pending` companions.
2. `main.cc` constructs all devices, services, application objects, and runtime callbacks.
3. `SafetyMonitor` converts physical left/right limit edges into debounced settled events.
4. `main.cc` translates physical settled events into business FSM events using current `parking_side`.
5. `RobotSupervisor` gates start/pause/return/terminate/reset requests and aggregates snapshots.
6. `RobotFsm` owns task-state progression and directly triggers motion actions.
7. `ThingsBoardControlPlane` maps shared attributes and RPC into config/supervisor calls.
8. `CloudService` owns ThingsBoard topic routing, RPC parsing, and telemetry/attribute publish.
9. Main loop periodically runs scheduler tick, low-battery safety tick, and reporting-period switching.

## 3. Flow Findings

### 3.1 Startup and Configuration Loading

**Intended behavior**

- Fixed local configuration should be loaded once and remain authoritative for hardware wiring.
- Cloud-mutable immediate configuration should take effect without waiting for the next task.
- Cloud-mutable next-task configuration should persist safely and take effect exactly at the next task boundary.
- Backup fallback should preserve startup availability without hiding the fact that the system is running from degraded configuration.

**Actual behavior in current code**

- `ConfigService::load()` loads the main file first, then `*.backup`, and only records a boolean `last_load_used_backup()`.
- `ThingsBoardConfigManager` owns `active_` and optional `pending_`, but currently only stages task-related fields plus schedules; fixed-local vs. immediate-vs-next-task ownership is not modeled explicitly.
- `main.cc` still reads startup wiring, battery thresholds, MQTT credentials, report periods, and parking-side self-check inputs directly from `ConfigService`.
- Startup fallback is only projected outward through `publish_backup_fallback_event()`, not as a first-class runtime mode used by later logic.

**Deterministic defects**

- Configuration ownership is not structurally separated into the three target classes. Current code mixes:
  - direct `ConfigService` reads in `main.cc`
  - `ThingsBoardConfigManager::active_config()`
  - `ThingsBoardConfigManager::pending_config()`
  This is a real defect because any future field can be misclassified and silently interpreted in multiple places.
- Battery thresholds are partially hard-coded to existing BMS constructor fields (`battery_full_soc`, `battery_low_soc`), but the target charging/start-gate model requires more than two thresholds. The current config model cannot express the confirmed target behavior cleanly.

**Semantic / responsibility risks**

- `main.cc` still acts as a runtime policy center, not only a composition root.
- Backup fallback is currently “observable as an event” but not “represented as runtime state”. This is not yet a proven misbehavior, but it is easy to misread downstream.

**Involved modules**

- `pv_cleaning_robot/main.cc`
- `pv_cleaning_robot/service/config_service.cc`
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`

### 3.2 Power-On Self-Check and Position Interpretation

**Intended behavior**

- Physical sensors should remain `left/right`.
- Business position should be derived from `parking_side`.
- If both endpoints are active:
  - report invalid position/state
  - do not allow task start
- If neither endpoint is active:
  - report invalid/off-endpoint state
  - do not start a task directly
  - attempt to return to the parking side

**Actual behavior in current code**

- `main.cc` computes `startup_facts` via `ParkingSideRuntime::from_physical_limits(...)`.
- If `!startup_facts.at_parking_side`, it only logs a warning.
- If `startup_facts.at_far_end`, it only logs a warning.
- No explicit invalid-position state is created for:
  - both sensors active
  - neither sensor active
- No automatic return-to-parking-side attempt is triggered at startup when the robot is off-endpoint.

**Deterministic defects**

- Dual-endpoint and no-endpoint startup states are only weak warnings today. That directly violates the confirmed target model.
- Startup location abnormality is not fed into `Supervisor`, `FSM`, or cloud status as a first-class state; therefore later logic cannot reliably distinguish “safe to start” from “position unknown but only warned”.

**Semantic / responsibility risks**

- `ParkingSideRuntime` is correct as a mapping helper, but the decision of “which parking-side truth to use now” still lives in call sites.
- `main.cc` owns startup position policy that should eventually move behind a dedicated runtime truth component.

**Involved modules**

- `pv_cleaning_robot/main.cc`
- `include/pv_cleaning_robot/app/parking_side_runtime.h`
- `pv_cleaning_robot/middleware/safety_monitor.cc`
- `pv_cleaning_robot/device/limit_switch.cc`

### 3.3 Scheduler Start and RPC Start

**Intended behavior**

- Task-start admission should have one business authority.
- `start` RPC should always mean “start a new task”.
- Scheduler-triggered start and RPC-triggered start should share the same admission logic.
- Start should fail when position is invalid or battery is below configured start threshold.

**Actual behavior in current code**

- Scheduler callback in `main.cc` directly calls `supervisor->start_task(facts.at_parking_side)`.
- RPC `start` is implemented in `ThingsBoardControlPlane`; it first checks `state == "Paused"` and resumes instead of starting a new task.
- `RobotSupervisor::start_task()` checks only:
  - current state is `Idle` or `Charging`
  - `at_parking_side == true`
  - pending promotion succeeds
- Start gating does not inspect battery thresholds; low battery is enforced later by `tick_safety()`, not by pre-start admission.

**Deterministic defects**

- RPC `start` still has resume semantics. This directly conflicts with the confirmed target contract.
- Start admission is distributed across:
  - `main.cc`
  - `ThingsBoardControlPlane`
  - `RobotSupervisor`
  - `RobotFsm`
  This is not just style debt; it already changes observable behavior because RPC `start` and scheduler start do not mean the same thing today.
- Battery-gate behavior is missing from pre-start admission. Under the new target model this is a concrete gap.

**Semantic / responsibility risks**

- `EvScheduleStart` is still the public start event for both scheduler and RPC, even though the future model is broader than “schedule-only”.
- `is_at_start_far_end` is still computed and logged through RPC registration, but start admission does not use it anymore.

**Involved modules**

- `pv_cleaning_robot/main.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/device/bms.cc`

### 3.4 Cleaning Round-Trip Main Flow

**Intended behavior**

- One pass means one full out-and-back trip.
- The task loop should remain anchored to parking-side semantics, not historical front/rear or forward/return naming.
- Task completion should occur only when the robot returns to the parking side and pass count is satisfied.

**Actual behavior in current code**

- `RobotFsm` still uses:
  - `StateCleanFwd`
  - `StateCleanReturn`
- `EvFarEndLimitSettled` transitions `CleanFwd -> CleanReturn`.
- `EvParkingSideLimitSettled` in `CleanReturn` either:
  - increments pass count and loops to `CleanFwd`
  - or transitions to `Charging`
- `main.cc` bridges physical `LimitSettledEvent` into one of those two FSM events using current active `parking_side`.

**Deterministic defects**

- Completion always enters `Charging`, regardless of battery level. This directly conflicts with the confirmed target model, which requires:
  - `Charging` only when battery is below charge-low threshold
  - otherwise `Idle`
- FSM progression trusts bridged semantic events blindly; there is no second-level consistency check once `main.cc` classifies the limit event.

**Semantic / responsibility risks**

- `CleanFwd / CleanReturn` names still encode historical direction semantics and will continue to attract incorrect mental models when parking side is `Right`.
- The main task loop is implemented correctly enough for the previous model, but it is richer than the desired “stable minimal base behavior” model.

**Involved modules**

- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/main.cc`
- `pv_cleaning_robot/service/motion_service.cc`

### 3.5 Stop and Task-Level Return

**Intended behavior**

- `stop` should terminate the current task and halt the robot.
- `return` should command the robot to go back to the parking side.
- There should be no pause/resume branch in the target model.

**Actual behavior in current code**

- `ThingsBoardControlPlane` maps:
  - `stop` -> `supervisor_->pause_task()`
  - `return` -> `supervisor_->return_task()`
  - `terminate` -> `supervisor_->terminate_task()`
- `RobotSupervisor` still exposes:
  - `pause_task()`
  - `resume_paused_task()`
  - `return_task()`
  - `terminate_task()`
- `RobotFsm` still supports:
  - `Paused`
  - `EvPauseTask`
  - `EvResumeTask`
  - `EvManualReturn`
  - `EvTerminateTask`

**Deterministic defects**

- RPC `stop` currently means pause, not terminate-and-halt. This is a direct behavioral mismatch.
- Task control is split into three distinct branches (`pause`, `return`, `terminate`) when the target model only needs two user-facing actions (`stop`, `return`).

**Semantic / responsibility risks**

- `Returning` currently mixes at least three meanings:
  - user-triggered return
  - low-battery return
  - P1 fault return
- The codebase still has explicit pause/resume behavior baked into tests, snapshots, telemetry, and state names; this is major simplification debt.

**Involved modules**

- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/app/robot_fsm.cc`

### 3.6 Reset and Reboot Behavior

**Intended behavior**

- `reset` should reboot the device.
- Reboot should be a device-lifecycle action, not an FSM reset-to-idle action.

**Actual behavior in current code**

- RPC `reset` calls `supervisor_->reset_task(at_parking_side)`.
- `RobotSupervisor::reset_task()` only dispatches `EvFaultReset` when state is `Fault` or `Terminated` and the robot is at the parking side.
- `RobotFsm::dispatch<EvFaultReset>` transitions to `Idle`.
- The only actual reboot implementation in the codebase is `OtaManager::apply_and_reboot()`, unrelated to ThingsBoard reset RPC.

**Deterministic defects**

- Reset semantics are completely different from the confirmed target model. Current `reset` is a state-machine reset, not a device reboot.
- Reboot logic already exists elsewhere but is not integrated into runtime task control, which makes the current RPC contract internally inconsistent.

**Semantic / responsibility risks**

- `EvFaultReset` currently conflates:
  - fault recovery
  - terminated-task recovery
- This event is likely to disappear or narrow sharply in the follow-on simplification phase.

**Involved modules**

- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/middleware/ota_manager.cc`

### 3.7 Fault Reporting and Safe Halt

**Intended behavior**

- Fault state must be reported.
- Safe halt behavior must be preserved where truly required for safety.
- Extra differentiated branches should be kept only if they are necessary for safe operation.

**Actual behavior in current code**

- `FaultService` publishes fault events and stores the last fault.
- `FaultHandler` applies differentiated behavior:
  - `P0`: `motion_->emergency_stop()`, then dispatch fault
  - `P1`: `motion_->pause_task()`, then dispatch fault
  - `P2`: report only
- `RobotFsm` still has:
  - `EvFaultP0 -> Fault`
  - `EvFaultP1 -> Returning`
  - `EvFaultP2` no state change
- `main.cc` watchdog timeout reports synthetic `P0`.

**Deterministic defects**

- Fault behavior is more branched than the confirmed target model and currently couples motion side-effects in both `FaultHandler` and `RobotFsm`.
- P1 fault still drives a special return branch, while the target model only requires reporting and simplification unless separate behavior is truly safety-critical.

**Semantic / responsibility risks**

- Fault semantics are split across three layers:
  - `FaultService`
  - `FaultHandler`
  - `RobotFsm`
- It is not yet explicit which differentiated behaviors are truly safety-required and which are historical workflow complexity.

**Involved modules**

- `pv_cleaning_robot/service/fault_service.cc`
- `pv_cleaning_robot/app/fault_handler.cc`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/main.cc`

### 3.8 Charging and Battery Gate Behavior

**Intended behavior**

- Low battery should block task start before entering task execution.
- Charging should depend on charge-low and charge-high thresholds.
- Returning to the parking side should only enter `Charging` when battery is below the configured low threshold.
- `Charging -> start` should be allowed only when battery is above configured start threshold.

**Actual behavior in current code**

- `BMS` exposes:
  - `low_soc_`
  - `full_soc_`
  - `is_low_battery()`
  - `is_fully_charged()`
- `RobotSupervisor::tick_safety()` injects `EvLowBattery` during runtime when the task is active.
- `RobotFsm` sends low battery into `Returning`.
- `RobotFsm` enters `Charging` on:
  - task completion
  - return completion
  regardless of current battery level.
- `RobotFsm` leaves `Charging` only via `EvChargeDone`.
- No explicit charge-low, charge-high, and start-threshold triplet exists in runtime config.

**Deterministic defects**

- Low battery is handled as an in-task return, not a pre-start gate. This directly violates the confirmed target model.
- Charging entry is unconditional on task-complete and return-complete paths.
- Charging exit is modeled only as `EvChargeDone`, while the target model requires threshold-driven transition.
- Start-from-Charging currently only checks supervisor state and parking-side position; it does not check whether battery is high enough.

**Semantic / responsibility risks**

- Battery semantics are partly in `BMS`, partly in `RobotSupervisor`, and partly in `RobotFsm`, without one coherent rule set.
- Existing tests and telemetry likely still encode the old “low battery means returning” mental model.

**Involved modules**

- `pv_cleaning_robot/device/bms.cc`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/main.cc`

### 3.9 Cloud Status Projection and Command Events

**Intended behavior**

- Cloud-visible command/state behavior should reflect the simplified runtime contract.
- RPC response, command events, and telemetry should describe the same business model.

**Actual behavior in current code**

- `CloudService` parses RPC and shared attributes and publishes RPC responses.
- `ThingsBoardControlPlane` publishes:
  - startup attributes
  - status events
  - command events
  - business telemetry
- `RobotSupervisor::snapshot()` projects:
  - `PausedTask`
  - `ReturningTask`
  - `ChargingTask`
  - `FaultedTask`
  - `TerminatedTask`
- Control-plane comments and implementation still advertise RPC `start / stop / return / terminate / reset`.

**Deterministic defects**

- Cloud contract still exposes `terminate` and state projections for `Paused`/`Terminated`, all of which conflict with the desired simplified contract.
- `task_state` projection is still built around the old richer task model.

**Semantic / responsibility risks**

- Cloud status remains a multi-view projection:
  - RPC response
  - command event
  - periodic business telemetry
  - health telemetry
- Until task control is simplified, platform-side meaning will remain heavier than necessary.

**Involved modules**

- `pv_cleaning_robot/service/cloud_service.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`

### 3.10 Thread, Event, Cache, and Runtime Support Layers

**Intended behavior**

- Runtime support layers should stay simple and reliable.
- Threading and event boundaries should not hide business truth or reintroduce redundant behavior.

**Actual behavior in current code**

- `MqttTransport` now uses a dedicated delivery thread and asynchronous publish semantics.
- `NetworkManager` remains a thin router.
- `DataCache` only persists telemetry publish payloads.
- `SafetyMonitor`:
  - immediately stops motion on left/right trigger
  - debounces settled events
  - suppresses repeated same-side trigger while pending
- Main loop still performs:
  - scheduler tick
  - dynamic cloud report-period switching
  - low-battery runtime safety injection
- `WatchdogMgr` reports `P0` faults on thread timeout.

**Deterministic defects**

- The low-battery runtime injection in the main loop is now a business-model defect, not merely an implementation detail.
- `DataCache` boundaries are still telemetry-only. That may be acceptable, but command/state event durability is explicitly not part of the current support model.

**Semantic / responsibility risks**

- Thread/event model is still mixed:
  - direct method calls
  - EventBus dispatch
  - periodic `ThreadExecutor` loops
  - main-loop ticking
- `main.cc` still uses runtime support layers to carry business policy, especially around scheduler and safety.

**Involved modules**

- `pv_cleaning_robot/middleware/mqtt_transport.cc`
- `pv_cleaning_robot/middleware/network_manager.cc`
- `pv_cleaning_robot/middleware/data_cache.cc`
- `pv_cleaning_robot/middleware/safety_monitor.cc`
- `pv_cleaning_robot/service/cloud_service.cc`
- `pv_cleaning_robot/app/watchdog_mgr.cc`
- `pv_cleaning_robot/main.cc`

## 4. Must-Fix Deterministic Defects

1. Configuration ownership is not cleanly split into fixed-local, cloud-immediate, and cloud-next-task classes.
   - Runtime evidence:
     - `main.cc` directly reads many mutable runtime values from `ConfigService`
     - `ThingsBoardConfigManager` only partially models staged runtime config
   - Impact:
     - future fields can be applied at the wrong time or interpreted by multiple modules
   - Involved modules:
     - `ConfigService`
     - `ThingsBoardConfigManager`
     - `main.cc`

2. Invalid startup position states are only warnings.
   - Runtime evidence:
     - startup self-check logs warnings for off-parking-side or opposite-side trigger, but does not produce a first-class invalid-position state
   - Impact:
     - invalid physical states are not enforced as “do not start” runtime truth
   - Involved modules:
     - `main.cc`
     - `ParkingSideRuntime`

3. RPC `start` still resumes paused tasks.
   - Runtime evidence:
     - `ThingsBoardControlPlane::register_rpc_handlers()` checks `state == "Paused"` and calls `resume_paused_task()`
   - Impact:
     - observable behavior conflicts with the confirmed command contract
   - Involved modules:
     - `ThingsBoardControlPlane`
     - `RobotSupervisor`

4. RPC `stop` currently means pause, not terminate-and-halt.
   - Runtime evidence:
     - `stop` routes to `pause_task()`
   - Impact:
     - external control contract is wrong
   - Involved modules:
     - `ThingsBoardControlPlane`
     - `RobotSupervisor`
     - `RobotFsm`

5. RPC `reset` currently means FSM reset-to-idle, not device reboot.
   - Runtime evidence:
     - `reset` routes to `reset_task()`
     - actual reboot logic only exists in `OtaManager`
   - Impact:
     - external control contract is wrong and misleading
   - Involved modules:
     - `ThingsBoardControlPlane`
     - `RobotSupervisor`
     - `RobotFsm`
     - `OtaManager`

6. Low battery is handled as an in-task return path instead of a pre-start gate.
   - Runtime evidence:
     - `main` calls `supervisor->tick_safety(bms->is_low_battery())`
     - `tick_safety()` dispatches `EvLowBattery`
   - Impact:
     - current runtime behavior directly conflicts with the target task model
   - Involved modules:
     - `main.cc`
     - `RobotSupervisor`
     - `RobotFsm`
     - `BMS`

7. `Charging` entry is unconditional after task completion or return completion.
   - Runtime evidence:
     - `EvParkingSideLimitSettled` always ends in `Charging` when passes complete or `Returning` completes
   - Impact:
     - state machine cannot represent “returned but battery high enough, stay Idle”
   - Involved modules:
     - `RobotFsm`

8. Start admission does not include battery-threshold gating.
   - Runtime evidence:
     - `RobotSupervisor::start_task()` checks state, position, and pending promotion only
   - Impact:
     - robot can still enter task start path under battery conditions that should be denied
   - Involved modules:
     - `RobotSupervisor`
     - `BMS`

## 5. High-Priority Semantic / Responsibility Risks

1. `main.cc` still contains too much runtime business knowledge.
   - Why it is risky:
     - startup self-check policy, scheduler admission, limit-event semantic bridging, and report-period policy all live in `main`
   - How it creates future defects:
     - later simplifications will continue to duplicate or bypass central business rules
   - Involved modules:
     - `main.cc`

2. FSM state model is richer than the target model.
   - Why it is risky:
     - `Paused`, `Terminated`, `Returning`, `CleanFwd`, `CleanReturn`, `LowBattery`, and differentiated fault branches preserve old semantics
   - How it creates future defects:
     - every upper layer must keep interpreting legacy branches the target product no longer wants
   - Involved modules:
     - `RobotFsm`
     - `RobotSupervisor`
     - `ThingsBoardControlPlane`

3. Fault behavior is spread across service, handler, and FSM layers.
   - Why it is risky:
     - safe-halt semantics are not yet clearly separated from historical task-branch semantics
   - How it creates future defects:
     - simplifying one layer without the others can break safety or preserve unnecessary branches
   - Involved modules:
     - `FaultService`
     - `FaultHandler`
     - `RobotFsm`

4. Cloud status remains a projection of the old control model.
   - Why it is risky:
     - telemetry, task_state, command events, and RPC handlers still encode pause/terminate/legacy reset assumptions
   - How it creates future defects:
     - platform behavior and runtime code can drift again even after simplification
   - Involved modules:
     - `CloudService`
     - `ThingsBoardControlPlane`
     - `RobotSupervisor`

5. Battery and charging semantics are split across BMS, main loop, supervisor, and FSM.
   - Why it is risky:
     - no single module currently owns the threshold-driven charging/start-gate truth
   - How it creates future defects:
     - charging and battery-gate behavior will be hard to simplify without reintroducing contradictions
   - Involved modules:
     - `BMS`
     - `main.cc`
     - `RobotSupervisor`
     - `RobotFsm`

## 6. Test Divergence and Sync Targets

### Startup / Position / Start Admission

- `test/integration/system_integration_test.cc`
  - mirrors main-program startup wiring and position interpretation
- `test/integration/task_chain_test.cc`
  - exercises task-start path through FSM and SafetyMonitor bridge
- `test/service/thingsboard_control_plane_test.cc`
  - currently still validates the richer RPC model and must follow the reduced contract
- `test/app/robot_fsm_test.cc`
  - currently covers self-check plus legacy pause/low-battery/terminate branches
- `test/app/robot_supervisor_test.cc`
  - must be updated once start admission includes battery gate and removes resume semantics

### Main Task / Branch Flows

- `test/app/robot_fsm_test.cc`
  - canonical unit coverage for branches that are now simplification candidates
- `test/app/robot_supervisor_test.cc`
  - admission, stop/return, and reset behavior
- `test/integration/task_chain_test.cc`
  - multi-component task-chain behavior through current FSM names and transitions
- `test/integration/hardware/clean_cycle_hw_test.cc`
  - contains legacy low-battery return expectations
- `test/integration/hardware/system_hw_test.cc`
  - contains real-device RPC terminate/reset expectations that conflict with the new target model

### Cloud / Runtime Support Layers

- `test/service/cloud_service_test.cc`
  - shared attributes, RPC parsing, and explicit reject behavior
- `test/service/thingsboard_control_plane_test.cc`
  - command contract and telemetry/event expectations; high-priority sync target
- `test/middleware/mqtt_transport_test.cc`
  - transport assumptions only, not target business truth
- `test/middleware/network_manager_test.cc`
  - transport routing assumptions only
- `test/integration/thingsboard_runtime_mock_integration_test.cc`
  - currently validates runtime RPC flow including `terminate/reset`; will need contract rewrite

### Battery / Charging

- `test/integration/hardware/bms_hw_test.cc`
  - validates raw BMS facts but not the target threshold-driven task/charging policy
- `test/integration/hardware/clean_cycle_hw_test.cc`
  - uses current `EvLowBattery -> Returning -> Charging` expectations
- tests reading `robot.battery_full_soc` / `robot.battery_low_soc`
  - must be revisited if the target model introduces separate start-threshold and charge-low/high thresholds

## 7. Simplification Inputs for the Next Phase

1. Centralize configuration ownership.
   - Split config into:
     - fixed local wiring/runtime constants
     - cloud-immediate runtime policy
     - cloud-next-task runtime policy
   - Remove ambiguous field handling between `main.cc` and `ThingsBoardConfigManager`.

2. Thin `main.cc`.
   - Move startup-position policy, start-precheck composition, and cloud-facing callback decisions out of `main`.
   - Keep `main` as composition root plus the thinnest possible bridge.

3. Collapse task admission into one authority.
   - Scheduler and RPC must both go through the same start-precheck path:
     - valid position
     - battery gate
     - pending promotion
     - state gate

4. Reduce the control contract.
   - Keep only:
     - `start`
     - `stop`
     - `return`
     - `reset` (device reboot)
   - Remove pause/resume/terminate semantics from the main business path.

5. Rename or replace direction-bound task states.
   - `CleanFwd / CleanReturn` should likely become parking-side semantics or a smaller state vocabulary aligned with the final model.

6. Replace low-battery-return with pre-start battery gating.
   - Battery logic should stop tasks from starting, not inject a separate mid-task branch unless safety strictly requires it.

7. Make charging threshold-driven.
   - Introduce one coherent policy for:
     - charge-low threshold
     - charge-high threshold
     - start-allowed threshold
   - Keep charging behavior tied to parking-side state.

8. Rewrite tests after runtime truth is simplified.
   - Current tests still preserve the richer historical control model.
   - Delete or rewrite them after the runtime contract is reduced, not before.
