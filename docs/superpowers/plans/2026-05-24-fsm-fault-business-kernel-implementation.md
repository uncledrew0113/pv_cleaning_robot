# FSM / Fault / Business Kernel Implementation Plan

Date: 2026-05-24
Status: Active

## Goal

Complete the remaining mission/FSM/fault refactor while preserving the already-applied external contract compaction.

This plan follows the approved constraints:

- simple, stable, reliable first
- no new code files
- prefer reducing structural noise over adding abstraction
- preserve SOLID
- do not change existing `HEALTH` and `DIAGNOSTICS` schemas in this phase

## Phase Status

### Phase 0: External Contract Compaction

Already completed in code:

- runtime config compacted to:
  - `passes`
  - `clean_speed_rpm`
  - `return_speed_rpm`
  - `brush_rpm`
  - `parking_side`
  - `min_battery_soc`
  - `charge_stop_soc`
  - `schedules`
- periodic business telemetry compacted to:
  - `state`
  - `fault`
  - `cfg_ver`
- status events compacted to:
  - `event`
  - `code`
- RPC replies compacted to:
  - `code`
- legacy outward fields removed:
  - `device_state`
  - `task_state`
  - `active_config_version`
  - `active_command`
  - `last_command`
  - `return_brush_rpm`
  - `start_battery_soc`
  - `charge_start_soc`
- command lifecycle remains local only
- heading UDS contract reduced to:
  - `valid`
  - `yaw_deg`
  - `confidence`

This compact outward contract is now baseline and must not be expanded again without a new design decision.

## Remaining Work

### Task 1: Replace Pass-Oriented Runtime Truth With Mission / Segment Truth

Files:

- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`

Goals:

- keep `passes` only as compatibility input
- introduce mission and segment runtime truth inside existing FSM/supervisor files
- stop using pass arithmetic as primary execution truth

Done criteria:

- internal execution can represent:
  - single-dock round trip
  - dual-dock dock-to-dock
  - single-leg test
- `passes` no longer drives low-level runtime transitions directly

### Task 2: Reduce FSM State Vocabulary

Files:

- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `test/app/robot_fsm_test.cc`
- `test/app/robot_supervisor_test.cc`
- `test/integration/hardware/system_hw_test.cc`

Goals:

- replace direction-coupled runtime truth with compact state set:
  - `Idle`
  - `SelfCheck`
  - `ExecutingSegment`
  - `SegmentBoundary`
  - `FaultStopped`
  - `Charging`
- remove direct dependence on `CleanFwd`, `CleanReturn`, and `Returning` as core business states

Done criteria:

- state machine is smaller
- transition logic is easier to read
- tests verify segment-oriented behavior instead of old direction-oriented state names

### Task 3: Rework P0 / P1 / P2 Handling Around Capability-Based Decisions

Files:

- `include/pv_cleaning_robot/app/fault_handler.h`
- `pv_cleaning_robot/app/fault_handler.cc`
- `include/pv_cleaning_robot/service/fault_service.h`
- `pv_cleaning_robot/service/fault_service.cc`
- `test/app/fault_handler_test.cc`

Goals:

- keep `P0` as immediate hard stop
- make `P1` decide between:
  - returnable fault-disposal segment
  - non-returnable stop
- keep `P2` as report-only
- avoid degraded-running business mode

Done criteria:

- business fault decision is explicit
- `P1` behavior depends on remaining mobility and context, not level alone
- fault code visibility remains compact through `fault`

### Task 4: Move Segment-to-Motor Translation Into Existing Motion Layer

Files:

- `include/pv_cleaning_robot/service/motion_service.h`
- `pv_cleaning_robot/service/motion_service.cc`
- `test/service/motion_service_test.cc`

Goals:

- keep segment spec business-level only
- resolve:
  - wheel direction
  - brush direction
  - target speed profile
  inside existing motion code

Done criteria:

- FSM does not carry motor-level fields
- dock/segment differences are resolved in motion logic, not FSM state naming

### Task 5: Align Tests And Observability With The New Kernel Truth

Files:

- `test/service/business_payload_builder_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/service/cloud_service_test.cc`
- `test/integration/system_integration_test.cc`
- `test/integration/task_chain_test.cc`
- `test/integration/thingsboard_real_integration_test.cc`

Goals:

- keep the already compact field contracts stable
- make tests assert:
  - `state`
  - `fault`
  - `cfg_ver`
  - `event`
  - `code`
- remove assertions that depend on deleted outward fields

Done criteria:

- no test still depends on removed external field names
- cloud-side tests match the compact contract exactly

## Explicit Non-Goals

This plan does not include:

- introducing `BusinessActionCode` into the outward contract
- changing `HEALTH` or `DIAGNOSTICS` schemas
- adding new helper modules such as `MissionBuilder`, `ProfileResolver`, or `FaultArbiter` files
- broad refactors unrelated to mission/FSM/fault simplification

## Verification

Required verification after each task:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Preferred runtime verification where host dependencies allow:

```bash
/usr/bin/qemu-aarch64-static -L /usr/aarch64-linux-gnu ./build/aarch64/bin/unit_tests
```

If qemu execution remains blocked by missing target shared libraries, report that explicitly and do not claim runtime test success.

## Completion Criteria

This plan is complete only when all of the following are true:

- external compact contracts remain unchanged and fully covered by tests
- FSM runtime truth is mission/segment-oriented
- old direction-driven core states are no longer the kernel truth
- P0/P1/P2 behavior matches the approved business model
- code remains simple, stable, reliable, and no new files were added
