# Hard Cut Domain/App Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove old pass/RPC-specialized compatibility semantics, delete redundant legacy tests, and add focused professional comments for the active `domain/app/service/middleware` boundaries.

**Architecture:** Keep `hal/driver/protocol/device` unchanged. Make `domain::MissionContext`, `app::RobotFsm::handle(...)`, `RobotSupervisor::submit_command(...)`, `MotionService::start_segment(...)`, and `ThingsBoardControlPlane` command mapping the only active business path.

**Tech Stack:** C++17, Catch2, CMake preset `rk3576-build`.

---

## Task 1: Remove Legacy Domain And FSM Surface

**Files:**
- Modify `include/pv_cleaning_robot/domain/robot_domain.h`
- Modify `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify `pv_cleaning_robot/app/robot_fsm.cc`

- [ ] Remove `passes`, `MissionType`, `SegmentDirection`, `CompletionCondition`, `RobotControlPort`, and segment compatibility constructors/aliases.
- [ ] Remove Boost.SML states/events and `dispatch<>`-based FSM implementation.
- [ ] Keep only explicit `RobotFsm::handle(...)` events/actions and state names from the approved FSM model.
- [ ] Verify compile errors point to callers that still use old APIs.

## Task 2: Remove Legacy Supervisor And Main Callers

**Files:**
- Modify `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify `pv_cleaning_robot/main.cc`
- Modify `test/integration/hardware/hw_config.h`

- [ ] Remove `start_task`, `start_task_from_current_position`, `stop_task`, `return_task`, and `make_control_port`.
- [ ] Rewire scheduler hits to submit `StartConfiguredMission`.
- [ ] Rewire limit-settled bridge to use `ExpectedEndpointSettled` and dispatch returned actions.
- [ ] Keep startup position checks and safety tick behavior.

## Task 3: Remove Redundant Legacy Tests

**Files:**
- Modify `test/CMakeLists.txt`
- Delete or rewrite legacy-heavy tests under `test/app`, `test/integration`, and stale service tests.

- [ ] Replace `robot_fsm_test.cc` with compact tests for new FSM command/action behavior.
- [ ] Replace `robot_supervisor_test.cc` with compact tests for four commands, self-check, and action dispatch.
- [ ] Remove `integration/task_chain_test.cc` and `integration/system_integration_test.cc` from `unit_tests` because they duplicate old event-path coverage.
- [ ] Remove stale `passes` assertions; keep only `repeat_count` behavior tests.

## Task 4: Tighten Service APIs And Comments

**Files:**
- Modify `include/pv_cleaning_robot/service/motion_service.h`
- Modify `pv_cleaning_robot/service/motion_service.cc`
- Modify `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify `include/pv_cleaning_robot/service/fault_service.h`
- Modify `include/pv_cleaning_robot/middleware/safety_monitor.h`
- Modify `include/pv_cleaning_robot/middleware/data_cache.h`
- Modify `include/pv_cleaning_robot/middleware/event_bus.h`

- [ ] Make old motion helpers private implementation details where feasible.
- [ ] Add comments only at business/safety/concurrency boundaries.
- [ ] Remove comments that describe old RPC, pass, or SML behavior.

## Task 5: Final Verification

- [ ] Run `cmake --build --preset rk3576-build --target unit_tests`.
- [ ] Run `git diff --check`.
- [ ] Report remaining risks, especially any hardware-only tests not built in this target.
