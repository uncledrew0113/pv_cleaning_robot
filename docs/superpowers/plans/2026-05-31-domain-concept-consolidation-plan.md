# Domain Concept Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Simplify the domain endpoint model by replacing scattered left/right, pose, lane, and parking-fact types with a compact endpoint-state model.

**Architecture:** Keep FSM behavior, RPC command semantics, and MotionService segment execution unchanged. Only change domain vocabulary and update callers to use `EndpointSide`, `EndpointState`, `LimitState`, and direct helper functions.

**Tech Stack:** C++17, Catch2, CMake preset `rk3576-build`.

---

## Task 1: Consolidate Domain Types

**Files:**
- Modify `include/pv_cleaning_robot/domain/robot_domain.h`
- Modify `test/domain/robot_domain_test.cc`

- [ ] Replace `PhysicalLimitSide`, `EndpointId`, `EndpointPoseKind`, `EndpointPose`, `LaneGeometry`, `ParkingSideFacts`, and `ParkingSideRuntime`.
- [ ] Add `EndpointSide`, `using ParkingSide = EndpointSide`, `LimitState`, `EndpointState`, and helper functions for endpoint estimation and mission-start checks.
- [ ] Keep `SegmentTarget`, `SegmentMode`, `MissionContext`, and `RobotCommandKind`.
- [ ] Update domain tests to assert the new endpoint-state API.

## Task 2: Update App And Service Callers

**Files:**
- Modify `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify `include/pv_cleaning_robot/service/motion_service.h`
- Modify `pv_cleaning_robot/service/motion_service.cc`
- Modify `include/pv_cleaning_robot/middleware/safety_monitor.h`
- Modify `pv_cleaning_robot/middleware/safety_monitor.cc`
- Modify `pv_cleaning_robot/main.cc`

- [ ] Replace endpoint-pose query with endpoint-state query.
- [ ] Replace lane-geometry query with parking-side query where motion only needs direction sign.
- [ ] Replace limit-settled events with `EndpointSide`.
- [ ] Preserve RPC command behavior and configured mission start constraints.

## Task 3: Update Tests

**Files:**
- Modify `test/app/robot_fsm_test.cc`
- Modify `test/app/robot_supervisor_test.cc`
- Modify `test/service/motion_service_test.cc`
- Modify `test/middleware/safety_monitor_test.cc`

- [ ] Replace old endpoint and limit types in tests.
- [ ] Keep assertions focused on behavior, not removed compatibility names.

## Task 4: Verify

- [ ] Run old-symbol scan for removed domain concepts.
- [ ] Run `cmake --build --preset rk3576-build --target unit_tests`.
- [ ] Run `cmake --build --preset rk3576-build --target hw_tests`.
- [ ] Run `git diff --check`.
