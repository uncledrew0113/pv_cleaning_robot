# Unattended Robot Architecture Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重构 app/middleware/service/main/test，使无人值守干挂式光伏清扫机器人业务保持完整，同时依赖清晰、职责单一、稳定可靠。

**Architecture:** 采用阶段化方案 C：先抽 domain/ports，再收窄 app 编排，再清理 service/middleware 反向依赖，最后瘦 main/test。每阶段行为可验证，不做大爆炸式重写。

**Tech Stack:** C++17, Boost.SML, Catch2, CMake preset `rk3576-build`, aarch64 target.

---

## File Strategy

不增加代码文件数量。需要承载 domain/ports 时，优先改名或复用现有文件：

- Modify/Rename: `include/pv_cleaning_robot/app/parking_side_runtime.h` -> domain 承载文件。
- Modify/Rename: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h` -> app ports/snapshot 承载文件。
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/app/robot_fsm.cc`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`, `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `include/pv_cleaning_robot/app/fault_handler.h`, `pv_cleaning_robot/app/fault_handler.cc`
- Modify: `include/pv_cleaning_robot/service/motion_service.h`, `pv_cleaning_robot/service/motion_service.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `include/pv_cleaning_robot/middleware/safety_monitor.h`, `pv_cleaning_robot/middleware/safety_monitor.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify tests under `test/app`, `test/service`, `test/middleware`, `test/integration`

## Task 1: Domain/Ports Extraction

**Goal:** 将业务类型和端口集中，先不改变行为。

- [ ] Move `ParkingSideFacts`, mission/segment enums, `FaultCode`, and runtime snapshot into domain/port headers using existing files.
- [ ] Update includes so `service` no longer needs app concrete classes.
- [ ] Keep adapter aliases if needed to avoid large call-site churn in the same step.
- [ ] Verify:
  - `cmake --build --preset rk3576-build --target unit_tests`
  - `cmake --build --preset rk3576-build --target hw_tests`
  - `git diff --check`

## Task 2: FSM Port Boundary

**Goal:** `RobotFsm` 只依赖 `MotionPort` 和 `FaultReporter`，不依赖完整 `MotionService/NavService`。

- [ ] Replace `std::shared_ptr<MotionService>` in `RobotFsm` with `std::shared_ptr<MotionPort>`.
- [ ] Remove unused `NavService` dependency from `RobotFsm` constructor.
- [ ] Keep FSM behavior unchanged: segment start, completion, P0/P1/P2, fault reset.
- [ ] Update app tests to use a minimal fake motion port instead of full service fixture where possible.
- [ ] Verify build targets and diff check.

## Task 3: Fault Handling Boundary

**Goal:** `FaultHandler` 不依赖完整 `MotionService`，只依赖 emergency/brush-safe motion port and dispatch callback。

- [ ] Replace concrete motion dependency with a narrow port.
- [ ] Keep P0 immediate stop, P1 no-brush return, P2 promotion.
- [ ] Ensure cloud publishing is not in EventBus synchronous fault path.
- [ ] Verify `fault_handler_test`, `robot_fsm_test`, integration fault chain.

## Task 4: SafetyMonitor Middleware Boundary

**Goal:** `SafetyMonitor` 不依赖 `WalkMotorGroup`，只依赖 `EmergencyStopPort`。

- [ ] Change constructor to accept an emergency stop port/callback.
- [ ] Keep immediate emergency override timing path.
- [ ] Keep stable limit event and release re-arm behavior.
- [ ] Update middleware tests to use fake emergency port.
- [ ] Verify safety monitor tests and build targets.

## Task 5: ThingsBoard Control Boundary

**Goal:** `ThingsBoardControlPlane` 只处理协议和 payload。

- [ ] Keep `RobotControlPort`, remove any remaining app concrete forward dependency.
- [ ] Move app-specific snapshot type out of `app` dependency path or alias through port header.
- [ ] Keep RPC response payload compact: reply only `code`, telemetry remains `state/fault/cfg_ver`.
- [ ] Verify ThingsBoard tests.

## Task 6: Supervisor Business Use Cases

**Goal:** `RobotSupervisor` 成为唯一业务用例入口。

- [ ] Centralize start/stop/return/scheduler/startup/limit/fault decisions.
- [ ] Keep single-dock boot at far end return behavior.
- [ ] Keep single-dock RPC far-end behavior: `SingleLegClean(ToParkingSide)`.
- [ ] Keep dual-dock任一端启动单段清扫。
- [ ] Keep wrong-end, conflicting-limits, unexpected-limit as P0.
- [ ] Verify app and integration tests.

## Task 7: Main as Composition Root

**Goal:** `main.cc` 不承载业务策略。

- [ ] Extract local lambdas only for hardware readings and app port binding.
- [ ] Move startup position status mapping into app/service helper if already represented by existing file; otherwise keep minimal call.
- [ ] Keep thread topology and RT priorities unchanged.
- [ ] Keep graceful shutdown order unchanged.
- [ ] Verify build targets.

## Task 8: Test Architecture Cleanup

**Goal:** 测试按边界组织，fixture 不复制生产业务。

- [ ] App tests use fake ports and domain types.
- [ ] Service tests verify protocol/payload/config only.
- [ ] Middleware tests verify infrastructure behavior only.
- [ ] Integration tests keep cross-layer business chains.
- [ ] Verify build targets; record qemu runtime limitation if encountered.

