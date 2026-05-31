# Minimal Domain/FSM Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current pass/RPC-specialized app model with a minimal domain model, command-source-agnostic FSM, and RPC command flow that supports configured missions and two directional cleaning commands.

**Architecture:** Keep `hal/driver/protocol/device` unchanged. Move business truth into compact `domain` types and pure helpers, make the app FSM execute `MissionContext` plus actions, keep RPC as protocol-to-command adaptation, and leave multi-stage recovery inside `service::RecoveryMotion`.

**Tech Stack:** C++17, Catch2, CMake preset `rk3576-build`, Boost currently present but the new FSM should prefer an explicit switch-based transition function unless preserving Boost.SML is proven simpler during implementation.

---

## File Map

- Modify `include/pv_cleaning_robot/domain/robot_domain.h`: compact domain enums, structs, and pure helper functions.
- Modify `test/CMakeLists.txt`: add a new domain test file to `unit_tests`.
- Create `test/domain/robot_domain_test.cc`: pure domain behavior tests.
- Modify `include/pv_cleaning_robot/app/robot_fsm.h`: define command/event/action types and FSM API.
- Modify `pv_cleaning_robot/app/robot_fsm.cc`: implement command-source-agnostic state transitions.
- Modify `test/app/robot_fsm_test.cc`: replace pass/RPC-specific tests with mission-context tests.
- Modify `include/pv_cleaning_robot/app/robot_supervisor.h`: expose `submit_command()` and command result status.
- Modify `pv_cleaning_robot/app/robot_supervisor.cc`: build mission contexts, validate starts, dispatch FSM actions.
- Modify `test/app/robot_supervisor_test.cc`: cover the four required RPC-equivalent commands through app API.
- Modify `include/pv_cleaning_robot/service/motion_service.h`: add a segment-based motion entry point.
- Modify `pv_cleaning_robot/service/motion_service.cc`: map `MissionSegment` target/mode to wheel and brush commands.
- Modify `test/service/motion_service_test.cc`: verify target/mode to motor command mapping.
- Create `include/pv_cleaning_robot/service/recovery_motion.h`: small multi-stage recovery controller interface.
- Create `pv_cleaning_robot/service/recovery_motion.cc`: implement recovery phase progression.
- Modify `test/CMakeLists.txt`: add `service/recovery_motion_test.cc` and `recovery_motion.cc`.
- Create `test/service/recovery_motion_test.cc`: cover stop, sample, micro-move, verify, done, failed.
- Modify `include/pv_cleaning_robot/service/thingsboard_control_plane.h`: depend on app command port instead of `domain::RobotControlPort`.
- Modify `pv_cleaning_robot/service/thingsboard_control_plane.cc`: map RPC methods to `RobotCommand`.
- Modify `test/service/thingsboard_control_plane_test.cc`: verify accepted/rejected and `command_id` close-loop.
- Modify `pv_cleaning_robot/main.cc`: wire the new command port and recovery service, keep device update threads unchanged.

## Task 1: Domain Model And Pure Rules

**Files:**
- Modify: `include/pv_cleaning_robot/domain/robot_domain.h`
- Create: `test/domain/robot_domain_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Add the failing domain tests**

Add `test/domain/robot_domain_test.cc`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include <optional>

#include "pv_cleaning_robot/domain/robot_domain.h"

using namespace robot::domain;

TEST_CASE("Domain endpoint role is derived from lane geometry", "[domain]") {
    LaneGeometry lane{DockMode::SingleDock, EndpointId::Left};

    CHECK(return_endpoint(lane) == EndpointId::Right);
    CHECK(is_parking_endpoint(lane, EndpointId::Left));
    CHECK_FALSE(is_parking_endpoint(lane, EndpointId::Right));
    CHECK(is_return_endpoint(lane, EndpointId::Right));
}

TEST_CASE("Domain trusted pose rejects unknown and inconsistent", "[domain]") {
    CHECK(is_trusted_pose(EndpointPose{EndpointPoseKind::AtEndpoint, EndpointId::Left}));
    CHECK(is_trusted_pose(EndpointPose{EndpointPoseKind::OnSegment, std::nullopt}));
    CHECK_FALSE(is_trusted_pose(EndpointPose{EndpointPoseKind::Unknown, std::nullopt}));
    CHECK_FALSE(is_trusted_pose(EndpointPose{EndpointPoseKind::Inconsistent, std::nullopt}));
}

TEST_CASE("Domain directional missions build single cleaning segment", "[domain]") {
    const auto to_return = build_directional_clean_context(
        MissionKind::CleanTowardReturnEndpoint, CommandSource::Rpc, "rpc-1");
    REQUIRE(to_return.segments.size() == 1);
    CHECK(to_return.kind == MissionKind::CleanTowardReturnEndpoint);
    CHECK(to_return.source == CommandSource::Rpc);
    CHECK(to_return.command_id == "rpc-1");
    CHECK(to_return.segments[0].target == SegmentTarget::ReturnEndpoint);
    CHECK(to_return.segments[0].mode == SegmentMode::Cleaning);

    const auto to_parking = build_directional_clean_context(
        MissionKind::CleanTowardParkingEndpoint, CommandSource::Rpc, "rpc-2");
    REQUIRE(to_parking.segments.size() == 1);
    CHECK(to_parking.segments[0].target == SegmentTarget::ParkingEndpoint);
    CHECK(to_parking.segments[0].mode == SegmentMode::Cleaning);
}

TEST_CASE("Domain configured mission builds single dock round trip", "[domain]") {
    LaneGeometry lane{DockMode::SingleDock, EndpointId::Left};
    EndpointPose pose{EndpointPoseKind::AtEndpoint, EndpointId::Left};

    const auto ctx = build_configured_mission_context(
        lane, pose, CommandSource::Scheduler, "schedule-1", 2);

    REQUIRE(ctx.segments.size() == 2);
    CHECK(ctx.kind == MissionKind::ConfiguredMission);
    CHECK(ctx.repeat_count == 2);
    CHECK(ctx.completed_cycles == 0);
    CHECK(ctx.segments[0].target == SegmentTarget::ReturnEndpoint);
    CHECK(ctx.segments[0].mode == SegmentMode::Cleaning);
    CHECK(ctx.segments[1].target == SegmentTarget::ParkingEndpoint);
    CHECK(ctx.segments[1].mode == SegmentMode::Cleaning);
}

TEST_CASE("Domain configured mission builds dual dock transfer", "[domain]") {
    LaneGeometry lane{DockMode::DualDock, EndpointId::Left};
    EndpointPose pose{EndpointPoseKind::AtEndpoint, EndpointId::Right};

    const auto ctx = build_configured_mission_context(
        lane, pose, CommandSource::Rpc, "rpc-3", 1);

    REQUIRE(ctx.segments.size() == 1);
    CHECK(ctx.segments[0].target == SegmentTarget::ParkingEndpoint);
    CHECK(ctx.segments[0].mode == SegmentMode::Cleaning);
}
```

- [ ] **Step 2: Register the domain test**

Modify `test/CMakeLists.txt` and add the file under `add_executable(unit_tests ...)` before the app tests:

```cmake
  # Domain 层
  domain/robot_domain_test.cc
```

- [ ] **Step 3: Run the focused build and verify the tests fail to compile**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `return_endpoint`, `is_trusted_pose`, `build_directional_clean_context`, and `build_configured_mission_context` are not defined yet.

- [ ] **Step 4: Implement the minimal domain types and helpers**

In `include/pv_cleaning_robot/domain/robot_domain.h`, replace the pass/RPC-specialized mission types with this compact model. Keep existing config and fault fields that other code still needs during migration, but mark `passes` compatibility as temporary in comments.

```cpp
enum class DockMode {
    SingleDock,
    DualDock,
};

enum class CommandSource {
    Rpc,
    Scheduler,
    Local,
    FaultPolicy,
    System,
};

enum class EndpointId {
    Left,
    Right,
};

enum class EndpointPoseKind {
    Unknown,
    OnSegment,
    AtEndpoint,
    Inconsistent,
};

struct EndpointPose {
    EndpointPoseKind kind{EndpointPoseKind::Unknown};
    std::optional<EndpointId> endpoint{};
};

struct LaneGeometry {
    DockMode dock_mode{DockMode::SingleDock};
    EndpointId parking_endpoint{EndpointId::Left};
};

enum class MissionKind {
    ConfiguredMission,
    CleanTowardReturnEndpoint,
    CleanTowardParkingEndpoint,
    BrushOffReturnHome,
};

enum class SegmentTarget {
    ParkingEndpoint,
    ReturnEndpoint,
};

enum class SegmentMode {
    Cleaning,
    BrushOffReturn,
};

struct MissionSegment {
    SegmentTarget target{SegmentTarget::ReturnEndpoint};
    SegmentMode mode{SegmentMode::Cleaning};
};

struct MissionContext {
    MissionKind kind{MissionKind::ConfiguredMission};
    CommandSource source{CommandSource::System};
    std::string command_id{};
    std::vector<MissionSegment> segments{};
    std::size_t current_segment_index{0};
    uint32_t repeat_count{1};
    uint32_t completed_cycles{0};
};

inline EndpointId opposite_endpoint(EndpointId endpoint) noexcept {
    return endpoint == EndpointId::Left ? EndpointId::Right : EndpointId::Left;
}

inline EndpointId return_endpoint(const LaneGeometry& lane) noexcept {
    return opposite_endpoint(lane.parking_endpoint);
}

inline bool is_parking_endpoint(const LaneGeometry& lane, EndpointId endpoint) noexcept {
    return lane.parking_endpoint == endpoint;
}

inline bool is_return_endpoint(const LaneGeometry& lane, EndpointId endpoint) noexcept {
    return return_endpoint(lane) == endpoint;
}

inline bool is_trusted_pose(const EndpointPose& pose) noexcept {
    return pose.kind == EndpointPoseKind::AtEndpoint || pose.kind == EndpointPoseKind::OnSegment;
}

inline bool is_at_target(const LaneGeometry& lane,
                         const EndpointPose& pose,
                         SegmentTarget target) noexcept {
    if (pose.kind != EndpointPoseKind::AtEndpoint || !pose.endpoint.has_value()) {
        return false;
    }
    return target == SegmentTarget::ParkingEndpoint
               ? is_parking_endpoint(lane, *pose.endpoint)
               : is_return_endpoint(lane, *pose.endpoint);
}

inline MissionContext build_directional_clean_context(MissionKind kind,
                                                       CommandSource source,
                                                       std::string command_id) {
    MissionContext ctx;
    ctx.kind = kind;
    ctx.source = source;
    ctx.command_id = std::move(command_id);
    ctx.repeat_count = 1;
    ctx.completed_cycles = 0;
    ctx.segments.push_back(MissionSegment{
        kind == MissionKind::CleanTowardParkingEndpoint ? SegmentTarget::ParkingEndpoint
                                                        : SegmentTarget::ReturnEndpoint,
        SegmentMode::Cleaning});
    return ctx;
}

inline MissionContext build_brush_off_return_context(CommandSource source,
                                                      std::string command_id) {
    MissionContext ctx;
    ctx.kind = MissionKind::BrushOffReturnHome;
    ctx.source = source;
    ctx.command_id = std::move(command_id);
    ctx.segments.push_back(MissionSegment{SegmentTarget::ParkingEndpoint,
                                          SegmentMode::BrushOffReturn});
    return ctx;
}

inline MissionContext build_configured_mission_context(const LaneGeometry& lane,
                                                       const EndpointPose& start_pose,
                                                       CommandSource source,
                                                       std::string command_id,
                                                       uint32_t repeat_count) {
    MissionContext ctx;
    ctx.kind = MissionKind::ConfiguredMission;
    ctx.source = source;
    ctx.command_id = std::move(command_id);
    ctx.repeat_count = repeat_count == 0 ? 1 : repeat_count;
    ctx.completed_cycles = 0;

    if (lane.dock_mode == DockMode::DualDock && start_pose.kind == EndpointPoseKind::AtEndpoint &&
        start_pose.endpoint.has_value() && *start_pose.endpoint != lane.parking_endpoint) {
        ctx.segments.push_back(MissionSegment{SegmentTarget::ParkingEndpoint,
                                              SegmentMode::Cleaning});
        return ctx;
    }

    if (lane.dock_mode == DockMode::DualDock) {
        ctx.segments.push_back(MissionSegment{SegmentTarget::ReturnEndpoint,
                                              SegmentMode::Cleaning});
        return ctx;
    }

    ctx.segments.push_back(MissionSegment{SegmentTarget::ReturnEndpoint, SegmentMode::Cleaning});
    ctx.segments.push_back(MissionSegment{SegmentTarget::ParkingEndpoint, SegmentMode::Cleaning});
    return ctx;
}
```

- [ ] **Step 5: Run the domain test build**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build reaches app/service compile failures caused by old types, or passes domain compilation if compatibility aliases remain. Do not change device/driver/protocol code.

- [ ] **Step 6: Commit the domain slice**

```bash
git add include/pv_cleaning_robot/domain/robot_domain.h test/domain/robot_domain_test.cc test/CMakeLists.txt
git commit -m "refactor: define minimal robot domain model"
```

## Task 2: Explicit FSM Events, Actions, And State Transitions

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Modify: `test/app/robot_fsm_test.cc`

- [ ] **Step 1: Replace pass/RPC-specific FSM tests with command-source-agnostic tests**

In `test/app/robot_fsm_test.cc`, add tests that exercise FSM without concrete motion services:

```cpp
TEST_CASE("FSM starts directional clean toward return endpoint", "[app][fsm]") {
    RobotFsm fsm;
    auto result = fsm.handle(StartRequested{
        domain::build_directional_clean_context(domain::MissionKind::CleanTowardReturnEndpoint,
                                                domain::CommandSource::Rpc,
                                                "rpc-clean-return")});

    REQUIRE(result.accepted);
    CHECK(fsm.current_state() == RobotState::SelfChecking);

    result = fsm.handle(SelfCheckPassed{});
    REQUIRE(result.accepted);
    CHECK(fsm.current_state() == RobotState::ExecutingMission);
    REQUIRE(result.actions.size() == 1);
    CHECK(result.actions[0].kind == RobotActionKind::StartSegmentMotion);
}

TEST_CASE("FSM completes single segment directional clean at endpoint", "[app][fsm]") {
    RobotFsm fsm;
    fsm.handle(StartRequested{
        domain::build_directional_clean_context(domain::MissionKind::CleanTowardParkingEndpoint,
                                                domain::CommandSource::Rpc,
                                                "rpc-clean-parking")});
    fsm.handle(SelfCheckPassed{});

    auto result = fsm.handle(ExpectedEndpointSettled{domain::SegmentTarget::ParkingEndpoint});

    REQUIRE(result.accepted);
    CHECK(fsm.current_state() == RobotState::Idle);
    REQUIRE(result.actions.size() >= 1);
    CHECK(result.has_action(RobotActionKind::StopMotion));
    CHECK(result.has_action(RobotActionKind::PublishCommandCompleted));
}

TEST_CASE("FSM stop cancels active mission and returns idle", "[app][fsm]") {
    RobotFsm fsm;
    fsm.handle(StartRequested{
        domain::build_directional_clean_context(domain::MissionKind::CleanTowardReturnEndpoint,
                                                domain::CommandSource::Rpc,
                                                "rpc-stop")});
    fsm.handle(SelfCheckPassed{});

    auto result = fsm.handle(StopRequested{domain::CommandSource::Rpc, "rpc-stop"});

    REQUIRE(result.accepted);
    CHECK(fsm.current_state() == RobotState::Idle);
    CHECK(result.has_action(RobotActionKind::StopMotion));
    CHECK(result.has_action(RobotActionKind::ClearMissionContext));
}
```

- [ ] **Step 2: Run the build and verify the tests fail to compile**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `RobotState`, `StartRequested`, `FsmResult`, and `RobotActionKind` are not implemented.

- [ ] **Step 3: Implement the compact FSM API**

In `include/pv_cleaning_robot/app/robot_fsm.h`, define the minimal app-level model:

```cpp
#include <algorithm>
#include <optional>
#include <string>
#include <vector>

enum class RobotState {
    Init,
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    ReturningHome,
    Paused,
    ProtectiveStopped,
    Recovering,
    Charging,
    FaultStopped,
};

enum class RobotActionKind {
    StartSegmentMotion,
    StartBrushOffReturn,
    StartRecoveryMotion,
    StopMotion,
    EmergencyStopMotion,
    StartEndpointSettlingTimer,
    PersistMissionContext,
    ClearMissionContext,
    PublishStatus,
    PublishCommandAccepted,
    PublishCommandRejected,
    PublishCommandCompleted,
    PublishCommandFailed,
    ReportFault,
};

struct RobotAction {
    RobotActionKind kind{RobotActionKind::PublishStatus};
    std::string command_id{};
    std::optional<domain::MissionSegment> segment{};
};

struct FsmResult {
    bool accepted{false};
    std::string reason{};
    std::vector<RobotAction> actions{};

    bool has_action(RobotActionKind kind) const {
        return std::any_of(actions.begin(), actions.end(), [kind](const RobotAction& action) {
            return action.kind == kind;
        });
    }
};

struct StartRequested {
    domain::MissionContext mission;
};

struct SelfCheckPassed {};

struct SelfCheckFailed {
    std::string reason;
    bool fatal{false};
};

struct ExpectedEndpointSettled {
    domain::SegmentTarget target{domain::SegmentTarget::ReturnEndpoint};
};

struct StopRequested {
    domain::CommandSource source{domain::CommandSource::System};
    std::string command_id{};
};

struct ReturnHomeRequested {
    domain::CommandSource source{domain::CommandSource::FaultPolicy};
    std::string command_id{};
};

struct FaultDetected {
    uint32_t code{0};
    bool immediate_stop{false};
};

class RobotFsm {
   public:
    RobotFsm();

    RobotState current_state() const noexcept;
    const std::optional<domain::MissionContext>& mission() const noexcept;

    FsmResult handle(const StartRequested& event);
    FsmResult handle(const SelfCheckPassed& event);
    FsmResult handle(const SelfCheckFailed& event);
    FsmResult handle(const ExpectedEndpointSettled& event);
    FsmResult handle(const StopRequested& event);
    FsmResult handle(const ReturnHomeRequested& event);
    FsmResult handle(const FaultDetected& event);

   private:
    RobotState state_{RobotState::Init};
    std::optional<domain::MissionContext> mission_{};
};
```

- [ ] **Step 4: Implement the transition function**

In `pv_cleaning_robot/app/robot_fsm.cc`, implement explicit transitions. The key endpoint-settled rule is:

```cpp
FsmResult RobotFsm::handle(const ExpectedEndpointSettled& event) {
    if (state_ != RobotState::ExecutingMission || !mission_.has_value()) {
        return {false, "endpoint_not_expected", {}};
    }

    auto& mission = *mission_;
    const auto* segment = mission.current_segment_index < mission.segments.size()
                              ? &mission.segments[mission.current_segment_index]
                              : nullptr;
    if (!segment || segment->target != event.target) {
        return {false, "wrong_endpoint", {}};
    }

    std::vector<RobotAction> actions{{RobotActionKind::StopMotion, mission.command_id, {}}};
    ++mission.current_segment_index;

    if (mission.current_segment_index < mission.segments.size()) {
        state_ = RobotState::ExecutingMission;
        actions.push_back(RobotAction{RobotActionKind::StartSegmentMotion,
                                      mission.command_id,
                                      mission.segments[mission.current_segment_index]});
        return {true, "next_segment", std::move(actions)};
    }

    ++mission.completed_cycles;
    if (mission.kind == domain::MissionKind::ConfiguredMission &&
        mission.completed_cycles < mission.repeat_count) {
        mission.current_segment_index = 0;
        actions.push_back(RobotAction{RobotActionKind::StartSegmentMotion,
                                      mission.command_id,
                                      mission.segments[0]});
        state_ = RobotState::ExecutingMission;
        return {true, "next_cycle", std::move(actions)};
    }

    actions.push_back(RobotAction{RobotActionKind::PublishCommandCompleted,
                                  mission.command_id,
                                  {}});
    actions.push_back(RobotAction{RobotActionKind::ClearMissionContext, mission.command_id, {}});
    mission_.reset();
    state_ = RobotState::Idle;
    return {true, "mission_done", std::move(actions)};
}
```

- [ ] **Step 5: Run FSM tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: `unit_tests` builds. If existing tests still fail to compile because they reference old `EvRpcStartTask` or string state names, update them in this same task to use `StartRequested` and `RobotState`.

- [ ] **Step 6: Commit the FSM slice**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h pv_cleaning_robot/app/robot_fsm.cc test/app/robot_fsm_test.cc
git commit -m "refactor: make robot fsm command-source agnostic"
```

## Task 3: Supervisor Command API And Mission Validation

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Add supervisor tests for the four required command semantics**

In `test/app/robot_supervisor_test.cc`, add tests through the app-level API:

```cpp
TEST_CASE("Supervisor accepts clean toward return from trusted on-segment pose",
          "[app][robot_supervisor]") {
    Fixture f;
    f.endpoint_pose = domain::EndpointPose{domain::EndpointPoseKind::OnSegment, std::nullopt};

    auto result = f.supervisor->submit_command(domain::RobotCommand{
        domain::RobotCommandKind::CleanTowardReturnEndpoint,
        domain::CommandSource::Rpc,
        "rpc-101"});

    REQUIRE(result.accepted);
    CHECK(f.fsm->current_state() == app::RobotState::SelfChecking);
}

TEST_CASE("Supervisor rejects directional clean from unknown pose", "[app][robot_supervisor]") {
    Fixture f;
    f.endpoint_pose = domain::EndpointPose{domain::EndpointPoseKind::Unknown, std::nullopt};

    auto result = f.supervisor->submit_command(domain::RobotCommand{
        domain::RobotCommandKind::CleanTowardParkingEndpoint,
        domain::CommandSource::Rpc,
        "rpc-102"});

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "robot_position_unknown");
}

TEST_CASE("Supervisor configured mission requires configured start endpoint",
          "[app][robot_supervisor]") {
    Fixture f;
    f.lane = domain::LaneGeometry{domain::DockMode::SingleDock, domain::EndpointId::Left};
    f.endpoint_pose = domain::EndpointPose{domain::EndpointPoseKind::AtEndpoint,
                                           domain::EndpointId::Left};

    auto result = f.supervisor->submit_command(domain::RobotCommand{
        domain::RobotCommandKind::StartConfiguredMission,
        domain::CommandSource::Rpc,
        "rpc-103"});

    REQUIRE(result.accepted);
    CHECK(f.fsm->current_state() == app::RobotState::SelfChecking);
}

TEST_CASE("Supervisor stop is idempotent when already idle", "[app][robot_supervisor]") {
    Fixture f;

    auto result = f.supervisor->submit_command(domain::RobotCommand{
        domain::RobotCommandKind::Stop,
        domain::CommandSource::Rpc,
        "rpc-104"});

    REQUIRE(result.accepted);
    CHECK(result.reason == "already_idle");
}
```

- [ ] **Step 2: Run the build and verify API failures**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile fails because `submit_command()` and command result fields are not available.

- [ ] **Step 3: Implement `submit_command()` as the only app command entry**

In `include/pv_cleaning_robot/app/robot_supervisor.h`, add:

```cpp
struct CommandResult {
    bool accepted{false};
    std::string reason{};
};

CommandResult submit_command(const domain::RobotCommand& command);
```

In `pv_cleaning_robot/app/robot_supervisor.cc`, implement command mapping:

```cpp
CommandResult RobotSupervisor::submit_command(const domain::RobotCommand& command) {
    if (command.kind == domain::RobotCommandKind::Stop) {
        const auto result = fsm_->handle(StopRequested{command.source, command.command_id});
        dispatch_actions(result.actions);
        return {true, result.reason.empty() ? "stopped" : result.reason};
    }

    const auto pose = endpoint_pose();
    if (command.kind == domain::RobotCommandKind::CleanTowardReturnEndpoint ||
        command.kind == domain::RobotCommandKind::CleanTowardParkingEndpoint) {
        if (!domain::is_trusted_pose(pose)) {
            return {false, pose.kind == domain::EndpointPoseKind::Inconsistent
                               ? "robot_position_inconsistent"
                               : "robot_position_unknown"};
        }
        auto mission = domain::build_directional_clean_context(
            command.kind == domain::RobotCommandKind::CleanTowardParkingEndpoint
                ? domain::MissionKind::CleanTowardParkingEndpoint
                : domain::MissionKind::CleanTowardReturnEndpoint,
            command.source,
            command.command_id);
        const auto result = fsm_->handle(StartRequested{std::move(mission)});
        dispatch_actions(result.actions);
        return {result.accepted, result.reason};
    }

    if (command.kind == domain::RobotCommandKind::StartConfiguredMission) {
        const auto lane = lane_geometry();
        if (!can_start_configured_mission(lane, pose)) {
            return {false, "configured_mission_requires_start_endpoint"};
        }
        auto mission = domain::build_configured_mission_context(
            lane, pose, command.source, command.command_id, configured_repeat_count());
        const auto result = fsm_->handle(StartRequested{std::move(mission)});
        dispatch_actions(result.actions);
        return {result.accepted, result.reason};
    }

    return {false, "command_not_supported"};
}
```

- [ ] **Step 4: Run supervisor tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: app tests compile and either pass or fail only on action dispatch expectations. Fix only `RobotSupervisor` dispatch glue in this task.

- [ ] **Step 5: Commit the supervisor slice**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h pv_cleaning_robot/app/robot_supervisor.cc test/app/robot_supervisor_test.cc
git commit -m "refactor: route robot commands through supervisor"
```

## Task 4: Segment-Based Motion Dispatch

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `test/service/motion_service_test.cc`

- [ ] **Step 1: Add motion tests for segment target and mode**

In `test/service/motion_service_test.cc`, add:

```cpp
TEST_CASE("MotionService starts cleaning segment toward return endpoint", "[service][motion]") {
    MotionFixture f;
    f.motion.set_lane_geometry_query([] {
        return domain::LaneGeometry{domain::DockMode::SingleDock, domain::EndpointId::Left};
    });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_segment(domain::MissionSegment{
        domain::SegmentTarget::ReturnEndpoint,
        domain::SegmentMode::Cleaning}));
    f.motion.update();

    CHECK_FALSE(f.can->sent_frames.empty());
    CHECK(f.serial->take_tx_text().find("v 0 ") != std::string::npos);
}

TEST_CASE("MotionService starts brush-off return segment with brush stopped", "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();

    REQUIRE(f.motion.start_segment(domain::MissionSegment{
        domain::SegmentTarget::ParkingEndpoint,
        domain::SegmentMode::BrushOffReturn}));
    f.motion.update();

    CHECK_FALSE(f.can->sent_frames.empty());
    CHECK(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
}
```

- [ ] **Step 2: Run build and verify missing method failures**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile fails because `start_segment()` and lane query methods do not exist.

- [ ] **Step 3: Add a segment-based motion method without deleting old wrappers yet**

In `include/pv_cleaning_robot/service/motion_service.h`, add:

```cpp
void set_lane_geometry_query(std::function<domain::LaneGeometry()> query);
bool start_segment(const domain::MissionSegment& segment);
```

In `pv_cleaning_robot/service/motion_service.cc`, implement `start_segment()` by mapping target to travel direction and mode to brush behavior:

```cpp
bool MotionService::start_segment(const domain::MissionSegment& segment) {
    sync_runtime_config();
    if (!enable_speed_mode()) {
        return false;
    }

    const auto lane = lane_geometry_query_
                          ? lane_geometry_query_()
                          : domain::LaneGeometry{domain::DockMode::SingleDock,
                                                 domain::EndpointId::Left};
    const auto target_endpoint = segment.target == domain::SegmentTarget::ParkingEndpoint
                                     ? lane.parking_endpoint
                                     : domain::return_endpoint(lane);
    set_base_speed_command(speed_command_toward(target_endpoint, segment.mode));

    if (segment.mode == domain::SegmentMode::Cleaning) {
        brush_->set_rpm(std::abs(cfg_.brush_rpm) * brush_direction_sign(target_endpoint));
    } else {
        brush_->stop();
    }

    walk_command_active_ = true;
    sync_heading_pid_enabled();
    group_->set_speeds(base_speed_cmd_);
    return true;
}
```

Keep `start_cleaning()`, `start_returning()`, and `start_returning_no_brush()` as compatibility wrappers that call `start_segment()` until app code is migrated.

- [ ] **Step 4: Run motion tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: motion tests pass. Existing wrapper tests still pass.

- [ ] **Step 5: Commit the motion slice**

```bash
git add include/pv_cleaning_robot/service/motion_service.h pv_cleaning_robot/service/motion_service.cc test/service/motion_service_test.cc
git commit -m "refactor: start motion from mission segments"
```

## Task 5: Multi-Stage RecoveryMotion Service

**Files:**
- Create: `include/pv_cleaning_robot/service/recovery_motion.h`
- Create: `pv_cleaning_robot/service/recovery_motion.cc`
- Create: `test/service/recovery_motion_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Add recovery tests**

Create `test/service/recovery_motion_test.cc`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include "pv_cleaning_robot/service/recovery_motion.h"

using robot::service::RecoveryMotion;

TEST_CASE("RecoveryMotion progresses stop sample micro-move verify done", "[service][recovery]") {
    RecoveryMotion recovery;
    recovery.start();

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Stabilizing);

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Sampling);

    recovery.set_pose_error_deg(3.0f);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::MicroMoving);

    recovery.set_pose_error_deg(0.3f);
    CHECK(recovery.step() == RecoveryMotion::Result::Done);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Done);
}

TEST_CASE("RecoveryMotion fails after retry limit", "[service][recovery]") {
    RecoveryMotion recovery;
    recovery.set_max_attempts(2);
    recovery.start();
    recovery.set_pose_error_deg(8.0f);

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Failed);
}
```

- [ ] **Step 2: Register the recovery files**

In `test/CMakeLists.txt`, add:

```cmake
  ${PROJ}/service/recovery_motion.cc
```

to `COMMON_SRCS`, and add:

```cmake
  service/recovery_motion_test.cc
```

to `unit_tests`.

- [ ] **Step 3: Run build and verify missing file failures**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `recovery_motion.h/.cc` do not exist.

- [ ] **Step 4: Implement the minimal recovery controller**

Create `include/pv_cleaning_robot/service/recovery_motion.h`:

```cpp
#pragma once

#include <cstdint>

namespace robot::service {

class RecoveryMotion {
   public:
    enum class Phase {
        Idle,
        Stop,
        Stabilizing,
        Sampling,
        MicroMoving,
        Verifying,
        Done,
        Failed,
    };

    enum class Result {
        Running,
        Done,
        Failed,
    };

    void start();
    Result step();
    Phase phase() const noexcept;
    void set_pose_error_deg(float value) noexcept;
    void set_max_attempts(uint32_t value) noexcept;

   private:
    Phase phase_{Phase::Idle};
    float pose_error_deg_{0.0f};
    uint32_t attempts_{0};
    uint32_t max_attempts_{3};
};

}  // namespace robot::service
```

Create `pv_cleaning_robot/service/recovery_motion.cc`:

```cpp
#include "pv_cleaning_robot/service/recovery_motion.h"

#include <cmath>

namespace robot::service {

void RecoveryMotion::start() {
    phase_ = Phase::Stop;
    attempts_ = 0;
}

RecoveryMotion::Result RecoveryMotion::step() {
    switch (phase_) {
    case Phase::Idle:
        return Result::Failed;
    case Phase::Stop:
        phase_ = Phase::Stabilizing;
        return Result::Running;
    case Phase::Stabilizing:
        phase_ = Phase::Sampling;
        return Result::Running;
    case Phase::Sampling:
        if (std::abs(pose_error_deg_) <= 0.5f) {
            phase_ = Phase::Done;
            return Result::Done;
        }
        phase_ = Phase::MicroMoving;
        return Result::Running;
    case Phase::MicroMoving:
        ++attempts_;
        if (attempts_ >= max_attempts_ && std::abs(pose_error_deg_) > 0.5f) {
            phase_ = Phase::Failed;
            return Result::Failed;
        }
        phase_ = Phase::Verifying;
        return Result::Running;
    case Phase::Verifying:
        if (std::abs(pose_error_deg_) <= 0.5f) {
            phase_ = Phase::Done;
            return Result::Done;
        }
        phase_ = Phase::Sampling;
        return Result::Running;
    case Phase::Done:
        return Result::Done;
    case Phase::Failed:
        return Result::Failed;
    }
    return Result::Failed;
}

RecoveryMotion::Phase RecoveryMotion::phase() const noexcept {
    return phase_;
}

void RecoveryMotion::set_pose_error_deg(float value) noexcept {
    pose_error_deg_ = value;
}

void RecoveryMotion::set_max_attempts(uint32_t value) noexcept {
    max_attempts_ = value == 0 ? 1 : value;
}

}  // namespace robot::service
```

- [ ] **Step 5: Run recovery tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: recovery tests build. Later integration with actual motion commands happens in Task 6.

- [ ] **Step 6: Commit the recovery slice**

```bash
git add include/pv_cleaning_robot/service/recovery_motion.h pv_cleaning_robot/service/recovery_motion.cc test/service/recovery_motion_test.cc test/CMakeLists.txt
git commit -m "feat: add multi-stage recovery motion controller"
```

## Task 6: Action Dispatch And Event Queue Integration

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Add action dispatch tests**

In `test/app/robot_supervisor_test.cc`, add:

```cpp
TEST_CASE("Supervisor dispatches start segment action to MotionService", "[app][robot_supervisor]") {
    Fixture f;
    f.endpoint_pose = domain::EndpointPose{domain::EndpointPoseKind::OnSegment, std::nullopt};

    auto result = f.supervisor->submit_command(domain::RobotCommand{
        domain::RobotCommandKind::CleanTowardReturnEndpoint,
        domain::CommandSource::Rpc,
        "rpc-201"});

    REQUIRE(result.accepted);
    f.supervisor->handle_self_check_passed();

    CHECK(f.motion->start_segment_called);
    CHECK(f.motion->last_segment.target == domain::SegmentTarget::ReturnEndpoint);
}
```

- [ ] **Step 2: Implement dispatch for FSM actions**

In `RobotSupervisor`, implement a single private dispatcher:

```cpp
void RobotSupervisor::dispatch_actions(const std::vector<RobotAction>& actions) {
    for (const auto& action : actions) {
        switch (action.kind) {
        case RobotActionKind::StartSegmentMotion:
            if (action.segment.has_value()) {
                motion_->start_segment(*action.segment);
            }
            break;
        case RobotActionKind::StartBrushOffReturn:
            motion_->start_segment(domain::MissionSegment{domain::SegmentTarget::ParkingEndpoint,
                                                          domain::SegmentMode::BrushOffReturn});
            break;
        case RobotActionKind::StartRecoveryMotion:
            recovery_motion_->start();
            break;
        case RobotActionKind::StopMotion:
            motion_->stop_cleaning();
            break;
        case RobotActionKind::EmergencyStopMotion:
            motion_->emergency_stop();
            break;
        case RobotActionKind::PublishCommandCompleted:
            publish_command_status(action.command_id, "completed");
            break;
        case RobotActionKind::PublishCommandFailed:
            publish_command_status(action.command_id, "failed");
            break;
        default:
            break;
        }
    }
}
```

- [ ] **Step 3: Keep device update calls in existing executors**

Verify `pv_cleaning_robot/main.cc` still has these periodic calls after supervisor changes:

```cpp
walk_exec.add_runnable(motion);
nav_exec.add_runnable(nav);
bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
    [&bms]() { bms->update(); }));
bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
    [&brush_motor]() { brush_motor->update(); }));
```

Do not move device `update()` into the app event queue.

- [ ] **Step 4: Run app tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: supervisor and FSM tests build.

- [ ] **Step 5: Commit the dispatch slice**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h pv_cleaning_robot/app/robot_supervisor.cc pv_cleaning_robot/main.cc test/app/robot_supervisor_test.cc
git commit -m "refactor: dispatch fsm actions from supervisor"
```

## Task 7: RPC Mapping And Command Close Loop

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Add RPC mapping tests**

In `test/service/thingsboard_control_plane_test.cc`, add or replace tests with:

```cpp
TEST_CASE("ThingsBoard RPC clean_to_return maps to directional command", "[service][tb_control_plane]") {
    Fixture f;
    f.mqtt->emit_rpc("301", R"({"method":"clean_to_return","params":{}})");

    CHECK(f.app_port.last_command.kind == domain::RobotCommandKind::CleanTowardReturnEndpoint);
    CHECK(f.app_port.last_command.source == domain::CommandSource::Rpc);
    CHECK(f.app_port.last_command.command_id == "301");
    const auto response = f.last_published_json("rpc/response/301");
    CHECK(response.find("accepted") != std::string::npos);
}

TEST_CASE("ThingsBoard RPC clean_to_parking maps to directional command",
          "[service][tb_control_plane]") {
    Fixture f;
    f.mqtt->emit_rpc("302", R"({"method":"clean_to_parking","params":{}})");

    CHECK(f.app_port.last_command.kind == domain::RobotCommandKind::CleanTowardParkingEndpoint);
    CHECK(f.app_port.last_command.command_id == "302");
}

TEST_CASE("ThingsBoard RPC start_configured maps to configured mission command",
          "[service][tb_control_plane]") {
    Fixture f;
    f.mqtt->emit_rpc("303", R"({"method":"start_configured","params":{}})");

    CHECK(f.app_port.last_command.kind == domain::RobotCommandKind::StartConfiguredMission);
    CHECK(f.app_port.last_command.command_id == "303");
}

TEST_CASE("ThingsBoard RPC stop maps to stop command", "[service][tb_control_plane]") {
    Fixture f;
    f.mqtt->emit_rpc("304", R"({"method":"stop","params":{}})");

    CHECK(f.app_port.last_command.kind == domain::RobotCommandKind::Stop);
    CHECK(f.app_port.last_command.command_id == "304");
}
```

- [ ] **Step 2: Replace `RobotControlPort` with an app command port**

In `include/pv_cleaning_robot/service/thingsboard_control_plane.h`, define a narrow port outside domain:

```cpp
struct RobotCommandPort {
    std::function<robot::app::CommandResult(const domain::RobotCommand&)> submit_command;
    std::function<domain::RobotRuntimeSnapshot()> snapshot;
};
```

Update `ThingsBoardControlPlane` constructor to take `RobotCommandPort`.

- [ ] **Step 3: Register the four required RPC handlers**

In `pv_cleaning_robot/service/thingsboard_control_plane.cc`, map methods directly:

```cpp
register_command_rpc("clean_to_return", domain::RobotCommandKind::CleanTowardReturnEndpoint);
register_command_rpc("clean_to_parking", domain::RobotCommandKind::CleanTowardParkingEndpoint);
register_command_rpc("start_configured", domain::RobotCommandKind::StartConfiguredMission);
register_command_rpc("stop", domain::RobotCommandKind::Stop);
```

Implement the helper:

```cpp
void ThingsBoardControlPlane::register_command_rpc(const char* method,
                                                   domain::RobotCommandKind kind) {
    cloud_->register_rpc(method, [this, kind](const std::string& request_id,
                                              const std::string&) {
        auto result = robot_.submit_command(domain::RobotCommand{
            kind, domain::CommandSource::Rpc, request_id});
        return result.accepted ? rpc_reply("accepted") : rpc_reply(result.reason);
    });
}
```

- [ ] **Step 4: Run ThingsBoard tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: ThingsBoard control plane tests compile. Existing old RPC method tests must be updated to the four names above or kept as compatibility aliases that map to the same command kinds.

- [ ] **Step 5: Commit the RPC slice**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h pv_cleaning_robot/service/thingsboard_control_plane.cc test/service/thingsboard_control_plane_test.cc
git commit -m "refactor: map rpc methods to robot commands"
```

## Task 8: Fault, Safety, And Recovery Transitions

**Files:**
- Modify: `include/pv_cleaning_robot/service/fault_service.h`
- Modify: `pv_cleaning_robot/service/fault_service.cc`
- Modify: `include/pv_cleaning_robot/middleware/safety_monitor.h`
- Modify: `pv_cleaning_robot/middleware/safety_monitor.cc`
- Modify: `test/service/fault_service_test.cc`
- Modify: `test/middleware/safety_monitor_test.cc`

- [ ] **Step 1: Add fault policy tests**

In `test/service/fault_service_test.cc`, add:

```cpp
TEST_CASE("FaultService maps transient attitude fault to recovery", "[service][fault]") {
    middleware::EventBus bus;
    FaultService faults(bus);
    auto decision = faults.decide(FaultEvent{FaultCode::kTransientAttitudeError});

    CHECK(decision.action == FaultAction::StartRecovery);
    CHECK(decision.latch == false);
}

TEST_CASE("FaultService maps conflicting limits to immediate stop", "[service][fault]") {
    middleware::EventBus bus;
    FaultService faults(bus);
    auto decision = faults.decide(FaultEvent{FaultCode::kConflictingLimitSides});

    CHECK(decision.action == FaultAction::ImmediateEmergencyStop);
    CHECK(decision.latch);
}
```

- [ ] **Step 2: Implement table-driven fault policy**

Use a small static table in `fault_service.cc`:

```cpp
static constexpr FaultRule kFaultRules[] = {
    {FaultCode::kStartRejectedLowBattery, FaultAction::RejectStart, false, true},
    {FaultCode::kTransientAttitudeError, FaultAction::StartRecovery, false, true},
    {FaultCode::kConflictingLimitSides, FaultAction::ImmediateEmergencyStop, true, true},
    {FaultCode::kCanCommunicationLost, FaultAction::ImmediateEmergencyStop, true, true},
    {FaultCode::kBrushFaultReturnRequired, FaultAction::BrushOffReturnHome, true, true},
};
```

- [ ] **Step 3: Preserve hard-stop bypass**

Keep `SafetyMonitor` hard stop behavior synchronous:

```cpp
emergency_stop_();
event_queue_.push(FaultDetected{FaultCode::kUnexpectedLimitSide, true});
```

Do not wait for `RobotEventQueue` before stopping motors.

- [ ] **Step 4: Run fault and middleware tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: fault and safety tests compile and pass.

- [ ] **Step 5: Commit the fault/safety slice**

```bash
git add include/pv_cleaning_robot/service/fault_service.h pv_cleaning_robot/service/fault_service.cc include/pv_cleaning_robot/middleware/safety_monitor.h pv_cleaning_robot/middleware/safety_monitor.cc test/service/fault_service_test.cc test/middleware/safety_monitor_test.cc
git commit -m "refactor: route faults through policy actions"
```

## Task 9: Config And Telemetry Naming Migration

**Files:**
- Modify: `include/pv_cleaning_robot/service/config_service.h`
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/config_service_runtime_patch_test.cc`
- Modify: `test/service/business_payload_builder_test.cc`

- [ ] **Step 1: Add config tests for repeat count and schedule immediacy**

In `test/service/config_service_runtime_patch_test.cc`, add:

```cpp
TEST_CASE("ConfigService maps legacy passes to repeat_count compatibility",
          "[service][config]") {
    ConfigService cfg;
    rapidjson::Document patch;
    patch.Parse(R"({"passes":2})");

    auto result = cfg.apply_runtime_patch(patch, nullptr);

    REQUIRE(result.accepted);
    CHECK(cfg.pending_runtime_config()->repeat_count == 2);
}

TEST_CASE("ConfigService applies schedules immediately and other runtime fields pending",
          "[service][config]") {
    ConfigService cfg;
    rapidjson::Document patch;
    patch.Parse(R"({"schedules":[{"hour":8,"minute":30}],"clean_speed_rpm":80})");

    auto result = cfg.apply_runtime_patch(patch, nullptr);

    REQUIRE(result.accepted);
    CHECK(cfg.active_runtime_config().schedules.size() == 1);
    CHECK(cfg.pending_runtime_config()->clean_speed_rpm == 80);
}
```

- [ ] **Step 2: Add `repeat_count` while keeping `passes` as compatibility input**

In runtime config, add:

```cpp
uint32_t repeat_count{1};
```

Parsing rule:

```cpp
if (attrs.HasMember("repeat_count")) {
    pending.repeat_count = clamp_repeat_count(attrs["repeat_count"].GetUint());
}
if (attrs.HasMember("passes")) {
    pending.repeat_count = clamp_repeat_count(static_cast<uint32_t>(attrs["passes"].GetDouble()));
}
```

Telemetry rule: publish `repeat_count` and `completed_cycles`; keep `passes` only if current cloud dashboards still require compatibility.

- [ ] **Step 3: Run config and payload tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: config and payload tests pass.

- [ ] **Step 4: Commit the config slice**

```bash
git add include/pv_cleaning_robot/service/config_service.h pv_cleaning_robot/service/config_service.cc pv_cleaning_robot/service/thingsboard_control_plane.cc test/service/config_service_runtime_patch_test.cc test/service/business_payload_builder_test.cc
git commit -m "refactor: migrate mission repeats from passes to repeat_count"
```

## Task 10: Main Wiring And Final Verification

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/integration/system_integration_test.cc`

- [ ] **Step 1: Update integration tests to use the four command semantics**

In `test/integration/task_chain_test.cc`, cover:

```cpp
TEST_CASE("Task chain: rpc directional clean to return completes at return endpoint",
          "[integration][task_chain]") {
    Fixture f;
    f.submit_rpc("clean_to_return", "401");
    f.self_check_passes();
    f.limit_settled(domain::SegmentTarget::ReturnEndpoint);

    CHECK(f.supervisor->current_state() == app::RobotState::Idle);
    CHECK(f.last_status_for("401") == "completed");
}
```

Add equivalent cases for `clean_to_parking`, `stop`, and `start_configured`.

- [ ] **Step 2: Wire app command port in `main.cc`**

Replace old control port wiring:

```cpp
auto command_port = robot::app::RobotSupervisor::make_command_port(supervisor);
auto tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
    cfg, &scheduler, cloud, command_tracker, command_port);
```

Keep thread executor update calls unchanged.

- [ ] **Step 3: Run final build validation**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
git diff --check
```

Expected: `unit_tests` target builds and `git diff --check` prints no whitespace errors. If QEMU or cross-runner cannot execute tests, record that the target builds but runtime execution is environment-limited.

- [ ] **Step 4: Commit final integration slice**

```bash
git add pv_cleaning_robot/main.cc test/integration/task_chain_test.cc test/integration/system_integration_test.cc
git commit -m "refactor: wire minimal command-based robot app"
```

## Self-Review Checklist

- The plan implements all four required RPC functions through `RobotCommandKind`.
- The plan does not add RPC-specific FSM states.
- The plan keeps device `update()` calls in `ThreadExecutor`.
- The plan preserves hard-stop bypass before queued fault state synchronization.
- The plan keeps recovery multi-stage inside `service::RecoveryMotion`, not the app FSM.
- The plan removes redundant domain fields from the implementation target.
- The plan does not modify `hal`, `driver`, `protocol`, or `device` responsibilities.
