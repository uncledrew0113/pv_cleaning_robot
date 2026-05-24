# FSM / Fault / Business Kernel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace pass-oriented task orchestration with a simple mission/segment kernel, simplify the FSM, implement action-oriented P0/P1/P2 handling, and expose unified business state/fault/action codes without adding new code files.

**Architecture:** Keep the existing app/service split, but move mission semantics into the current FSM/supervisor files and move segment-to-motor translation into the current `MotionService`. Do not add code files; instead, simplify in place, collapse obsolete pass/state branches, and keep business truth centralized in existing headers and snapshot/telemetry paths.

**Tech Stack:** C++17, Boost.SML, Catch2, RapidJSON, existing app/service/device layers, `cmake --build --preset rk3576-build --target unit_tests`, optional `/usr/bin/qemu-aarch64*_static` for target-binary verification

---

## File Structure

### Files To Modify

- `include/pv_cleaning_robot/service/config_service.h`
  - add `LaneMode` and retain `passes` only as a single-dock repeat-count compatibility field
- `pv_cleaning_robot/service/config_service.cc`
  - parse/validate/serialize `lane_mode`
  - keep `passes` validation for round-trip repeat count only
- `include/pv_cleaning_robot/app/robot_fsm.h`
  - replace old state/event vocabulary with mission/segment/business-code vocabulary
- `pv_cleaning_robot/app/robot_fsm.cc`
  - implement segment-driven FSM transitions and fault-disposal segment injection
- `include/pv_cleaning_robot/app/robot_supervisor.h`
  - expose mission-oriented start/return helpers
- `pv_cleaning_robot/app/robot_supervisor.cc`
  - build missions from config/start context, update state predicates, update snapshot fields
- `include/pv_cleaning_robot/service/motion_service.h`
  - replace directional wrapper API with a segment-oriented start API
- `pv_cleaning_robot/service/motion_service.cc`
  - resolve segment business semantics into motor profiles in place
- `include/pv_cleaning_robot/app/fault_handler.h`
  - define fault decision vocabulary in the existing fault-handler boundary
- `pv_cleaning_robot/app/fault_handler.cc`
  - implement in-place fault arbitration without adding a new file
- `include/pv_cleaning_robot/service/fault_service.h`
  - extend fault event storage/access to support business fault code reporting safely
- `pv_cleaning_robot/service/fault_service.cc`
  - store and expose last fault in a way telemetry can read safely
- `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
  - add `BusinessStateCode`, `BusinessActionCode`, `BusinessFaultCode`
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
  - no API expansion beyond what telemetry build needs
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
  - publish business codes and self-check/fault visibility
- `test/app/robot_fsm_test.cc`
  - rewrite around mission/segment states, remove old `CleanFwd/CleanReturn/Returning` assertions
- `test/app/robot_supervisor_test.cc`
  - align startup/return/task-state behavior with mission semantics
- `test/app/fault_handler_test.cc`
  - cover P0 hard stop, P1 returnable/non-returnable, P2 report-only
- `test/service/motion_service_test.cc`
  - cover segment-to-profile resolution
- `test/service/fault_service_test.cc`
  - cover last-fault snapshot behavior
- `test/service/business_payload_builder_test.cc`
  - assert business state/action/fault telemetry payload
- `test/service/thingsboard_control_plane_test.cc`
  - assert cloud-side start/return/fault payload semantics
- `test/integration/hardware/system_hw_test.cc`
  - update log expectations and state assertions to new business vocabulary
- `test/integration/hardware/hw_config.h`
  - update fixture defaults if mission config fields are needed

### Files To Keep But Simplify

- `include/pv_cleaning_robot/app/parking_side_runtime.h`
  - keep as-is unless a tiny helper is needed; do not expand it into a mission planner

### Files Explicitly Not Created

- no new `MissionBuilder`, `ProfileResolver`, `FaultArbiter`, or `BusinessCode` files
- no new helper directories
- no new app/service modules

The rule for implementation is: if a helper is small and used only by one current file, keep it private in that existing file.

## Compatibility Rule Used By This Plan

To keep the refactor small and reliable:

- `RuntimeConfig::passes` stays in config for now
- it is reinterpreted as:
  - single-dock `RoundTripClean`: number of round-trip repetitions
  - dual-dock `DockToDockClean`: ignored or forced to `1`
  - single-leg test: not sourced from config; only constructed explicitly by code/tests

This preserves current cloud/runtime config compatibility while removing `passes` from the FSM’s internal truth model.

### Task 1: Introduce Kernel Vocabulary In Existing Headers

**Files:**
- Modify: `include/pv_cleaning_robot/service/config_service.h`
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Test: `test/app/robot_fsm_test.cc`
- Test: `test/service/business_payload_builder_test.cc`

- [ ] **Step 1: Write the failing tests for lane mode, mission kernel types, and business codes**

```cpp
TEST_CASE("FSM mission kernel accepts dual-dock dock-to-dock mission", "[app][fsm]") {
    FsmFixture f;

    MissionSpec mission;
    mission.lane_mode = LaneMode::DualDockLane;
    mission.type = MissionType::DockToDockClean;
    mission.segments = {
        SegmentSpec{
            TerminalSide::LeftDock,
            TerminalSide::RightDock,
            SegmentMode::Clean,
            CompletionCondition::ReachRightTerminal,
        },
    };

    f.fsm.dispatch(EvStartMission{mission});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");
    REQUIRE(f.fsm.business_state_code() == BusinessStateCode::EXECUTING_SEGMENT);
}

TEST_CASE("ThingsBoardJsonCodec emits business state action and fault codes",
          "[service][tb_json_codec]") {
    robot::app::RobotRuntimeSnapshot snap;
    snap.business_state = robot::app::BusinessStateCode::EXECUTING_SEGMENT;
    snap.business_action = robot::app::BusinessActionCode::EXECUTING_CLEAN_SEGMENT;
    snap.last_business_fault = robot::app::BusinessFaultCode::NONE;

    char out[512];
    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_business_telemetry(snap, out, sizeof(out));

    REQUIRE(len > 0);
    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["business_state"].GetString()) == "EXECUTING_SEGMENT");
    CHECK(std::string(payload["business_action"].GetString()) == "EXECUTING_CLEAN_SEGMENT");
    CHECK(std::string(payload["business_fault"].GetString()) == "NONE");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fsm]" "[service][tb_json_codec]"
```

Expected:

- build or test failure because `MissionSpec`, `SegmentSpec`, `LaneMode`, `BusinessStateCode`, `business_state_code()`, or payload keys do not exist yet

- [ ] **Step 3: Add the minimal shared vocabulary in existing headers**

```cpp
// include/pv_cleaning_robot/service/config_service.h
enum class LaneMode {
    SingleDockLane,
    DualDockLane,
};

inline const char* lane_mode_config_string(LaneMode value) noexcept {
    switch (value) {
    case LaneMode::SingleDockLane:
        return "single_dock_lane";
    case LaneMode::DualDockLane:
        return "dual_dock_lane";
    }
    return "single_dock_lane";
}

struct RuntimeConfig {
    double passes{1.0};  // compatibility repeat count for single-dock round-trip only
    LaneMode lane_mode{LaneMode::SingleDockLane};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    int return_brush_rpm{1000};
    ParkingSide parking_side{ParkingSide::Left};
    double start_battery_soc{30.0};
    double charge_start_soc{15.0};
    double charge_stop_soc{95.0};
    std::vector<RuntimeScheduleEntry> schedules;
};
```

```cpp
// include/pv_cleaning_robot/service/config_service.h
private:
    static LaneMode parse_lane_mode_string(const std::string& value);
```

```cpp
// include/pv_cleaning_robot/app/robot_fsm.h
enum class TerminalSide {
    LeftDock,
    RightDock,
    LeftFar,
    RightFar,
};

enum class MissionType {
    RoundTripClean,
    DockToDockClean,
    SingleLegTest,
};

enum class SegmentMode {
    Clean,
    ReturnNoBrush,
};

enum class CompletionCondition {
    ReachLeftTerminal,
    ReachRightTerminal,
};

struct SegmentSpec {
    TerminalSide from_terminal{TerminalSide::LeftDock};
    TerminalSide to_terminal{TerminalSide::RightFar};
    SegmentMode segment_mode{SegmentMode::Clean};
    CompletionCondition completion_condition{CompletionCondition::ReachRightTerminal};
};

struct MissionSpec {
    service::LaneMode lane_mode{service::LaneMode::SingleDockLane};
    MissionType type{MissionType::RoundTripClean};
    std::vector<SegmentSpec> segments;
    std::size_t current_segment_index{0};
};
```

```cpp
// include/pv_cleaning_robot/app/robot_runtime_snapshot.h
enum class BusinessStateCode {
    IDLE,
    SELF_CHECK,
    EXECUTING_SEGMENT,
    SEGMENT_BOUNDARY,
    CHARGING,
    FAULT_STOPPED,
};

enum class BusinessActionCode {
    NONE,
    STARTING_MISSION,
    EXECUTING_CLEAN_SEGMENT,
    SWITCHING_SEGMENT,
    RETURNING_TO_DOCK,
    STOPPING_IMMEDIATELY,
    WAITING_MANUAL_RESET,
};

enum class BusinessFaultCode {
    NONE,
    STARTUP_POSITION_INVALID,
    BMS_UNAVAILABLE,
    BRUSH_FAILURE_RETURN_REQUIRED,
    WALK_DRIVE_FAILURE_STOP_REQUIRED,
    LIMIT_SIGNAL_CONFLICT,
    PARKING_TARGET_UNREACHABLE,
};

struct RobotRuntimeSnapshot {
    std::string device_state;
    std::string task_state;
    BusinessStateCode business_state{BusinessStateCode::IDLE};
    BusinessActionCode business_action{BusinessActionCode::NONE};
    BusinessFaultCode last_business_fault{BusinessFaultCode::NONE};
    int target_passes{0};
    int completed_passes{0};
    int clean_count{0};
    uint64_t active_config_version{0};
    std::optional<robot::service::RuntimeConfig> active_config;
    std::optional<robot::service::RuntimeConfig> pending_config;
    std::optional<robot::service::CommandSnapshot> active_command;
    std::optional<robot::service::CommandSnapshot> last_command;
};
```

- [ ] **Step 4: Implement config parsing/serialization for `lane_mode`**

```cpp
// pv_cleaning_robot/service/config_service.cc
bool is_supported_runtime_patch_field(const std::string& key)
{
    return key == "passes" || key == "lane_mode" || key == "clean_speed_rpm" ||
           key == "return_speed_rpm" || key == "brush_rpm" || key == "return_brush_rpm" ||
           key == "parking_side" || key == "start_battery_soc" ||
           key == "charge_start_soc" || key == "charge_stop_soc" || key == "schedules";
}

LaneMode ConfigService::parse_lane_mode_string(const std::string& value)
{
    if (value == lane_mode_config_string(LaneMode::SingleDockLane)) {
        return LaneMode::SingleDockLane;
    }
    if (value == lane_mode_config_string(LaneMode::DualDockLane)) {
        return LaneMode::DualDockLane;
    }
    throw std::runtime_error("lane_mode must be single_dock_lane or dual_dock_lane");
}

if (const auto it = robot.FindMember("lane_mode");
    it != robot.MemberEnd() && it->value.IsString()) {
    cfg.lane_mode = parse_lane_mode_string(it->value.GetString());
}

set_string_member(*robot,
                  "lane_mode",
                  lane_mode_config_string(config.lane_mode),
                  root.GetAllocator());
```

- [ ] **Step 5: Re-run the targeted tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fsm]" "[service][tb_json_codec]"
```

Expected:

- compile still fails on old FSM API/state names, but lane-mode and business-code symbols now exist

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/config_service.h \
        pv_cleaning_robot/service/config_service.cc \
        include/pv_cleaning_robot/app/robot_fsm.h \
        include/pv_cleaning_robot/app/robot_runtime_snapshot.h \
        test/app/robot_fsm_test.cc \
        test/service/business_payload_builder_test.cc
git commit -m "refactor: add mission kernel vocabulary"
```

### Task 2: Replace Pass-Oriented FSM With Segment-Oriented Execution

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_fsm_test.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Rewrite the failing FSM tests around the new state vocabulary**

```cpp
TEST_CASE("single-dock round-trip mission advances segment by segment", "[app][fsm]") {
    FsmFixture f;

    MissionSpec mission;
    mission.lane_mode = LaneMode::SingleDockLane;
    mission.type = MissionType::RoundTripClean;
    mission.segments = {
        {TerminalSide::LeftDock, TerminalSide::RightFar, SegmentMode::Clean,
         CompletionCondition::ReachRightTerminal},
        {TerminalSide::RightFar, TerminalSide::LeftDock, SegmentMode::Clean,
         CompletionCondition::ReachLeftTerminal},
    };

    f.fsm.dispatch(EvStartMission{mission});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");

    f.fsm.dispatch(EvSegmentTerminalReached{device::LimitSide::RIGHT, true});
    REQUIRE(f.fsm.current_state() == "ExecutingSegment");

    f.fsm.dispatch(EvSegmentTerminalReached{device::LimitSide::LEFT, true});
    REQUIRE(f.fsm.current_state() == "Charging");
}

TEST_CASE("single-leg test mission completes on far terminal", "[app][fsm]") {
    FsmFixture f;

    MissionSpec mission;
    mission.lane_mode = LaneMode::SingleDockLane;
    mission.type = MissionType::SingleLegTest;
    mission.segments = {
        {TerminalSide::LeftDock, TerminalSide::RightFar, SegmentMode::Clean,
         CompletionCondition::ReachRightTerminal},
    };

    f.fsm.dispatch(EvStartMission{mission});
    f.fsm.dispatch(EvSegmentTerminalReached{device::LimitSide::RIGHT, false});
    REQUIRE(f.fsm.current_state() == "Idle");
}
```

- [ ] **Step 2: Run tests to verify old FSM implementation fails**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fsm]" "[app][robot_supervisor]"
```

Expected:

- failures because `EvStartMission`, `EvSegmentTerminalReached`, `ExecutingSegment`, `SegmentBoundary`, and mission-driven transitions are not implemented yet

- [ ] **Step 3: Replace old state/event definitions in `robot_fsm.h`**

```cpp
// remove:
// StateCleanFwd, StateCleanReturn, StateReturning, StateStopped, StateFault
// EvScheduleStart, EvRpcStartTask carrying only passes
// EvFarEndLimitSettled / EvParkingSideLimitSettled as the primary runtime events

struct StateInit {};
struct StateIdle {};
struct StateSelfCheck {};
struct StateExecutingSegment {};
struct StateSegmentBoundary {};
struct StateCharging {};
struct StateFaultStopped {};

struct EvStartMission {
    MissionSpec mission;
};

struct EvSegmentTerminalReached {
    device::LimitSide side;
    bool should_charge{true};
};

struct EvStopTask {};
struct EvFaultReset {};
struct EvChargeDone {};

struct EvFaultHardStop {};
struct EvFaultInjectReturnSegment {
    SegmentSpec segment;
};
```

```cpp
// include/pv_cleaning_robot/app/robot_fsm.h
class RobotFsm {
public:
    void dispatch(EvStartMission e);
    void dispatch(EvSegmentTerminalReached e);
    void dispatch(EvFaultHardStop e);
    void dispatch(EvFaultInjectReturnSegment e);

    BusinessStateCode business_state_code() const;
    BusinessActionCode business_action_code() const;
    BusinessFaultCode business_fault_code() const;
private:
    std::optional<MissionSpec> active_mission_;
    std::optional<SegmentSpec> active_segment_;
    BusinessStateCode business_state_code_{BusinessStateCode::IDLE};
    BusinessActionCode business_action_code_{BusinessActionCode::NONE};
    BusinessFaultCode business_fault_code_{BusinessFaultCode::NONE};
};
```

- [ ] **Step 4: Implement the minimal segment-driven FSM in `robot_fsm.cc` and adapt `RobotSupervisor`**

```cpp
// pv_cleaning_robot/app/robot_fsm.cc
template <>
void RobotFsm::dispatch<EvStartMission>(EvStartMission e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        active_mission_ = std::move(e.mission);
        active_segment_ = active_mission_->segments.at(0);
        business_state_code_ = BusinessStateCode::EXECUTING_SEGMENT;
        business_action_code_ = active_segment_->segment_mode == SegmentMode::ReturnNoBrush
            ? BusinessActionCode::RETURNING_TO_DOCK
            : BusinessActionCode::EXECUTING_CLEAN_SEGMENT;
        state_name_ = "ExecutingSegment";
        action = [this]() { motion_->start_segment(*active_segment_); };
    }
    if (action) action();
}

template <>
void RobotFsm::dispatch<EvSegmentTerminalReached>(EvSegmentTerminalReached e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        business_state_code_ = BusinessStateCode::SEGMENT_BOUNDARY;
        business_action_code_ = BusinessActionCode::SWITCHING_SEGMENT;

        const bool last_segment =
            active_mission_->current_segment_index + 1 >= active_mission_->segments.size();

        if (last_segment) {
            state_name_ = e.should_charge ? "Charging" : "Idle";
            business_state_code_ = e.should_charge ? BusinessStateCode::CHARGING
                                                   : BusinessStateCode::IDLE;
            business_action_code_ = BusinessActionCode::NONE;
            active_segment_.reset();
            action = [this]() { motion_->stop_cleaning(); };
        } else {
            ++active_mission_->current_segment_index;
            active_segment_ =
                active_mission_->segments.at(active_mission_->current_segment_index);
            state_name_ = "ExecutingSegment";
            business_state_code_ = BusinessStateCode::EXECUTING_SEGMENT;
            business_action_code_ = active_segment_->segment_mode == SegmentMode::ReturnNoBrush
                ? BusinessActionCode::RETURNING_TO_DOCK
                : BusinessActionCode::EXECUTING_CLEAN_SEGMENT;
            action = [this]() { motion_->start_segment(*active_segment_); };
        }
    }
    if (action) action();
}
```

```cpp
// pv_cleaning_robot/app/robot_supervisor.cc
bool RobotSupervisor::start_task(bool at_parking_side, bool position_valid, float battery_soc) {
    if (!is_new_task_start_state(fsm_->current_state()) || !position_valid || !at_parking_side) {
        return false;
    }

    MissionSpec mission = build_start_mission(/*allow_single_leg_test=*/false);
    fsm_->dispatch(EvStartMission{mission});
    return fsm_->business_state_code() == BusinessStateCode::EXECUTING_SEGMENT;
}
```

```cpp
// include/pv_cleaning_robot/app/robot_supervisor.h
private:
    MissionSpec build_start_mission(bool allow_single_leg_test) const;
    static BusinessFaultCode map_fault_event_to_business_fault(
        const service::FaultService::FaultEvent& evt);
```

- [ ] **Step 5: Update supervisor state mapping and snapshot filling**

```cpp
// pv_cleaning_robot/app/robot_supervisor.cc
bool RobotSupervisor::is_new_task_start_state(const std::string& state) {
    return state == "Idle" || state == "Charging";
}

MissionSpec RobotSupervisor::build_start_mission(bool allow_single_leg_test) const {
    MissionSpec mission;
    mission.lane_mode = config_.active_runtime_config().lane_mode;

    if (allow_single_leg_test) {
        mission.type = MissionType::SingleLegTest;
        mission.segments = {
            {TerminalSide::LeftDock, TerminalSide::RightFar, SegmentMode::Clean,
             CompletionCondition::ReachRightTerminal},
        };
        return mission;
    }

    if (mission.lane_mode == service::LaneMode::DualDockLane) {
        mission.type = MissionType::DockToDockClean;
        mission.segments = {
            {TerminalSide::LeftDock, TerminalSide::RightDock, SegmentMode::Clean,
             CompletionCondition::ReachRightTerminal},
        };
        return mission;
    }

    mission.type = MissionType::RoundTripClean;
    mission.segments = {
        {TerminalSide::LeftDock, TerminalSide::RightFar, SegmentMode::Clean,
         CompletionCondition::ReachRightTerminal},
        {TerminalSide::RightFar, TerminalSide::LeftDock, SegmentMode::Clean,
         CompletionCondition::ReachLeftTerminal},
    };
    return mission;
}

std::string RobotSupervisor::task_state_from_device_state(const std::string& device_state) {
    if (device_state == "ExecutingSegment") return "RunningTask";
    if (device_state == "Charging") return "ChargingTask";
    if (device_state == "FaultStopped") return "FaultedTask";
    return "IdleTask";
}

RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.device_state = fsm_->current_state();
    snap.task_state = task_state_from_device_state(snap.device_state);
    snap.business_state = fsm_->business_state_code();
    snap.business_action = fsm_->business_action_code();
    snap.last_business_fault = fsm_->business_fault_code();
    return snap;
}
```

- [ ] **Step 6: Re-run the app-level tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fsm]" "[app][robot_supervisor]"
```

Expected:

- `robot_fsm_test` and supervisor mission-state tests pass
- legacy assertions on `CleanFwd/CleanReturn/Returning/Stopped/Fault` are removed or updated

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc \
        include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_fsm_test.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: replace pass-based fsm with segment execution"
```

### Task 3: Move Direction Logic Behind `MotionService::start_segment`

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Test: `test/service/motion_service_test.cc`

- [ ] **Step 1: Write the failing motion tests for segment resolution**

```cpp
TEST_CASE("MotionService clean segment resolves to brush-on profile", "[service][motion]") {
    MotionFixture f;

    robot::app::SegmentSpec segment{
        robot::app::TerminalSide::LeftDock,
        robot::app::TerminalSide::RightFar,
        robot::app::SegmentMode::Clean,
        robot::app::CompletionCondition::ReachRightTerminal,
    };

    REQUIRE(f.motion.start_segment(segment));
    const auto tx = f.brush_serial->take_tx_text();
    CHECK(tx.find("v 0 0.000 0\n") == std::string::npos);
}

TEST_CASE("MotionService return-no-brush segment resolves to brush-off profile",
          "[service][motion]") {
    MotionFixture f;

    robot::app::SegmentSpec segment{
        robot::app::TerminalSide::RightFar,
        robot::app::TerminalSide::LeftDock,
        robot::app::SegmentMode::ReturnNoBrush,
        robot::app::CompletionCondition::ReachLeftTerminal,
    };

    REQUIRE(f.motion.start_segment(segment));
    const auto tx = f.brush_serial->take_tx_text();
    CHECK(tx.find("v 0 0.000 0\n") != std::string::npos);
}
```

- [ ] **Step 2: Run the motion tests to verify they fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[service][motion]"
```

Expected:

- failure because `start_segment()` and segment-based motion resolution do not exist

- [ ] **Step 3: Replace wrapper-style motion start API with a segment-oriented API**

```cpp
// include/pv_cleaning_robot/service/motion_service.h
bool start_segment(const app::SegmentSpec& segment);
void stop_cleaning();
void emergency_stop();

private:
    bool apply_segment_profile(const app::SegmentSpec& segment);
    device::WalkMotorGroup::SpeedCmd resolve_walk_profile(const app::SegmentSpec& segment) const;
    int resolve_brush_rpm(const app::SegmentSpec& segment) const;
```

```cpp
// pv_cleaning_robot/service/motion_service.cc
bool MotionService::start_segment(const app::SegmentSpec& segment) {
    sync_runtime_config();
    sync_heading_pid_enabled();
    return apply_segment_profile(segment);
}

bool MotionService::apply_segment_profile(const app::SegmentSpec& segment) {
    if (!enable_speed_mode()) {
        return false;
    }

    const auto cmd = resolve_walk_profile(segment);
    set_base_speed_command(cmd);
    walk_command_active_ = true;

    if (segment.segment_mode == app::SegmentMode::ReturnNoBrush) {
        brush_->set_speed(0.0f);
    } else {
        brush_->set_speed(static_cast<float>(resolve_brush_rpm(segment)));
    }
    return true;
}
```

- [ ] **Step 4: Resolve wheel/brush behavior inside the current `motion_service.cc`**

```cpp
device::WalkMotorGroup::SpeedCmd MotionService::resolve_walk_profile(
    const app::SegmentSpec& segment) const
{
    const int sign = task_direction_sign();
    const float clean = cfg_.clean_speed_rpm;
    const float ret = cfg_.return_speed_rpm;

    if (segment.segment_mode == app::SegmentMode::ReturnNoBrush) {
        motion_phase_ = HeadingCorrector::MotionPhase::Return;
        return device::WalkMotorGroup::SpeedCmd{-sign * ret, -sign * ret, sign * ret, sign * ret};
    }

    motion_phase_ = HeadingCorrector::MotionPhase::CleanFwd;
    const bool left_to_right =
        segment.from_terminal == app::TerminalSide::LeftDock ||
        segment.from_terminal == app::TerminalSide::LeftFar;
    const int lane_sign = left_to_right ? sign : -sign;
    return device::WalkMotorGroup::SpeedCmd{lane_sign * clean,
                                            lane_sign * clean,
                                            -lane_sign * clean,
                                            -lane_sign * clean};
}
```

- [ ] **Step 5: Re-run the motion tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[service][motion]"
```

Expected:

- segment-based motion tests pass
- no direct test depends on `start_cleaning()` / `start_returning()` wrappers anymore

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/motion_service.h \
        pv_cleaning_robot/service/motion_service.cc \
        test/service/motion_service_test.cc
git commit -m "refactor: resolve motion by segment semantics"
```

### Task 4: Implement P0 / P1 / P2 Business Decisions In `FaultHandler`

**Files:**
- Modify: `include/pv_cleaning_robot/app/fault_handler.h`
- Modify: `pv_cleaning_robot/app/fault_handler.cc`
- Modify: `include/pv_cleaning_robot/service/fault_service.h`
- Modify: `pv_cleaning_robot/service/fault_service.cc`
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Test: `test/app/fault_handler_test.cc`
- Test: `test/service/fault_service_test.cc`

- [ ] **Step 1: Rewrite failing fault tests around business decisions**

```cpp
TEST_CASE("FaultHandler P1 returnable injects return-no-brush segment", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.current_segment = robot::app::SegmentSpec{
        robot::app::TerminalSide::LeftDock,
        robot::app::TerminalSide::RightFar,
        robot::app::SegmentMode::Clean,
        robot::app::CompletionCondition::ReachRightTerminal,
    };

    f.fault_svc.report(FaultLevel::P1, 0x2001, "brush blocked");

    REQUIRE(f.injected_return_segment.has_value());
    CHECK(f.injected_return_segment->segment_mode == robot::app::SegmentMode::ReturnNoBrush);
}

TEST_CASE("FaultHandler P2 only reports and does not change motion", "[app][fault_handler]") {
    FaultHandlerFixture f;
    f.fault_svc.report(FaultLevel::P2, 0x3001, "gps stale");
    REQUIRE(f.dispatched_hard_stop == false);
    REQUIRE_FALSE(f.injected_return_segment.has_value());
}
```

- [ ] **Step 2: Run fault tests to verify they fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fault_handler]" "[service][fault]"
```

Expected:

- failure because there is no business fault decision path yet

- [ ] **Step 3: Add a small decision vocabulary inside the existing `fault_handler.h`**

```cpp
enum class FaultTargetAction {
    ReportOnly,
    HardStop,
    ReturnToDock,
};

struct FaultDecision {
    app::BusinessFaultCode business_fault{app::BusinessFaultCode::NONE};
    FaultTargetAction action{FaultTargetAction::ReportOnly};
    std::optional<app::SegmentSpec> return_segment;
};

class FaultHandler {
private:
    FaultDecision decide_fault(const service::FaultService::FaultEvent& evt) const;
    std::function<void(const app::SegmentSpec&, app::BusinessFaultCode)> dispatch_return_segment_;
};
```

- [ ] **Step 4: Implement in-place arbitration and FSM hooks**

```cpp
FaultDecision FaultHandler::decide_fault(const service::FaultService::FaultEvent& evt) const {
    using Level = service::FaultService::FaultEvent::Level;

    if (evt.level == Level::P0) {
        return {app::BusinessFaultCode::WALK_DRIVE_FAILURE_STOP_REQUIRED,
                FaultTargetAction::HardStop,
                std::nullopt};
    }

    if (evt.level == Level::P1) {
        app::SegmentSpec segment{
            app::TerminalSide::RightFar,
            app::TerminalSide::LeftDock,
            app::SegmentMode::ReturnNoBrush,
            app::CompletionCondition::ReachLeftTerminal,
        };
        return {app::BusinessFaultCode::BRUSH_FAILURE_RETURN_REQUIRED,
                FaultTargetAction::ReturnToDock,
                segment};
    }

    return {app::BusinessFaultCode::NONE, FaultTargetAction::ReportOnly, std::nullopt};
}

void FaultHandler::on_fault(const service::FaultService::FaultEvent& evt) {
    const auto decision = decide_fault(evt);
    switch (decision.action) {
    case FaultTargetAction::HardStop:
        motion_->emergency_stop();
        dispatch_fn_(evt);
        break;
    case FaultTargetAction::ReturnToDock:
        dispatch_return_segment_(*decision.return_segment, decision.business_fault);
        break;
    case FaultTargetAction::ReportOnly:
        break;
    }
}
```

```cpp
// include/pv_cleaning_robot/app/fault_handler.h
using FsmHardStopFn = std::function<void()>;
using FsmReturnSegmentFn =
    std::function<void(const app::SegmentSpec&, app::BusinessFaultCode)>;

FaultHandler(std::shared_ptr<service::MotionService> motion,
             middleware::EventBus& bus,
             FsmHardStopFn dispatch_hard_stop,
             FsmReturnSegmentFn dispatch_return_segment);
```

```cpp
// pv_cleaning_robot/app/robot_fsm.cc
template <>
void RobotFsm::dispatch<EvFaultHardStop>(EvFaultHardStop) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    state_name_ = "FaultStopped";
    business_state_code_ = BusinessStateCode::FAULT_STOPPED;
    business_action_code_ = BusinessActionCode::STOPPING_IMMEDIATELY;
}

template <>
void RobotFsm::dispatch<EvFaultInjectReturnSegment>(EvFaultInjectReturnSegment e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        active_segment_ = e.segment;
        state_name_ = "ExecutingSegment";
        business_state_code_ = BusinessStateCode::EXECUTING_SEGMENT;
        business_action_code_ = BusinessActionCode::RETURNING_TO_DOCK;
        action = [this, segment = e.segment]() { motion_->start_segment(segment); };
    }
    if (action) action();
}
```

- [ ] **Step 5: Make `FaultService` snapshot-safe for telemetry consumption**

```cpp
// include/pv_cleaning_robot/service/fault_service.h
std::optional<FaultEvent> last_fault_snapshot() const;
```

```cpp
// pv_cleaning_robot/service/fault_service.cc
std::optional<FaultService::FaultEvent> FaultService::last_fault_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_fault_) {
        return std::nullopt;
    }
    return last_fault_;
}
```

- [ ] **Step 6: Re-run the fault tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fault_handler]" "[service][fault]"
```

Expected:

- `P0` hard-stop path passes
- `P1` inject-return path passes
- `P2` report-only path passes

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/fault_handler.h \
        pv_cleaning_robot/app/fault_handler.cc \
        include/pv_cleaning_robot/service/fault_service.h \
        pv_cleaning_robot/service/fault_service.cc \
        include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc \
        test/app/fault_handler_test.cc \
        test/service/fault_service_test.cc
git commit -m "refactor: make fault handling action-oriented"
```

### Task 5: Publish Unified Business Codes Through Supervisor And ThingsBoard

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Test: `test/service/business_payload_builder_test.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Write failing telemetry and RPC tests against the new payload**

```cpp
TEST_CASE("business telemetry contains kernel truth fields", "[service][tb_json_codec]") {
    robot::app::RobotRuntimeSnapshot snap;
    snap.business_state = robot::app::BusinessStateCode::FAULT_STOPPED;
    snap.business_action = robot::app::BusinessActionCode::WAITING_MANUAL_RESET;
    snap.last_business_fault =
        robot::app::BusinessFaultCode::BRUSH_FAILURE_RETURN_REQUIRED;

    char out[512];
    const size_t len =
        robot::service::ThingsBoardJsonCodec::build_business_telemetry(snap, out, sizeof(out));

    const auto payload = parse_json(out, len);
    CHECK(std::string(payload["business_state"].GetString()) == "FAULT_STOPPED");
    CHECK(std::string(payload["business_action"].GetString()) == "WAITING_MANUAL_RESET");
    CHECK(std::string(payload["business_fault"].GetString()) ==
          "BRUSH_FAILURE_RETURN_REQUIRED");
}

TEST_CASE("self-check failure is visible through published status event",
          "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers(/*position_valid=*/false);
    f.mqtt->emit_rpc("1", R"({"method":"start","params":{}})");

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["event"].GetString()) == "startup_position_invalid");
    CHECK(j.HasMember("business_fault"));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[service][tb_json_codec]" "[service][tb_control_plane]"
```

Expected:

- payload tests fail because new business keys are not emitted yet

- [ ] **Step 3: Emit business-code strings from `ThingsBoardJsonCodec`**

```cpp
// pv_cleaning_robot/service/thingsboard_control_plane.cc
namespace {
const char* to_string(robot::app::BusinessStateCode value) {
    switch (value) {
    case robot::app::BusinessStateCode::IDLE: return "IDLE";
    case robot::app::BusinessStateCode::SELF_CHECK: return "SELF_CHECK";
    case robot::app::BusinessStateCode::EXECUTING_SEGMENT: return "EXECUTING_SEGMENT";
    case robot::app::BusinessStateCode::SEGMENT_BOUNDARY: return "SEGMENT_BOUNDARY";
    case robot::app::BusinessStateCode::CHARGING: return "CHARGING";
    case robot::app::BusinessStateCode::FAULT_STOPPED: return "FAULT_STOPPED";
    }
    return "IDLE";
}

const char* to_string(robot::app::BusinessActionCode value) {
    switch (value) {
    case robot::app::BusinessActionCode::NONE: return "NONE";
    case robot::app::BusinessActionCode::STARTING_MISSION: return "STARTING_MISSION";
    case robot::app::BusinessActionCode::EXECUTING_CLEAN_SEGMENT: return "EXECUTING_CLEAN_SEGMENT";
    case robot::app::BusinessActionCode::SWITCHING_SEGMENT: return "SWITCHING_SEGMENT";
    case robot::app::BusinessActionCode::RETURNING_TO_DOCK: return "RETURNING_TO_DOCK";
    case robot::app::BusinessActionCode::STOPPING_IMMEDIATELY: return "STOPPING_IMMEDIATELY";
    case robot::app::BusinessActionCode::WAITING_MANUAL_RESET: return "WAITING_MANUAL_RESET";
    }
    return "NONE";
}

const char* to_string(robot::app::BusinessFaultCode value) {
    switch (value) {
    case robot::app::BusinessFaultCode::NONE: return "NONE";
    case robot::app::BusinessFaultCode::STARTUP_POSITION_INVALID: return "STARTUP_POSITION_INVALID";
    case robot::app::BusinessFaultCode::BMS_UNAVAILABLE: return "BMS_UNAVAILABLE";
    case robot::app::BusinessFaultCode::BRUSH_FAILURE_RETURN_REQUIRED: return "BRUSH_FAILURE_RETURN_REQUIRED";
    case robot::app::BusinessFaultCode::WALK_DRIVE_FAILURE_STOP_REQUIRED: return "WALK_DRIVE_FAILURE_STOP_REQUIRED";
    case robot::app::BusinessFaultCode::LIMIT_SIGNAL_CONFLICT: return "LIMIT_SIGNAL_CONFLICT";
    case robot::app::BusinessFaultCode::PARKING_TARGET_UNREACHABLE: return "PARKING_TARGET_UNREACHABLE";
    }
    return "NONE";
}
}  // namespace

writer.Key("business_state");
writer.String(to_string(view.business_state));
writer.Key("business_action");
writer.String(to_string(view.business_action));
writer.Key("business_fault");
writer.String(to_string(view.last_business_fault));
writer.Key("active_config_version");
writer.Uint64(view.active_config_version);
```

```cpp
// keep device_state/task_state only if still needed by existing cloud dashboards
writer.Key("device_state");
writer.String(view.device_state.c_str());
writer.Key("task_state");
writer.String(view.task_state.c_str());
```

- [ ] **Step 4: Fill snapshot truth from supervisor/fault state**

```cpp
RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.device_state = fsm_->current_state();
    snap.task_state = task_state_from_device_state(snap.device_state);
    snap.business_state = fsm_->business_state_code();
    snap.business_action = fsm_->business_action_code();

    if (const auto last_fault = fault_->last_fault_snapshot()) {
        snap.last_business_fault = map_fault_event_to_business_fault(*last_fault);
    }
    return snap;
}
```

```cpp
// pv_cleaning_robot/app/robot_supervisor.cc
BusinessFaultCode RobotSupervisor::map_fault_event_to_business_fault(
    const service::FaultService::FaultEvent& evt) {
    switch (evt.level) {
    case service::FaultService::FaultEvent::Level::P0:
        return BusinessFaultCode::WALK_DRIVE_FAILURE_STOP_REQUIRED;
    case service::FaultService::FaultEvent::Level::P1:
        return BusinessFaultCode::BRUSH_FAILURE_RETURN_REQUIRED;
    case service::FaultService::FaultEvent::Level::P2:
    case service::FaultService::FaultEvent::Level::P3:
        return BusinessFaultCode::NONE;
    }
    return BusinessFaultCode::NONE;
}
```

- [ ] **Step 5: Re-run the telemetry/control-plane tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[service][tb_json_codec]" "[service][tb_control_plane]"
```

Expected:

- payload tests pass with business-state/action/fault fields
- start rejection and fault paths expose business fault reason

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_runtime_snapshot.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        test/service/business_payload_builder_test.cc \
        test/service/thingsboard_control_plane_test.cc
git commit -m "refactor: publish business kernel telemetry"
```

### Task 6: Update Hardware-Oriented Tests And Run Full Verification

**Files:**
- Modify: `test/integration/hardware/system_hw_test.cc`
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/app/robot_supervisor_test.cc`
- Modify: `test/app/robot_fsm_test.cc`
- Modify: `test/service/motion_service_test.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Update hardware-test assertions to the new vocabulary**

```cpp
// test/integration/hardware/system_hw_test.cc
CHECK(log_text.find("state=ExecutingSegment") != std::string::npos);
CHECK(log_text.find("business_state=EXECUTING_SEGMENT") != std::string::npos);
CHECK(log_text.find("business_action=RETURNING_TO_DOCK") != std::string::npos);
CHECK(log_text.find("state=FaultStopped") != std::string::npos);
```

```cpp
// remove assertions that require:
// "CleanFwd", "CleanReturn", "Returning", "Stopped", "Fault"
```

- [ ] **Step 2: Run the focused unit test set**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[app][fsm]" "[app][robot_supervisor]" "[app][fault_handler]" "[service][motion]" "[service][fault]" "[service][tb_json_codec]" "[service][tb_control_plane]"
```

Expected:

- all targeted unit/integration-mock tests pass

- [ ] **Step 3: Run hardware-test binary build and selected hardware test filter**

Run:

```bash
cmake --build build --target hw_tests -j4
./build/aarch64/bin/hw_tests "[hw_system][pid_combined]"
```

Expected:

- build succeeds
- log vocabulary reflects `ExecutingSegment`, `FaultStopped`, and business actions

- [ ] **Step 4: Verify target binaries with host-side qemu when direct execution is needed**

Run:

```bash
ls /usr/bin/qemu-aarch64*_static
/usr/bin/qemu-aarch64-static ./build/aarch64/bin/unit_tests "[app][fsm]"
```

Expected:

- qemu binary exists
- target-architecture `unit_tests` can at least start and execute the selected filter if host execution is otherwise blocked

- [ ] **Step 5: Run the broad regression set**

Run:

```bash
./build/aarch64/bin/unit_tests "[app]" "[service]"
```

Expected:

- no regressions in adjacent app/service behavior after the vocabulary/state refactor

- [ ] **Step 6: Commit**

```bash
git add test/integration/hardware/system_hw_test.cc \
        test/integration/hardware/hw_config.h \
        test/app/robot_supervisor_test.cc \
        test/app/robot_fsm_test.cc \
        test/service/motion_service_test.cc \
        test/service/thingsboard_control_plane_test.cc
git commit -m "test: align hardware and service coverage with mission kernel"
```

## Self-Review

### Spec Coverage

- mission/segment model: covered by Tasks 1-2
- simple FSM with reduced states: covered by Task 2
- no degraded mode: enforced in Task 4 and telemetry checks in Task 5
- P0/P1/P2 action semantics: covered by Task 4
- business state/fault/action codes: covered by Tasks 1 and 5
- single-dock/dual-dock completion rules: covered by Task 2 and Task 6
- cloud visibility of self-check/fault truth: covered by Task 5
- no new code files / prefer simplification in place: enforced by file list and all tasks
- qemu verification note: covered by Task 6

### Placeholder Scan

- no `TODO` / `TBD`
- every code-changing step includes concrete code snippets
- every validation step includes exact commands and expected outcomes

### Type Consistency

The plan consistently uses:

- `LaneMode`
- `MissionType`
- `SegmentMode`
- `CompletionCondition`
- `MissionSpec`
- `SegmentSpec`
- `BusinessStateCode`
- `BusinessActionCode`
- `BusinessFaultCode`
- `EvStartMission`
- `EvSegmentTerminalReached`
- `EvFaultHardStop`
- `EvFaultInjectReturnSegment`

No alternate names are used later in the plan.
