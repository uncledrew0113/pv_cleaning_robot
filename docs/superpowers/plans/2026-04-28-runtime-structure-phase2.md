# Runtime Structure Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split `ThingsBoardRuntime` into focused cloud-boundary classes and move business telemetry snapshot truth into `RobotSupervisor`.

**Architecture:** Phase 2 keeps the Phase 1 runtime behavior intact, but removes the all-in-one `ThingsBoardRuntime` coordinator shape. `RobotSupervisor` becomes the upstream source of runtime snapshot data, `ThingsBoardControlPlane` owns RPC/shared-attribute ingress, and `ThingsBoardTelemetryPublisher` owns startup attributes plus event/periodic telemetry egress.

**Tech Stack:** C++17, Catch2, CMake, existing `RobotSupervisor`/`CloudService`/`ThingsBoardConfigManager`/`CommandTracker` stack

---

## File Structure

**Create**
- `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
  Runtime-facing DTO for task/device state, progress, config visibility, and command visibility.
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
  Cloud ingress boundary for shared attributes and RPC registration.
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
  Concrete RPC/shared-attribute implementation using `CloudService`, `ThingsBoardConfigManager`, `CommandTracker`, and `RobotSupervisor`.
- `include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h`
  Cloud egress boundary for startup attributes, event telemetry, and periodic business telemetry.
- `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
  Concrete telemetry publisher using fixed-buffer builders and `RobotRuntimeSnapshot`.
- `test/service/thingsboard_control_plane_test.cc`
  Focused ingress tests for shared attributes and RPC control.
- `test/service/thingsboard_telemetry_publisher_test.cc`
  Focused egress tests for startup attributes, command events, backup fallback, and periodic business telemetry.

**Delete**
- `include/pv_cleaning_robot/service/thingsboard_runtime.h`
- `pv_cleaning_robot/service/thingsboard_runtime.cc`
- `test/service/thingsboard_runtime_test.cc`

**Modify**
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
- `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
- `pv_cleaning_robot/main.cc`
- `test/app/robot_supervisor_test.cc`
- `test/CMakeLists.txt`

## Task 1: Introduce `RobotRuntimeSnapshot`

**Files:**
- Create: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Add failing tests for snapshot contents**

```cpp
TEST_CASE("RobotSupervisor snapshot reflects active task progress",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(EvFrontLimitSettled{});

    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.device_state == "CleanReturn");
    REQUIRE(snap.task_state == "RunningTask");
    REQUIRE(snap.target_half_passes == 4);
    REQUIRE(snap.completed_half_passes == 1);
}

TEST_CASE("RobotSupervisor snapshot includes config and command visibility",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.apply_pending_config_with_passes(3);
    const auto snap = f.supervisor->snapshot();
    REQUIRE(snap.active_config.has_value());
    REQUIRE(snap.pending_config.has_value());
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails because `RobotRuntimeSnapshot` and `RobotSupervisor::snapshot()` do not exist yet

- [ ] **Step 3: Add the runtime snapshot type and minimal supervisor API**

```cpp
struct RobotRuntimeSnapshot {
    std::string device_state;
    std::string task_state;
    int target_half_passes{0};
    int completed_half_passes{0};
    int clean_count{0};
    uint64_t active_config_version{0};
    std::optional<robot::service::TbRuntimeConfig> active_config;
    std::optional<robot::service::TbRuntimeConfig> pending_config;
    std::optional<robot::service::CommandSnapshot> active_command;
    std::optional<robot::service::CommandSnapshot> last_command;
};

RobotRuntimeSnapshot snapshot() const;
```

```cpp
RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.device_state = fsm_->current_state();
    snap.task_state = snap.device_state == "Paused" ? "PausedTask" : "IdleTask";
    return snap;
}
```

- [ ] **Step 4: Build to verify the API compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- runtime tests would still fail until task-state mapping and config/command data are filled in

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_runtime_snapshot.h \
        include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc
git commit -m "feat: add robot runtime snapshot"
```

## Task 2: Make `RobotSupervisor` The Upstream Snapshot Truth

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Extend tests for full task-state mapping**

```cpp
TEST_CASE("RobotSupervisor snapshot maps device states to task states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;

    REQUIRE(f.supervisor->snapshot().task_state == "IdleTask");

    f.fsm->dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.supervisor->snapshot().task_state == "RunningTask");

    f.fsm->dispatch(EvPauseTask{});
    REQUIRE(f.supervisor->snapshot().task_state == "PausedTask");
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- runtime tests would fail because snapshot mapping/config version are still incomplete

- [ ] **Step 3: Fill in snapshot mapping, config, and progress fields**

```cpp
static std::string task_state_from_device_state(const std::string& device_state) {
    if (device_state == "CleanFwd" || device_state == "CleanReturn")
        return "RunningTask";
    if (device_state == "Paused")
        return "PausedTask";
    if (device_state == "Returning")
        return "ReturningTask";
    if (device_state == "Charging")
        return "ChargingTask";
    if (device_state == "Fault")
        return "FaultedTask";
    if (device_state == "Terminated")
        return "TerminatedTask";
    return "IdleTask";
}

RobotRuntimeSnapshot RobotSupervisor::snapshot() const {
    RobotRuntimeSnapshot snap;
    snap.device_state = fsm_->current_state();
    snap.task_state = task_state_from_device_state(snap.device_state);
    snap.target_half_passes = fsm_->target_half_passes();
    snap.completed_half_passes = fsm_->completed_half_passes();
    snap.clean_count = snap.completed_half_passes / 2;
    snap.active_config = tb_cfg_->active_config();
    snap.pending_config = tb_cfg_->pending_config();
    snap.active_config_version = runtime_config_version(*snap.active_config);
    return snap;
}
```

- [ ] **Step 4: Build to verify snapshot behavior compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- snapshot logic now lives in `RobotSupervisor` instead of ThingsBoard code

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: make supervisor the snapshot source"
```

## Task 3: Extract `ThingsBoardControlPlane`

**Files:**
- Create: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Create: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write failing ingress tests**

```cpp
TEST_CASE("ThingsBoardControlPlane shared attributes update pending config and emit event",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->subscribe_shared_attributes();
    f.mqtt->emit_attributes(R"({"passes":2.5})");

    REQUIRE(f.tb_cfg->pending_config().has_value());
    REQUIRE(f.last_telemetry_json().at("event").get<std::string>() == "shared_attr_update");
}

TEST_CASE("ThingsBoardControlPlane start RPC launches new task from idle",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->register_rpc_handlers([] { return true; }, [] { return false; });
    f.mqtt->emit_rpc("42", R"({"method":"start","params":{}})");

    REQUIRE(f.fsm->current_state() == "CleanFwd");
    REQUIRE(f.last_rpc_response_json("42").at("accepted").get<bool>() == true);
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails because `ThingsBoardControlPlane` does not exist yet

- [ ] **Step 3: Move shared-attribute and RPC logic into the new class**

```cpp
class ThingsBoardControlPlane {
public:
    ThingsBoardControlPlane(std::shared_ptr<CloudService> cloud,
                            std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
                            std::shared_ptr<CommandTracker> command_tracker,
                            std::shared_ptr<robot::app::RobotSupervisor> supervisor,
                            std::function<void(const char*, const CommandSnapshot&)> publish_command_event,
                            std::function<void(bool, const char*)> publish_status_event);

    void subscribe_shared_attributes();
    void register_rpc_handlers(const std::function<bool()>& is_at_home,
                               const std::function<bool()>& is_at_front);
};
```

- [ ] **Step 4: Build to verify the extraction compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- shared attributes and RPC logic no longer live in the monolithic runtime class

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/CMakeLists.txt
git commit -m "refactor: extract thingsboard control plane"
```

## Task 4: Extract `ThingsBoardTelemetryPublisher`

**Files:**
- Create: `include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h`
- Create: `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
- Modify: `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
- Modify: `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
- Test: `test/service/thingsboard_telemetry_publisher_test.cc`
- Test: `test/service/business_telemetry_snapshot_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write failing egress tests**

```cpp
TEST_CASE("ThingsBoardTelemetryPublisher publishes startup attributes",
          "[service][tb_telemetry]") {
    Fixture f;
    f.publisher->publish_startup_attributes();
    REQUIRE(f.last_attributes_json().at("software_version").get<std::string>() == "2.0.0");
}

TEST_CASE("ThingsBoardTelemetryPublisher publishes business telemetry from supervisor snapshot",
          "[service][tb_telemetry]") {
    Fixture f;
    f.fsm->dispatch(EvScheduleStart{true, false, 2.0f});
    f.publisher->publish_business_telemetry();

    REQUIRE(f.last_telemetry_json().at("task_state").get<std::string>() == "RunningTask");
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails because `ThingsBoardTelemetryPublisher` does not exist yet

- [ ] **Step 3: Move startup/event/periodic telemetry logic into the new class**

```cpp
class ThingsBoardTelemetryPublisher {
public:
    ThingsBoardTelemetryPublisher(ConfigService& config,
                                  std::shared_ptr<CloudService> cloud,
                                  std::shared_ptr<robot::app::RobotSupervisor> supervisor);

    void publish_startup_attributes() const;
    void publish_backup_fallback_event() const;
    void publish_command_event(const char* event_name, const CommandSnapshot& snapshot) const;
    void publish_business_telemetry() const;
};
```

```cpp
BusinessTelemetrySnapshot to_business_telemetry(const robot::app::RobotRuntimeSnapshot& snap) {
    BusinessTelemetrySnapshot out;
    out.device_state = snap.device_state;
    out.task_state = snap.task_state;
    out.target_half_passes = snap.target_half_passes;
    out.completed_half_passes = snap.completed_half_passes;
    out.clean_count = snap.clean_count;
    out.active_config_version = snap.active_config_version;
    out.active_config = snap.active_config;
    out.pending_config = snap.pending_config;
    out.active_command = snap.active_command;
    out.last_command = snap.last_command;
    return out;
}
```

- [ ] **Step 4: Build to verify the extraction compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- business telemetry derives from `RobotSupervisor::snapshot()` rather than inline ThingsBoard logic

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h \
        pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc \
        include/pv_cleaning_robot/service/business_telemetry_snapshot.h \
        pv_cleaning_robot/service/business_telemetry_snapshot.cc \
        test/service/thingsboard_telemetry_publisher_test.cc \
        test/service/business_telemetry_snapshot_test.cc \
        test/CMakeLists.txt
git commit -m "refactor: extract thingsboard telemetry publisher"
```

## Task 5: Wire Main And Delete `ThingsBoardRuntime`

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/service/thingsboard_telemetry_publisher_test.cc`
- Modify: `test/CMakeLists.txt`
- Delete: `include/pv_cleaning_robot/service/thingsboard_runtime.h`
- Delete: `pv_cleaning_robot/service/thingsboard_runtime.cc`
- Delete: `test/service/thingsboard_runtime_test.cc`

- [ ] **Step 1: Remove the last wiring references to `ThingsBoardRuntime`**

```cpp
auto tb_telemetry = std::make_shared<robot::service::ThingsBoardTelemetryPublisher>(
    cfg, cloud, supervisor);
auto tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
    cloud, tb_cfg, command_tracker, supervisor,
    [tb_telemetry](const char* event_name, const CommandSnapshot& snapshot) {
        tb_telemetry->publish_command_event(event_name, snapshot);
    },
    [tb_telemetry](bool accepted, const char* reason) {
        tb_telemetry->publish_status_event("shared_attr_update", accepted, reason);
    });
```

- [ ] **Step 2: Build to verify removal reveals any remaining `ThingsBoardRuntime` references**

Run:

```bash
cmake --build build --target unit_tests pv_cleaning_robot -j4
```

Expected:
- compile fails at any remaining include, type alias, or fixture that still references `ThingsBoardRuntime`

- [ ] **Step 3: Delete the old class and finish the new wiring**

```bash
rg -n "ThingsBoardRuntime" include pv_cleaning_robot test
```

Expected:
- no remaining live-code matches once rewiring is complete

- [ ] **Step 4: Build to verify Phase 2 compiles end-to-end**

Run:

```bash
cmake --build build --target unit_tests pv_cleaning_robot -j4
```

Expected:
- compile succeeds
- `main.cc` wires control plane + telemetry publisher directly

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: split thingsboard runtime"
```

## Task 6: Final Phase-2 Verification Pass

**Files:**
- Modify as needed: any files touched by Tasks 1-5

- [ ] **Step 1: Rebuild both the test target and product target**

Run:

```bash
cmake --build build --target unit_tests pv_cleaning_robot -j4
```

Expected:
- successful build with `ThingsBoardRuntime` removed

- [ ] **Step 2: Run targeted tests if a runnable `aarch64` environment is available**

Run:

```bash
./build/unit_tests "[app][robot_supervisor]"
./build/unit_tests "[service][tb_control_plane]"
./build/unit_tests "[service][tb_telemetry]"
./build/unit_tests "[integration][task_chain]"
```

Expected:
- all filters pass in a runnable test environment

- [ ] **Step 3: If this environment still cannot execute `aarch64` binaries, record the limitation explicitly**

```text
Verification note:
- `cmake --build build --target unit_tests pv_cleaning_robot -j4` passed
- runtime execution of `./build/unit_tests` is still blocked on this machine because `qemu-aarch64` is unavailable
```

- [ ] **Step 4: Review the split result**

```bash
wc -l pv_cleaning_robot/main.cc \
      pv_cleaning_robot/app/robot_supervisor.cc \
      pv_cleaning_robot/service/thingsboard_control_plane.cc \
      pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc
rg -n "ThingsBoardRuntime|publish_business_telemetry|register_rpc\\(|subscribe_shared_attributes" \
      include pv_cleaning_robot test -S
```

Expected:
- `ThingsBoardRuntime` no longer exists
- ingress logic lives in `ThingsBoardControlPlane`
- egress logic lives in `ThingsBoardTelemetryPublisher`
- business telemetry derives from supervisor snapshot

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: complete runtime structure phase 2"
```

## Self-Review

- Spec coverage:
  - split `ThingsBoardRuntime`: covered by Tasks 3-5
  - runtime snapshot unification in `RobotSupervisor`: covered by Tasks 1-2
  - keep lower layers stable: preserved by file scope
- Placeholder scan:
  - no `TBD`/`TODO`
  - each task includes exact files, commands, and expected outcomes
- Type consistency:
  - `RobotRuntimeSnapshot`, `ThingsBoardControlPlane`, and `ThingsBoardTelemetryPublisher` names are used consistently across later tasks
