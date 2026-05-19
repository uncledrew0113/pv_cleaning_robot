# Runtime, Motion, and Supervisor Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `ConfigService` the single configuration truth source, centralize motion-direction rules inside `MotionService`, move application orchestration out of `main` into `RobotSupervisor`, minimize non-health cloud telemetry, and remove OTA code from the control process.

**Architecture:** Keep the existing top-level components, but reassign responsibilities instead of adding new framework layers. `ConfigService` will own runtime-config semantics and persistence, `MotionService` will own wheel/brush direction rules from absolute-value config, `RobotSupervisor` will become the application orchestrator, `ThingsBoardControlPlane` will shrink to protocol ingress/egress, and `main` will reduce to dependency wiring plus thread startup.

**Tech Stack:** C++17, RapidJSON, spdlog, existing device/service/app layers, Catch2 tests, CMake.

---

## File Structure

### Core files to modify
- `include/pv_cleaning_robot/service/config_service.h`
- `pv_cleaning_robot/service/config_service.cc`
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `include/pv_cleaning_robot/service/motion_service.h`
- `pv_cleaning_robot/service/motion_service.cc`
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/main.cc`
- `pv_cleaning_robot/CMakeLists.txt`

### Files likely to delete
- `pv_cleaning_robot/middleware/ota_manager.cc`
- `include/pv_cleaning_robot/middleware/ota_manager.h`

### Tests to update
- `test/service/config_service_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/service/motion_service_test.cc`
- `test/app/robot_supervisor_test.cc`
- `test/integration/system_integration_test.cc`
- `test/integration/hardware/system_hw_test.cc`

### Docs/config to update
- `config/config.runtime.json`
- `config/config.fixed.json`
- `README.md`
- `doc/API_REFERENCE.md`
- `doc/thingsboard_config.md`

---

### Task 1: Move runtime-config semantics into `ConfigService`

**Files:**
- Modify: `include/pv_cleaning_robot/service/config_service.h`
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Test: `test/service/config_service_test.cc`

- [ ] **Step 1: Write failing tests for runtime-config ownership in `ConfigService`**

Add test coverage for:
- active runtime config parse from runtime JSON
- pending runtime config parse/load
- shared-attributes patch application
- promote-pending-to-active

Code to add in `test/service/config_service_test.cc`:

```cpp
TEST_CASE("ConfigService: parses active runtime config", "[service][config_service]") {
    ConfigService cfg(runtime_path, fixed_path);
    REQUIRE(cfg.load());
    const auto runtime = cfg.active_runtime_config();
    CHECK(runtime.parking_side == ParkingSide::Right);
    CHECK(runtime.clean_speed_rpm == Approx(20.0));
}

TEST_CASE("ConfigService: promotes pending runtime config into active runtime config",
          "[service][config_service]") {
    ConfigService cfg(runtime_path, fixed_path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.save_pending_runtime_config(test_pending_runtime()));
    REQUIRE(cfg.promote_pending_runtime_to_active());
    CHECK(cfg.active_runtime_config().passes == Approx(3.0));
    CHECK_FALSE(cfg.pending_runtime_config().has_value());
}
```

- [ ] **Step 2: Run config-service tests and confirm they fail**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][config_service]"
```

Expected: compile or test failure because `active_runtime_config()`, `pending_runtime_config()`, or promotion helpers do not exist yet.

- [ ] **Step 3: Add runtime-config type and ownership APIs to `ConfigService`**

Add a configuration type owned by `ConfigService`, replacing the current `TbRuntimeConfig` ownership in `thingsboard_control_plane.h`.

Code sketch for `include/pv_cleaning_robot/service/config_service.h`:

```cpp
enum class ParkingSide { Left, Right };

struct RuntimeScheduleEntry {
    int hour{0};
    int minute{0};
};

struct RuntimeConfig {
    double passes{1.0};
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

class ConfigService {
public:
    RuntimeConfig active_runtime_config() const;
    std::optional<RuntimeConfig> pending_runtime_config() const;
    bool has_pending_runtime_config() const;
    SharedAttrApplyResult apply_runtime_patch(const rapidjson::Value& attrs);
    bool save_pending_runtime_config(const RuntimeConfig& pending);
    bool promote_pending_runtime_to_active();
    uint64_t runtime_config_version(const RuntimeConfig& config) const;
};
```

Implement parse/serialize/apply logic in `pv_cleaning_robot/service/config_service.cc` by moving and adapting logic currently in `ThingsBoardConfigManager`.

- [ ] **Step 4: Update `ThingsBoardControlPlane` public types to use `ConfigService::RuntimeConfig`**

Replace old declarations in `include/pv_cleaning_robot/service/thingsboard_control_plane.h`:

```cpp
using RuntimeConfig = robot::service::RuntimeConfig;
```

Remove `ThingsBoardConfigManager` declarations from the header after call sites are updated.

- [ ] **Step 5: Re-run focused tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][config_service]"
```

Expected: `ConfigService` tests pass.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/config_service.h \
        pv_cleaning_robot/service/config_service.cc \
        include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        test/service/config_service_test.cc
git commit -m "refactor: move runtime config semantics into config service"
```

---

### Task 2: Shrink `ThingsBoardControlPlane` to protocol-only behavior

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Write failing tests proving control-plane delegates config mutation**

Add tests asserting that:
- shared attributes call `ConfigService::apply_runtime_patch(...)`
- promote-at-task-boundary no longer depends on `ThingsBoardConfigManager`

Code to add in `test/service/thingsboard_control_plane_test.cc`:

```cpp
TEST_CASE("ThingsBoardControlPlane: shared attributes delegate to ConfigService",
          "[service][thingsboard_control_plane]") {
    auto attrs = parse_json(R"({"passes":2,"parking_side":"right"})");
    auto result = f.control.apply_shared_attributes(attrs);
    CHECK(result.accepted);
    CHECK(f.cfg.active_runtime_config().parking_side == ParkingSide::Left);
    CHECK(f.cfg.pending_runtime_config()->parking_side == ParkingSide::Right);
}
```

- [ ] **Step 2: Run focused tests and confirm failures**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][thingsboard_control_plane]"
```

Expected: failing compile/test because `ThingsBoardControlPlane` still depends on `ThingsBoardConfigManager`.

- [ ] **Step 3: Replace `ThingsBoardConfigManager` dependency with `ConfigService`**

Update constructor shape in `include/pv_cleaning_robot/service/thingsboard_control_plane.h`:

```cpp
ThingsBoardControlPlane(ConfigService& config,
                        std::shared_ptr<CloudService> cloud,
                        std::shared_ptr<CommandTracker> command_tracker,
                        std::shared_ptr<app::RobotSupervisor> supervisor);
```

Update implementation so:
- `apply_shared_attributes()` calls `config_.apply_runtime_patch(attrs)`
- `active_config()` returns `config_.active_runtime_config()`
- `pending_config()` returns `config_.pending_runtime_config()`
- `promote_pending_to_active()` calls `config_.promote_pending_runtime_to_active()`

- [ ] **Step 4: Remove old manager-specific code paths**

Delete:
- `ThingsBoardConfigManager` implementation in `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- any tests/fixtures that still instantiate it directly

Keep `ThingsBoardControlPlane` focused on:
- MQTT/Cloud subscribe/register
- RPC replies
- startup/status/command/business payload publishing

- [ ] **Step 5: Re-run focused tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][thingsboard_control_plane]"
```

Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        test/service/thingsboard_control_plane_test.cc
git commit -m "refactor: make thingsboard control plane protocol-only"
```

---

### Task 3: Centralize motion direction and brush direction rules in `MotionService`

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `test/service/motion_service_test.cc`

- [ ] **Step 1: Write failing tests for absolute-value motion rules**

Add tests for the four sign combinations the user specified:
- right parking + start
- right parking + return
- left parking + start
- left parking + return

Code to add in `test/service/motion_service_test.cc`:

```cpp
TEST_CASE("MotionService: right parking start uses + + - - wheels and negative brush",
          "[service][motion_service]") {
    f.motion.set_parking_side_query([] { return ParkingSide::Right; });
    REQUIRE(f.motion.start_cleaning());
    CHECK(f.group.last_speed_cmd() == SpeedCmd{20.0f, 20.0f, -20.0f, -20.0f});
    CHECK(f.brush.last_rpm() == -1000);
}

TEST_CASE("MotionService: left parking return uses + + - - wheels and negative brush",
          "[service][motion_service]") {
    f.motion.set_parking_side_query([] { return ParkingSide::Left; });
    REQUIRE(f.motion.start_returning());
    CHECK(f.group.last_speed_cmd() == SpeedCmd{20.0f, 20.0f, -20.0f, -20.0f});
    CHECK(f.brush.last_rpm() == -1000);
}
```

- [ ] **Step 2: Run focused tests and confirm current mismatches**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][motion_service]"
```

Expected: some direction/brush sign tests fail with current split logic.

- [ ] **Step 3: Add a single internal motion-model helper**

Keep public methods unchanged, but centralize internal command generation in `pv_cleaning_robot/service/motion_service.cc`.

Code sketch:

```cpp
struct MotionProfile {
    device::WalkMotorGroup::SpeedCmd wheels{};
    int brush_rpm{0};
    bool run_brush{false};
};

MotionProfile MotionService::build_clean_profile() const;
MotionProfile MotionService::build_return_profile(bool run_brush) const;
```

Rules:
- interpret config magnitudes with `std::abs(...)`
- derive signs solely from `parking_side`
- `start_returning_no_brush()` reuses return wheel signs but forces `brush_rpm = 0`

- [ ] **Step 4: Use the same base command as PID input**

Ensure `base_speed_cmd_` always comes from the centralized motion model before PID correction, so wheel direction truth is not duplicated.

Code in `pv_cleaning_robot/service/motion_service.cc`:

```cpp
const auto profile = build_clean_profile();
set_base_speed_command(profile.wheels);
if (profile.run_brush) {
    brush_->set_rpm(profile.brush_rpm);
}
```

- [ ] **Step 5: Re-run focused tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][motion_service]"
./build/aarch64/bin/unit_tests "[service][heading_pid]"
```

Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/motion_service.h \
        pv_cleaning_robot/service/motion_service.cc \
        test/service/motion_service_test.cc
git commit -m "refactor: centralize motion direction model in motion service"
```

---

### Task 4: Move application orchestration from `main` into `RobotSupervisor`

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/app/robot_supervisor_test.cc`
- Modify: `test/integration/system_integration_test.cc`

- [ ] **Step 1: Write failing tests for supervisor-owned orchestration**

Add tests for:
- translating physical limit side into parking/far-end business events
- startup-position handling
- scheduler-triggered start path

Code sketch for `test/app/robot_supervisor_test.cc`:

```cpp
TEST_CASE("RobotSupervisor: maps physical limit to parking-side event using active parking side",
          "[app][robot_supervisor]") {
    f.cfg.set_active_runtime_parking_side(ParkingSide::Left);
    f.supervisor->on_limit_settled(robot::device::LimitSide::LEFT, 20.0f);
    CHECK(f.fsm->current_state() == "Charging");
}
```

- [ ] **Step 2: Run focused tests and confirm missing API**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[app][robot_supervisor]"
```

Expected: compile/test failure because orchestration helpers are still in `main`.

- [ ] **Step 3: Add orchestration entry points to `RobotSupervisor`**

Extend `include/pv_cleaning_robot/app/robot_supervisor.h` with methods such as:

```cpp
ParkingSideFacts current_active_parking_facts() const;
ParkingSideFacts current_start_parking_facts() const;
void on_limit_settled(robot::device::LimitSide side, float battery_soc);
bool start_task_from_scheduler(float battery_soc);
void handle_startup_position();
```

Implement them in `pv_cleaning_robot/app/robot_supervisor.cc` by moving logic currently in:
- `read_parking_side_facts(...)`
- `publish_startup_position_status(...)`
- `register_limit_settled_bridge(...)`
- `register_scheduler_start(...)`

- [ ] **Step 4: Slim `main` down to construction/wiring**

Remove those helper functions from `main.cc` and replace with direct wiring:

```cpp
event_bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
    [supervisor, current_battery_soc](const auto& evt) {
        supervisor->on_limit_settled(evt.side, current_battery_soc());
    });

scheduler.set_on_window_hit([supervisor, current_battery_soc]() {
    supervisor->start_task_from_scheduler(current_battery_soc());
});

supervisor->handle_startup_position();
```

- [ ] **Step 5: Re-run focused tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[app][robot_supervisor]"
./build/aarch64/bin/unit_tests "[integration]"
```

Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        pv_cleaning_robot/main.cc \
        test/app/robot_supervisor_test.cc \
        test/integration/system_integration_test.cc
git commit -m "refactor: move app orchestration into robot supervisor"
```

---

### Task 5: Minimize non-health cloud telemetry and command/status fields

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/business_payload_builder_test.cc`
- Modify: `test/service/cloud_service_test.cc`

- [ ] **Step 1: Write failing tests for the reduced payload shape**

Add tests asserting that periodic business telemetry contains only the minimum set:
- `device_state`
- `task_state`
- `active_config_version`

Code sketch:

```cpp
TEST_CASE("Business telemetry: emits minimal runtime truth only",
          "[service][business_payload]") {
    auto payload = parse_json(build_business_payload(snapshot));
    CHECK(payload.HasMember("device_state"));
    CHECK(payload.HasMember("task_state"));
    CHECK(payload.HasMember("active_config_version"));
    CHECK_FALSE(payload.HasMember("active_config"));
    CHECK_FALSE(payload.HasMember("pending_config"));
}
```

- [ ] **Step 2: Run focused tests and confirm current payload is too large**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][business_payload]"
```

Expected: fail because current payload still includes extra fields.

- [ ] **Step 3: Shrink runtime snapshot and publisher output**

Update `RobotRuntimeSnapshot` and `ThingsBoardJsonCodec::build_business_telemetry(...)` so the periodic payload keeps only the minimum required fields.

Code sketch:

```cpp
struct RobotRuntimeSnapshot {
    std::string device_state;
    std::string task_state;
    uint64_t active_config_version{0};
};
```

And:

```cpp
writer.Key("device_state");
writer.String(view.device_state.c_str());
writer.Key("task_state");
writer.String(view.task_state.c_str());
writer.Key("active_config_version");
writer.Uint64(view.active_config_version);
```

- [ ] **Step 4: Reduce startup/status/command publishing to minimum**

Keep:
- startup attributes with only identity/version fields
- RPC reply payloads

Delete or stop publishing:
- command lifecycle cloud events unless still strictly needed by tests/contracts
- redundant status-event variants

- [ ] **Step 5: Re-run focused tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
./build/aarch64/bin/unit_tests "[service][business_payload]"
./build/aarch64/bin/unit_tests "[service][cloud_service]"
```

Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_runtime_snapshot.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        test/service/business_payload_builder_test.cc \
        test/service/cloud_service_test.cc
git commit -m "refactor: minimize non-health telemetry payloads"
```

---

### Task 6: Remove OTA from the control-process build and clean config/docs

**Files:**
- Modify: `pv_cleaning_robot/CMakeLists.txt`
- Delete: `pv_cleaning_robot/middleware/ota_manager.cc`
- Delete: `include/pv_cleaning_robot/middleware/ota_manager.h`
- Modify: `config/config.runtime.json`
- Modify: `config/config.fixed.json`
- Modify: `README.md`
- Modify: `doc/API_REFERENCE.md`
- Modify: `doc/thingsboard_config.md`

- [ ] **Step 1: Search for remaining OTA references and write down exact removals**

Run:

```bash
rg -n "OTA|ota_manager|PV_ENABLE_OTA|firmware update|A/B 分区" \
   pv_cleaning_robot include config README.md doc -S
```

Expected: list only the current residual OTA files and doc references.

- [ ] **Step 2: Remove OTA from the build**

Update `pv_cleaning_robot/CMakeLists.txt` to stop conditionally compiling:

```cmake
if(NOT PV_ENABLE_OTA)
    list(REMOVE_ITEM ...)
endif()
```

Instead, remove `middleware/ota_manager.cc` from the library source list entirely.

- [ ] **Step 3: Delete OTA implementation files**

Delete:

```text
pv_cleaning_robot/middleware/ota_manager.cc
include/pv_cleaning_robot/middleware/ota_manager.h
```

- [ ] **Step 4: Remove OTA-related config/doc references**

Update config/docs so they describe OTA as an external process responsibility, not part of the control binary.

Example wording for docs:

```md
OTA is handled by a separate system process. `pv_cleaning_robot` does not download, stage, or apply firmware updates.
```

- [ ] **Step 5: Run full build verification**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests -j4
cmake --build --preset rk3576-build --target hw_tests -j4
git diff --check
```

Expected:
- both targets build
- no whitespace or patch-format issues

- [ ] **Step 6: Commit**

```bash
git add pv_cleaning_robot/CMakeLists.txt \
        config/config.runtime.json \
        config/config.fixed.json \
        README.md \
        doc/API_REFERENCE.md \
        doc/thingsboard_config.md
git rm pv_cleaning_robot/middleware/ota_manager.cc \
       include/pv_cleaning_robot/middleware/ota_manager.h
git commit -m "refactor: remove ota from control process"
```

---

### Task 7: Final integration verification

**Files:**
- Verify only; no planned file creation

- [ ] **Step 1: Run focused service/app tests**

Run:

```bash
./build/aarch64/bin/unit_tests "[service][config_service]"
./build/aarch64/bin/unit_tests "[service][thingsboard_control_plane]"
./build/aarch64/bin/unit_tests "[service][motion_service]"
./build/aarch64/bin/unit_tests "[service][heading_pid]"
./build/aarch64/bin/unit_tests "[app][robot_supervisor]"
```

Expected: all pass.

- [ ] **Step 2: Run key integration tests**

Run:

```bash
./build/aarch64/bin/unit_tests "[integration]"
```

Expected: integration suite passes or any hardware-excluded tests remain green.

- [ ] **Step 3: Rebuild hardware tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests -j4
```

Expected: build success.

- [ ] **Step 4: Run final patch hygiene checks**

Run:

```bash
git diff --check
rg -n "ThingsBoardConfigManager|ota_manager|PV_ENABLE_OTA" \
   pv_cleaning_robot include test config doc -S
```

Expected:
- `git diff --check` clean
- no stale `ThingsBoardConfigManager`
- no stale OTA control-process references

- [ ] **Step 5: Final commit**

```bash
git add -A
git commit -m "refactor: centralize config motion and supervisor responsibilities"
```

---

## Self-Review

### Spec coverage
- Config centralization: covered by Tasks 1-2
- Motion model in `MotionService`: covered by Task 3
- Supervisor/main boundary cleanup: covered by Task 4
- Telemetry minimization: covered by Task 5
- OTA removal: covered by Task 6

### Placeholder scan
- No `TODO` / `TBD` placeholders remain.
- All planned code changes include target files and concrete commands.

### Type consistency
- `RuntimeConfig` is owned by `ConfigService` throughout the plan.
- `ParkingSide` remains the shared enum for motion/config/orchestration.
- `MotionService` public methods stay intact; only internal model changes.

