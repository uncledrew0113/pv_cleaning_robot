# Runtime Config And Code Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split fixed config from ThingsBoard-managed runtime config, centralize runtime config semantics in one place, reduce duplicated runtime/ThingsBoard code, and add focused explanatory comments.

**Architecture:** Keep the existing `ConfigService + ThingsBoardConfigManager` structure, but split persisted configuration into fixed, runtime-active, and runtime-pending files. `ConfigService` remains a light JSON file service, while `ThingsBoardConfigManager` becomes the only owner of active-vs-pending runtime semantics. `main.cc` and test fixtures stop interpreting runtime config directly and instead consume `active_config()` or the promotion API.

**Tech Stack:** C++17, Catch2, RapidJSON, existing `ConfigService`, `ThingsBoardConfigManager`, `ThingsBoardControlPlane`, existing unit/integration tests.

---

## File Structure

**Create:**
- `docs/superpowers/plans/2026-05-08-runtime-config-and-code-simplification.md`

**Modify:**
- `config/config.json`
- `include/pv_cleaning_robot/service/config_service.h`
- `pv_cleaning_robot/service/config_service.cc`
- `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- `pv_cleaning_robot/main.cc`
- `test/service/config_service_test.cc`
- `test/service/thingsboard_config_manager_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/app/robot_supervisor_test.cc`
- `test/integration/system_integration_test.cc`
- `test/integration/task_chain_test.cc`
- `test/integration/thingsboard_runtime_mock_integration_test.cc`
- `test/integration/thingsboard_real_integration_test.cc`
- `test/integration/hardware/hw_config.h`
- `test/integration/hardware/system_hw_test.cc`

**Potentially modify if duplication is confirmed while implementing:**
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `pv_cleaning_robot/app/robot_supervisor.h`
- `test/app/fault_handler_test.cc`
- `test/app/robot_fsm_test.cc`

**Test:**
- `test/service/config_service_test.cc`
- `test/service/thingsboard_config_manager_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/app/robot_supervisor_test.cc`
- `test/integration/task_chain_test.cc`
- `test/integration/system_integration_test.cc`
- `test/integration/thingsboard_runtime_mock_integration_test.cc`
- `test/integration/thingsboard_real_integration_test.cc`

---

### Task 1: Add fixed/runtime/pending file support to `ConfigService`

**Files:**
- Modify: `include/pv_cleaning_robot/service/config_service.h`
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Test: `test/service/config_service_test.cc`

- [ ] **Step 1: Write the failing tests for split config paths**

Add tests that assert:

```cpp
TEST_CASE("ConfigService loads fixed, runtime, and pending files independently",
          "[service][config]") {
    SplitConfigFixture f;
    ConfigService cfg(f.runtime_path, f.fixed_path);

    REQUIRE(cfg.load());
    REQUIRE(cfg.load_fixed());

    CHECK(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(320.0f));
    CHECK(cfg.get_fixed<int>("gpio.left_limit.line", -1) == 12);

    auto pending = cfg.load_pending();
    REQUIRE(pending.has_value());
    CHECK((*pending)["robot"]["passes"].GetDouble() == Approx(3.0));
}

TEST_CASE("ConfigService clears runtime pending file when requested",
          "[service][config]") {
    SplitConfigFixture f;
    ConfigService cfg(f.runtime_path, f.fixed_path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.clear_pending());
    CHECK_FALSE(std::filesystem::exists(f.pending_path));
}
```

- [ ] **Step 2: Run the focused config tests to verify they fail**

Run: `./aarch64/bin/unit_tests "[service][config]"`

Expected: FAIL because `ConfigService` does not yet expose a separate fixed-config path or the new split-file helpers.

- [ ] **Step 3: Add the minimal `ConfigService` API for split files**

Extend the public API with exact methods needed by the tests and by the later runtime-manager changes, for example:

```cpp
class ConfigService {
   public:
    explicit ConfigService(const std::string& runtime_path = "",
                           const std::string& fixed_path = "");

    bool load();
    bool load_fixed();

    template <typename T>
    T get_fixed(const std::string& path, const T& default_value) const;

    bool save_pending(const rapidjson::Document& pending);
    std::optional<rapidjson::Document> load_pending() const;
    bool clear_pending();

    const std::string& runtime_path() const noexcept;
    const std::string& fixed_path() const noexcept;
    const std::string& pending_path() const noexcept;
};
```

Keep `ConfigService` as a file/JSON utility only. Do not put runtime semantics into it.

- [ ] **Step 4: Implement the split-file behavior in `config_service.cc`**

Implement path derivation and file I/O with the existing save/load style:

```cpp
ConfigService::ConfigService(const std::string& runtime_path,
                             const std::string& fixed_path)
    : runtime_path_(runtime_path)
    , fixed_path_(fixed_path.empty() ? derive_fixed_path(runtime_path) : fixed_path)
    , pending_path_(derive_pending_path(runtime_path)) {}

bool ConfigService::load_fixed() {
    if (fixed_path_.empty())
        return false;
    return load_document_from_path(fixed_path_, fixed_doc_);
}

template <typename T>
T ConfigService::get_fixed(const std::string& path, const T& default_value) const {
    return get_from_document<T>(fixed_doc_, path, default_value);
}
```

Use the same helper style already present in the file. Do not redesign the JSON machinery.

- [ ] **Step 5: Run the focused config tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[service][config]"`

Expected: PASS for new split-file tests and existing `ConfigService` behavior.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/config_service.h \
        pv_cleaning_robot/service/config_service.cc \
        test/service/config_service_test.cc
git commit -m "refactor: add split fixed and runtime config support"
```

---

### Task 2: Move runtime config semantics fully into `ThingsBoardConfigManager`

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Test: `test/service/thingsboard_config_manager_test.cc`

- [ ] **Step 1: Write the failing tests for scheduler-immediate vs pending runtime updates**

Add tests like:

```cpp
TEST_CASE("ThingsBoardConfigManager applies scheduler windows immediately",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"schedules":[{"hour":9,"minute":15}]})");

    auto result = f.manager->apply_shared_attributes(attrs);

    REQUIRE(result.accepted);
    CHECK(f.manager->active_config().schedules.size() == 1);
    CHECK(f.scheduler.windows().size() == 1);
    CHECK_FALSE(f.manager->pending_config().has_value());
}

TEST_CASE("ThingsBoardConfigManager writes non-scheduler runtime attrs to pending only",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"passes":2.0,"parking_side":"right"})");

    auto before = f.manager->active_config();
    auto result = f.manager->apply_shared_attributes(attrs);

    REQUIRE(result.accepted);
    CHECK(f.manager->active_config().passes == before.passes);
    REQUIRE(f.manager->pending_config().has_value());
    CHECK(f.manager->pending_config()->passes == Approx(2.0));
}
```

- [ ] **Step 2: Run the focused ThingsBoard config tests to verify they fail**

Run: `./aarch64/bin/unit_tests "[service][tb_config]"`

Expected: FAIL because current tests and implementation still allow more immediate updates than the new design permits.

- [ ] **Step 3: Update the manager interface to expose only the needed active/pending operations**

Use a narrow API:

```cpp
class ThingsBoardConfigManager {
   public:
    const TbRuntimeConfig& active_config() const noexcept;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const noexcept;
    bool promote_pending_to_active();

    SharedAttrApplyResult apply_shared_attributes(const rapidjson::Value& attrs);
};
```

Do not create a new config-store class. Keep this manager as the semantics owner.

- [ ] **Step 4: Implement the exact field split**

In `apply_shared_attributes`, enforce:

```cpp
if (attrs.HasMember("schedules")) {
    apply_schedules_immediately(...);
}

if (attrs.HasMember("passes") ||
    attrs.HasMember("parking_side") ||
    attrs.HasMember("clean_speed_rpm") ||
    attrs.HasMember("return_speed_rpm") ||
    attrs.HasMember("brush_rpm") ||
    attrs.HasMember("return_brush_rpm") ||
    attrs.HasMember("start_battery_soc") ||
    attrs.HasMember("charge_start_soc") ||
    attrs.HasMember("charge_stop_soc")) {
    write_runtime_fields_to_pending(...);
}
```

The implementation must:

- validate all runtime fields first
- update active only for `schedules`
- update pending only for every other runtime field
- persist active runtime to `config.runtime.json`
- persist pending runtime to `config.runtime.pending.json`
- delete the pending file when pending is empty

- [ ] **Step 5: Make promotion consume pending only after start approval**

Implement:

```cpp
bool ThingsBoardConfigManager::promote_pending_to_active() {
    if (!pending_)
        return false;
    active_ = merge_pending_into_active(active_, *pending_);
    persist_active_runtime(active_);
    pending_.reset();
    cfg_.clear_pending();
    scheduler_.replace_windows(active_.schedules);
    return true;
}
```

Keep `schedules` in active so the scheduler always reflects `active_config()`.

- [ ] **Step 6: Run the focused ThingsBoard config tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[service][tb_config]"`

Expected: PASS for:

- immediate scheduler update behavior
- pending-only runtime update behavior
- promotion behavior
- validation behavior

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_config_manager.h \
        pv_cleaning_robot/service/thingsboard_config_manager.cc \
        test/service/thingsboard_config_manager_test.cc
git commit -m "refactor: centralize runtime config semantics in tb manager"
```

---

### Task 3: Make `RobotSupervisor` the only pending-promotion trigger

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Write the failing tests for promotion timing**

Add tests like:

```cpp
TEST_CASE("RobotSupervisor promotes pending only on new task start",
          "[app][robot_supervisor]") {
    Fixture f;
    f.apply_pending_runtime_attrs(parse_json(R"({"passes":3.0})"));

    REQUIRE(f.supervisor->start_task(true, true, 80.0f));
    CHECK(f.tb_cfg->active_config().passes == Approx(3.0));
    CHECK_FALSE(f.tb_cfg->has_pending_config());
}

TEST_CASE("Charging to Idle does not auto-promote pending",
          "[app][robot_supervisor]") {
    Fixture f;
    f.apply_pending_runtime_attrs(parse_json(R"({"passes":4.0})"));
    f.fsm->dispatch(start_from_parking_side(1.0f));
    f.fsm->dispatch(EvFarEndLimitSettled{});
    f.fsm->dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm->current_state() == "Charging");

    f.fsm->dispatch(EvChargeDone{});

    CHECK(f.tb_cfg->active_config().passes == Approx(1.0));
    CHECK(f.tb_cfg->has_pending_config());
}
```

- [ ] **Step 2: Run the supervisor tests to verify they fail**

Run: `./aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: FAIL if promotion still happens in the wrong place or tests do not reflect the new contract.

- [ ] **Step 3: Keep promotion in `start_task()` and nowhere else**

Use the exact order:

```cpp
bool RobotSupervisor::start_task(bool at_parking_side,
                                 bool position_valid,
                                 float battery_soc) {
    if (!is_new_task_start_state(fsm_->current_state()))
        return false;
    if (!position_valid || !at_parking_side)
        return false;
    if (battery_soc < tb_cfg_->active_config().start_battery_soc)
        return false;

    if (tb_cfg_->has_pending_config())
        tb_cfg_->promote_pending_to_active();

    fsm_->dispatch(EvScheduleStart{true, false, tb_cfg_->active_config().passes});
    return fsm_->current_state() == "CleanFwd";
}
```

Do not move promotion into the FSM, `main`, or `ThingsBoardControlPlane`.

- [ ] **Step 4: Run the supervisor tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: PASS for promotion timing and existing start/stop/return behavior.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: promote pending runtime config only on new task start"
```

---

### Task 4: Remove scattered runtime-config interpretation from `main.cc`

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Test: `test/integration/system_integration_test.cc`
- Test: `test/integration/task_chain_test.cc`

- [ ] **Step 1: Write the failing integration assertions for active-config reads**

Add or update tests to assert runtime decisions come from active config only:

```cpp
TEST_CASE("System start path uses active runtime config after promotion",
          "[integration][system]") {
    SystemFixture f;
    f.apply_pending_runtime_attrs(parse_json(R"({"parking_side":"right","passes":2.0})"));

    REQUIRE(f.supervisor->start_task(true, true, 80.0f));

    CHECK(f.tb_cfg->active_config().parking_side == robot::service::ParkingSide::Right);
    CHECK(f.tb_cfg->active_config().passes == Approx(2.0));
}
```

- [ ] **Step 2: Run task-chain and system tests to verify they fail where `main` or fixtures still read config directly**

Run: `./aarch64/bin/unit_tests "[integration][task_chain],[integration][system]"`

Expected: FAIL if fixtures or integration assembly still bypass the manager contract.

- [ ] **Step 3: Replace direct runtime reads in `main.cc` with manager-backed reads**

Follow this shape:

```cpp
const auto current_active_parking_facts = [&tb_cfg, physical_limit_facts_for]() {
    return physical_limit_facts_for(tb_cfg->active_config().parking_side);
};

const auto current_start_parking_facts = [&tb_cfg, physical_limit_facts_for]() {
    const auto pending = tb_cfg->pending_config();
    const auto parking_side =
        pending ? pending->parking_side : tb_cfg->active_config().parking_side;
    return physical_limit_facts_for(parking_side);
};
```

But remove any remaining direct JSON-path runtime reads from `ConfigService` in `main.cc` after manager construction.

- [ ] **Step 4: Update the integration fixtures to mirror the main-path contract**

In system/task-chain/runtime-mock fixtures:

- build `ThingsBoardConfigManager`
- set `MotionService` parking-side provider from `tb_cfg->active_config().parking_side`
- read runtime values from `active_config()`, not from raw config files

Example:

```cpp
motion->set_parking_side_provider(
    [this]() { return tb_cfg->active_config().parking_side; });
```

- [ ] **Step 5: Run task-chain and system tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[integration][task_chain],[integration][system]"`

Expected: PASS for runtime-config-dependent startup and task-chain behavior.

- [ ] **Step 6: Commit**

```bash
git add pv_cleaning_robot/main.cc \
        test/integration/system_integration_test.cc \
        test/integration/task_chain_test.cc
git commit -m "refactor: route runtime config reads through active manager state"
```

---

### Task 5: Update ThingsBoard control-plane behavior and comments

**Files:**
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`
- Test: `test/integration/thingsboard_runtime_mock_integration_test.cc`
- Test: `test/integration/thingsboard_real_integration_test.cc`

- [ ] **Step 1: Write the failing tests for runtime shared attributes and promotion semantics**

Add assertions like:

```cpp
TEST_CASE("ThingsBoard shared attributes update scheduler immediately but defer runtime task params",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->subscribe_shared_attributes();

    f.mqtt->emit_attributes(R"({"passes":2.0,"schedules":[{"hour":10,"minute":30}]})");

    CHECK(f.tb_cfg->active_config().schedules.size() == 1);
    CHECK(f.tb_cfg->active_config().passes == Approx(1.0));
    REQUIRE(f.tb_cfg->pending_config().has_value());
    CHECK(f.tb_cfg->pending_config()->passes == Approx(2.0));
}
```

- [ ] **Step 2: Run the control-plane and runtime-mock tests to verify they fail**

Run: `./aarch64/bin/unit_tests "[service][tb_control_plane],[integration][thingsboard][runtime_mock]"`

Expected: FAIL where tests still assume a different runtime-config update model.

- [ ] **Step 3: Make the control plane depend only on manager semantics**

In shared-attribute handling:

```cpp
auto result = tb_cfg_->apply_shared_attributes(attrs);
publish_shared_attr_update_event(result);
```

Avoid control-plane-local field classification.

In comments, add concise explanations such as:

```cpp
// Shared attributes never rewrite an active task's runtime parameters directly.
// Scheduler windows are the only immediate runtime update; all other task
// parameters are persisted to pending and promoted only on the next new-task start.
```

- [ ] **Step 4: Update the runtime payload builder comments and keys only where needed**

Keep runtime payloads aligned with active config:

```cpp
writer.Key("start_battery_soc");
writer.Double(config.start_battery_soc);
```

Do not add speculative extra telemetry fields.

- [ ] **Step 5: Run the control-plane and runtime-mock tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[service][tb_control_plane],[integration][thingsboard][runtime_mock]"`

Expected: PASS for:

- shared-attribute runtime deferral
- scheduler immediate effect
- runtime-mock next-task promotion semantics

- [ ] **Step 6: Run the real shared-attribute tests**

Run: `TB_REAL_TEST=1 ./aarch64/bin/unit_tests "[integration][thingsboard][real][shared_attr],[integration][thingsboard][real][shared_attr][reconnect]"`

Expected: PASS with the new active/pending behavior.

- [ ] **Step 7: Commit**

```bash
git add pv_cleaning_robot/service/thingsboard_control_plane.cc \
        pv_cleaning_robot/service/thingsboard_event_payload_builder.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/integration/thingsboard_runtime_mock_integration_test.cc \
        test/integration/thingsboard_real_integration_test.cc
git commit -m "refactor: align thingsboard runtime config behavior and comments"
```

---

### Task 6: Reduce fixture duplication around split config paths

**Files:**
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/system_integration_test.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Write one focused test or assertion proving split-path helpers are reused consistently**

Add a fixture-level assertion or helper test like:

```cpp
TEST_CASE("Runtime fixtures derive fixed/runtime/pending paths consistently",
          "[integration][config_paths]") {
    RuntimePaths paths{"/tmp/example_runtime.json"};
    CHECK(paths.fixed_path.filename() == "example_fixed.json");
    CHECK(paths.pending_path.filename() == "example_runtime.pending.json");
}
```

- [ ] **Step 2: Run the narrow affected tests to verify they fail before helper extraction**

Run: `./aarch64/bin/unit_tests "[app][robot_supervisor],[service][tb_control_plane],[integration][system],[integration][task_chain]"`

Expected: FAIL or compile error if helpers are not yet unified.

- [ ] **Step 3: Extract and reuse the path/helper pattern in test code**

Use one narrow helper style, for example:

```cpp
struct RuntimeConfigPaths {
    std::string fixed_path;
    std::string runtime_path;
    std::string pending_path;
};

RuntimeConfigPaths make_runtime_paths(const std::string& runtime_path);
```

Use this only where duplication is currently real. Do not create a generic utility library for tests.

- [ ] **Step 4: Run the narrow affected tests to verify they pass**

Run: `./aarch64/bin/unit_tests "[app][robot_supervisor],[service][tb_control_plane],[integration][system],[integration][task_chain]"`

Expected: PASS with less duplicated fixture setup.

- [ ] **Step 5: Commit**

```bash
git add test/integration/hardware/hw_config.h \
        test/integration/system_integration_test.cc \
        test/integration/task_chain_test.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: deduplicate runtime config fixture setup"
```

---

### Task 7: Add focused explanatory comments and remove stale local duplication

**Files:**
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Modify: `pv_cleaning_robot/main.cc`

- [ ] **Step 1: Identify the smallest set of non-obvious code paths that need comments**

Add comments only to code paths like:

```cpp
// Scheduler windows are the only runtime shared attributes applied immediately.
// All other runtime values are deferred so an active task never changes its
// direction, speeds, or battery thresholds halfway through execution.
```

and:

```cpp
// The motion direction baseline is defined for parking_side=right.
// When the active parking side is left, MotionService flips walk and brush
// signs so "start_cleaning" still means "depart from parking side".
```

- [ ] **Step 2: Remove any stale duplicate key lists or compatibility branches confirmed unnecessary**

Delete only code that is redundant after the earlier tasks. For example:

```cpp
// delete local duplicated shared-attribute key arrays if the manager already owns them
// delete raw config-path guessing in fixtures once helpers exist
```

Do not merge files or rename broad subsystems here.

- [ ] **Step 3: Run the focused build and ThingsBoard/system regression**

Run: `cmake --build --preset rk3576-build --target unit_tests -- -j4`

Expected: successful build with no newly introduced warnings from this task.

- [ ] **Step 4: Run the final targeted regression suite**

Run:

```bash
./aarch64/bin/unit_tests "[service][config],[service][tb_config],[service][tb_control_plane],[app][robot_supervisor],[integration][task_chain],[integration][system]"
TB_REAL_TEST=1 ./aarch64/bin/unit_tests "[integration][thingsboard][real][shared_attr],[integration][thingsboard][real][shared_attr][reconnect],[integration][thingsboard][real][runtime_mock]"
```

Expected: PASS for the full runtime-config and ThingsBoard regression surface touched by this plan.

- [ ] **Step 5: Commit**

```bash
git add pv_cleaning_robot/service/thingsboard_config_manager.cc \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        pv_cleaning_robot/service/cloud_service.cc \
        pv_cleaning_robot/main.cc
git commit -m "docs: clarify runtime config and thingsboard behavior"
```

---

## Self-Review

### Spec coverage

This plan covers:

- split fixed/runtime/pending file model
- scheduler-immediate vs all-other-pending semantics
- manager-owned runtime semantics
- main-path runtime read cleanup
- test-fixture deduplication
- ThingsBoard comment additions

### Placeholder scan

No `TODO`, `TBD`, or “similar to above” placeholders are left in the task steps. Each code-changing task includes concrete file paths, example code, and commands.

### Type consistency

The plan consistently uses:

- `ConfigService`
- `ThingsBoardConfigManager`
- `active_config()`
- `pending_config()`
- `promote_pending_to_active()`
- `SchedulerService`

The pending-promotion trigger remains `RobotSupervisor::start_task()`, consistent with the approved design.
