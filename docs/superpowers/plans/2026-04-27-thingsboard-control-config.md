# ThingsBoard Control/Config Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the approved ThingsBoard RPC, telemetry, shared-attribute, and client-attribute feature set while fixing the blocking business bugs in the existing runtime.

**Architecture:** Introduce a thin ThingsBoard business layer on top of the existing transport stack. Keep `CloudService` transport-focused, move command semantics into a new `CommandOrchestrator`, move shared-attribute validation/persistence into a new `ThingsBoardConfigManager`, and extend `RobotFsm` plus a new `TaskRuntimeContext` to represent pause/terminate/resume behavior correctly.

**Tech Stack:** C++17, nlohmann/json, Catch2, Boost.SML, existing `ThreadExecutor`, `CloudService`, `ConfigService`, `SchedulerService`, `HealthService`

---

## File Structure

### New files

- `include/pv_cleaning_robot/service/task_runtime_context.h`
  - task-scoped runtime state shared by FSM, orchestrator, and telemetry
- `include/pv_cleaning_robot/service/command_tracker.h`
  - active/last-completed command tracking model
- `pv_cleaning_robot/service/command_tracker.cc`
  - thread-safe command tracking implementation
- `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
  - shared-attribute parsing, validation, active/pending/backup persistence
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
  - manager implementation
- `include/pv_cleaning_robot/service/command_orchestrator.h`
  - RPC business semantics and admissibility checks
- `pv_cleaning_robot/service/command_orchestrator.cc`
  - orchestrator implementation
- `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
  - DTO for ThingsBoard-facing business telemetry
- `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
  - snapshot assembly helpers
- `test/service/thingsboard_config_manager_test.cc`
  - validation, staging, backup, rollback tests
- `test/service/command_tracker_test.cc`
  - active/last-command state tests
- `test/service/command_orchestrator_test.cc`
  - RPC acceptance/rejection and workflow tests
- `test/service/business_telemetry_snapshot_test.cc`
  - business state and cadence tests

### Modified files

- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `include/pv_cleaning_robot/service/motion_service.h`
- `pv_cleaning_robot/service/motion_service.cc`
- `include/pv_cleaning_robot/app/fault_handler.h`
- `pv_cleaning_robot/app/fault_handler.cc`
- `include/pv_cleaning_robot/service/config_service.h`
- `pv_cleaning_robot/service/config_service.cc`
- `include/pv_cleaning_robot/service/scheduler_service.h`
- `pv_cleaning_robot/service/scheduler_service.cc`
- `include/pv_cleaning_robot/service/health_service.h`
- `pv_cleaning_robot/service/health_service.cc`
- `include/pv_cleaning_robot/service/cloud_service.h`
- `pv_cleaning_robot/service/cloud_service.cc`
- `include/pv_cleaning_robot/middleware/data_cache.h`
- `pv_cleaning_robot/middleware/data_cache.cc`
- `pv_cleaning_robot/main.cc`
- `pv_cleaning_robot/CMakeLists.txt`
- `test/CMakeLists.txt`
- `test/app/robot_fsm_test.cc`
- `test/app/fault_handler_test.cc`
- `test/middleware/data_cache_test.cc`
- `test/service/config_service_test.cc`
- `test/service/cloud_service_test.cc`
- `test/service/motion_service_test.cc`
- `test/service/scheduler_service_test.cc`
- `test/integration/task_chain_test.cc`
- `test/integration/system_integration_test.cc`

### Decomposition notes

- Keep the new ThingsBoard business classes in `service/`, not `app/`, because they coordinate transport-facing business rules rather than low-level motion primitives.
- Keep `RobotFsm` focused on motion/task state transitions, not persistence or command response formatting.
- Keep telemetry assembly separate from `HealthService` so cadence and business fields can evolve without folding more policy into the health collector.

### Build/test conventions

- Build unit tests: `cmake --build build --target unit_tests -j4`
- Run focused Catch2 tags: `./build/aarch64/bin/unit_tests "[tag]"`
- Run integration tests: `./build/aarch64/bin/unit_tests "[integration][...]" -s`

### Task 1: Fix Offline Cache Consistency First

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h`
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
- Test: `test/middleware/data_cache_test.cc`

- [ ] **Step 1: Write the failing test**

```cpp
TEST_CASE("DataCache: full-queue eviction does not drop a record when ack journal append fails",
          "[middleware][data_cache]") {
    const std::string path = "/tmp/data_cache_eviction_failure.jsonl";
    std::filesystem::remove(path);

    robot::middleware::DataCache cache(path, 2);
    REQUIRE(cache.open());
    REQUIRE(cache.push("t/a", R"({"v":1})"));
    REQUIRE(cache.push("t/b", R"({"v":2})"));

    // Inject a failure on the eviction ack append path.
    // The implementation change in this task adds a small file-writer seam for tests.
    cache.set_test_append_hook([](const nlohmann::json& j) {
        return j.value("op", "") != "ack";
    });

    REQUIRE_FALSE(cache.push("t/c", R"({"v":3})"));
    CHECK(cache.size() == 2);
    const auto batch = cache.pop_batch(10);
    CHECK(batch.size() == 2);
    CHECK(batch[0].payload == R"({"v":1})");
    CHECK(batch[1].payload == R"({"v":2})");
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[middleware][data_cache]" -s`
Expected: FAIL because `DataCache` has no append hook seam and the queue is mutated before persistence is guaranteed.

- [ ] **Step 3: Write minimal implementation**

```cpp
// include/pv_cleaning_robot/middleware/data_cache.h
using AppendHook = std::function<bool(const nlohmann::json&)>;
void set_test_append_hook(AppendHook hook);

// pv_cleaning_robot/middleware/data_cache.cc
if (queue_.size() >= max_rows_ && !queue_.empty()) {
    const Record dropped = queue_.front();
    if (!append_ack_record_locked(dropped.id)) {
        spdlog::warn("[DataCache] eviction ack append failed: {}", dropped.id);
        return false;
    }
    queue_.pop_front();
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./build/aarch64/bin/unit_tests "[middleware][data_cache]" -s`
Expected: PASS, with the queue unchanged when the persistence step fails.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/middleware/data_cache.h \
        pv_cleaning_robot/middleware/data_cache.cc \
        test/middleware/data_cache_test.cc
git commit -m "fix: preserve cache consistency on eviction failure"
```

### Task 2: Split Motion Stop Semantics And Fix P1 Brush Behavior

**Files:**
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `include/pv_cleaning_robot/app/fault_handler.h`
- Modify: `pv_cleaning_robot/app/fault_handler.cc`
- Test: `test/service/motion_service_test.cc`
- Test: `test/app/fault_handler_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("MotionService: pause keeps drivetrain enabled and zeros speed",
          "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion->start_cleaning());

    f.motion->pause_task();

    CHECK(f.group->disable_all_called == false);
    CHECK(f.group->last_uniform_speed == Approx(0.0f));
}

TEST_CASE("FaultHandler: P1 path does not reverse the brush before FSM handoff",
          "[app][fault_handler]") {
    FaultHandlerFixture f;

    f.handler.on_fault({Level::P1, 0x2001, "p1", 0});

    CHECK(f.motion_spy.start_returning_called == false);
    CHECK(f.motion_spy.start_returning_no_brush_called == true);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][motion],[app][fault_handler]" -s`
Expected: FAIL because `pause_task()` does not exist and `FaultHandler` currently calls `start_returning()`.

- [ ] **Step 3: Write minimal implementation**

```cpp
// include/pv_cleaning_robot/service/motion_service.h
void pause_task();
bool resume_forward_task();
bool resume_return_task(bool with_brush);

// pv_cleaning_robot/service/motion_service.cc
void MotionService::pause_task() {
    brush_->stop();
    group_->enable_heading_control(false);
    group_->set_speed_uniform(0.0f);
}

// pv_cleaning_robot/app/fault_handler.cc
case Level::P1:
    motion_->pause_task();
    dispatch_fn_(evt);
    break;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[service][motion],[app][fault_handler]" -s`
Expected: PASS, with no brush reverse on the P1 path and a separate pause primitive available.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/motion_service.h \
        pv_cleaning_robot/service/motion_service.cc \
        include/pv_cleaning_robot/app/fault_handler.h \
        pv_cleaning_robot/app/fault_handler.cc \
        test/service/motion_service_test.cc \
        test/app/fault_handler_test.cc
git commit -m "fix: separate pause semantics from fault return"
```

### Task 3: Harden And Extend RobotFsm

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Create: `include/pv_cleaning_robot/service/task_runtime_context.h`
- Test: `test/app/robot_fsm_test.cc`
- Test: `test/integration/task_chain_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("FSM: EvFaultReset outside Fault does not force Idle",
          "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.fsm.dispatch(EvFaultReset{});
    CHECK(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("FSM: stop transitions CleanFwd to Paused",
          "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});

    f.fsm.dispatch(EvPauseTask{});
    CHECK(f.fsm.current_state() == "Paused");
}

TEST_CASE("FSM: terminate transitions CleanReturn to Terminated",
          "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFrontLimitSettled{});

    f.fsm.dispatch(EvTerminateTask{});
    CHECK(f.fsm.current_state() == "Terminated");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[app][fsm],[integration][task_chain]" -s`
Expected: FAIL because `EvPauseTask`/`EvTerminateTask` do not exist and illegal reset currently forces `Idle`.

- [ ] **Step 3: Write minimal implementation**

```cpp
// include/pv_cleaning_robot/app/robot_fsm.h
struct StatePaused {};
struct StateTerminated {};
struct EvPauseTask {};
struct EvResumeTask {};
struct EvManualReturn {};
struct EvTerminateTask {};
struct EvResetReady {};

bool can_start_new_task() const;
bool can_resume_paused_task() const;

// pv_cleaning_robot/app/robot_fsm.cc
if (!sm_->process_event(e)) {
    spdlog::warn("[FSM] ignored event in state {}", state_name_);
    return;
}
state_name_ = "Paused";
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[app][fsm],[integration][task_chain]" -s`
Expected: PASS, with illegal events ignored and new paused/terminated states available.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc \
        include/pv_cleaning_robot/service/task_runtime_context.h \
        test/app/robot_fsm_test.cc \
        test/integration/task_chain_test.cc
git commit -m "feat: extend FSM for paused and terminated task states"
```

### Task 4: Add Command Tracking Primitives

**Files:**
- Create: `include/pv_cleaning_robot/service/command_tracker.h`
- Create: `pv_cleaning_robot/service/command_tracker.cc`
- Test: `test/service/command_tracker_test.cc`
- Modify: `pv_cleaning_robot/CMakeLists.txt`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

```cpp
TEST_CASE("CommandTracker: moves active command into last completed on success",
          "[service][command_tracker]") {
    robot::service::CommandTracker tracker;

    const auto id = tracker.accept("start", "rpc-001");
    auto active = tracker.active();
    REQUIRE(active.has_value());
    CHECK(active->name == "start");
    CHECK(active->phase == CommandPhase::Accepted);

    tracker.mark_running(id);
    tracker.finish_success(id, "completed");

    CHECK_FALSE(tracker.active().has_value());
    auto last = tracker.last_completed();
    REQUIRE(last.has_value());
    CHECK(last->name == "start");
    CHECK(last->phase == CommandPhase::Succeeded);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][command_tracker]" -s`
Expected: FAIL because `CommandTracker` does not exist.

- [ ] **Step 3: Write minimal implementation**

```cpp
enum class CommandPhase { Accepted, Running, Succeeded, Failed, Rejected };

struct CommandSnapshot {
    std::string id;
    std::string name;
    std::string request_id;
    CommandPhase phase;
    std::string reason;
    uint64_t accepted_at_ms{0};
    uint64_t finished_at_ms{0};
};
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./build/aarch64/bin/unit_tests "[service][command_tracker]" -s`
Expected: PASS, with active and last-completed command snapshots behaving deterministically.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/command_tracker.h \
        pv_cleaning_robot/service/command_tracker.cc \
        test/service/command_tracker_test.cc \
        pv_cleaning_robot/CMakeLists.txt \
        test/CMakeLists.txt
git commit -m "feat: add command tracking primitives"
```

### Task 5: Build ThingsBoardConfigManager

**Files:**
- Create: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Create: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Modify: `include/pv_cleaning_robot/service/config_service.h`
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Test: `test/service/thingsboard_config_manager_test.cc`
- Test: `test/service/config_service_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("ThingsBoardConfigManager: invalid speed rejects whole update",
          "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager.active_config();

    const nlohmann::json attrs{
        {"walk_speed_percent", 30},
        {"brush_speed_percent", 101}
    };

    const auto result = f.manager.apply_shared_attributes(attrs);
    CHECK(result.accepted == false);
    CHECK(f.manager.active_config() == before);
}

TEST_CASE("ThingsBoardConfigManager: schedule applies immediately, passes stay pending",
          "[service][tb_config]") {
    Fixture f;
    const nlohmann::json attrs{
        {"schedules", {{{"hour", 7}, {"minute", 30}}}},
        {"passes", 2}
    };

    const auto result = f.manager.apply_shared_attributes(attrs);
    REQUIRE(result.accepted);
    CHECK(f.scheduler_snapshot().size() == 1);
    CHECK(f.manager.pending_config().passes == 2.0f);
    CHECK(f.manager.active_config().passes != 2.0f);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][tb_config],[service][config]" -s`
Expected: FAIL because `ThingsBoardConfigManager` and staged config behavior do not exist.

- [ ] **Step 3: Write minimal implementation**

```cpp
struct TbRuntimeConfig {
    std::string parking_side{"right"};
    double passes{1.0};
    std::vector<ScheduleEntry> schedules;
    int low_battery_threshold{15};
    int temperature_threshold{60};
    int overcurrent_threshold{0};
    int task_timeout_sec{3600};
    int walk_speed_percent{100};
    int brush_speed_percent{100};
    uint64_t version{0};
};

struct SharedAttrApplyResult {
    bool accepted{false};
    std::string reason;
};
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[service][tb_config],[service][config]" -s`
Expected: PASS, with whole-update rejection and immediate-vs-pending behavior implemented.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_config_manager.h \
        pv_cleaning_robot/service/thingsboard_config_manager.cc \
        include/pv_cleaning_robot/service/config_service.h \
        pv_cleaning_robot/service/config_service.cc \
        test/service/thingsboard_config_manager_test.cc \
        test/service/config_service_test.cc
git commit -m "feat: add staged ThingsBoard config manager"
```

### Task 6: Update Scheduler For Immediate Schedule Replacement

**Files:**
- Modify: `include/pv_cleaning_robot/service/scheduler_service.h`
- Modify: `pv_cleaning_robot/service/scheduler_service.cc`
- Test: `test/service/scheduler_service_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("SchedulerService: replace_windows swaps the full active schedule atomically",
          "[service][scheduler]") {
    SchedulerService svc;
    svc.replace_windows({{8, 0}, {14, 30}});
    CHECK(svc.window_count_for_test() == 2);

    svc.replace_windows({{9, 15}});
    CHECK(svc.window_count_for_test() == 1);
}

TEST_CASE("SchedulerService: paused task flag suppresses auto-restart",
          "[service][scheduler]") {
    SchedulerService svc;
    svc.set_task_blocked(true);
    svc.replace_windows({local_now_window()});

    int count = 0;
    svc.set_on_task_start([&] { ++count; });
    svc.tick();

    CHECK(count == 0);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][scheduler]" -s`
Expected: FAIL because `replace_windows()` and task-block suppression do not exist.

- [ ] **Step 3: Write minimal implementation**

```cpp
void replace_windows(std::vector<TimeWindow> windows);
void set_task_blocked(bool blocked);

if (task_blocked_) {
    in_window_ = in_window_now;
    return;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[service][scheduler]" -s`
Expected: PASS, with immediate full replacement and no scheduler-driven resume while paused.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/scheduler_service.h \
        pv_cleaning_robot/service/scheduler_service.cc \
        test/service/scheduler_service_test.cc
git commit -m "feat: support atomic schedule replacement and pause blocking"
```

### Task 7: Add CommandOrchestrator And Replace Direct RPC Lambdas

**Files:**
- Create: `include/pv_cleaning_robot/service/command_orchestrator.h`
- Create: `pv_cleaning_robot/service/command_orchestrator.cc`
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Test: `test/service/command_orchestrator_test.cc`
- Test: `test/service/cloud_service_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("CommandOrchestrator: start from Idle away from home is rejected",
          "[service][command_orch]") {
    Fixture f;
    f.runtime.set_state("Idle");
    f.runtime.set_at_home(false);

    const auto response = f.orchestrator.handle_start("rpc-100");
    CHECK(response.accepted == false);
    CHECK(response.reason == "not_at_home_for_new_task");
}

TEST_CASE("CommandOrchestrator: stop from CleanFwd transitions into paused",
          "[service][command_orch]") {
    Fixture f;
    f.runtime.set_state("CleanFwd");

    const auto response = f.orchestrator.handle_stop("rpc-101");
    REQUIRE(response.accepted);
    CHECK(f.fsm_spy.last_event == "EvPauseTask");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][command_orch],[service][cloud]" -s`
Expected: FAIL because there is no orchestrator and RPC logic still lives in `main.cc`.

- [ ] **Step 3: Write minimal implementation**

```cpp
struct CommandResponse {
    bool accepted{false};
    std::string command_id;
    std::string reason;
};

CommandResponse CommandOrchestrator::handle_start(const std::string& request_id);
CommandResponse CommandOrchestrator::handle_stop(const std::string& request_id);
CommandResponse CommandOrchestrator::handle_return(const std::string& request_id);
CommandResponse CommandOrchestrator::handle_terminate(const std::string& request_id);
CommandResponse CommandOrchestrator::handle_reset(const std::string& request_id);
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[service][command_orch],[service][cloud]" -s`
Expected: PASS, with immediate RPC acceptance/rejection behavior moved out of `main.cc`.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/command_orchestrator.h \
        pv_cleaning_robot/service/command_orchestrator.cc \
        include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc \
        pv_cleaning_robot/main.cc \
        test/service/command_orchestrator_test.cc \
        test/service/cloud_service_test.cc
git commit -m "feat: add command orchestrator for ThingsBoard RPC"
```

### Task 8: Add Business Telemetry Snapshot And State-Aware Cadence

**Files:**
- Create: `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
- Create: `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
- Modify: `include/pv_cleaning_robot/service/health_service.h`
- Modify: `pv_cleaning_robot/service/health_service.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Test: `test/service/business_telemetry_snapshot_test.cc`
- Test: `test/integration/system_integration_test.cc`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("BusinessTelemetry: active command and last command are serialized",
          "[service][business_telemetry]") {
    robot::service::BusinessTelemetrySnapshot snap;
    snap.device_state = "Paused";
    snap.active_command = CommandSnapshot{.id = "cmd-1", .name = "stop", .phase = CommandPhase::Running};
    snap.last_command = CommandSnapshot{.id = "cmd-0", .name = "start", .phase = CommandPhase::Succeeded};

    const auto j = snap.to_json();
    CHECK(j.at("device_state").get<std::string>() == "Paused");
    CHECK(j.at("active_command").at("name").get<std::string>() == "stop");
    CHECK(j.at("last_command").at("name").get<std::string>() == "start");
}

TEST_CASE("Health cadence: active task reports at 1s and idle reports at 5min",
          "[integration][system][telemetry_cadence]") {
    Fixture f;
    f.runtime.set_has_active_task(true);
    CHECK(f.report_policy.next_interval_ms() == 1000);

    f.runtime.set_has_active_task(false);
    CHECK(f.report_policy.next_interval_ms() == 300000);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][business_telemetry],[integration][system][telemetry_cadence]" -s`
Expected: FAIL because business telemetry and cadence policy do not exist.

- [ ] **Step 3: Write minimal implementation**

```cpp
struct BusinessTelemetrySnapshot {
    std::string device_state;
    std::string task_state;
    int target_half_passes{0};
    int completed_half_passes{0};
    uint64_t active_config_version{0};
    std::optional<CommandSnapshot> active_command;
    std::optional<CommandSnapshot> last_command;

    nlohmann::json to_json() const;
};
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/aarch64/bin/unit_tests "[service][business_telemetry],[integration][system][telemetry_cadence]" -s`
Expected: PASS, with business fields present and interval policy switching between 1 second and 5 minutes.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/business_telemetry_snapshot.h \
        pv_cleaning_robot/service/business_telemetry_snapshot.cc \
        include/pv_cleaning_robot/service/health_service.h \
        pv_cleaning_robot/service/health_service.cc \
        pv_cleaning_robot/main.cc \
        test/service/business_telemetry_snapshot_test.cc \
        test/integration/system_integration_test.cc
git commit -m "feat: add business telemetry snapshot and cadence policy"
```

### Task 9: Wire Startup, Reset, And End-To-End ThingsBoard Flow

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/integration/system_integration_test.cc`
- Modify: `test/app/robot_fsm_test.cc`
- Modify: `test/service/thingsboard_config_manager_test.cc`

- [ ] **Step 1: Write the failing integration tests**

```cpp
TEST_CASE("System: terminate requires home plus reset before returning to idle",
          "[integration][system][terminate_reset]") {
    Fixture f;
    f.start_task_from_home();

    REQUIRE(f.rpc("terminate").accepted);
    CHECK(f.fsm.current_state() == "Terminated");

    auto reset_away = f.rpc("reset");
    CHECK(reset_away.accepted == false);

    f.set_at_home(true);
    auto reset_home = f.rpc("reset");
    CHECK(reset_home.accepted == true);
    CHECK(f.fsm.current_state() == "Idle");
}

TEST_CASE("System: schedule applies immediately while passes wait for next task",
          "[integration][system][shared_attributes]") {
    Fixture f;
    f.apply_shared_attrs({{"schedules", {{{"hour", 9}, {"minute", 0}}}}, {"passes", 2}});

    CHECK(f.scheduler_window_count() == 1);
    CHECK(f.active_passes() == 1.0);
    CHECK(f.pending_passes() == 2.0);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[integration][system],[integration][task_chain]" -s`
Expected: FAIL because the reset preconditions, staged config promotion, and new RPC semantics are not fully wired.

- [ ] **Step 3: Write minimal implementation**

```cpp
// pv_cleaning_robot/main.cc
auto tb_cfg = std::make_shared<robot::service::ThingsBoardConfigManager>(...);
auto tracker = std::make_shared<robot::service::CommandTracker>();
auto orchestrator = std::make_shared<robot::service::CommandOrchestrator>(...);

cloud->register_rpc("start", [&](const std::string& params) {
    return orchestrator->handle_start(params).to_json().dump();
});
cloud->subscribe_shared_attributes([&](const nlohmann::json& attrs) {
    tb_cfg->apply_shared_attributes(attrs);
});
```

- [ ] **Step 4: Run full verification**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[app],[service],[middleware],[integration]" -s`
Expected: PASS, with the new end-to-end ThingsBoard behavior and the pre-existing blocking bugs covered by tests.

- [ ] **Step 5: Commit**

```bash
git add pv_cleaning_robot/main.cc \
        test/integration/task_chain_test.cc \
        test/integration/system_integration_test.cc \
        test/app/robot_fsm_test.cc \
        test/service/thingsboard_config_manager_test.cc
git commit -m "feat: wire ThingsBoard control and config runtime"
```

## Self-Review

### Spec coverage

- RPC semantics: covered by Tasks 3, 4, 7, and 9.
- Shared attribute validation and staging: covered by Tasks 5 and 9.
- Client attributes and business telemetry: covered by Task 8 and final wiring in Task 9.
- Pre-implementation bug fixes: covered by Tasks 1, 2, and 3.
- Scheduler behavior and immediate schedule application: covered by Task 6.
- Reset preconditions and terminated flow: covered by Tasks 7 and 9.

No spec section is left without at least one implementing task.

### Placeholder scan

- No `TODO`, `TBD`, or “implement later” placeholders remain.
- Each task contains file paths, test code, implementation code, run commands, and commit commands.

### Type consistency

- `CommandTracker` defines `CommandPhase` and `CommandSnapshot`, reused by orchestrator and telemetry tasks.
- `ThingsBoardConfigManager` owns `active` and `pending` config layers consistently across Tasks 5 and 9.
- `EvPauseTask`, `EvTerminateTask`, and `TaskRuntimeContext` are introduced before later tasks depend on them.

