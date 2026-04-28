# Runtime Structure Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Introduce `RobotSupervisor`, reroute runtime policy through it, and remove the dead `CleanTask` model without changing approved robot behavior.

**Architecture:** Phase 1 keeps the existing `RobotFsm`, `ThingsBoardConfigManager`, `SchedulerService`, and `ThingsBoardRuntime`, but inserts a single business orchestrator above them. `main.cc` becomes a thinner bootstrap/wiring file, while `RobotSupervisor` owns task admission, safety-policy routing, and cloud cadence decisions.

**Tech Stack:** C++17, Catch2, CMake, existing `RobotFsm`/`ThingsBoardRuntime`/`SchedulerService` stack

---

## File Structure

**Create**
- `include/pv_cleaning_robot/app/robot_supervisor.h`
  Runtime orchestration interface for schedule start, RPC control, safety tick, cadence policy, and telemetry-facing snapshot access.
- `pv_cleaning_robot/app/robot_supervisor.cc`
  Minimal implementation that delegates to `RobotFsm`, `ThingsBoardConfigManager`, and `FaultService`.
- `test/app/robot_supervisor_test.cc`
  Focused unit tests for admission, control routing, safety routing, and cadence policy.

**Delete**
- `include/pv_cleaning_robot/app/clean_task.h`
- `pv_cleaning_robot/app/clean_task.cc`

**Modify**
- `pv_cleaning_robot/main.cc`
  Remove `CleanTask`, route schedule start and main-loop policy through `RobotSupervisor`, keep bootstrap/thread wiring only.
- `include/pv_cleaning_robot/service/thingsboard_runtime.h`
  Replace direct FSM-oriented RPC position callbacks with supervisor-oriented control entrypoints.
- `pv_cleaning_robot/service/thingsboard_runtime.cc`
  Delegate RPC semantics to `RobotSupervisor` instead of owning runtime business decisions locally.
- `include/pv_cleaning_robot/service/scheduler_service.h`
  Tighten comments and callback naming so it is explicitly a time trigger.
- `pv_cleaning_robot/service/scheduler_service.cc`
  Keep behavior stable, only align terminology and callback handling to the narrowed role.
- `test/integration/task_chain_test.cc`
  Update schedule-trigger integration to drive `RobotSupervisor` instead of dispatching FSM events inline.
- `test/service/thingsboard_runtime_test.cc`
  Update fixture wiring so RPC tests assert delegation through `RobotSupervisor`.
- `test/CMakeLists.txt`
  Add `robot_supervisor` sources/tests and remove `clean_task` references.

## Task 1: Add `RobotSupervisor` Shell And Failing Tests

**Files:**
- Create: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Create: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_CASE("RobotSupervisor rejects schedule start when robot is not at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_scheduled_task(false, false));
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("RobotSupervisor promotes pending config before scheduled task start",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.apply_pending_config_with_passes(3);

    REQUIRE(f.supervisor->start_scheduled_task(true, false));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE(f.tb_cfg->active_config().passes == 3);
}

TEST_CASE("RobotSupervisor resumes paused task without requiring home position",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm.dispatch(EvPauseTask{});

    REQUIRE(f.supervisor->resume_paused_task());
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor reports active cadence for running states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.supervisor->desired_cloud_period_ms(1000, 300000) == 1000);
}
```

- [ ] **Step 2: Build tests to verify they fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- build fails with missing `robot_supervisor` header/symbols referenced by the new test

- [ ] **Step 3: Add the minimal `RobotSupervisor` interface and stub implementation**

```cpp
class RobotSupervisor {
public:
    RobotSupervisor(std::shared_ptr<RobotFsm> fsm,
                    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg,
                    std::shared_ptr<service::FaultService> fault,
                    std::shared_ptr<service::NavService> nav);

    bool start_scheduled_task(bool at_home, bool at_front);
    bool start_manual_task(bool at_home, bool at_front);
    bool resume_paused_task();
    bool pause_task();
    bool return_task();
    bool terminate_task();
    bool reset_task(bool at_home);
    void tick_safety(bool low_battery);
    int desired_cloud_period_ms(int active_ms, int idle_ms) const;

private:
    std::shared_ptr<RobotFsm> fsm_;
    std::shared_ptr<service::ThingsBoardConfigManager> tb_cfg_;
    std::shared_ptr<service::FaultService> fault_;
    std::shared_ptr<service::NavService> nav_;
};
```

```cpp
bool RobotSupervisor::start_scheduled_task(bool at_home, bool at_front) {
    (void)at_home;
    (void)at_front;
    return false;
}
```

- [ ] **Step 4: Build tests to verify the shell compiles and behavior tests still fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- the new tests would fail at runtime because the stub returns the wrong values

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc \
        test/CMakeLists.txt
git commit -m "test: add robot supervisor scaffolding"
```

## Task 2: Implement Schedule Start And Manual Start Through `RobotSupervisor`

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Extend tests for manual-start semantics and pending promotion failure**

```cpp
TEST_CASE("RobotSupervisor rejects manual start from idle when robot is not at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->start_manual_task(false, false));
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("RobotSupervisor starts manual task from charging when robot is at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.force_state_for_test("Charging");

    REQUIRE(f.supervisor->start_manual_task(true, false));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("RobotSupervisor rejects start when pending config promotion fails",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.install_invalid_pending_config_for_test();

    REQUIRE_FALSE(f.supervisor->start_scheduled_task(true, false));
    REQUIRE(f.fsm.current_state() == "Idle");
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- runtime would fail because `start_manual_task()` and promotion handling are not implemented yet

- [ ] **Step 3: Implement start admission helpers and config promotion**

```cpp
namespace {
bool is_new_task_start_state(const std::string& state) {
    return state == "Idle" || state == "Charging";
}
}

bool RobotSupervisor::start_scheduled_task(bool at_home, bool at_front) {
    if (!is_new_task_start_state(fsm_->current_state()) || !at_home)
        return false;
    if (tb_cfg_->has_pending_config() && !tb_cfg_->promote_pending_to_active())
        return false;
    fsm_->dispatch(EvScheduleStart{
        at_home, at_front, static_cast<float>(tb_cfg_->active_config().passes)});
    return fsm_->current_state() == "CleanFwd";
}

bool RobotSupervisor::start_manual_task(bool at_home, bool at_front) {
    return start_scheduled_task(at_home, at_front);
}

bool RobotSupervisor::resume_paused_task() {
    if (fsm_->current_state() != "Paused")
        return false;
    fsm_->dispatch(EvResumeTask{});
    return fsm_->current_state() == "CleanFwd" ||
           fsm_->current_state() == "CleanReturn";
}
```

- [ ] **Step 4: Build to verify the task compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds with `RobotSupervisor` providing schedule/manual start behavior

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc
git commit -m "feat: route task start admission through supervisor"
```

## Task 3: Move RPC Control Semantics From `ThingsBoardRuntime` To `RobotSupervisor`

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_runtime.h`
- Modify: `pv_cleaning_robot/service/thingsboard_runtime.cc`
- Test: `test/app/robot_supervisor_test.cc`
- Test: `test/service/thingsboard_runtime_test.cc`

- [ ] **Step 1: Add failing tests for pause/return/terminate/reset routing**

```cpp
TEST_CASE("RobotSupervisor pauses only from cleaning states", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->pause_task());

    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.supervisor->pause_task());
    REQUIRE(f.fsm.current_state() == "Paused");
}

TEST_CASE("RobotSupervisor returns from paused or cleaning states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.supervisor->return_task());
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("RobotSupervisor resets only from fault or terminated at home",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE_FALSE(f.supervisor->reset_task(true));
}
```

```cpp
TEST_CASE("ThingsBoardRuntime start RPC delegates new-task start to supervisor",
          "[service][thingsboard_runtime]") {
    RuntimeFixture f;
    f.runtime->register_rpc_handlers(
        []() { return true; },
        []() { return false; });

    const auto reply = f.invoke_rpc("start", "{}");
    REQUIRE(reply.contains("\"accepted\":true"));
    REQUIRE(f.supervisor->last_call() == "start_manual_task");
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails until `ThingsBoardRuntime` constructor and test fixtures are updated for supervisor injection

- [ ] **Step 3: Inject `RobotSupervisor` into `ThingsBoardRuntime` and delegate RPC actions**

```cpp
class ThingsBoardRuntime {
public:
    ThingsBoardRuntime(ConfigService& config,
                       std::shared_ptr<CloudService> cloud,
                       std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
                       std::shared_ptr<CommandTracker> command_tracker,
                       std::shared_ptr<app::RobotFsm> fsm,
                       std::shared_ptr<app::RobotSupervisor> supervisor);

    void register_rpc_handlers(const std::function<bool()>& is_at_home,
                               const std::function<bool()>& is_at_front);
};
```

```cpp
cloud_->register_rpc("stop", [this](const std::string&) {
    if (!supervisor_->pause_task()) {
        command_tracker_->reject("stop", "", "stop_not_allowed_in_current_state");
        publish_command_event("command_rejected", *command_tracker_->last_completed());
        return rpc_reply(false, "stop_not_allowed_in_current_state");
    }
    const auto cmd_id = command_tracker_->accept("stop", "");
    publish_command_event("command_accepted", *command_tracker_->active());
    command_tracker_->mark_running(cmd_id);
    command_tracker_->finish_success(cmd_id, "paused_task");
    publish_command_event("command_completed", *command_tracker_->last_completed());
    return rpc_reply(true);
});
```

- [ ] **Step 4: Build to verify the refactor compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- `ThingsBoardRuntime` no longer owns direct state-admission rules beyond command tracking and replies

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        include/pv_cleaning_robot/service/thingsboard_runtime.h \
        pv_cleaning_robot/service/thingsboard_runtime.cc \
        test/app/robot_supervisor_test.cc \
        test/service/thingsboard_runtime_test.cc
git commit -m "refactor: move rpc task control into supervisor"
```

## Task 4: Move Schedule Trigger And Main-Loop Safety Policy Into `RobotSupervisor`

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `include/pv_cleaning_robot/service/scheduler_service.h`
- Modify: `pv_cleaning_robot/service/scheduler_service.cc`
- Test: `test/app/robot_supervisor_test.cc`
- Test: `test/integration/task_chain_test.cc`

- [ ] **Step 1: Add failing tests for low-battery and spin-free routing**

```cpp
TEST_CASE("RobotSupervisor low battery triggers returning from active task",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});

    f.supervisor->tick_safety(true);
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("RobotSupervisor spin-free reports P0 fault outside idle states",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    f.nav->set_spin_free_for_test(true);

    f.supervisor->tick_safety(false);
    REQUIRE(f.fault_events.size() == 1);
    REQUIRE(f.fault_events[0].code == 0x0002);
}
```

```cpp
TEST_CASE("TaskChain: Scheduler trigger delegates through supervisor",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.scheduler.set_on_task_start([&] { f.supervisor->start_scheduled_task(true, false); });
    f.scheduler.tick();
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}
```

- [ ] **Step 2: Build to verify the new tests fail**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails until integration fixtures construct `RobotSupervisor`

- [ ] **Step 3: Implement `tick_safety()` and reroute `main.cc`**

```cpp
void RobotSupervisor::tick_safety(bool low_battery) {
    const auto state = fsm_->current_state();
    const bool active = state == "CleanFwd" || state == "CleanReturn" ||
                        state == "Returning" || state == "Paused";
    if (low_battery && state != "Idle" && state != "Paused" &&
        state != "Returning" && state != "Charging" &&
        state != "Fault" && state != "Terminated") {
        fsm_->dispatch(EvLowBattery{});
    }
    if (active && nav_->get_pose().spin_free_detected) {
        fault_->report(service::FaultService::FaultEvent::Level::P0,
                       0x0002,
                       "wheel spin-free detected");
        nav_->clear_spin_detection();
    }
}

int RobotSupervisor::desired_cloud_period_ms(int active_ms, int idle_ms) const {
    const auto state = fsm_->current_state();
    return (state == "CleanFwd" || state == "CleanReturn" ||
            state == "Returning" || state == "Paused")
               ? active_ms
               : idle_ms;
}
```

```cpp
scheduler.set_on_task_start([supervisor, &rear_switch, &front_switch, &rear_open_ok, &front_open_ok] {
    const bool at_home = rear_open_ok && !rear_switch->read_current_level();
    const bool at_front = front_open_ok && !front_switch->read_current_level();
    supervisor->start_scheduled_task(at_home, at_front);
});

const int desired_report_period =
    supervisor->desired_cloud_period_ms(active_report_period, idle_report_period);
if (cloud_exec.period_ms() != desired_report_period)
    cloud_exec.set_period_ms(desired_report_period);

supervisor->tick_safety(bms->is_low_battery());
```

- [ ] **Step 4: Build to verify phase wiring compiles**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- `main.cc` no longer contains inline schedule-start admission or inline safety-policy branches

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        pv_cleaning_robot/main.cc \
        include/pv_cleaning_robot/service/scheduler_service.h \
        pv_cleaning_robot/service/scheduler_service.cc \
        test/app/robot_supervisor_test.cc \
        test/integration/task_chain_test.cc
git commit -m "refactor: move runtime policy into supervisor"
```

## Task 5: Remove `CleanTask` And Update Build/Test Graph

**Files:**
- Delete: `include/pv_cleaning_robot/app/clean_task.h`
- Delete: `pv_cleaning_robot/app/clean_task.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/CMakeLists.txt`
- Modify: any test files still including `clean_task.h`

- [ ] **Step 1: Add a compile guard by removing the last product reference**

```cpp
// Delete from main.cc:
robot::app::CleanTask::Config task_cfg;
task_cfg.track_length_m = cfg.get<float>("robot.track_length_m", 1000.0f);
task_cfg.passes = static_cast<float>(tb_cfg->active_config().passes);
auto clean_task = std::make_shared<robot::app::CleanTask>(motion, nav, task_cfg);
```

- [ ] **Step 2: Build to verify the tree still references `CleanTask` where expected**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile fails at any remaining include or source reference to `CleanTask`

- [ ] **Step 3: Remove `CleanTask` sources from the repository build graph**

```cmake
# Delete from test/CMakeLists.txt:
${CMAKE_SOURCE_DIR}/include/pv_cleaning_robot/app/clean_task.h
${CMAKE_SOURCE_DIR}/pv_cleaning_robot/app/clean_task.cc
```

```bash
rg -n "CleanTask|clean_task" include pv_cleaning_robot test
```

Expected:
- no remaining matches outside git history or stale documentation that is being intentionally left unchanged in phase 1

- [ ] **Step 4: Build to verify `CleanTask` is fully removed from the live code path**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- compile succeeds
- no product or test target references `CleanTask`

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: remove dead clean task model"
```

## Task 6: Final Phase-1 Verification Pass

**Files:**
- Modify as needed: any files touched by Tasks 1-5

- [ ] **Step 1: Rebuild the unit-test target from a clean incremental state**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:
- successful build with `RobotSupervisor` integrated and `CleanTask` removed

- [ ] **Step 2: Run targeted tests if a runnable `aarch64` environment is available**

Run:

```bash
./build/unit_tests "[app][robot_supervisor]"
./build/unit_tests "[service][thingsboard_runtime]"
./build/unit_tests "[integration][task_chain]"
```

Expected:
- all three filters pass in a runnable test environment

- [ ] **Step 3: If this environment still cannot execute `aarch64` binaries, record the limitation explicitly**

```text
Verification note:
- `cmake --build build --target unit_tests -j4` passed
- runtime execution of `./build/unit_tests` is still blocked on this machine because `qemu-aarch64` is unavailable
```

- [ ] **Step 4: Review `main.cc` and `ThingsBoardRuntime` for phase-1 success criteria**

```bash
wc -l pv_cleaning_robot/main.cc \
      include/pv_cleaning_robot/service/thingsboard_runtime.h \
      pv_cleaning_robot/service/thingsboard_runtime.cc
rg -n "EvLowBattery|spin_free_detected|set_on_task_start|register_rpc\\(" \
      pv_cleaning_robot/main.cc \
      pv_cleaning_robot/service/thingsboard_runtime.cc \
      include/pv_cleaning_robot/app/robot_supervisor.h \
      pv_cleaning_robot/app/robot_supervisor.cc
```

Expected:
- `main.cc` keeps bootstrap/wiring responsibilities
- runtime policy references move primarily into `RobotSupervisor`
- `ThingsBoardRuntime` delegates instead of owning task-admission logic

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: complete runtime structure phase 1"
```

## Self-Review

- Spec coverage:
  - `RobotSupervisor` introduction: covered by Tasks 1-4
  - schedule/RPC/main-loop inputs converging on supervisor: covered by Tasks 2-4
  - `CleanTask` removal: covered by Task 5
  - `main.cc` bootstrap shrink: covered by Tasks 4 and 6
  - phase-1-only scope, no deep lower-layer churn: preserved by file list and task boundaries
- Placeholder scan:
  - no `TBD`/`TODO`
  - each task lists exact files, commands, and expected outcomes
- Type consistency:
  - `RobotSupervisor` methods are named consistently across tests and implementation tasks
  - `ThingsBoardRuntime::register_rpc_handlers(const std::function<bool()>&, const std::function<bool()>&)` keeps the existing position-sensor callback shape while delegating policy to `RobotSupervisor`
