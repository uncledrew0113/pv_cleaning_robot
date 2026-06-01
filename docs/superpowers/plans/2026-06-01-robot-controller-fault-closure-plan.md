# RobotController Fault Closure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a single-threaded `RobotController` business kernel that owns robot state, mission context, fault closure, and all business-impacting event handling while preserving hard safety stop and existing production behavior.

**Architecture:** `RobotController` owns the business event queue, `RobotState`, active `MissionContext`, and active fault state. `SafetyMonitor`, ThingsBoard RPC, scheduler, watchdog, recovery, motion start failure, and fault detection become event producers; hard endpoint stop still bypasses the queue before posting an event. `FaultPolicy` maps fault facts to actions, while `FaultDetector` observes device/service state and emits facts without mutating business state.

**Tech Stack:** C++17, Catch2 tests, existing `ThreadExecutor`, `EventBus`, `ConfigService`, `MotionService`, `NavService`, `HealthService`, `ThingsBoardControlPlane`, CMake preset `rk3576-build`.

---

## File Structure

- Create `include/pv_cleaning_robot/app/robot_controller.h`: public controller API, event types, snapshot access, command submission, thread lifecycle.
- Create `pv_cleaning_robot/app/robot_controller.cc`: event queue, state transitions, command handling, action dispatch.
- Create `include/pv_cleaning_robot/app/fault_policy.h`: fault fact and policy decision types.
- Create `pv_cleaning_robot/app/fault_policy.cc`: simple table mapping fault facts to actions.
- Create `include/pv_cleaning_robot/app/fault_detector.h`: non-real-time detector interface and snapshot input ports.
- Create `pv_cleaning_robot/app/fault_detector.cc`: minimal runtime fault detection from existing service/device states.
- Modify `include/pv_cleaning_robot/app/robot_supervisor.h`: remove after controller migration, or leave only until final deletion task.
- Modify `pv_cleaning_robot/app/robot_supervisor.cc`: remove after controller migration, or leave only until final deletion task.
- Modify `include/pv_cleaning_robot/app/robot_fsm.h`: remove after controller migration, or leave only until final deletion task.
- Modify `pv_cleaning_robot/app/robot_fsm.cc`: remove after controller migration, or leave only until final deletion task.
- Modify `include/pv_cleaning_robot/app/fault_handler.h`: remove after controller accepts fault events directly.
- Modify `pv_cleaning_robot/app/fault_handler.cc`: remove after controller accepts fault events directly.
- Modify `include/pv_cleaning_robot/service/thingsboard_control_plane.h`: replace direct synchronous supervisor dependency with bounded controller command port.
- Modify `pv_cleaning_robot/service/thingsboard_control_plane.cc`: remove `reset` reboot RPC and generic status-event publishing.
- Modify `include/pv_cleaning_robot/service/fault_service.h`: reduce to active fault store/reporter or remove policy decision methods.
- Modify `pv_cleaning_robot/service/fault_service.cc`: remove `decide()` once `FaultPolicy` owns decisions.
- Modify `include/pv_cleaning_robot/middleware/safety_monitor.h`: expose settled and unstable callback setters for controller event posting.
- Modify `pv_cleaning_robot/middleware/safety_monitor.cc`: keep hard stop; ensure business callbacks only post to controller.
- Modify `pv_cleaning_robot/main.cc`: compose `RobotController`, `FaultPolicy`, `FaultDetector`; wire RPC, scheduler, watchdog, safety, recovery, telemetry.
- Modify `test/CMakeLists.txt`: add new controller, policy, detector tests and source files; remove obsolete app tests after replacement coverage exists.
- Create `test/app/fault_policy_test.cc`: policy table tests.
- Create `test/app/robot_controller_test.cc`: state transition and command tests.
- Create `test/app/fault_detector_test.cc`: detector fact tests.
- Modify `test/service/thingsboard_control_plane_test.cc`: verify retained RPCs and removed reboot RPC/status events.
- Modify `test/integration/hardware/system_hw_test.cc`: preserve equivalent hardware scenarios with `RobotController`.

---

### Task 1: Add FaultPolicy

**Files:**
- Create: `include/pv_cleaning_robot/app/fault_policy.h`
- Create: `pv_cleaning_robot/app/fault_policy.cc`
- Create: `test/app/fault_policy_test.cc`
- Modify: `include/pv_cleaning_robot/domain/robot_domain.h`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write failing policy tests**

Add `test/app/fault_policy_test.cc`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include "pv_cleaning_robot/app/fault_policy.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::FaultAction;
using robot::app::FaultFact;
using robot::app::FaultPolicy;
using robot::app::FaultSource;
using robot::domain::FaultCode;

TEST_CASE("FaultPolicy maps endpoint conflicts to emergency latch", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::SafetyMonitor, FaultCode::kConflictingLimitSides, "both_limits"});

    REQUIRE(decision.action == FaultAction::EmergencyStopAndLatch);
    REQUIRE(decision.latch);
}

TEST_CASE("FaultPolicy maps transient attitude to recovery", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::FaultDetector, FaultCode::kTransientAttitudeError, "tilt"});

    REQUIRE(decision.action == FaultAction::StartRecovery);
    REQUIRE_FALSE(decision.latch);
}

TEST_CASE("FaultPolicy maps low battery start gate to reject start", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::SelfCheck, FaultCode::kStartRejectedLowBattery, "soc_low"});

    REQUIRE(decision.action == FaultAction::RejectStart);
    REQUIRE_FALSE(decision.latch);
}

TEST_CASE("FaultPolicy defaults 0x1xxx faults to emergency latch", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::Watchdog, 0x1F55u, "unknown_p0"});

    REQUIRE(decision.action == FaultAction::EmergencyStopAndLatch);
    REQUIRE(decision.latch);
}
```

- [ ] **Step 2: Register the test and verify it fails**

Modify `test/CMakeLists.txt` by adding:

```cmake
  app/fault_policy_test.cc
```

to the unit test source list, and adding:

```cmake
  ${PROJ}/app/fault_policy.cc
```

to the test support implementation list.

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `pv_cleaning_robot/app/fault_policy.h` does not exist.

- [ ] **Step 3: Implement FaultPolicy**

Update `include/pv_cleaning_robot/domain/robot_domain.h` in `namespace FaultCode`:

```cpp
static constexpr uint32_t kTaskContextInconsistent = 0x1103u;
```

Add `include/pv_cleaning_robot/app/fault_policy.h`:

```cpp
#pragma once

#include <cstdint>
#include <string>

namespace robot::app {

enum class FaultSource {
    SafetyMonitor,
    Watchdog,
    FaultDetector,
    SelfCheck,
    Motion,
    Recovery,
    Controller,
};

struct FaultFact {
    FaultSource source{FaultSource::Controller};
    uint32_t code{0};
    std::string detail;
};

enum class FaultAction {
    WarnOnly,
    RejectStart,
    StartRecovery,
    EmergencyStopAndLatch,
};

struct FaultDecision {
    FaultAction action{FaultAction::WarnOnly};
    bool latch{false};
};

class FaultPolicy {
public:
    FaultDecision decide(const FaultFact& fact) const noexcept;
};

}  // namespace robot::app
```

Add `pv_cleaning_robot/app/fault_policy.cc`:

```cpp
#include "pv_cleaning_robot/app/fault_policy.h"

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {
namespace {

bool is_p0_code(uint32_t code) noexcept {
    return (code & 0xF000u) == 0x1000u;
}

}  // namespace

FaultDecision FaultPolicy::decide(const FaultFact& fact) const noexcept {
    namespace FaultCode = robot::domain::FaultCode;

    switch (fact.code) {
    case FaultCode::kStartRejectedLowBattery:
    case FaultCode::kStartRejectedInvalidPosition:
    case FaultCode::kStartRejectedBusy:
        return {FaultAction::RejectStart, false};
    case FaultCode::kTransientAttitudeError:
        return {FaultAction::StartRecovery, false};
    case FaultCode::kWheelSpinFree:
    case FaultCode::kCanCommunicationLost:
    case FaultCode::kSegmentStartFailed:
    case FaultCode::kP1DuringReturnEscalatedToP0:
    case FaultCode::kTaskContextInconsistent:
    case FaultCode::kUnexpectedLimitSide:
    case FaultCode::kConflictingLimitSides:
    case FaultCode::kLimitUnstableAfterEmergencyStop:
        return {FaultAction::EmergencyStopAndLatch, true};
    default:
        if (is_p0_code(fact.code)) {
            return {FaultAction::EmergencyStopAndLatch, true};
        }
        return {FaultAction::WarnOnly, false};
    }
}

}  // namespace robot::app
```

- [ ] **Step 4: Verify policy tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][fault_policy]"
```

Expected: all `[app][fault_policy]` tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/fault_policy.h pv_cleaning_robot/app/fault_policy.cc include/pv_cleaning_robot/domain/robot_domain.h test/app/fault_policy_test.cc test/CMakeLists.txt
git commit -m "feat: add robot fault policy"
```

---

### Task 2: Add RobotController Command Core

**Files:**
- Create: `include/pv_cleaning_robot/app/robot_controller.h`
- Create: `pv_cleaning_robot/app/robot_controller.cc`
- Create: `test/app/robot_controller_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write failing command-state tests**

Add `test/app/robot_controller_test.cc`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include "pv_cleaning_robot/app/robot_controller.h"

using robot::app::RobotController;
using robot::app::RobotState;
using robot::domain::CommandSource;
using robot::domain::RobotCommand;
using robot::domain::RobotCommandKind;

TEST_CASE("RobotController starts configured mission through SelfChecking", "[app][robot_controller]") {
    RobotController controller;

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController rejects start while busy", "[app][robot_controller]") {
    RobotController controller;
    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-2"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "busy");
}

TEST_CASE("RobotController stop is only accepted while mission is active", "[app][robot_controller]") {
    RobotController controller;

    auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-idle"});
    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "not_running");

    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-running"});
    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
}
```

- [ ] **Step 2: Register the controller test and verify it fails**

Modify `test/CMakeLists.txt` by adding:

```cmake
  app/robot_controller_test.cc
```

and:

```cmake
  ${PROJ}/app/robot_controller.cc
```

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `robot_controller.h` does not exist.

- [ ] **Step 3: Implement minimal controller command core**

Add `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {

enum class RobotState {
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    Recovering,
    Charging,
    FaultStopped,
};

struct CommandResult {
    bool accepted{false};
    std::string reason;
};

struct RobotControllerSnapshot {
    std::string state{"Idle"};
    std::optional<uint32_t> fault;
    int completed_cycles{0};
};

class RobotController {
public:
    CommandResult submit_command(const domain::RobotCommand& command);
    RobotControllerSnapshot snapshot() const;

    void complete_self_check_for_test(bool ok);

private:
    static const char* state_name(RobotState state) noexcept;
    bool mission_active() const noexcept;
    CommandResult start_command_locked(const domain::RobotCommand& command);
    CommandResult stop_locked();

    mutable std::mutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
    std::optional<uint32_t> active_fault_;
};

}  // namespace robot::app
```

Add `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
#include "pv_cleaning_robot/app/robot_controller.h"

namespace robot::app {

const char* RobotController::state_name(RobotState state) noexcept {
    switch (state) {
    case RobotState::Idle:
        return "Idle";
    case RobotState::SelfChecking:
        return "SelfChecking";
    case RobotState::ExecutingMission:
        return "ExecutingMission";
    case RobotState::SettlingEndpoint:
        return "SettlingEndpoint";
    case RobotState::Recovering:
        return "Recovering";
    case RobotState::Charging:
        return "Charging";
    case RobotState::FaultStopped:
        return "FaultStopped";
    }
    return "Unknown";
}

bool RobotController::mission_active() const noexcept {
    return state_ == RobotState::SelfChecking ||
           state_ == RobotState::ExecutingMission ||
           state_ == RobotState::SettlingEndpoint ||
           state_ == RobotState::Recovering;
}

CommandResult RobotController::submit_command(const domain::RobotCommand& command) {
    std::lock_guard<std::mutex> lk(mtx_);
    switch (command.kind) {
    case domain::RobotCommandKind::StartConfiguredMission:
    case domain::RobotCommandKind::CleanTowardOppositeEndpoint:
    case domain::RobotCommandKind::CleanTowardPrimaryDock:
        return start_command_locked(command);
    case domain::RobotCommandKind::Stop:
        return stop_locked();
    case domain::RobotCommandKind::FaultReset:
        if (state_ != RobotState::FaultStopped) {
            return {false, "not_fault_stopped"};
        }
        active_fault_.reset();
        mission_.reset();
        state_ = RobotState::Idle;
        return {true, "accepted"};
    }
    return {false, "unknown_command"};
}

CommandResult RobotController::start_command_locked(const domain::RobotCommand& command) {
    if (state_ != RobotState::Idle) {
        return {false, "busy"};
    }
    domain::LaneConfig lane{};
    lane.primary_dock = domain::Endpoint::A;
    lane.dock_mode = domain::DockMode::SingleDock;

    if (command.kind == domain::RobotCommandKind::CleanTowardOppositeEndpoint) {
        mission_ = domain::build_directional_clean_context(
            domain::MissionKind::CleanTowardOppositeEndpoint,
            lane,
            command.source,
            command.command_id);
    } else if (command.kind == domain::RobotCommandKind::CleanTowardPrimaryDock) {
        mission_ = domain::build_directional_clean_context(
            domain::MissionKind::CleanTowardPrimaryDock,
            lane,
            command.source,
            command.command_id);
    } else {
        mission_ = domain::build_configured_mission_context(
            lane,
            domain::PositionState::AtA,
            command.source,
            command.command_id,
            1);
    }
    state_ = RobotState::SelfChecking;
    return {true, "accepted"};
}

CommandResult RobotController::stop_locked() {
    if (!mission_active()) {
        return {false, "not_running"};
    }
    mission_.reset();
    state_ = RobotState::Idle;
    return {true, "accepted"};
}

RobotControllerSnapshot RobotController::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    RobotControllerSnapshot snap;
    snap.state = state_name(state_);
    snap.fault = active_fault_;
    return snap;
}

void RobotController::complete_self_check_for_test(bool ok) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != RobotState::SelfChecking) {
        return;
    }
    if (!ok) {
        mission_.reset();
        state_ = RobotState::Idle;
        return;
    }
    state_ = RobotState::ExecutingMission;
}

}  // namespace robot::app
```

- [ ] **Step 4: Verify controller command tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all `[app][robot_controller]` tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc test/CMakeLists.txt
git commit -m "feat: add robot controller command core"
```

---

### Task 3: Add Controller Event Queue

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `test/app/robot_controller_test.cc`

- [ ] **Step 1: Add failing queue serialization test**

Append to `test/app/robot_controller_test.cc`:

```cpp
#include <atomic>
#include <thread>
#include <vector>

TEST_CASE("RobotController serializes posted callbacks on controller thread", "[app][robot_controller]") {
    RobotController controller;
    controller.start();

    std::atomic<int> count{0};
    std::vector<std::thread> workers;
    for (int i = 0; i < 8; ++i) {
        workers.emplace_back([&controller, &count] {
            for (int j = 0; j < 25; ++j) {
                controller.post_for_test([&count] { count.fetch_add(1); });
            }
        });
    }
    for (auto& worker : workers) {
        worker.join();
    }

    controller.drain_for_test();
    controller.stop();

    REQUIRE(count.load() == 200);
}
```

- [ ] **Step 2: Run test and verify it fails**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `start`, `stop`, `post_for_test`, and `drain_for_test` do not exist.

- [ ] **Step 3: Add minimal event queue**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
#include <condition_variable>
#include <deque>
#include <functional>
#include <thread>
```

Add public methods:

```cpp
    void start();
    void stop();
    void post_for_test(std::function<void()> fn);
    void drain_for_test();
```

Add private members:

```cpp
    void loop();
    void post(std::function<void()> fn);

    mutable std::mutex queue_mtx_;
    std::condition_variable queue_cv_;
    std::deque<std::function<void()>> queue_;
    bool running_{false};
    bool stop_requested_{false};
    std::thread worker_;
```

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
void RobotController::start() {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    if (running_) {
        return;
    }
    running_ = true;
    stop_requested_ = false;
    worker_ = std::thread([this] { loop(); });
}

void RobotController::stop() {
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        stop_requested_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lk(queue_mtx_);
    running_ = false;
}

void RobotController::post(std::function<void()> fn) {
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        queue_.push_back(std::move(fn));
    }
    queue_cv_.notify_one();
}

void RobotController::post_for_test(std::function<void()> fn) {
    post(std::move(fn));
}

void RobotController::drain_for_test() {
    for (;;) {
        {
            std::lock_guard<std::mutex> lk(queue_mtx_);
            if (queue_.empty()) {
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void RobotController::loop() {
    for (;;) {
        std::function<void()> fn;
        {
            std::unique_lock<std::mutex> lk(queue_mtx_);
            queue_cv_.wait(lk, [this] { return stop_requested_ || !queue_.empty(); });
            if (stop_requested_ && queue_.empty()) {
                return;
            }
            fn = std::move(queue_.front());
            queue_.pop_front();
        }
        fn();
    }
}
```

Add include to `robot_controller.cc`:

```cpp
#include <chrono>
```

- [ ] **Step 4: Verify queue test passes**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all controller tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc
git commit -m "feat: serialize robot controller events"
```

---

### Task 4: Make Command Submission Queue-Bound

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `test/app/robot_controller_test.cc`

- [ ] **Step 1: Add failing bounded submit test**

Append to `test/app/robot_controller_test.cc`:

```cpp
TEST_CASE("RobotController submit_command works through running queue", "[app][robot_controller]") {
    RobotController controller;
    controller.start();

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-queued"});

    controller.stop();

    REQUIRE(result.accepted);
    REQUIRE(result.reason == "accepted");
    REQUIRE(controller.snapshot().state == "SelfChecking");
}
```

- [ ] **Step 2: Verify current direct implementation fails the intended design review**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: test may pass, but code review fails because `submit_command()` still mutates state on the caller thread when the controller is running.

- [ ] **Step 3: Route submit_command through queue when started**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
#include <future>
```

Add private method:

```cpp
    CommandResult submit_command_locked(const domain::RobotCommand& command);
```

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
CommandResult RobotController::submit_command(const domain::RobotCommand& command) {
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        if (!running_) {
            std::lock_guard<std::mutex> state_lk(mtx_);
            return submit_command_locked(command);
        }
    }

    auto promise = std::make_shared<std::promise<CommandResult>>();
    auto future = promise->get_future();
    post([this, command, promise] {
        std::lock_guard<std::mutex> lk(mtx_);
        promise->set_value(submit_command_locked(command));
    });

    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return {false, "controller_timeout"};
    }
    return future.get();
}

CommandResult RobotController::submit_command_locked(const domain::RobotCommand& command) {
    switch (command.kind) {
    case domain::RobotCommandKind::StartConfiguredMission:
    case domain::RobotCommandKind::CleanTowardOppositeEndpoint:
    case domain::RobotCommandKind::CleanTowardPrimaryDock:
        return start_command_locked(command);
    case domain::RobotCommandKind::Stop:
        return stop_locked();
    case domain::RobotCommandKind::FaultReset:
        if (state_ != RobotState::FaultStopped) {
            return {false, "not_fault_stopped"};
        }
        active_fault_.reset();
        mission_.reset();
        state_ = RobotState::Idle;
        return {true, "accepted"};
    }
    return {false, "unknown_command"};
}
```

Replace the old body of `submit_command()` with the queue-bound version above.

- [ ] **Step 4: Verify command tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all controller tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc
git commit -m "feat: route controller commands through queue"
```

---

### Task 5: Add Endpoint and Mission Segment Transitions

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `test/app/robot_controller_test.cc`

- [ ] **Step 1: Add failing endpoint tests**

Append to `test/app/robot_controller_test.cc`:

```cpp
TEST_CASE("RobotController completes configured single-dock mission through two endpoints",
          "[app][robot_controller]") {
    RobotController controller;
    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    controller.complete_self_check_for_test(true);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    REQUIRE(controller.snapshot().state == "Idle");
}

TEST_CASE("RobotController unexpected endpoint enters FaultStopped", "[app][robot_controller]") {
    RobotController controller;
    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault.has_value());
}
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `handle_limit_settled_for_test()` does not exist.

- [ ] **Step 3: Implement endpoint handling**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
    void handle_limit_settled_for_test(domain::Endpoint endpoint);
```

Add private method:

```cpp
    void handle_limit_settled_locked(domain::Endpoint endpoint);
```

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
void RobotController::handle_limit_settled_for_test(domain::Endpoint endpoint) {
    std::lock_guard<std::mutex> lk(mtx_);
    handle_limit_settled_locked(endpoint);
}

void RobotController::handle_limit_settled_locked(domain::Endpoint endpoint) {
    namespace FaultCode = robot::domain::FaultCode;
    if (state_ != RobotState::ExecutingMission || !mission_) {
        active_fault_ = FaultCode::kUnexpectedLimitSide;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return;
    }

    const auto* segment = mission_->current_segment();
    if (segment == nullptr || segment->target != endpoint) {
        active_fault_ = FaultCode::kUnexpectedLimitSide;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return;
    }

    state_ = RobotState::SettlingEndpoint;
    ++mission_->current_segment_index;
    if (mission_->current_segment_index >= mission_->segments.size()) {
        ++mission_->completed_cycles;
    }
    if (mission_->completed_cycles >= mission_->repeat_count) {
        mission_.reset();
        state_ = RobotState::Idle;
        return;
    }
    mission_->current_segment_index = 0;
    state_ = RobotState::ExecutingMission;
}
```

- [ ] **Step 4: Verify endpoint tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all controller tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc
git commit -m "feat: add controller endpoint transitions"
```

---

### Task 6: Add Fault Application to Controller

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `test/app/robot_controller_test.cc`

- [ ] **Step 1: Add failing fault tests**

Append to `test/app/robot_controller_test.cc`:

```cpp
#include "pv_cleaning_robot/app/fault_policy.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

TEST_CASE("RobotController P0 fault enters FaultStopped and reset returns Idle",
          "[app][robot_controller]") {
    RobotController controller;
    controller.handle_fault_for_test(robot::app::FaultFact{
        robot::app::FaultSource::Watchdog,
        robot::domain::FaultCode::kCanCommunicationLost,
        "can_lost"});

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kCanCommunicationLost);

    const auto reset = controller.submit_command(
        RobotCommand{RobotCommandKind::FaultReset, CommandSource::Rpc, "reset-1"});
    REQUIRE(reset.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController recoverable fault enters Recovering from ExecutingMission",
          "[app][robot_controller]") {
    RobotController controller;
    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_fault_for_test(robot::app::FaultFact{
        robot::app::FaultSource::FaultDetector,
        robot::domain::FaultCode::kTransientAttitudeError,
        "tilt"});

    REQUIRE(controller.snapshot().state == "Recovering");
}
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `handle_fault_for_test()` does not exist.

- [ ] **Step 3: Implement fault application**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
#include "pv_cleaning_robot/app/fault_policy.h"
```

Add public method:

```cpp
    void handle_fault_for_test(const FaultFact& fact);
```

Add private members:

```cpp
    void handle_fault_locked(const FaultFact& fact);
    FaultPolicy fault_policy_;
```

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
void RobotController::handle_fault_for_test(const FaultFact& fact) {
    std::lock_guard<std::mutex> lk(mtx_);
    handle_fault_locked(fact);
}

void RobotController::handle_fault_locked(const FaultFact& fact) {
    const auto decision = fault_policy_.decide(fact);
    switch (decision.action) {
    case FaultAction::WarnOnly:
        return;
    case FaultAction::RejectStart:
        if (state_ == RobotState::SelfChecking) {
            mission_.reset();
            state_ = RobotState::Idle;
        }
        return;
    case FaultAction::StartRecovery:
        if (state_ == RobotState::ExecutingMission && mission_) {
            state_ = RobotState::Recovering;
        }
        return;
    case FaultAction::EmergencyStopAndLatch:
        active_fault_ = fact.code;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return;
    }
}
```

- [ ] **Step 4: Verify fault tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all controller tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc
git commit -m "feat: apply faults in robot controller"
```

---

### Task 7: Add FaultDetector

**Files:**
- Create: `include/pv_cleaning_robot/app/fault_detector.h`
- Create: `pv_cleaning_robot/app/fault_detector.cc`
- Create: `test/app/fault_detector_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write failing detector tests**

Add `test/app/fault_detector_test.cc`:

```cpp
#include <catch2/catch_test_macros.hpp>

#include "pv_cleaning_robot/app/fault_detector.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::FaultDetector;
using robot::app::FaultFact;

TEST_CASE("FaultDetector emits conflicting limit fault", "[app][fault_detector]") {
    FaultDetector detector;
    auto facts = detector.detect(FaultDetector::Input{
        .left_limit_active = true,
        .right_limit_active = true,
    });

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kConflictingLimitSides);
}

TEST_CASE("FaultDetector emits spin-free fault only while executing", "[app][fault_detector]") {
    FaultDetector detector;
    auto facts = detector.detect(FaultDetector::Input{
        .executing_mission = true,
        .spin_free_detected = true,
    });

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kWheelSpinFree);

    facts = detector.detect(FaultDetector::Input{
        .executing_mission = false,
        .spin_free_detected = true,
    });
    REQUIRE(facts.empty());
}

TEST_CASE("FaultDetector emits transient attitude fault for recoverable tilt",
          "[app][fault_detector]") {
    FaultDetector detector;
    auto facts = detector.detect(FaultDetector::Input{
        .executing_mission = true,
        .imu_fresh = true,
        .attitude_out_of_range = true,
    });

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kTransientAttitudeError);
}
```

- [ ] **Step 2: Register and verify failure**

Modify `test/CMakeLists.txt` by adding:

```cmake
  app/fault_detector_test.cc
```

and:

```cmake
  ${PROJ}/app/fault_detector.cc
```

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `fault_detector.h` does not exist.

- [ ] **Step 3: Implement detector**

Add `include/pv_cleaning_robot/app/fault_detector.h`:

```cpp
#pragma once

#include <vector>

#include "pv_cleaning_robot/app/fault_policy.h"

namespace robot::app {

class FaultDetector {
public:
    struct Input {
        bool executing_mission{false};
        bool left_limit_active{false};
        bool right_limit_active{false};
        bool spin_free_detected{false};
        bool imu_fresh{true};
        bool attitude_out_of_range{false};
        bool bms_critical_alarm{false};
        bool brush_critical_fault{false};
        bool motor_driver_fault{false};
    };

    std::vector<FaultFact> detect(const Input& input) const;
};

}  // namespace robot::app
```

Add `pv_cleaning_robot/app/fault_detector.cc`:

```cpp
#include "pv_cleaning_robot/app/fault_detector.h"

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {

std::vector<FaultFact> FaultDetector::detect(const Input& input) const {
    namespace FaultCode = robot::domain::FaultCode;
    std::vector<FaultFact> facts;

    if (input.left_limit_active && input.right_limit_active) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kConflictingLimitSides,
                         "conflicting_limit_sides"});
    }
    if (input.executing_mission && input.spin_free_detected) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kWheelSpinFree,
                         "wheel_spin_free"});
    }
    if (input.executing_mission && input.imu_fresh && input.attitude_out_of_range) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kTransientAttitudeError,
                         "transient_attitude_error"});
    }
    if (!input.imu_fresh) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kCanCommunicationLost,
                         "imu_stale"});
    }
    if (input.bms_critical_alarm) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kCanCommunicationLost,
                         "bms_critical_alarm"});
    }
    if (input.brush_critical_fault) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kCanCommunicationLost,
                         "brush_critical_fault"});
    }
    if (input.motor_driver_fault) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kCanCommunicationLost,
                         "motor_driver_fault"});
    }

    return facts;
}

}  // namespace robot::app
```

- [ ] **Step 4: Verify detector tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][fault_detector]"
```

Expected: all detector tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/fault_detector.h pv_cleaning_robot/app/fault_detector.cc test/app/fault_detector_test.cc test/CMakeLists.txt
git commit -m "feat: add runtime fault detector"
```

---

### Task 8: Wire Motion Actions into RobotController

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `test/app/robot_controller_test.cc`

- [ ] **Step 1: Add failing action-port test**

Append to `test/app/robot_controller_test.cc`:

```cpp
struct RecordingRobotActions {
    int start_segment_count{0};
    int stop_count{0};
    int emergency_stop_count{0};

    robot::app::RobotController::ActionPorts ports() {
        return robot::app::RobotController::ActionPorts{
            .start_segment = [this](const robot::domain::MissionSegment&) {
                ++start_segment_count;
                return true;
            },
            .stop_motion = [this] { ++stop_count; },
            .emergency_stop = [this] { ++emergency_stop_count; },
            .start_recovery = [] {},
            .clear_fault = [] {},
        };
    }
};

TEST_CASE("RobotController starts motion after successful self check", "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());

    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(actions.start_segment_count == 1);
}

TEST_CASE("RobotController converts motion start failure into FaultStopped",
          "[app][robot_controller]") {
    auto ports = RecordingRobotActions{}.ports();
    ports.start_segment = [](const robot::domain::MissionSegment&) { return false; };
    RobotController controller(ports);

    REQUIRE(controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kSegmentStartFailed);
}
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `ActionPorts` and constructor overload do not exist.

- [ ] **Step 3: Add action ports and start-segment dispatch**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
    struct ActionPorts {
        std::function<bool(const domain::MissionSegment&)> start_segment;
        std::function<void()> stop_motion;
        std::function<void()> emergency_stop;
        std::function<void()> start_recovery;
        std::function<void()> clear_fault;
    };

    RobotController() = default;
    explicit RobotController(ActionPorts ports);
```

Add private member:

```cpp
    bool start_current_segment_locked();
    ActionPorts actions_{};
```

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
RobotController::RobotController(ActionPorts ports) : actions_(std::move(ports)) {}

bool RobotController::start_current_segment_locked() {
    namespace FaultCode = robot::domain::FaultCode;
    if (!mission_) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        state_ = RobotState::FaultStopped;
        return false;
    }
    const auto* segment = mission_->current_segment();
    if (segment == nullptr) {
        active_fault_ = FaultCode::kTaskContextInconsistent;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        return false;
    }
    if (actions_.start_segment && !actions_.start_segment(*segment)) {
        active_fault_ = FaultCode::kSegmentStartFailed;
        mission_.reset();
        state_ = RobotState::FaultStopped;
        if (actions_.emergency_stop) {
            actions_.emergency_stop();
        }
        return false;
    }
    return true;
}
```

Update `complete_self_check_for_test(true)` success branch:

```cpp
    state_ = RobotState::ExecutingMission;
    start_current_segment_locked();
```

Update `stop_locked()` before clearing mission:

```cpp
    if (actions_.stop_motion) {
        actions_.stop_motion();
    }
```

- [ ] **Step 4: Verify action tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller]"
```

Expected: all controller tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc test/app/robot_controller_test.cc
git commit -m "feat: wire robot controller actions"
```

---

### Task 9: Rewire ThingsBoard RPCs and Remove Reboot RPC

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Add failing ThingsBoard RPC scope tests**

Append to `test/service/thingsboard_control_plane_test.cc`:

```cpp
TEST_CASE("ThingsBoardControlPlane does not register device reboot reset RPC",
          "[service][thingsboard]") {
    ThingsBoardFixture f;
    f.control_plane->register_rpc_handlers();

    REQUIRE_FALSE(f.cloud->has_rpc("reset"));
    REQUIRE(f.cloud->has_rpc("fault_reset"));
}

TEST_CASE("ThingsBoardControlPlane maps retained RPCs to robot command port",
          "[service][thingsboard]") {
    ThingsBoardFixture f;
    std::vector<robot::domain::RobotCommandKind> received;
    f.robot_port.submit_command = [&](const robot::domain::RobotCommand& command) {
        received.push_back(command.kind);
        return robot::service::RobotCommandResult{true, "accepted"};
    };
    f.control_plane->register_rpc_handlers();

    f.cloud->emit_rpc("1", R"({"method":"clean_to_return","params":{}})");
    f.cloud->emit_rpc("2", R"({"method":"clean_to_parking","params":{}})");
    f.cloud->emit_rpc("3", R"({"method":"start_configured","params":{}})");
    f.cloud->emit_rpc("4", R"({"method":"stop","params":{}})");
    f.cloud->emit_rpc("5", R"({"method":"fault_reset","params":{}})");

    REQUIRE(received.size() == 5);
}
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][thingsboard]"
```

Expected: tests fail because `register_rpc_handlers()` currently requires reboot callback and registers `reset`.

- [ ] **Step 3: Remove reboot RPC from control plane**

Update `include/pv_cleaning_robot/service/thingsboard_control_plane.h`:

```cpp
    void register_rpc_handlers();
```

Remove the `std::function<void()> reboot_device` parameter.

Update `pv_cleaning_robot/service/thingsboard_control_plane.cc`:

```cpp
void ThingsBoardControlPlane::register_rpc_handlers() {
    register_command_rpc("clean_to_return", domain::RobotCommandKind::CleanTowardOppositeEndpoint);
    register_command_rpc("clean_to_parking",
                         domain::RobotCommandKind::CleanTowardPrimaryDock);
    register_command_rpc("start_configured", domain::RobotCommandKind::StartConfiguredMission);
    register_command_rpc("stop", domain::RobotCommandKind::Stop);
    register_command_rpc("fault_reset", domain::RobotCommandKind::FaultReset);
}
```

Delete the old `cloud_->register_rpc("reset", ...)` block.

Update `main.cc` call later in Task 12 from:

```cpp
tb_control->register_rpc_handlers([&log]() { ... });
```

to:

```cpp
tb_control->register_rpc_handlers();
```

- [ ] **Step 4: Verify ThingsBoard tests pass**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][thingsboard]"
```

Expected: ThingsBoard retained RPC tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h pv_cleaning_robot/service/thingsboard_control_plane.cc test/service/thingsboard_control_plane_test.cc
git commit -m "refactor: remove thingsboard reboot rpc"
```

---

### Task 10: Remove Generic ThingsBoard Status Events

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/service/thingsboard_event_payload_builder_test.cc`

- [ ] **Step 1: Add failing test for shared attribute update without status event**

Append to `test/service/thingsboard_control_plane_test.cc`:

```cpp
TEST_CASE("ThingsBoardControlPlane applies shared attributes without status event telemetry",
          "[service][thingsboard]") {
    ThingsBoardFixture f;
    f.control_plane->subscribe_shared_attributes();

    const auto before = f.cloud->published_telemetry.size();
    f.cloud->emit_shared_attributes(R"({"clean_speed_rpm":120})");

    REQUIRE(f.cloud->published_telemetry.size() == before);
}
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][thingsboard]"
```

Expected: test fails if shared attribute update still publishes `shared_attr_update`.

- [ ] **Step 3: Remove status event API and calls**

Update `include/pv_cleaning_robot/service/thingsboard_control_plane.h` by removing:

```cpp
    void publish_backup_fallback_event() const;
    void publish_status_event(const char* event_name, const char* code) const;
    static size_t build_status_event(const StatusEventView& view, char* out, size_t cap) noexcept;
```

Remove `StatusEventView`.

Update `pv_cleaning_robot/service/thingsboard_control_plane.cc`:

```cpp
void ThingsBoardControlPlane::subscribe_shared_attributes() {
    cloud_->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
        const auto result = config_.apply_runtime_patch(attrs, scheduler_);
        if (!result.accepted) {
            spdlog::warn("[ThingsBoardControlPlane] shared attributes rejected: {}",
                         result.reason);
        }
    });
}
```

Delete `publish_backup_fallback_event()`, `publish_status_event()`, and `ThingsBoardJsonCodec::build_status_event()`.

Update `main.cc` later in Task 12 by removing:

```cpp
if (cfg.last_load_used_backup()) {
    tb_control->publish_backup_fallback_event();
}
```

- [ ] **Step 4: Remove obsolete status-event tests**

In `test/service/thingsboard_event_payload_builder_test.cc`, delete tests that only validate `build_status_event`. Keep startup attribute tests.

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][thingsboard]"
```

Expected: ThingsBoard tests pass, and no code references `publish_status_event`.

- [ ] **Step 5: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h pv_cleaning_robot/service/thingsboard_control_plane.cc test/service/thingsboard_control_plane_test.cc test/service/thingsboard_event_payload_builder_test.cc
git commit -m "refactor: remove thingsboard status events"
```

---

### Task 11: Rewire Safety, Watchdog, Scheduler, and Recovery to Controller Events

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_controller.h`
- Modify: `pv_cleaning_robot/app/robot_controller.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `pv_cleaning_robot/middleware/safety_monitor.cc`
- Modify: `include/pv_cleaning_robot/middleware/safety_monitor.h`
- Modify: `test/app/robot_controller_test.cc`
- Modify: `test/middleware/safety_monitor_test.cc`

- [ ] **Step 1: Add controller producer methods**

Update `include/pv_cleaning_robot/app/robot_controller.h`:

```cpp
    void post_limit_settled(domain::Endpoint endpoint);
    void post_limit_unstable(domain::Endpoint endpoint);
    void post_watchdog_timeout(std::string thread_name);
    void post_recovery_finished(bool ok);
```

Add private handlers:

```cpp
    void handle_limit_unstable_locked(domain::Endpoint endpoint);
    void handle_watchdog_timeout_locked(const std::string& thread_name);
    void handle_recovery_finished_locked(bool ok);
```

- [ ] **Step 2: Implement producer methods through queue**

Update `pv_cleaning_robot/app/robot_controller.cc`:

```cpp
void RobotController::post_limit_settled(domain::Endpoint endpoint) {
    post([this, endpoint] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_limit_settled_locked(endpoint);
    });
}

void RobotController::post_limit_unstable(domain::Endpoint endpoint) {
    post([this, endpoint] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_limit_unstable_locked(endpoint);
    });
}

void RobotController::post_watchdog_timeout(std::string thread_name) {
    post([this, thread_name = std::move(thread_name)] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_watchdog_timeout_locked(thread_name);
    });
}

void RobotController::post_recovery_finished(bool ok) {
    post([this, ok] {
        std::lock_guard<std::mutex> lk(mtx_);
        handle_recovery_finished_locked(ok);
    });
}

void RobotController::handle_limit_unstable_locked(domain::Endpoint) {
    handle_fault_locked(FaultFact{FaultSource::SafetyMonitor,
                                  domain::FaultCode::kLimitUnstableAfterEmergencyStop,
                                  "limit_unstable_after_hard_stop"});
}

void RobotController::handle_watchdog_timeout_locked(const std::string& thread_name) {
    handle_fault_locked(FaultFact{FaultSource::Watchdog,
                                  domain::FaultCode::kCanCommunicationLost,
                                  "watchdog_timeout:" + thread_name});
}

void RobotController::handle_recovery_finished_locked(bool ok) {
    if (state_ != RobotState::Recovering) {
        return;
    }
    if (!ok) {
        handle_fault_locked(FaultFact{FaultSource::Recovery,
                                      domain::FaultCode::kP1DuringReturnEscalatedToP0,
                                      "recovery_failed"});
        return;
    }
    state_ = RobotState::ExecutingMission;
    start_current_segment_locked();
}
```

- [ ] **Step 3: Rewire main composition**

In `pv_cleaning_robot/main.cc`, replace old supervisor bridge wiring:

```cpp
supervisor->register_limit_settled_bridge(event_bus, current_limit_levels, current_battery_soc);
supervisor->register_scheduler_window(scheduler, current_limit_levels, current_battery_soc);
robot::app::FaultHandler fault_handler(...);
fault_handler.start_listening();
watchdog.set_timeout_callback([&fault](const std::string& thread_name) {
    fault->report(...);
});
```

with controller wiring:

```cpp
safety_monitor.set_limit_settled_callback(
    [controller](robot::domain::Endpoint endpoint) {
        controller->post_limit_settled(endpoint);
    });
safety_monitor.set_limit_unstable_callback(
    [controller](robot::domain::Endpoint endpoint) {
        controller->post_limit_unstable(endpoint);
    });
scheduler.set_on_window_hit([controller] {
    controller->post_schedule_window_hit();
});
watchdog.set_timeout_callback([controller](const std::string& thread_name) {
    controller->post_watchdog_timeout(thread_name);
});
```

Add `post_schedule_window_hit()` in `RobotController` in the same style as other post methods.

- [ ] **Step 4: Keep SafetyMonitor hard stop unchanged**

In `pv_cleaning_robot/middleware/safety_monitor.cc`, keep:

```cpp
emergency_stop_();
```

inside `SafetyMonitor::on_limit_trigger()`.

Change only the settled/unstable business notification path so it invokes callbacks that post to `RobotController`, instead of directly depending on business subscribers.

- [ ] **Step 5: Verify safety and controller tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[app][robot_controller],[middleware][safety_monitor]"
```

Expected: controller events are serialized; SafetyMonitor still verifies emergency stop frame behavior.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_controller.h pv_cleaning_robot/app/robot_controller.cc pv_cleaning_robot/main.cc include/pv_cleaning_robot/middleware/safety_monitor.h pv_cleaning_robot/middleware/safety_monitor.cc test/app/robot_controller_test.cc test/middleware/safety_monitor_test.cc
git commit -m "refactor: route business producers through robot controller"
```

---

### Task 12: Compose RobotController in main

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Replace supervisor construction**

In `pv_cleaning_robot/main.cc`, replace:

```cpp
auto fsm = std::make_shared<robot::app::RobotFsm>();
auto supervisor =
    std::make_shared<robot::app::RobotSupervisor>(fsm, cfg, fault, nav);
```

with:

```cpp
auto controller = std::make_shared<robot::app::RobotController>(
    robot::app::RobotController::ActionPorts{
        .start_segment = [motion](const robot::domain::MissionSegment& segment) {
            return motion->start_segment(segment);
        },
        .stop_motion = [motion] { motion->stop_cleaning(); },
        .emergency_stop = [motion] { motion->emergency_stop(); },
        .start_recovery = [recovery] { recovery->start(); },
        .clear_fault = [fault] { fault->clear_active_fault(); },
    });
controller->start();
```

- [ ] **Step 2: Replace ThingsBoard command port**

Replace:

```cpp
auto command_port = robot::service::RobotCommandPort{
    [supervisor](const robot::domain::RobotCommand& command) {
        const auto result = supervisor->submit_command(command);
        return robot::service::RobotCommandResult{result.accepted, result.reason};
    },
    [supervisor]() { return supervisor->snapshot(); }};
```

with:

```cpp
auto command_port = robot::service::RobotCommandPort{
    [controller](const robot::domain::RobotCommand& command) {
        const auto result = controller->submit_command(command);
        return robot::service::RobotCommandResult{result.accepted, result.reason};
    },
    [controller]() {
        const auto snap = controller->snapshot();
        robot::domain::RobotRuntimeSnapshot runtime;
        runtime.state = snap.state;
        runtime.fault = snap.fault;
        runtime.completed_cycles = snap.completed_cycles;
        return runtime;
    }};
```

- [ ] **Step 3: Replace lifecycle stop**

Before shutdown exits, add:

```cpp
controller->stop();
```

before stopping service executors if business events need to stop first, or after stopping producers if no new events should enter the queue. Use this order:

```cpp
walk_exec.stop();
nav_exec.stop();
bms_exec.stop();
cloud_exec.stop();
safety_monitor.stop();
watchdog.stop();
controller->stop();
motion->emergency_stop();
```

- [ ] **Step 4: Remove direct supervisor ticks**

Delete:

```cpp
supervisor->tick_safety();
supervisor->tick_recovery();
```

Replace with:

```cpp
controller->post_tick();
```

where `post_tick()` runs fault detection and recovery polling on the controller thread.

- [ ] **Step 5: Build**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build passes. If any old supervisor reference remains, remove that reference in this step before proceeding.

- [ ] **Step 6: Commit**

```bash
git add pv_cleaning_robot/main.cc test/CMakeLists.txt
git commit -m "refactor: compose robot controller in main"
```

---

### Task 13: Remove RobotSupervisor, RobotFsm, and FaultHandler

**Files:**
- Delete: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Delete: `pv_cleaning_robot/app/robot_supervisor.cc`
- Delete: `include/pv_cleaning_robot/app/robot_fsm.h`
- Delete: `pv_cleaning_robot/app/robot_fsm.cc`
- Delete: `include/pv_cleaning_robot/app/fault_handler.h`
- Delete: `pv_cleaning_robot/app/fault_handler.cc`
- Delete or rewrite: `test/app/robot_supervisor_test.cc`
- Delete or rewrite: `test/app/robot_fsm_test.cc`
- Delete or rewrite: `test/app/fault_handler_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Search remaining references**

Run:

```bash
rg -n "RobotSupervisor|RobotFsm|FaultHandler|robot_supervisor|robot_fsm|fault_handler" pv_cleaning_robot include test
```

Expected: references remain only in old files and CMake.

- [ ] **Step 2: Remove old files from CMake**

In `test/CMakeLists.txt`, remove:

```cmake
  ${PROJ}/app/robot_supervisor.cc
  ${PROJ}/app/robot_fsm.cc
  ${PROJ}/app/fault_handler.cc
  app/robot_supervisor_test.cc
  app/robot_fsm_test.cc
  app/fault_handler_test.cc
```

Keep replacement coverage in:

```cmake
  ${PROJ}/app/robot_controller.cc
  ${PROJ}/app/fault_policy.cc
  ${PROJ}/app/fault_detector.cc
  app/robot_controller_test.cc
  app/fault_policy_test.cc
  app/fault_detector_test.cc
```

- [ ] **Step 3: Delete old files**

Use:

```bash
git rm include/pv_cleaning_robot/app/robot_supervisor.h pv_cleaning_robot/app/robot_supervisor.cc
git rm include/pv_cleaning_robot/app/robot_fsm.h pv_cleaning_robot/app/robot_fsm.cc
git rm include/pv_cleaning_robot/app/fault_handler.h pv_cleaning_robot/app/fault_handler.cc
git rm test/app/robot_supervisor_test.cc test/app/robot_fsm_test.cc test/app/fault_handler_test.cc
```

- [ ] **Step 4: Verify no old references**

Run:

```bash
rg -n "RobotSupervisor|RobotFsm|FaultHandler|robot_supervisor|robot_fsm|fault_handler" pv_cleaning_robot include test
```

Expected: no output.

- [ ] **Step 5: Build unit tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build passes.

- [ ] **Step 6: Commit**

```bash
git add test/CMakeLists.txt
git commit -m "refactor: remove old supervisor fsm fault handler"
```

---

### Task 14: Preserve Health Telemetry and Local JSONL

**Files:**
- Modify: `test/service/health_payload_builder_test.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Verify: `pv_cleaning_robot/service/health_service.cc`
- Verify: `include/pv_cleaning_robot/service/health_service.h`

- [ ] **Step 1: Run existing health tests before code changes**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][health]"
```

Expected: health payload tests pass.

- [ ] **Step 2: Confirm HealthService still uploads through CloudService**

Verify `pv_cleaning_robot/service/health_service.cc` keeps:

```cpp
if (cloud_)
    cloud_->publish_telemetry(payload_cache_);
```

and local JSONL keeps:

```cpp
local_log_->log(spdlog::level::info,
                spdlog::string_view_t(payload_cache_.data(), payload_cache_.size()));
```

- [ ] **Step 3: Add regression test for health service independence**

Append to `test/service/health_payload_builder_test.cc`:

```cpp
TEST_CASE("HealthPayloadBuilder remains independent of business state",
          "[service][health][payload]") {
    robot::service::HealthPayloadBuilder::HealthView view{};
    view.ts_ms = 123;
    view.bms.soc_pct = 88.0f;
    view.imu.pitch_deg = 1.5f;

    char out[1024]{};
    const size_t len = robot::service::HealthPayloadBuilder::build_health(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    const std::string payload(out, len);
    REQUIRE(payload.find("\"bat_soc\":88.000000") != std::string::npos);
    REQUIRE(payload.find("\"imu_p\":1.500000") != std::string::npos);
}
```

- [ ] **Step 4: Verify health tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
./build/rk3576-build/test/unit_tests "[service][health]"
```

Expected: health tests pass.

- [ ] **Step 5: Commit**

```bash
git add test/service/health_payload_builder_test.cc
git commit -m "test: preserve health telemetry contract"
```

---

### Task 15: Update Hardware System Tests to Controller

**Files:**
- Modify: `test/integration/hardware/system_hw_test.cc`
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Replace old app construction in hardware fixture**

In `test/integration/hardware/system_hw_test.cc`, replace `RobotSupervisor`, `RobotFsm`, and `FaultHandler` construction with:

```cpp
controller = std::make_shared<robot::app::RobotController>(
    robot::app::RobotController::ActionPorts{
        .start_segment = [this](const robot::domain::MissionSegment& segment) {
            return motion->start_segment(segment);
        },
        .stop_motion = [this] { motion->stop_cleaning(); },
        .emergency_stop = [this] { motion->emergency_stop(); },
        .start_recovery = [this] {
            if (recovery) {
                recovery->start();
            }
        },
        .clear_fault = [this] { fault->clear_active_fault(); },
    });
controller->start();
```

Add fixture member:

```cpp
std::shared_ptr<robot::app::RobotController> controller;
```

- [ ] **Step 2: Replace command submissions**

Replace old calls:

```cpp
f.supervisor->submit_command(command)
```

with:

```cpp
f.controller->submit_command(command)
```

Replace old state reads:

```cpp
f.supervisor->current_state()
```

with:

```cpp
f.controller->snapshot().state
```

- [ ] **Step 3: Replace P0 fault chain test**

In the P0 chain test, replace direct `fault->report(P0, ...)` with:

```cpp
f.controller->handle_fault_for_test(robot::app::FaultFact{
    robot::app::FaultSource::Watchdog,
    robot::domain::FaultCode::kCanCommunicationLost,
    "test_p0_fault"});
```

Then reset through RPC-equivalent command:

```cpp
const auto reset = f.controller->submit_command(robot::domain::RobotCommand{
    robot::domain::RobotCommandKind::FaultReset,
    robot::domain::CommandSource::Rpc,
    "fault-reset"});
REQUIRE(reset.accepted);
REQUIRE(f.controller->snapshot().state == "Idle");
```

- [ ] **Step 4: Verify hardware test build**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: hardware test target builds.

- [ ] **Step 5: Run non-motion hardware-safe tests first**

Run:

```bash
./build/rk3576-build/test/hw_tests "[hw_system][watchdog_heartbeat]"
./build/rk3576-build/test/hw_tests "[hw_system][p0_fault_chain]"
```

Expected: both pass without requiring a full cleaning run.

- [ ] **Step 6: Commit**

```bash
git add test/integration/hardware/system_hw_test.cc test/integration/hardware/hw_config.h test/CMakeLists.txt
git commit -m "test: migrate hardware system tests to robot controller"
```

---

### Task 16: Final Cleanup and Full Verification

**Files:**
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-06-01-robot-controller-fault-closure-design.md`

- [ ] **Step 1: Search for forbidden old business paths**

Run:

```bash
rg -n "FaultHandler|RobotSupervisor|RobotFsm|fault->report\\(.*P0|FaultService::decide|publish_status_event|register_rpc\\(\\s*\"reset\"" pv_cleaning_robot include test
```

Expected: no output.

- [ ] **Step 2: Search for direct business state mutation outside controller**

Run:

```bash
rg -n "state_\\s*=|mission_\\s*=|active_fault_|FaultStopped|SelfChecking|ExecutingMission|Recovering" pv_cleaning_robot include test
```

Expected: production state mutations appear in `robot_controller.cc`. Tests may assert states.

- [ ] **Step 3: Build unit tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build passes.

- [ ] **Step 4: Run unit tests**

Run:

```bash
./build/rk3576-build/test/unit_tests
```

Expected: all unit tests pass.

- [ ] **Step 5: Build hardware tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: build passes.

- [ ] **Step 6: Run selected hardware-safe tests**

Run:

```bash
./build/rk3576-build/test/hw_tests "[hw_system][watchdog_heartbeat]"
./build/rk3576-build/test/hw_tests "[hw_system][p0_fault_chain]"
```

Expected: both pass.

- [ ] **Step 7: Commit final cleanup**

```bash
git add README.md docs/superpowers/specs/2026-06-01-robot-controller-fault-closure-design.md
git commit -m "docs: align robot controller architecture docs"
```

When neither file changed, run `git status --short README.md docs/superpowers/specs/2026-06-01-robot-controller-fault-closure-design.md` and expect no output; do not create a final documentation commit.

---

## Self-Review

- Spec coverage: The plan covers controller ownership, event queue, P0 source closure, hard safety bypass, FaultPolicy, FaultDetector, ThingsBoard retained RPCs, reboot RPC removal, status event removal, health telemetry preservation, scheduler routing, watchdog routing, recovery routing, and old compatibility deletion.
- Placeholder scan: No task uses deferred placeholders. Each task lists concrete files, code snippets, commands, expected results, and commit scope.
- Type consistency: The plan consistently uses `RobotController`, `FaultPolicy`, `FaultDetector`, `FaultFact`, `FaultAction`, `FaultSource`, `CommandResult`, `RobotControllerSnapshot`, and retained `domain::RobotCommand`.
- Scope risk: This is a large refactor, but it is one coherent business-kernel change. The task order keeps each commit buildable and testable.
