# Runtime Semantics And Codebase Re-Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct the runtime model so terminal-aware parking and charging semantics are first-class, then converge duplicated telemetry/config structures, repair tests that encode stale behavior, and synchronize the maintained docs.

**Architecture:** Introduce an explicit terminal semantics model that decouples business logic from physical `front/rear` sensor names. Feed that model through config parsing, runtime orchestration, FSM completion logic, telemetry snapshots, and startup attribute synchronization before converging duplicated builders and updating tests and docs.

**Tech Stack:** C++17, Boost.SML, nlohmann/json, Catch2, fixed-buffer JSON payload builders, MQTT/ThingsBoard integration

---

## File Structure

### New files

- Create: `include/pv_cleaning_robot/app/terminal_semantics.h`
  - Shared business enums and helpers for `TerminalSide`, `ParkingPolicy`, and `ChargingSide`
- Create: `test/app/terminal_semantics_test.cc`
  - Focused unit tests for terminal-policy helper behavior

### Existing files to modify

- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
  - Extend runtime config with terminal-aware semantics
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
  - Replace `at_home/at_front`-centric admission logic with terminal-aware runtime logic
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
  - Stop treating `Charging` as a generic task-end state and wire task completion around runtime terminal/charging truth
- Modify: `pv_cleaning_robot/main.cc`
  - Register shared attributes before network connect and pass terminal-aware runtime signals into supervisor
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
  - Accept terminal-aware admission callbacks and remove stale `home/front` assumptions
- Modify: `include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h`
- Modify: `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
- Modify: `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
- Modify: `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
- Modify: `include/pv_cleaning_robot/service/business_payload_builder.h`
- Modify: `pv_cleaning_robot/service/business_payload_builder.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_event_payload_builder.h`
- Modify: `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
  - Converge duplicated snapshot and payload-writer logic
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
  - Make shared-attribute startup behavior explicit and safe
- Modify: `include/pv_cleaning_robot/middleware/event_bus.h`
  - Make subscriber truncation observable
- Modify: `config/config.json`
  - Add `parking_policy` and `charging_side` sample config
- Modify: `README.md`
- Modify: `doc/API_REFERENCE.md`
- Modify: `doc/thingsboard_config.md`
  - Synchronize maintained docs with the corrected semantics

### Existing tests to modify

- Modify: `test/app/robot_supervisor_test.cc`
- Modify: `test/app/robot_fsm_test.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/service/cloud_service_test.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/service/thingsboard_telemetry_publisher_test.cc`
- Modify: `test/service/thingsboard_config_manager_test.cc`
- Modify: `test/CMakeLists.txt`

---

### Task 1: Add Terminal Semantics Baseline Tests

**Files:**
- Create: `include/pv_cleaning_robot/app/terminal_semantics.h`
- Create: `test/app/terminal_semantics_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing terminal semantics tests**

```cpp
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/terminal_semantics.h"

using robot::app::ChargingSide;
using robot::app::ParkingPolicy;
using robot::app::TerminalSide;
using robot::app::allows_half_pass;
using robot::app::can_start_from_terminal;
using robot::app::supports_charging_at;

TEST_CASE("Terminal semantics: half-pass requires both parking policy", "[app][terminal]") {
    CHECK_FALSE(allows_half_pass(ParkingPolicy::TerminalAOnly));
    CHECK_FALSE(allows_half_pass(ParkingPolicy::TerminalBOnly));
    CHECK(allows_half_pass(ParkingPolicy::Both));
}

TEST_CASE("Terminal semantics: start admission respects parking policy", "[app][terminal]") {
    CHECK(can_start_from_terminal(ParkingPolicy::TerminalAOnly, TerminalSide::A));
    CHECK_FALSE(can_start_from_terminal(ParkingPolicy::TerminalAOnly, TerminalSide::B));
    CHECK(can_start_from_terminal(ParkingPolicy::Both, TerminalSide::A));
    CHECK(can_start_from_terminal(ParkingPolicy::Both, TerminalSide::B));
}

TEST_CASE("Terminal semantics: charging support is independent from parking policy", "[app][terminal]") {
    CHECK(supports_charging_at(ChargingSide::TerminalA, TerminalSide::A));
    CHECK_FALSE(supports_charging_at(ChargingSide::TerminalA, TerminalSide::B));
    CHECK(supports_charging_at(ChargingSide::Both, TerminalSide::A));
    CHECK(supports_charging_at(ChargingSide::Both, TerminalSide::B));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[app][terminal]"`

Expected: compile or link failure because `terminal_semantics.h` and helpers do not exist yet.

- [ ] **Step 3: Write the minimal terminal semantics header**

```cpp
#pragma once

namespace robot::app {

enum class TerminalSide { Unknown, A, B };
enum class ParkingPolicy { TerminalAOnly, TerminalBOnly, Both };
enum class ChargingSide { TerminalA, TerminalB, Both };

inline bool allows_half_pass(ParkingPolicy policy) noexcept {
    return policy == ParkingPolicy::Both;
}

inline bool can_start_from_terminal(ParkingPolicy policy, TerminalSide side) noexcept {
    switch (policy) {
    case ParkingPolicy::TerminalAOnly:
        return side == TerminalSide::A;
    case ParkingPolicy::TerminalBOnly:
        return side == TerminalSide::B;
    case ParkingPolicy::Both:
        return side == TerminalSide::A || side == TerminalSide::B;
    }
    return false;
}

inline bool supports_charging_at(ChargingSide side_cfg, TerminalSide side) noexcept {
    switch (side_cfg) {
    case ChargingSide::TerminalA:
        return side == TerminalSide::A;
    case ChargingSide::TerminalB:
        return side == TerminalSide::B;
    case ChargingSide::Both:
        return side == TerminalSide::A || side == TerminalSide::B;
    }
    return false;
}

}  // namespace robot::app
```

- [ ] **Step 4: Register the new test source**

```cmake
    app/terminal_semantics_test.cc
```

Add the line above in `test/CMakeLists.txt` next to the other `test/app/*` sources.

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[app][terminal]"`

Expected: test compile passes; runtime execution may still be skipped on the host environment if `aarch64` execution is unavailable.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/terminal_semantics.h \
        test/app/terminal_semantics_test.cc \
        test/CMakeLists.txt
git commit -m "test: add terminal semantics baseline"
```

---

### Task 2: Extend Runtime Config With Parking And Charging Semantics

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Modify: `config/config.json`
- Test: `test/service/thingsboard_config_manager_test.cc`

- [ ] **Step 1: Write failing config-manager tests for parking and charging fields**

```cpp
TEST_CASE("ThingsBoardConfigManager persists parking and charging semantics",
          "[service][tb_config]") {
    auto result = mgr.apply_shared_attributes(nlohmann::json{
        {"parking_policy", "both"},
        {"charging_side", "terminal_a"}
    });

    REQUIRE(result.accepted);
    REQUIRE(mgr.pending_config().has_value());
    CHECK(mgr.pending_config()->parking_policy == robot::app::ParkingPolicy::Both);
    CHECK(mgr.pending_config()->charging_side == robot::app::ChargingSide::TerminalA);
}

TEST_CASE("ThingsBoardConfigManager rejects half-pass when parking policy is not both",
          "[service][tb_config]") {
    auto result = mgr.apply_shared_attributes(nlohmann::json{
        {"parking_policy", "left_only"},
        {"passes", 0.5}
    });

    REQUIRE_FALSE(result.accepted);
    CHECK(result.reason == "half_pass_requires_both_parking_policy");
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][tb_config]"`

Expected: compile or assertion failure because `TbRuntimeConfig` has no `parking_policy` or `charging_side`.

- [ ] **Step 3: Extend `TbRuntimeConfig` and supported fields**

```cpp
struct TbRuntimeConfig {
    double passes{1.0};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    robot::app::ParkingPolicy parking_policy{robot::app::ParkingPolicy::TerminalAOnly};
    robot::app::ChargingSide charging_side{robot::app::ChargingSide::TerminalA};
    std::vector<TbScheduleEntry> schedules;
};
```

And expand the supported field list:

```cpp
return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
       key == "brush_rpm" || key == "schedules" ||
       key == "parking_policy" || key == "charging_side";
```

- [ ] **Step 4: Add parsing and validation for the new string fields**

```cpp
static robot::app::ParkingPolicy parse_parking_policy(const nlohmann::json& value) {
    const auto text = value.get<std::string>();
    if (text == "left_only" || text == "terminal_a_only") {
        return robot::app::ParkingPolicy::TerminalAOnly;
    }
    if (text == "right_only" || text == "terminal_b_only") {
        return robot::app::ParkingPolicy::TerminalBOnly;
    }
    if (text == "both") {
        return robot::app::ParkingPolicy::Both;
    }
    throw std::runtime_error("invalid parking_policy");
}

static robot::app::ChargingSide parse_charging_side(const nlohmann::json& value) {
    const auto text = value.get<std::string>();
    if (text == "left" || text == "terminal_a") {
        return robot::app::ChargingSide::TerminalA;
    }
    if (text == "right" || text == "terminal_b") {
        return robot::app::ChargingSide::TerminalB;
    }
    if (text == "both") {
        return robot::app::ChargingSide::Both;
    }
    throw std::runtime_error("invalid charging_side");
}
```

Add the semantic guard:

```cpp
if (pending_root_after["robot"].value("passes", 1.0) == 0.5 &&
    parse_parking_policy(pending_root_after["robot"]["parking_policy"]) != robot::app::ParkingPolicy::Both) {
    throw std::runtime_error("half_pass_requires_both_parking_policy");
}
```

- [ ] **Step 5: Update the sample config**

```json
"robot": {
  "passes": 1.0,
  "parking_policy": "terminal_a_only",
  "charging_side": "terminal_a",
  "clean_speed_rpm": 300.0,
  "return_speed_rpm": 300.0,
  "return_brush_rpm": 1000,
  "brush_rpm": 1000
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][tb_config]"`

Expected: compile passes; runtime execution may still be skipped on the host environment if `aarch64` execution is unavailable.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_config_manager.h \
        pv_cleaning_robot/service/thingsboard_config_manager.cc \
        config/config.json \
        test/service/thingsboard_config_manager_test.cc
git commit -m "feat: add terminal-aware runtime config"
```

---

### Task 3: Repair Startup Attribute Synchronization

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Test: `test/service/cloud_service_test.cc`

- [ ] **Step 1: Rewrite the cloud-service test so startup shared attributes are not silently dropped**

```cpp
TEST_CASE("CloudService dispatches shared attributes after callback registration completes",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_test.jsonl");
    CloudService cloud(net, cache);

    int call_count = 0;
    cloud.subscribe_shared_attributes([&](const nlohmann::json& attrs) {
        REQUIRE(attrs.at("passes").get<int>() == 2);
        ++call_count;
    });

    mqtt->emit_attributes(R"({"passes":2})");
    REQUIRE(call_count == 1);
}
```

- [ ] **Step 2: Run test to verify current startup behavior fails the new expectation**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][cloud]"`

Expected: current test shape no longer matches desired startup behavior.

- [ ] **Step 3: Move shared-attribute callback installation ahead of network connect**

In `main.cc`, reorder these calls:

```cpp
tb_control->subscribe_shared_attributes();
tb_control->register_rpc_handlers(...);
net_mgr->connect();
```

instead of connecting the network before the shared-attribute callback is installed.

- [ ] **Step 4: Make callback presence explicit in `CloudService`**

Add a small helper and use it in the attribute path:

```cpp
bool CloudService::has_shared_attribute_callback() const
{
    std::lock_guard<std::mutex> lk(attr_cb_mtx_);
    return static_cast<bool>(attr_cb_);
}
```

And log startup loss if a message still arrives before wiring is complete:

```cpp
if (!cb) {
    spdlog::warn("[CloudService] Dropped shared attributes before callback installation");
    return;
}
```

- [ ] **Step 5: Run focused tests and compile the main target**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build succeeds with the new wiring order.

- [ ] **Step 6: Commit**

```bash
git add pv_cleaning_robot/main.cc \
        include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc \
        test/service/cloud_service_test.cc
git commit -m "fix: synchronize shared attribute startup wiring"
```

---

### Task 4: Introduce Terminal-Aware Runtime State In Supervisor And FSM

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Test: `test/app/robot_supervisor_test.cc`
- Test: `test/app/robot_fsm_test.cc`

- [ ] **Step 1: Add failing tests for terminal-aware admission and end-state semantics**

```cpp
TEST_CASE("RobotSupervisor allows integer-pass start from terminal_b when parking policy is both",
          "[app][robot_supervisor]") {
    f.apply_terminal_policy(robot::app::ParkingPolicy::Both, robot::app::ChargingSide::TerminalA);
    REQUIRE(f.supervisor->start_manual_task(robot::app::TerminalSide::B, 1.0));
}

TEST_CASE("RobotSupervisor maps half-pass completion at non-charging terminal to idle",
          "[app][robot_supervisor]") {
    f.apply_terminal_policy(robot::app::ParkingPolicy::Both, robot::app::ChargingSide::TerminalA);
    REQUIRE(f.supervisor->start_manual_task(robot::app::TerminalSide::A, 0.5));
    f.fsm->dispatch(EvFrontLimitSettled{});
    f.supervisor->sync_terminal_state(robot::app::TerminalSide::B);
    REQUIRE(f.supervisor->current_state() == "Idle");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: compile or assertion failures because supervisor and FSM do not model terminal-aware completion yet.

- [ ] **Step 3: Extend the runtime snapshot and supervisor state**

Add runtime fields:

```cpp
std::optional<robot::app::TerminalSide> start_terminal;
std::optional<robot::app::TerminalSide> current_terminal;
bool current_terminal_supports_charging{false};
```

Add supervisor methods:

```cpp
bool start_manual_task(robot::app::TerminalSide start_terminal, double passes);
bool start_scheduled_task(robot::app::TerminalSide start_terminal, double passes);
void sync_terminal_state(robot::app::TerminalSide current_terminal);
```

- [ ] **Step 4: Stop using `Charging` as the generic completion target**

Introduce an internal FSM completion event that separates “task finished” from “charge-capable parked state”:

```cpp
struct EvTaskParkedIdle {};
struct EvTaskParkedCharging {};
```

And update transition rows:

```cpp
state<StateCleanFwd>    + event<EvTaskParkedIdle>     = state<StateIdle>,
state<StateCleanFwd>    + event<EvTaskParkedCharging> = state<StateCharging>,
state<StateCleanReturn> + event<EvTaskParkedIdle>     = state<StateIdle>,
state<StateCleanReturn> + event<EvTaskParkedCharging> = state<StateCharging>,
state<StateReturning>   + event<EvTaskParkedIdle>     = state<StateIdle>,
state<StateReturning>   + event<EvTaskParkedCharging> = state<StateCharging>,
```

- [ ] **Step 5: Drive completion from terminal and charging truth in `RobotSupervisor`**

After limit-settled completion, the supervisor should decide:

```cpp
if (supports_charging_at(active_cfg.charging_side, current_terminal)) {
    fsm_->dispatch(EvTaskParkedCharging{});
} else {
    fsm_->dispatch(EvTaskParkedIdle{});
}
```

Also gate half-pass start legality:

```cpp
if (passes == 0.5 && !allows_half_pass(active_cfg.parking_policy)) {
    return false;
}
```

- [ ] **Step 6: Run focused tests and compile**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build succeeds with the new terminal-aware model skeleton.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_runtime_snapshot.h \
        include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc \
        test/app/robot_supervisor_test.cc \
        test/app/robot_fsm_test.cc
git commit -m "feat: add terminal-aware runtime semantics"
```

---

### Task 5: Update ThingsBoard Control And Telemetry To Use Terminal Truth

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h`
- Modify: `pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`
- Test: `test/service/thingsboard_telemetry_publisher_test.cc`

- [ ] **Step 1: Write failing tests for terminal-aware start and telemetry state**

```cpp
TEST_CASE("ThingsBoardControlPlane accepts start from terminal_b when parking policy is both",
          "[service][tb_control_plane]") {
    f.apply_terminal_policy(robot::app::ParkingPolicy::Both, robot::app::ChargingSide::TerminalA);
    f.control_plane->register_rpc_handlers(
        []() { return robot::app::TerminalSide::B; },
        []() { return robot::app::TerminalSide::B; });

    f.mqtt->emit_rpc("42", R"({"method":"start","params":{"passes":1}})");
    CHECK(f.fsm->current_state() == "CleanReturn");
}

TEST_CASE("ThingsBoardTelemetryPublisher includes terminal-aware runtime fields",
          "[service][tb_telemetry]") {
    auto payload = f.capture_business_payload();
    CHECK(payload.at("current_terminal").get<std::string>() == "terminal_b");
    CHECK(payload.at("current_terminal_supports_charging").get<bool>() == false);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][tb_control_plane]"`

Expected: compile or assertion failures because the API still uses home/front callbacks and telemetry lacks terminal fields.

- [ ] **Step 3: Update the control-plane callback contract**

Replace the old callbacks:

```cpp
void register_rpc_handlers(const std::function<TerminalSide()>& current_terminal);
```

And route `start` through terminal-aware supervisor methods instead of `at_home/at_front`.

- [ ] **Step 4: Add terminal fields to telemetry publishing**

Update the business telemetry payload to include:

```cpp
"start_terminal"
"current_terminal"
"current_terminal_supports_charging"
"parking_policy"
"charging_side"
```

Populate them from the unified runtime snapshot rather than recomputing them in the publisher.

- [ ] **Step 5: Run focused tests and compile**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build succeeds with terminal-aware control and telemetry interfaces.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        include/pv_cleaning_robot/service/thingsboard_telemetry_publisher.h \
        pv_cleaning_robot/service/thingsboard_telemetry_publisher.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/service/thingsboard_telemetry_publisher_test.cc
git commit -m "feat: wire terminal semantics into thingsboard runtime"
```

---

### Task 6: Converge Snapshot And Payload-Builder Duplication

**Files:**
- Modify: `include/pv_cleaning_robot/service/business_telemetry_snapshot.h`
- Modify: `pv_cleaning_robot/service/business_telemetry_snapshot.cc`
- Modify: `include/pv_cleaning_robot/service/business_payload_builder.h`
- Modify: `pv_cleaning_robot/service/business_payload_builder.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_event_payload_builder.h`
- Modify: `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- Test: `test/service/business_payload_builder_test.cc`
- Test: `test/service/business_telemetry_snapshot_test.cc`
- Test: `test/service/thingsboard_event_payload_builder_test.cc`

- [ ] **Step 1: Write tests that prove one runtime snapshot shape drives both telemetry and event payloads**

```cpp
TEST_CASE("Business payload builder serializes unified runtime snapshot fields",
          "[service][business_payload]") {
    robot::app::RobotRuntimeSnapshot snap;
    snap.device_state = "Idle";
    snap.current_terminal = robot::app::TerminalSide::B;
    snap.current_terminal_supports_charging = false;

    std::array<char, 1024> buf{};
    const auto len = BusinessPayloadBuilder::build(snap, buf.data(), buf.size());
    REQUIRE(len > 0);
    const auto json = nlohmann::json::parse(std::string(buf.data(), len));
    CHECK(json.at("current_terminal").get<std::string>() == "terminal_b");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target unit_tests -j4 && ./build/aarch64/bin/unit_tests "[service][business_payload]"`

Expected: type mismatch or compile failure because `BusinessPayloadBuilder` still depends on `BusinessTelemetrySnapshot`.

- [ ] **Step 3: Collapse the duplicate business snapshot layer**

Change:

```cpp
static size_t build(const BusinessTelemetrySnapshot& view, char* out, size_t cap) noexcept;
```

to:

```cpp
static size_t build(const robot::app::RobotRuntimeSnapshot& view, char* out, size_t cap) noexcept;
```

and delete or demote the product-facing `to_json()` path:

```cpp
// Remove product-path use of BusinessTelemetrySnapshot::to_json()
// Keep only test-only helpers if still needed.
```

- [ ] **Step 4: Extract shared fixed-buffer JSON helpers**

Introduce a shared helper in the existing builders by moving the repeated writer and phase-mapping code into one place:

```cpp
namespace {
class FixedJsonWriter { /* shared implementation */ };
const char* command_phase_name(CommandPhase phase) noexcept;
}
```

One builder source should own the common implementation; the other should reuse it through a small internal header if needed.

- [ ] **Step 5: Run focused tests and compile**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build succeeds and the duplicate JSON path is no longer part of the product flow.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/business_telemetry_snapshot.h \
        pv_cleaning_robot/service/business_telemetry_snapshot.cc \
        include/pv_cleaning_robot/service/business_payload_builder.h \
        pv_cleaning_robot/service/business_payload_builder.cc \
        include/pv_cleaning_robot/service/thingsboard_event_payload_builder.h \
        pv_cleaning_robot/service/thingsboard_event_payload_builder.cc \
        test/service/business_payload_builder_test.cc \
        test/service/business_telemetry_snapshot_test.cc \
        test/service/thingsboard_event_payload_builder_test.cc
git commit -m "refactor: converge runtime telemetry payload encoding"
```

---

### Task 7: Repair Tests That Encode Stale Charging Semantics

**Files:**
- Modify: `test/app/robot_fsm_test.cc`
- Modify: `test/app/robot_supervisor_test.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/service/cloud_service_test.cc`

- [ ] **Step 1: Replace stale “end means Charging” assertions with terminal-aware expectations**

Example updates:

```cpp
SECTION("half pass ends idle when terminal_b cannot charge") {
    f.apply_terminal_policy(robot::app::ParkingPolicy::Both, robot::app::ChargingSide::TerminalA);
    REQUIRE(f.supervisor->start_manual_task(robot::app::TerminalSide::A, 0.5));
    f.fsm->dispatch(EvFrontLimitSettled{});
    f.supervisor->sync_terminal_state(robot::app::TerminalSide::B);
    REQUIRE(f.supervisor->current_state() == "Idle");
}
```

And:

```cpp
SECTION("integer pass returns to start terminal and charges only if that terminal can charge") {
    f.apply_terminal_policy(robot::app::ParkingPolicy::Both, robot::app::ChargingSide::TerminalB);
    REQUIRE(f.supervisor->start_manual_task(robot::app::TerminalSide::B, 1.0));
    // drive task completion...
    REQUIRE(f.supervisor->current_state() == "Charging");
}
```

- [ ] **Step 2: Replace the cloud startup-loss test expectation**

```cpp
TEST_CASE("CloudService warns or blocks until shared attribute callback is installed",
          "[service][cloud]") {
    // This test should no longer assert that startup attribute messages are silently dropped.
}
```

- [ ] **Step 3: Run focused tests and compile**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build succeeds with the new terminal-aware test expectations.

- [ ] **Step 4: Commit**

```bash
git add test/app/robot_fsm_test.cc \
        test/app/robot_supervisor_test.cc \
        test/integration/task_chain_test.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/service/cloud_service_test.cc
git commit -m "test: replace stale charging semantics assumptions"
```

---

### Task 8: Synchronize Maintained Documentation And Config Guidance

**Files:**
- Modify: `README.md`
- Modify: `doc/API_REFERENCE.md`
- Modify: `doc/thingsboard_config.md`
- Modify: `config/config.json`

- [ ] **Step 1: Rewrite the user-facing semantics in `doc/thingsboard_config.md`**

Update the control/config description to say:

```markdown
- `passes = 0.5` is valid only when `parking_policy = both`
- task completion may end in `Idle` or `Charging` depending on `charging_side`
- integer-pass tasks end at the terminal where that task started
- business-facing runtime terminology uses `terminal_a` / `terminal_b`
```

- [ ] **Step 2: Fix stale API reference sections**

Replace the old scheduler API description:

```cpp
void set_on_window_hit(TriggerCallback cb);
void set_on_window_expired(TriggerCallback cb);
```

Remove stale `CleanTask` references and add:

```markdown
- `Paused`
- `Terminated`
- terminal-aware runtime semantics
```

Also update the Charging state description:

```markdown
`StateCharging` — parked at a valid terminal and charging conditions are satisfied
```

- [ ] **Step 3: Fix the README runtime semantics summary**

Update the configuration section:

```markdown
| `robot.passes` | Task passes. `0.5` is valid only when parking policy allows both terminals. |
| `robot.parking_policy` | `terminal_a_only` / `terminal_b_only` / `both` |
| `robot.charging_side` | `terminal_a` / `terminal_b` / `both` |
```

And correct the OTA note so it matches the actual build behavior.

- [ ] **Step 4: Run a final compile-only verification**

Run: `cmake --build build --target unit_tests pv_cleaning_robot -j4`

Expected: build still succeeds after config and doc synchronization.

- [ ] **Step 5: Commit**

```bash
git add README.md \
        doc/API_REFERENCE.md \
        doc/thingsboard_config.md \
        config/config.json
git commit -m "docs: align runtime semantics and config guidance"
```

---

## Self-Review

### Spec coverage

- Runtime semantic drift around parking, charging, and terminal-aware starts is covered by Tasks 1 through 5.
- Snapshot and payload-builder duplication is covered by Task 6.
- Test drift is covered by Task 7.
- Documentation and config drift is covered by Task 8.
- Startup shared-attribute source-of-truth loss is covered by Task 3.

No approved spec requirement is currently without a matching task.

### Placeholder scan

- No `TBD`, `TODO`, or “implement later” placeholders remain.
- All tasks include concrete files, code, and commands.

### Type consistency

- Terminal business vocabulary is consistently `TerminalSide`, `ParkingPolicy`, and `ChargingSide`.
- Runtime truth convergence consistently uses `RobotRuntimeSnapshot` as the intended long-term payload input.
- ThingsBoard ingress consistently moves from `at_home/at_front` callbacks toward terminal-aware callbacks.

