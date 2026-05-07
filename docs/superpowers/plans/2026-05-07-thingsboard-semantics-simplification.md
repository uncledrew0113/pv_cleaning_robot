# ThingsBoard Semantics Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Simplify the ThingsBoard runtime model so config, sensor interpretation, supervisor admission, FSM flow, telemetry, and tests all use one left/right parking-side semantics.

**Architecture:** The implementation removes the old multi-terminal model instead of adapting it. First shrink the config model to `parking_side` plus task parameters, then centralize sensor-to-business mapping, then simplify supervisor and FSM to a strict out-and-back task flow, and finally update ThingsBoard RPC/telemetry plumbing and tests to match.

**Tech Stack:** C++17, RapidJSON, Catch2, existing service/app layers, ThingsBoard MQTT integration

---

## File Structure

- `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
  Owns the simplified shared-attribute schema and the runtime config struct.
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
  Validates and persists `parking_side`-based config, schedule handling, and pending promotion.
- `include/pv_cleaning_robot/app/parking_side_runtime.h`
  New narrow helper that converts physical left/right sensor truth into business facts (`at_parking_side`, `at_far_end`).
- `include/pv_cleaning_robot/app/robot_supervisor.h`
  Declares the reduced supervisor API with one task-start entry point.
- `pv_cleaning_robot/app/robot_supervisor.cc`
  Implements start/pause/return/terminate/reset admission using parking-side facts.
- `include/pv_cleaning_robot/app/robot_fsm.h`
  Declares the simplified start event payload and state-direction semantics.
- `pv_cleaning_robot/app/robot_fsm.cc`
  Implements strict integer-pass out-and-back flow.
- `include/pv_cleaning_robot/service/cloud_service.h`
  Extends the RPC handler contract to pass `request_id`.
- `pv_cleaning_robot/service/cloud_service.cc`
  Extracts the request id once and passes it through the handler chain.
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
  Declares parking-side-aware RPC registration helpers.
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
  Uses the new supervisor API and preserves RPC request ids in command events.
- `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
  Removes deleted config fields from telemetry payloads.
- `pv_cleaning_robot/main.cc`
  Wires left/right sensor facts to the new parking-side helper and new supervisor API.
- `test/service/thingsboard_config_manager_test.cc`
  Rewritten for `parking_side` + integer-pass config only.
- `test/app/terminal_semantics_test.cc`
  Replaced with parking-side sensor mapping tests.
- `test/app/robot_supervisor_test.cc`
  Rewritten for unified start admission and parking-side-aware reset/return behavior.
- `test/service/cloud_service_test.cc`
  Rewritten to verify RPC request-id propagation.
- `test/service/thingsboard_control_plane_test.cc`
  Rewritten for parking-side semantics and preserved request ids.
- `test/integration/thingsboard_runtime_mock_integration_test.cc`
  Rewritten to validate live shared-attribute `parking_side`, strict out-and-back state flow, and telemetry consistency.

### Task 1: Simplify the ThingsBoard Config Schema

**Files:**
- Create: `none`
- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Test: `test/service/thingsboard_config_manager_test.cc`

- [ ] **Step 1: Write the failing config-manager tests**

```cpp
TEST_CASE("ThingsBoardConfigManager accepts parking_side left and right only",
          "[service][tb_config]") {
    Fixture f;

    auto left_attrs = parse_json(R"({"parking_side":"left","passes":2.0})");
    REQUIRE(f.manager->apply_shared_attributes(left_attrs).accepted);
    REQUIRE(f.manager->pending_config().has_value());
    CHECK(f.manager->pending_config()->parking_side ==
          robot::service::ParkingSide::Left);

    auto bad_attrs = parse_json(R"({"parking_side":"both"})");
    const auto bad = f.manager->apply_shared_attributes(bad_attrs);
    CHECK_FALSE(bad.accepted);
    CHECK(bad.reason == "parking_side must be left or right");
}

TEST_CASE("ThingsBoardConfigManager rejects non-integer passes",
          "[service][tb_config]") {
    Fixture f;

    auto attrs = parse_json(R"({"passes":0.5})");
    const auto result = f.manager->apply_shared_attributes(attrs);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "passes must be a positive integer");
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build --preset rk3576-build --target unit_tests && ./build/aarch64/bin/unit_tests "[service][tb_config]"`

Expected: FAIL because `ParkingSide` and `"parking_side"` handling do not exist yet, and old tests still expect `parking_policy` / `charging_side`.

- [ ] **Step 3: Replace the config enums and struct fields with the minimal model**

```cpp
enum class ParkingSide {
    Left,
    Right,
};

inline const char* parking_side_config_string(ParkingSide value) noexcept {
    switch (value) {
    case ParkingSide::Left:
        return "left";
    case ParkingSide::Right:
        return "right";
    }
    return "left";
}

struct TbRuntimeConfig {
    double passes{1.0};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    ParkingSide parking_side{ParkingSide::Left};
    std::vector<TbScheduleEntry> schedules;
    // keep operator== updated to use parking_side only
};
```

- [ ] **Step 4: Implement the minimal parsing and validation changes**

```cpp
ParkingSide parse_parking_side_string(const std::string& value)
{
    if (value == "left") {
        return ParkingSide::Left;
    }
    if (value == "right") {
        return ParkingSide::Right;
    }
    throw std::runtime_error("parking_side must be left or right");
}

bool is_supported_field(const std::string& key)
{
    return key == "passes" || key == "clean_speed_rpm" ||
           key == "return_speed_rpm" || key == "brush_rpm" ||
           key == "parking_side" || key == "schedules";
}

void validate_runtime_config(const TbRuntimeConfig& cfg)
{
    if (!is_integer_passes(cfg.passes)) {
        throw std::runtime_error("passes must be a positive integer");
    }
}
```

- [ ] **Step 5: Update shared-attribute persistence to use `parking_side`**

```cpp
if (const auto it = attrs.FindMember("parking_side"); it != attrs.MemberEnd()) {
    if (!it->value.IsString()) {
        throw std::runtime_error("parking_side must be left or right");
    }
    const auto side = parse_parking_side_string(it->value.GetString());
    set_string_member(*robot,
                      "parking_side",
                      parking_side_config_string(side),
                      pending_root_after.GetAllocator());
    touches_pending = true;
}
```

- [ ] **Step 6: Run the config-manager tests**

Run: `./build/aarch64/bin/unit_tests "[service][tb_config]"`

Expected: PASS with no remaining references to `parking_policy` or `charging_side`.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/thingsboard_config_manager.h \
        pv_cleaning_robot/service/thingsboard_config_manager.cc \
        test/service/thingsboard_config_manager_test.cc
git commit -m "refactor: simplify thingsboard config semantics"
```

### Task 2: Centralize Parking-Side Sensor Mapping

**Files:**
- Create: `include/pv_cleaning_robot/app/parking_side_runtime.h`
- Modify: `pv_cleaning_robot/main.cc`
- Test: `test/app/terminal_semantics_test.cc`

- [ ] **Step 1: Write the failing sensor-mapping tests**

```cpp
TEST_CASE("Parking side runtime maps left parking side to front_limit",
          "[app][parking_side]") {
    const auto facts = robot::app::ParkingSideRuntime::from_physical_limits(
        robot::service::ParkingSide::Left,
        /*front_limit_active=*/true,
        /*rear_limit_active=*/false);

    CHECK(facts.at_parking_side);
    CHECK_FALSE(facts.at_far_end);
}

TEST_CASE("Parking side runtime maps right parking side to rear_limit",
          "[app][parking_side]") {
    const auto facts = robot::app::ParkingSideRuntime::from_physical_limits(
        robot::service::ParkingSide::Right,
        /*front_limit_active=*/false,
        /*rear_limit_active=*/true);

    CHECK(facts.at_parking_side);
    CHECK_FALSE(facts.at_far_end);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./build/aarch64/bin/unit_tests "[app][parking_side]"`

Expected: FAIL because `ParkingSideRuntime` does not exist yet.

- [ ] **Step 3: Add the narrow runtime mapping helper**

```cpp
namespace robot::app {

struct ParkingSideFacts {
    bool at_parking_side{false};
    bool at_far_end{false};
};

struct ParkingSideRuntime {
    static ParkingSideFacts from_physical_limits(service::ParkingSide parking_side,
                                                 bool front_limit_active,
                                                 bool rear_limit_active) {
        if (parking_side == service::ParkingSide::Left) {
            return {.at_parking_side = front_limit_active,
                    .at_far_end = rear_limit_active};
        }
        return {.at_parking_side = rear_limit_active,
                .at_far_end = front_limit_active};
    }
};

}  // namespace robot::app
```

- [ ] **Step 4: Update `main.cc` to stop passing old home/front truth**

```cpp
const auto facts = robot::app::ParkingSideRuntime::from_physical_limits(
    tb_cfg->active_config().parking_side,
    front_open_ok && !front_switch->read_current_level(),
    rear_open_ok && !rear_switch->read_current_level());

if (!supervisor->start_task(facts.at_parking_side, facts.at_far_end)) {
    log->warn("[Main] 调度启动被拒绝");
}
```

- [ ] **Step 5: Run the mapping tests**

Run: `./build/aarch64/bin/unit_tests "[app][parking_side]"`

Expected: PASS, proving business logic can now reason in parking-side terms instead of physical names.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/parking_side_runtime.h \
        pv_cleaning_robot/main.cc \
        test/app/terminal_semantics_test.cc
git commit -m "refactor: centralize parking side sensor mapping"
```

### Task 3: Reduce `RobotSupervisor` to One Start Path

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Write the failing supervisor tests**

```cpp
TEST_CASE("RobotSupervisor start_task requires parking-side position",
          "[app][robot_supervisor]") {
    SupervisorFixture f;

    REQUIRE_FALSE(f.supervisor->start_task(false, false));
    REQUIRE(f.fsm->current_state() == "Idle");
}

TEST_CASE("RobotSupervisor start_task promotes pending config before launch",
          "[app][robot_supervisor]") {
    SupervisorFixture f;
    f.apply_pending_runtime_attrs(parse_json(R"({"passes":3.0,"parking_side":"right"})"));

    REQUIRE(f.supervisor->start_task(true, false));
    REQUIRE(f.tb_cfg->active_config().passes == Approx(3.0));
}
```

- [ ] **Step 2: Run the supervisor tests**

Run: `./build/aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: FAIL because `start_task()` does not exist and old tests still call `start_manual_task()` / `start_scheduled_task()`.

- [ ] **Step 3: Replace the duplicated start API in the header**

```cpp
class RobotSupervisor {
public:
    bool start_task(bool at_parking_side, bool at_far_end);
    bool pause_task();
    bool return_task();
    bool terminate_task();
    bool reset_task(bool at_parking_side);
    RobotRuntimeSnapshot snapshot() const;
};
```

- [ ] **Step 4: Implement the minimal unified admission logic**

```cpp
bool RobotSupervisor::start_task(bool at_parking_side, bool /*at_far_end*/) {
    if (!is_new_task_start_state(fsm_->current_state()) || !at_parking_side) {
        return false;
    }
    if (tb_cfg_->has_pending_config() && !tb_cfg_->promote_pending_to_active()) {
        return false;
    }
    fsm_->dispatch(EvTaskStart{static_cast<float>(tb_cfg_->active_config().passes)});
    return fsm_->current_state() == "CleanOut" ||
           fsm_->current_state() == "CleanFwd";
}
```

- [ ] **Step 5: Remove old cadence and legacy start helpers**

```cpp
// delete:
// bool start_scheduled_task(bool at_home, bool at_front);
// bool start_manual_task(bool at_home, bool at_front);
// int desired_cloud_period_ms(int active_ms, int idle_ms) const;
```

- [ ] **Step 6: Run the supervisor tests**

Run: `./build/aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: PASS, with no remaining test references to `at_home` or duplicated start methods.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h \
        pv_cleaning_robot/app/robot_supervisor.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: simplify robot supervisor start flow"
```

### Task 4: Simplify the FSM to Strict Out-and-Back Passes

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Write the failing state-flow tests**

```cpp
TEST_CASE("One pass completes only after out-and-back cycle",
          "[app][robot_supervisor][fsm]") {
    SupervisorFixture f;

    REQUIRE(f.supervisor->start_task(true, false));
    REQUIRE(f.fsm->current_state() == "CleanOut");

    f.fsm->dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm->current_state() == "CleanBack");

    f.fsm->dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm->current_state() == "Charging");
    REQUIRE(f.fsm->completed_passes() == 1);
}
```

- [ ] **Step 2: Run the FSM-focused tests**

Run: `./build/aarch64/bin/unit_tests "[app][robot_supervisor][fsm]"`

Expected: FAIL because the current FSM still uses `EvScheduleStart`, half-pass counting, and front/rear completion semantics.

- [ ] **Step 3: Replace the start and limit events with parking-side semantics**

```cpp
struct EvTaskStart {
    float passes{1.0f};
};

struct EvFarEndLimitSettled {};
struct EvParkingSideLimitSettled {};
```

- [ ] **Step 4: Update the transition table to strict outward/return flow**

```cpp
return make_transition_table(
    *state<StateInit> + event<EvInitDone> = state<StateIdle>,
    state<StateIdle> + event<EvTaskStart> = state<StateCleanOut>,
    state<StateCharging> + event<EvTaskStart> = state<StateCleanOut>,
    state<StateCleanOut> + event<EvFarEndLimitSettled> = state<StateCleanBack>,
    state<StateCleanBack> + event<EvParkingSideLimitSettled> = state<StateCleanOut>,
    state<StateCleanBack> + event<EvTaskComplete> = state<StateCharging>,
    state<StateReturning> + event<EvParkingSideLimitSettled> = state<StateCharging>
);
```

- [ ] **Step 5: Implement pass counting as full cycles**

```cpp
if (sm_->is(sml::state<StateCleanBack>)) {
    completed_passes_++;
    if (completed_passes_ >= target_passes_) {
        sm_->process_event(EvTaskComplete{});
        state_name_ = "Charging";
        action = [this]() { motion_->stop_cleaning(); };
    } else {
        sm_->process_event(EvParkingSideLimitSettled{});
        state_name_ = "CleanOut";
        action = [this]() { motion_->start_cleaning(); };
    }
}
```

- [ ] **Step 6: Run the FSM and supervisor tests**

Run: `./build/aarch64/bin/unit_tests "[app][robot_supervisor]"`

Expected: PASS with no remaining normal-task completion path at the far end.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc \
        test/app/robot_supervisor_test.cc
git commit -m "refactor: simplify task fsm pass semantics"
```

### Task 5: Preserve RPC `request_id` Through the Command Path

**Files:**
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Test: `test/service/cloud_service_test.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: Write the failing request-id tests**

```cpp
TEST_CASE("CloudService passes RPC request_id to the handler",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_rpc_test.jsonl");
    CloudService cloud(net, cache);

    std::string seen_request_id;
    cloud.register_rpc("start", [&](const std::string& request_id,
                                    const std::string&) {
        seen_request_id = request_id;
        return std::string{R"({"ok":true})"};
    });

    mqtt->emit_rpc("42", R"({"method":"start","params":{}})");
    CHECK(seen_request_id == "42");
}
```

- [ ] **Step 2: Run the RPC tests**

Run: `./build/aarch64/bin/unit_tests "[service][cloud]" "[service][tb_control_plane]"`

Expected: FAIL because the RPC handler signature does not accept `request_id`, and command events still store an empty request id.

- [ ] **Step 3: Extend the `CloudService` handler type**

```cpp
class CloudService : public middleware::IRunnable {
public:
    using RpcHandler =
        std::function<std::string(const std::string& request_id,
                                  const std::string& params)>;
};
```

- [ ] **Step 4: Pass the extracted id into the handler**

```cpp
response = it->second(request_id, params);
```

- [ ] **Step 5: Thread the id through `ThingsBoardControlPlane` command tracking**

```cpp
std::string ThingsBoardControlPlane::complete_rpc_command(const char* command_name,
                                                          const std::string& request_id,
                                                          const char* completion_reason) {
    const auto cmd_id = command_tracker_->accept(command_name, request_id);
    publish_command_event("command_accepted", *command_tracker_->active());
    command_tracker_->mark_running(cmd_id);
    command_tracker_->finish_success(cmd_id, completion_reason);
    publish_command_event("command_completed", *command_tracker_->last_completed());
    return rpc_reply(true);
}
```

- [ ] **Step 6: Run the RPC and control-plane tests**

Run: `./build/aarch64/bin/unit_tests "[service][cloud]" "[service][tb_control_plane]"`

Expected: PASS, and command events now preserve `request_id`.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc \
        include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc \
        test/service/cloud_service_test.cc \
        test/service/thingsboard_control_plane_test.cc
git commit -m "fix: preserve thingsboard rpc request ids"
```

### Task 6: Remove Deleted Semantics From Telemetry and Integration Tests

**Files:**
- Modify: `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`
- Test: `test/integration/thingsboard_runtime_mock_integration_test.cc`

- [ ] **Step 1: Write the failing telemetry assertions**

```cpp
TEST_CASE("ThingsBoard business telemetry emits parking_side only",
          "[service][tb_control_plane][telemetry]") {
    Fixture f;
    f.control_plane->publish_business_telemetry();

    const auto j = f.last_published_json("telemetry");
    REQUIRE(j["active_config"].IsObject());
    CHECK(std::string(j["active_config"]["parking_side"].GetString()) == "left");
    CHECK(j["active_config"].FindMember("parking_policy") ==
          j["active_config"].MemberEnd());
    CHECK(j["active_config"].FindMember("charging_side") ==
          j["active_config"].MemberEnd());
}
```

- [ ] **Step 2: Run the telemetry and mock integration tests**

Run: `./build/aarch64/bin/unit_tests "[service][tb_control_plane][telemetry]" "[integration][thingsboard][runtime_mock]"`

Expected: FAIL because payloads still serialize deleted fields and integration tests still assume old state names and start semantics.

- [ ] **Step 3: Remove deleted config fields from the payload builder**

```cpp
template <typename WriterT>
void write_runtime_config(const char* key, const TbRuntimeConfig& config, WriterT& writer) {
    writer.Key(key);
    writer.StartObject();
    writer.Key("passes");
    writer.Double(config.passes);
    writer.Key("clean_speed_rpm");
    writer.Double(config.clean_speed_rpm);
    writer.Key("return_speed_rpm");
    writer.Double(config.return_speed_rpm);
    writer.Key("brush_rpm");
    writer.Int(config.brush_rpm);
    writer.Key("parking_side");
    writer.String(parking_side_config_string(config.parking_side));
    writer.Key("schedules");
    write_schedule_entries(config.schedules, writer);
    writer.EndObject();
}
```

- [ ] **Step 4: Update `main.cc` event wiring to the new limit semantics**

```cpp
event_bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
    [&fsm, &tb_cfg, &log](const robot::middleware::SafetyMonitor::LimitSettledEvent& evt) {
        const bool parking_left =
            tb_cfg->active_config().parking_side == robot::service::ParkingSide::Left;
        const bool parking_side_hit =
            (parking_left && evt.side == robot::device::LimitSide::FRONT) ||
            (!parking_left && evt.side == robot::device::LimitSide::REAR);

        if (parking_side_hit) {
            log->info("[Limit] 停机侧防抖完成");
            fsm->dispatch(robot::app::EvParkingSideLimitSettled{});
        } else {
            log->info("[Limit] 对侧防抖完成");
            fsm->dispatch(robot::app::EvFarEndLimitSettled{});
        }
    });
```

- [ ] **Step 5: Rewrite the mock integration expectations around the new model**

```cpp
REQUIRE(f.tb_cfg->apply_shared_attributes(parse_json(
    R"({"parking_side":"right","passes":1.0})")).accepted);
REQUIRE(f.connect());
REQUIRE(f.wait_state_with_cloud({"CleanOut"}, std::chrono::seconds(30)));
```

- [ ] **Step 6: Run the final focused test suite**

Run: `./build/aarch64/bin/unit_tests "[service][tb_config]" "[app][parking_side]" "[app][robot_supervisor]" "[service][cloud]" "[service][tb_control_plane]" "[integration][thingsboard][runtime_mock]"`

Expected: PASS, with no remaining references to removed terminal or half-pass semantics in ThingsBoard-related coverage.

- [ ] **Step 7: Commit**

```bash
git add pv_cleaning_robot/service/thingsboard_event_payload_builder.cc \
        pv_cleaning_robot/main.cc \
        test/service/thingsboard_control_plane_test.cc \
        test/integration/thingsboard_runtime_mock_integration_test.cc
git commit -m "refactor: align thingsboard telemetry with parking side semantics"
```
