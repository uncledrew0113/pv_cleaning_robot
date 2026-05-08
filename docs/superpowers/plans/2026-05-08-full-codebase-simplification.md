# Full Codebase Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce file count, merge over-split ThingsBoard code, remove redundant runtime and test code, and improve beginner-readable comments without adding new abstraction layers.

**Architecture:** Consolidate the ThingsBoard runtime chain into one primary implementation unit, tighten `main`/`RobotSupervisor` runtime ownership around that shape, then collapse repeated test fixtures into existing shared support. Keep the runtime flow direct: fewer files, fewer indirections, clearer comments, and deleted obsolete compatibility code instead of preserving parallel paths.

**Tech Stack:** C++17, RapidJSON, Catch2, spdlog, CMake

---

### Task 1: Consolidate the ThingsBoard service implementation

**Files:**
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Modify: `include/pv_cleaning_robot/service/motion_service.h`
- Modify: `include/pv_cleaning_robot/app/robot_runtime_snapshot.h`
- Modify: `include/pv_cleaning_robot/app/parking_side_runtime.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `pv_cleaning_robot/main.cc`
- Delete: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Delete: `include/pv_cleaning_robot/service/thingsboard_event_payload_builder.h`
- Delete: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Delete: `pv_cleaning_robot/service/thingsboard_event_payload_builder.cc`

- [ ] **Step 1: Move runtime-config and payload declarations into the primary ThingsBoard header**

```cpp
// include/pv_cleaning_robot/service/thingsboard_control_plane.h
namespace robot::service {

enum class ParkingSide { Left, Right };
const char* parking_side_config_string(ParkingSide value) noexcept;

struct TbScheduleEntry {
    int hour;
    int minute;
};
struct TbRuntimeConfig {
    double passes;
    double clean_speed_rpm;
    double return_speed_rpm;
    int brush_rpm;
    int return_brush_rpm;
    ParkingSide parking_side;
    double start_battery_soc;
    double charge_start_soc;
    double charge_stop_soc;
    std::vector<TbScheduleEntry> schedules;
};
struct SharedAttrApplyResult {
    bool accepted;
    std::string reason;
};

class ThingsBoardJsonCodec {
public:
    struct StartupAttributesView {
        const char* software_version;
        const char* hardware_version;
        const char* device_model;
        const char* device_id;
    };
    struct StatusEventView {
        const char* event_name;
        bool accepted;
        const char* reason;
    };
    struct CommandEventView {
        const char* event_name;
        const CommandSnapshot* command;
    };
    static size_t build_startup_attributes(const StartupAttributesView& view,
                                           char* out,
                                           size_t cap) noexcept;
    static size_t build_status_event(const StatusEventView& view,
                                     char* out,
                                     size_t cap) noexcept;
    static size_t build_command_event(const CommandEventView& view,
                                      char* out,
                                      size_t cap) noexcept;
    static size_t build_business_telemetry(const app::RobotRuntimeSnapshot& view,
                                           char* out,
                                           size_t cap) noexcept;
};

class ThingsBoardControlPlane {
public:
    SharedAttrApplyResult apply_shared_attributes(const rapidjson::Value& attrs);
    bool promote_pending_to_active();
    TbRuntimeConfig active_config() const;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const;
    void subscribe_shared_attributes();
    void request_shared_attributes_snapshot() const;
    void register_rpc_handlers(const std::function<bool()>& is_start_position_valid,
                               const std::function<bool()>& is_at_start_parking_side,
                               const std::function<bool()>& is_at_active_parking_side,
                               const std::function<float()>& current_battery_soc,
                               std::function<void()> reboot_device);
};
}
```

- [ ] **Step 2: Inline the old config-manager and payload-builder implementation into the primary ThingsBoard source**

```cpp
// pv_cleaning_robot/service/thingsboard_control_plane.cc
namespace {
bool is_supported_tb_field(const std::string& key) {
    return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
           key == "brush_rpm" || key == "return_brush_rpm" || key == "parking_side" ||
           key == "start_battery_soc" || key == "charge_start_soc" ||
           key == "charge_stop_soc" || key == "schedules";
}
void validate_runtime_config(const TbRuntimeConfig& cfg) {
    /* keep the existing integer-passes, SOC-range, and threshold-order checks */
}
rapidjson::Document merge_runtime_root(const rapidjson::Value& active_root,
                                       const rapidjson::Value& pending_patch) {
    /* keep the existing deep object merge used by active + pending runtime config */
}
}

SharedAttrApplyResult ThingsBoardControlPlane::apply_shared_attributes(
    const rapidjson::Value& attrs) { /* move old config-manager body here */ }
bool ThingsBoardControlPlane::promote_pending_to_active() { /* move old config-manager body here */ }
size_t ThingsBoardJsonCodec::build_business_telemetry(
    const app::RobotRuntimeSnapshot& view, char* out, size_t cap) noexcept {
    /* move old payload-builder body here */
}
```

- [ ] **Step 3: Keep `CloudService` generic, but remove duplicated ThingsBoard reply construction**

```cpp
// keep topic routing in CloudService, but use one local helper for
// shared-attributes parsing and one reply-shape builder instead of scattered JSON generation
std::string build_rpc_response(bool accepted, const char* reason);
std::optional<rapidjson::Document> parse_small_object_json(const std::string& payload,
                                                           size_t pool_bytes);
```

- [ ] **Step 4: Update all includes and call sites to use the consolidated public surface**

```cpp
// examples
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

motion->set_parking_side_provider([tb_control]() {
    return tb_control->active_config().parking_side;
});
motion->set_runtime_config_provider([tb_control]() {
    return tb_control->active_config();
});
```

- [ ] **Step 5: Add boundary comments for beginner readability**

```cpp
// Explain in one place:
// 1. which shared attributes take effect immediately
// 2. which ones wait until next task
// 3. why schedules are special
// 4. how RPC/result/event/telemetry map to local runtime truth
```

- [ ] **Step 6: Build and run focused ThingsBoard tests**

Run: `cmake --build build --target unit_tests -j4`
Expected: build succeeds

Run: `./build/aarch64/bin/unit_tests "[service][tb_control_plane]" "[service][tb_config]" "[service][tb_json_codec]"`
Expected: all touched ThingsBoard unit tests pass

### Task 2: Simplify app-layer runtime orchestration without adding builders

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`

- [ ] **Step 1: Replace scattered `main.cc` runtime-policy blocks with a few local static functions**

```cpp
namespace {
ParkingSideFacts current_parking_facts(robot::service::ThingsBoardControlPlane& tb,
                                       const std::function<ParkingSideFacts(robot::service::ParkingSide)>& facts_for,
                                       bool use_pending_for_start);
void publish_startup_position_status(robot::service::ThingsBoardControlPlane& tb,
                                     const ParkingSideFacts& startup_facts,
                                     const std::shared_ptr<robot::middleware::Logger>& log);
void register_limit_settled_bridge(robot::middleware::EventBus& bus,
                                   const std::shared_ptr<robot::app::RobotFsm>& fsm,
                                   robot::service::ThingsBoardControlPlane& tb,
                                   const std::function<float()>& current_battery_soc,
                                   const std::shared_ptr<robot::middleware::Logger>& log);
void register_scheduler_start(robot::service::SchedulerService& scheduler,
                              const std::shared_ptr<robot::app::RobotSupervisor>& supervisor,
                              const std::function<ParkingSideFacts()>& start_facts,
                              const std::function<float()>& current_battery_soc,
                              const std::shared_ptr<robot::middleware::Logger>& log);
}
```

- [ ] **Step 2: Keep `RobotSupervisor` as the single admission point for `start`, `stop`, and `return`**

```cpp
// RobotSupervisor
bool start_task(bool at_parking_side, bool position_valid, float battery_soc);
bool stop_task();
bool return_task(bool at_parking_side);

// collapse repeated state predicates near these entry points
static bool is_new_task_start_state(const std::string& state);
static bool is_return_allowed_state(const std::string& state);
```

- [ ] **Step 3: Remove repeated active/pending config lookup patterns from app call sites**

```cpp
const auto runtime_cfg = tb_cfg_->has_pending_config() ? *tb_cfg_->pending_config()
                                                       : tb_cfg_->active_config();
if (battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)) {
    return false;
}
```

- [ ] **Step 4: Tighten `RobotFsm` comments and remove low-value local sprawl**

```cpp
// keep dispatch specializations, but rewrite comments around:
// - start only from parking side
// - out-and-back pass semantics
// - stop vs return vs charge completion
```

- [ ] **Step 5: Run focused app-layer tests**

Run: `./build/aarch64/bin/unit_tests "[app][robot_supervisor]" "[app][robot_fsm]" "[integration][task_chain]"`
Expected: app/runtime-flow tests pass

### Task 3: Clean adjacent service and middleware redundancy exposed by the consolidation

**Files:**
- Modify: `pv_cleaning_robot/service/config_service.cc`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `pv_cleaning_robot/service/health_service.cc`
- Modify: `include/pv_cleaning_robot/middleware/network_manager.h`
- Modify: `pv_cleaning_robot/middleware/network_manager.cc`
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
- Modify: `pv_cleaning_robot/middleware/safety_monitor.cc`

- [ ] **Step 1: Remove low-value duplication in config and runtime sync helpers**

```cpp
// examples:
// - keep one JSON clone path per concern
// - keep one write/rollback path per persisted config operation
// - trim repeated config lookups where one grouped read is enough
```

- [ ] **Step 2: Clarify runtime-sync comments where middleware participates in business truth**

```cpp
// SafetyMonitor: physical emergency stop + debounced business-edge handoff
// DataCache: offline durability only, not business-state truth
// NetworkManager: transport multiplexing only, not cloud semantics owner
```

- [ ] **Step 3: Keep changes surgical and delete dead helper code rather than wrapping it**

```cpp
// if a helper is now one-call-site-only after consolidation,
// inline or delete it instead of preserving pseudo-reuse
```

- [ ] **Step 4: Run touched middleware/service tests**

Run: `./build/aarch64/bin/unit_tests "[middleware]" "[service][config]" "[service][motion]" "[integration][system]"`
Expected: touched non-TB runtime tests pass

### Task 4: Consolidate duplicated test fixtures into shared support

**Files:**
- Modify: `test/integration/thingsboard_test_support.h`
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/service/thingsboard_control_plane_test.cc`
- Modify: `test/app/robot_supervisor_test.cc`
- Modify: `test/integration/task_chain_test.cc`
- Modify: `test/integration/thingsboard_runtime_mock_integration_test.cc`
- Modify: `test/integration/thingsboard_real_integration_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Expand existing shared support instead of inventing a new fixture framework**

```cpp
// test/integration/thingsboard_test_support.h
struct TempSplitConfigPaths {
    fs::path runtime_path;
    fs::path fixed_path;
    fs::path pending_path;
    fs::path backup_path;
    fs::path cache_path;
};
void write_runtime_config(const fs::path& path, const std::string& json_text);
void write_fixed_config(const fs::path& path, const std::string& json_text);
robot::service::MotionService::Config make_test_motion_config();
```

- [ ] **Step 2: Factor repeated runtime/TB object-graph setup into direct shared helpers**

```cpp
struct MockThingsBoardRuntime {
    robot::service::ConfigService cfg;
    robot::service::SchedulerService scheduler;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::shared_ptr<robot::service::ThingsBoardControlPlane> tb;
    std::shared_ptr<robot::service::CommandTracker> command_tracker;
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;
};
```

- [ ] **Step 3: Replace near-copy fixtures in service/app/integration tests with shared setup**

```cpp
// each test file should keep only:
// - scenario-specific knobs
// - scenario-specific assertions
// and delete repeated config-file writes and repeated object-graph wiring
```

- [ ] **Step 4: Keep hardware shared support in `hw_config.h`, but reuse the common TB support where appropriate**

```cpp
inline robot::middleware::MqttTransport::Config build_tb_mqtt_config(
    robot::service::ConfigService& cfg, const std::filesystem::path& repo_root) {
    return tb_test_support::build_mqtt_config(cfg, repo_root, "_hw_tb_itest");
}
```

- [ ] **Step 5: Run the full touched test slices**

Run: `./build/aarch64/bin/unit_tests "[service][tb_control_plane]" "[app][robot_supervisor]" "[integration][task_chain]" "[integration][thingsboard]"`
Expected: consolidated fixture users pass

### Task 5: Final deletion pass and verification

**Files:**
- Modify: `pv_cleaning_robot/CMakeLists.txt`
- Modify: `test/CMakeLists.txt`
- Modify: any touched headers/sources from Tasks 1-4

- [ ] **Step 1: Remove deleted ThingsBoard files from build lists**

```cmake
# delete references to:
# service/thingsboard_config_manager.cc
# service/thingsboard_event_payload_builder.cc
```

- [ ] **Step 2: Remove obsolete test includes and stale file references**

```cmake
# update test source lists to match the reduced file set
```

- [ ] **Step 3: Run full build for the test target**

Run: `cmake --build build --target unit_tests -j4`
Expected: build succeeds without missing file references

- [ ] **Step 4: Run broader regression coverage**

Run: `./build/aarch64/bin/unit_tests "[app]" "[service]" "[middleware]" "[integration]"`
Expected: touched software-path regression suite passes

- [ ] **Step 5: Review result for the intended simplification outcome**

```bash
git diff --stat
rg -n "thingsboard_config_manager|thingsboard_event_payload_builder" include pv_cleaning_robot test
```

Expected:
- fewer files than before in the ThingsBoard service area
- no stale references to deleted files
- comments present at the consolidated runtime boundaries
