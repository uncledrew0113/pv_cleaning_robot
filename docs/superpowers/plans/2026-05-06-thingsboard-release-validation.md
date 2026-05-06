# ThingsBoard Release Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add real-board ThingsBoard validation coverage for telemetry, RPC, cloud configuration, and runtime state transitions; document platform-side validation steps; and add focused inline comments to the core ThingsBoard code path.

**Architecture:** Keep the current `MqttTransport -> NetworkManager -> CloudService -> ThingsBoardControlPlane/ThingsBoardConfigManager` path unchanged. Extend the existing real ThingsBoard smoke test for connection and configuration validation, add real hardware/runtime validation through the existing hardware fixture stack, and update operator-facing documentation to match the currently supported release scope only.

**Tech Stack:** C++17, Catch2, RapidJSON, Paho MQTT C++, spdlog, existing `hw_config.h` fixtures, existing ThingsBoard MQTT mutual TLS configuration

---

## File Map

- Modify: `test/integration/thingsboard_real_integration_test.cc`
  - Real cloud smoke tests
  - Real shared-attribute validation tests
  - Real rejected-shared-attribute validation tests
- Modify: `test/integration/hardware/hw_config.h`
  - Add a ThingsBoard-aware hardware/runtime fixture that reuses real board devices and current cloud stack without going through `main`
- Modify: `test/integration/hardware/system_hw_test.cc`
  - Add real hardware + real ThingsBoard runtime validation tests driven by platform RPC/manual steps
- Modify: `doc/thingsboard_config.md`
  - Rewrite to actual implemented release scope
  - Add platform-side setup and test execution procedure
- Modify: `include/pv_cleaning_robot/middleware/mqtt_transport.h`
- Modify: `pv_cleaning_robot/middleware/mqtt_transport.cc`
  - Add targeted comments for TLS verification behavior
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
  - Add comments for topic routing and callback behavior
- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
  - Add comments for active/pending/promotion rules and release restrictions
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
  - Add comments for ingress/egress responsibilities

## Task 1: Expand Real ThingsBoard Smoke And Config Validation

**Files:**
- Modify: `test/integration/thingsboard_real_integration_test.cc`
- Test: `test/integration/thingsboard_real_integration_test.cc`

- [ ] **Step 1: Write the failing shared-attribute acceptance test**

Add two new real integration test cases below the existing publish test:

```cpp
TEST_CASE("Real ThingsBoard shared attributes update local pending config",
          "[integration][thingsboard][real][shared_attr]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard shared attribute test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.net->connect());

    spdlog::warn("[TB real test] ACTION REQUIRED: in ThingsBoard shared attributes, set "
                 "passes=2, clean_speed_rpm=320, charging_side=terminal_a");

    REQUIRE(wait_until(
        [&] {
            const auto pending = f.tb_cfg->pending_config();
            return pending.has_value() &&
                   pending->passes == Approx(2.0) &&
                   pending->clean_speed_rpm == Approx(320.0) &&
                   pending->charging_side == robot::service::ChargingSide::TerminalA;
        },
        std::chrono::seconds(120)));
}

TEST_CASE("Real ThingsBoard rejects unsupported shared attributes for this release",
          "[integration][thingsboard][real][shared_attr][rejected]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard rejected shared attribute test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.net->connect());

    const auto before = f.tb_cfg->active_config();
    spdlog::warn("[TB real test] ACTION REQUIRED: in ThingsBoard shared attributes, set "
                 "passes=0.5 or parking_policy=both");

    REQUIRE(wait_until(
        [&] { return f.tb_cfg->active_config() == before && !f.tb_cfg->pending_config().has_value(); },
        std::chrono::seconds(120)));
}
```

- [ ] **Step 2: Add the minimal wait helper and file checks**

Add helper code near the current local helpers:

```cpp
template <typename Pred>
bool wait_until(Pred pred, std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return pred();
}

bool file_contains_key(const fs::path& path, const char* key)
{
    robot::service::ConfigService cfg(path.string());
    if (!cfg.load()) {
        return false;
    }
    return cfg.snapshot().HasMember(key);
}
```

Also add concrete checks inside the shared-attribute test for:

```cpp
const auto pending_cfg_path = paths->repo_root / "config" / "config.pending.json";
REQUIRE(fs::exists(pending_cfg_path));
```

- [ ] **Step 3: Run the shared-attribute tests to verify they fail before platform action**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/unit_tests "[integration][thingsboard][real][shared_attr]"
```

Expected:

- the test starts
- it logs the required platform action
- it fails by timeout unless the matching ThingsBoard shared attributes are actually set

- [ ] **Step 4: Implement the final operator-assisted checks**

Add explicit config assertions after `wait_until(...)`:

```cpp
const auto pending = f.tb_cfg->pending_config();
REQUIRE(pending.has_value());
CHECK(pending->passes == Approx(2.0));
CHECK(pending->clean_speed_rpm == Approx(320.0));
CHECK(pending->charging_side == robot::service::ChargingSide::TerminalA);
```

And for the rejected case:

```cpp
CHECK(f.tb_cfg->active_config() == before);
CHECK_FALSE(f.tb_cfg->pending_config().has_value());
```

- [ ] **Step 5: Run the tests again with platform interaction**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/unit_tests "[integration][thingsboard][real]"
```

Expected:

- smoke tests still pass
- shared-attribute acceptance test passes after platform-side attribute update
- rejected test passes after platform-side unsupported attribute update leaves runtime unchanged

- [ ] **Step 6: Commit**

```bash
git add test/integration/thingsboard_real_integration_test.cc
git commit -m "test: add real thingsboard shared attribute validation"
```

## Task 2: Add Real Hardware + Real ThingsBoard RPC And State Validation

**Files:**
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Test: `test/integration/hardware/system_hw_test.cc`

- [ ] **Step 1: Write the failing hardware/runtime fixture for ThingsBoard**

Add a new fixture in `hw_config.h` that reuses the current real-hardware stack plus the current cloud stack:

```cpp
struct ThingsBoardRuntimeFixture : FullSystemFixture {
    std::string tb_cache_path{"/tmp/hw_tb_runtime_cache.jsonl"};
    robot::service::ConfigService tb_cfg_file{"config/config.json"};
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net_mgr;
    std::shared_ptr<robot::middleware::DataCache> data_cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::shared_ptr<robot::service::ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<robot::service::CommandTracker> command_tracker;
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;
    std::shared_ptr<robot::service::ThingsBoardControlPlane> tb_control;

    bool init_thingsboard_runtime() {
        if (!FullSystemFixture::init(p.health_jsonl_path)) {
            return false;
        }

        REQUIRE(tb_cfg_file.load());
        auto mqtt_cfg = robot::middleware::MqttTransport::Config{};
        mqtt_cfg.broker_uri = tb_cfg_file.get<std::string>("network.mqtt.broker_uri", "");
        mqtt_cfg.client_id = tb_cfg_file.get<std::string>("network.mqtt.client_id", "pv_robot_001");
        mqtt_cfg.username = tb_cfg_file.get<std::string>("network.mqtt.username", "");
        mqtt_cfg.password = tb_cfg_file.get<std::string>("network.mqtt.password", "");
        mqtt_cfg.tls_enabled = tb_cfg_file.get<bool>("network.mqtt.tls_enabled", false);
        mqtt_cfg.ca_cert_path = tb_cfg_file.get<std::string>("network.mqtt.ca_cert_path", "");
        mqtt_cfg.client_cert_path = tb_cfg_file.get<std::string>("network.mqtt.client_cert_path", "");
        mqtt_cfg.client_key_path = tb_cfg_file.get<std::string>("network.mqtt.client_key_path", "");
        mqtt_cfg.insecure_skip_server_name_check =
            tb_cfg_file.get<bool>("network.mqtt.insecure_skip_server_name_check", false);
        mqtt_cfg.keep_alive_sec = tb_cfg_file.get<int>("network.mqtt.keep_alive_s", 60);
        mqtt_cfg.qos = tb_cfg_file.get<int>("network.mqtt.qos", 1);

        mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);
        net_mgr = std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
        data_cache = std::make_shared<robot::middleware::DataCache>(tb_cache_path);
        REQUIRE(data_cache->open());
        cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
        tb_cfg = std::make_shared<robot::service::ThingsBoardConfigManager>(tb_cfg_file, scheduler);
        command_tracker = std::make_shared<robot::service::CommandTracker>();
        supervisor = std::make_shared<robot::app::RobotSupervisor>(fsm, tb_cfg, command_tracker, fault, nav);
        tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
            tb_cfg_file, cloud, tb_cfg, command_tracker, supervisor);

        tb_control->subscribe_shared_attributes();
        tb_control->register_rpc_handlers(
            [this] { return !rear_sw->read_current_level(); },
            [this] { return !front_sw->read_current_level(); });

        return net_mgr->connect();
    }
};
```

- [ ] **Step 2: Write the failing RPC/state tests in `system_hw_test.cc`**

Add two new tests:

```cpp
TEST_CASE("System（真实硬件）ThingsBoard RPC start/stop drives runtime state",
          "[hw_system][tb_rpc_runtime]") {
    hw::ThingsBoardRuntimeFixture f;
    REQUIRE(f.init_thingsboard_runtime());

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: invoke RPC start from ThingsBoard");
    REQUIRE(f.wait_state("CleanFwd", std::chrono::milliseconds(120000)));

    f.tb_control->publish_business_telemetry();
    CHECK(f.fsm->current_state() == "CleanFwd");

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: invoke RPC stop from ThingsBoard");
    REQUIRE(f.wait_state("Paused", std::chrono::milliseconds(120000)));

    f.tb_control->publish_business_telemetry();
    CHECK(f.fsm->current_state() == "Paused");
}

TEST_CASE("System（真实硬件）ThingsBoard RPC return/reset updates runtime and telemetry",
          "[hw_system][tb_state_telemetry]") {
    hw::ThingsBoardRuntimeFixture f;
    REQUIRE(f.init_thingsboard_runtime());

    spdlog::warn("[hw_system][tb_state_telemetry] ACTION REQUIRED: invoke RPC start then return");
    REQUIRE(f.wait_state("CleanFwd", std::chrono::milliseconds(120000)));
    REQUIRE(f.wait_state("CleanReturn", std::chrono::milliseconds(120000)));

    f.tb_control->publish_business_telemetry();
    CHECK(f.fsm->current_state() == "CleanReturn");
}
```

- [ ] **Step 3: Run the hardware/runtime tests to confirm they fail before platform action**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_rpc_runtime]"
```

Expected:

- test starts
- board connects to ThingsBoard
- test times out unless the operator actually invokes RPC from the platform

- [ ] **Step 4: Add minimal local telemetry evidence capture**

Inside each test, immediately after `publish_business_telemetry()`, read the local health/telemetry artifacts that already exist in the fixture and check the runtime state locally:

```cpp
CHECK(f.fsm->current_state() == "CleanFwd");
CHECK(f.supervisor->snapshot().device_state == "CleanFwd");
```

and later:

```cpp
CHECK(f.fsm->current_state() == "Paused");
CHECK(f.supervisor->snapshot().device_state == "Paused");
```

This keeps the tests grounded in actual runtime-state change, not only the RPC response path.

- [ ] **Step 5: Run both hardware/runtime tests with real platform actions**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_rpc_runtime]"
TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_state_telemetry]"
```

Expected:

- `start` produces `CleanFwd`
- `stop` produces `Paused`
- `return` produces `CleanReturn`
- telemetry publish calls succeed while those states are active

- [ ] **Step 6: Commit**

```bash
git add test/integration/hardware/hw_config.h \
        test/integration/hardware/system_hw_test.cc
git commit -m "test: add real thingsboard hardware runtime validation"
```

## Task 3: Rewrite Operator-Facing ThingsBoard Validation Documentation

**Files:**
- Modify: `doc/thingsboard_config.md`
- Test: `doc/thingsboard_config.md` (manual review)

- [ ] **Step 1: Replace the outdated capability list with the actual release scope**

Rewrite the supported-scope section to match the current code exactly:

```md
#### 1、控制类（通过 RPC 实现）

- 启动：`start`
- 停止：`stop`
- 返回：`return`
- 终止：`terminate`
- 复位：`reset`

#### 2、遥测类（通过遥测实现）

- startup attributes
- command events
- shared attribute result events
- business telemetry:
  - `device_state`
  - `task_state`
  - `target_half_passes`
  - `completed_half_passes`
  - `active_config`
  - `pending_config`
  - `last_command`

#### 3、配置类（通过共享属性实现）

- `passes`（仅正整数）
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy`（仅 `terminal_a_only` / `terminal_b_only`）
- `charging_side`
- `schedules`（每日多个时刻）
```

- [ ] **Step 2: Add explicit unsupported-items section**

Add:

```md
#### 当前版本明确不支持

- `passes = 0.5`
- `parking_policy = both`
- 星期几调度规则
- 欠压/温度/过流/任务超时共享属性配置
- LoRaWAN / OTA 验收范围
```

- [ ] **Step 3: Add platform-side setup instructions**

Write the exact operator steps:

```md
#### 平台侧准备

1. 创建设备并绑定当前客户端证书
2. 确认 MQTT over TLS 入口可达
3. 配置共享属性面板，至少包含：
   - `passes`
   - `clean_speed_rpm`
   - `return_speed_rpm`
   - `brush_rpm`
   - `parking_policy`
   - `charging_side`
   - `schedules`
4. 配置 RPC 调用入口：
   - `start`
   - `stop`
   - `return`
   - `terminate`
   - `reset`
5. 在仪表盘上暴露：
   - `device_state`
   - `task_state`
   - `target_half_passes`
   - `completed_half_passes`
   - `active_config`
   - `pending_config`
   - `last_command`
```

- [ ] **Step 4: Add real-board validation procedure**

Add a concrete manual procedure:

```md
#### 真实板子联调顺序

1. 先运行：
   `TB_REAL_TEST=1 ./build/aarch64/bin/unit_tests "[integration][thingsboard][real]"`
2. 在平台侧完成 shared attribute 修改
3. 再运行：
   `TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_rpc_runtime]"`
4. 依次触发：
   - `start`
   - `stop`
   - `return`
   - `reset`
5. 观察仪表盘中的状态变化与事件上报
```

- [ ] **Step 5: Review the doc for contradictions**

Check that `doc/thingsboard_config.md` no longer claims support for:

- half-pass
- both-side parking
- weekday schedules
- unsupported protection attributes

Expected:

- document matches actual code scope exactly

- [ ] **Step 6: Commit**

```bash
git add doc/thingsboard_config.md
git commit -m "docs: align thingsboard config guide with current release scope"
```

## Task 4: Add Focused Inline Comments To Core ThingsBoard Files

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/mqtt_transport.h`
- Modify: `pv_cleaning_robot/middleware/mqtt_transport.cc`
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- Modify: `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Modify: `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`

- [ ] **Step 1: Add TLS comment in `MqttTransport::Config`**

Add:

```cpp
bool insecure_skip_server_name_check{false};  ///< 仅临时调试/验收使用：保留 CA 链校验，但跳过 broker 主机名/IP 与证书 SAN/CN 的匹配检查
```

And above the TLS block in `connect()`:

```cpp
// 注意：enable_server_cert_auth=true 负责 CA/服务端证书链认证；
// verify=true 还会继续做主机名/IP 与证书 SAN/CN 匹配检查。
// 当前开关仅用于真实板子验收阶段临时绕过“证书签给域名、实际用 IP 连接”的场景。
```

- [ ] **Step 2: Add routing comments in `CloudService`**

Add comments near constructor subscriptions and handlers:

```cpp
// CloudService 只负责 ThingsBoard topic 级别路由：
// - RPC request topic -> method handler
// - shared attributes topic -> attr callback
// 不在这里承载业务语义判定。
```

and:

```cpp
// 共享属性消息在这里解析并分发，但 active/pending 规则由
// ThingsBoardConfigManager 负责，CloudService 只做传输层入口。
```

- [ ] **Step 3: Add active/pending comments in `ThingsBoardConfigManager`**

Add comments like:

```cpp
// 当前版本规则：
// - schedules 立即进入 active，并同步写入 pending 视图
// - 任务参数先进入 pending，等待下一任务提升
// - passes=0.5 / parking_policy=both 在本版本显式拒绝
```

and:

```cpp
// promote_pending_to_active() 是“下一任务前提升”的唯一正式入口。
```

- [ ] **Step 4: Add responsibility comments in `ThingsBoardControlPlane`**

Add comments like:

```cpp
// ThingsBoardControlPlane 负责：
// 1. shared attributes ingress -> ThingsBoardConfigManager
// 2. RPC ingress -> RobotSupervisor / CommandTracker
// 3. startup/status/command/business telemetry egress
// 不负责直接持久化配置，也不负责定义任务状态机语义。
```

and for `register_rpc_handlers(...)`:

```cpp
// 这里的 is_at_home / is_at_front 仅提供当前物理位置观测，
// 具体是否允许启动/复位由 RobotSupervisor 当前规则决定。
```

- [ ] **Step 5: Build after comment-only edits**

Run:

```bash
cmake --build build --target unit_tests pv_cleaning_robot hw_tests -j4
```

Expected:

- clean compile
- no behavior change introduced

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/middleware/mqtt_transport.h \
        pv_cleaning_robot/middleware/mqtt_transport.cc \
        include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc \
        include/pv_cleaning_robot/service/thingsboard_config_manager.h \
        pv_cleaning_robot/service/thingsboard_config_manager.cc \
        include/pv_cleaning_robot/service/thingsboard_control_plane.h \
        pv_cleaning_robot/service/thingsboard_control_plane.cc
git commit -m "docs: clarify thingsboard runtime behavior in code comments"
```

## Task 5: Final Release-Validation Sweep

**Files:**
- Modify: `test/integration/thingsboard_real_integration_test.cc` (only if small tag/log cleanup is needed)
- Modify: `test/integration/hardware/system_hw_test.cc` (only if timeout/log cleanup is needed)
- Modify: `doc/thingsboard_config.md` (only if validation wording needs final alignment)

- [ ] **Step 1: Run the real cloud smoke suite**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/unit_tests "[integration][thingsboard][real]"
```

Expected:

- real TLS connection succeeds
- startup attributes publish succeeds
- business telemetry publish succeeds
- shared-attribute validation cases pass when the operator performs the instructed platform action

- [ ] **Step 2: Run the hardware/runtime suite**

Run:

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_rpc_runtime]"
TB_REAL_TEST=1 ./build/aarch64/bin/hw_tests "[hw_system][tb_state_telemetry]"
```

Expected:

- `start`, `stop`, `return`, and `reset` produce observable runtime state changes
- telemetry publish calls succeed while those runtime states are active

- [ ] **Step 3: Verify platform-side observation checklist**

Confirm in ThingsBoard dashboard or telemetry browser that the board exposes:

```text
device_state
task_state
target_half_passes
completed_half_passes
active_config
pending_config
last_command
```

Expected:

- each field is visible and changes during at least one real task-state sequence

- [ ] **Step 4: Write the final release verdict note in the doc**

Append a short operator note to `doc/thingsboard_config.md`:

```md
#### 快速上线验收结论

只有在以下四项全部完成后，当前版本才可判定为 ThingsBoard 路径满足快速上线基本要求：

- 遥测可见
- RPC 可控
- 云端配置可下发并按规则生效
- 至少一轮真实状态变化可在平台侧观察到
```

- [ ] **Step 5: Run the compile sanity build one last time**

Run:

```bash
cmake --build build -j4
```

Expected:

- `pv_cleaning_robot`
- `unit_tests`
- `hw_tests`

all build successfully

- [ ] **Step 6: Commit**

```bash
git add test/integration/thingsboard_real_integration_test.cc \
        test/integration/hardware/system_hw_test.cc \
        doc/thingsboard_config.md
git commit -m "test: finalize thingsboard release validation flow"
```

## Self-Review

### Spec coverage

- Real-board connectivity: covered by Task 1 and Task 5
- Real shared-attribute validation: covered by Task 1
- Real RPC validation: covered by Task 2
- Real state-transition + telemetry validation: covered by Task 2 and Task 5
- Platform-side required work: covered by Task 3
- Inline code comments: covered by Task 4
- Current release scope only: enforced by Task 3 and the exact supported/unsupported lists

### Placeholder scan

- No `TODO`/`TBD`
- All tasks name exact files
- All test steps include concrete commands
- All code steps include concrete code blocks

### Type consistency

- Reused current class names:
  - `MqttTransport`
  - `CloudService`
  - `ThingsBoardConfigManager`
  - `ThingsBoardControlPlane`
  - `RobotSupervisor`
  - `FullSystemFixture`
- Reused current telemetry field names:
  - `device_state`
  - `task_state`
  - `target_half_passes`
  - `completed_half_passes`
  - `active_config`
  - `pending_config`
  - `last_command`
