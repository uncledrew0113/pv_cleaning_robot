# Cloud Service Thread Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the confirmed `CloudService::attr_cb_` data race and make shared-attribute delivery structurally safe regardless of callback registration timing.

**Architecture:** Keep `CloudService` as the routing point for RPC and shared attributes, but split callback publication from invocation. The implementation should snapshot the shared-attribute callback under a dedicated lock, release the lock, then invoke the snapshot outside the critical section. Tests should validate the new delivery seam and the late-registration behavior visible from `main.cc`.

**Tech Stack:** C++17, Catch2, existing `CloudService`, `NetworkManager`, `INetworkTransport`, `nlohmann::json`

---

## File Structure

- Create: `test/service/cloud_service_test.cc`
  - Unit tests for shared-attribute delivery semantics and the new delivery seam.
- Modify: `test/CMakeLists.txt`
  - Register the new service test.
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
  - Add a private delivery helper and separate attribute-callback lock/state.
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
  - Route subscribed attribute payloads through the helper, snapshot callback under lock, invoke outside the lock.
- Modify: `pv_cleaning_robot/main.cc`
  - Register shared-attribute callback before connecting the network so startup order is no longer relying on a race window.

### Task 1: Add a Deterministic Shared-Attribute Delivery Test Seam

**Files:**
- Create: `test/service/cloud_service_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing test file**

Create `test/service/cloud_service_test.cc` with this content:

```cpp
#include <catch2/catch.hpp>

#include <memory>
#include <string>

#include "pv_cleaning_robot/service/cloud_service.h"

using robot::middleware::DataCache;
using robot::middleware::INetworkTransport;
using robot::middleware::NetworkManager;
using robot::service::CloudService;

namespace {

struct MockTransport final : INetworkTransport {
    MessageCallback subscribed_cb;
    bool connect() override { return true; }
    void disconnect() override {}
    bool is_connected() const override { return true; }
    bool publish(const std::string&, const std::string&) override { return true; }
    bool subscribe(const std::string&, MessageCallback cb) override {
        subscribed_cb = std::move(cb);
        return true;
    }
    void emit(const std::string& topic, const std::string& payload) {
        REQUIRE(static_cast<bool>(subscribed_cb));
        subscribed_cb(topic, payload);
    }
};

}  // namespace

TEST_CASE("CloudService shared attributes ignore messages before callback registration",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_test.jsonl");
    CloudService cloud(net, cache);

    int call_count = 0;

    mqtt->emit("v1/devices/me/attributes", R"({"passes":2})");
    cloud.subscribe_shared_attributes([&](const nlohmann::json& attrs) {
        REQUIRE(attrs.at("passes").get<int>() == 3);
        ++call_count;
    });
    mqtt->emit("v1/devices/me/attributes", R"({"passes":3})");

    REQUIRE(call_count == 1);
}
```

- [ ] **Step 2: Register the new test in `test/CMakeLists.txt`**

Add this line under the existing service-layer tests:

```cmake
  service/cloud_service_test.cc
```

- [ ] **Step 3: Run the test target to confirm the new test fails**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The build fails because the test tries to emit messages before a callback exists and the current `MockTransport::emit()` expectation or current setup path is not yet aligned with a deterministic delivery seam.

- [ ] **Step 4: Commit the red test**

Run:

```bash
git add test/service/cloud_service_test.cc test/CMakeLists.txt
git commit -m "test: add cloud service shared attribute regression"
```

### Task 2: Remove the Data Race in `CloudService`

**Files:**
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`

- [ ] **Step 1: Add a dedicated shared-attribute delivery helper and lock**

In `include/pv_cleaning_robot/service/cloud_service.h`, make these changes:

```cpp
private:
    void on_rpc_message(const std::string& topic, const std::string& payload);
    void on_shared_attributes_message(const std::string& payload);

    static constexpr size_t kMaxRpcParamsBytes = 4096;

    std::shared_ptr<middleware::NetworkManager> network_;
    std::shared_ptr<middleware::DataCache> cache_;
    Topics topics_;
    std::unordered_map<std::string, RpcHandler> rpc_handlers_;
    mutable std::mutex rpc_mtx_;
    mutable std::mutex attr_cb_mtx_;
    AttrCallback attr_cb_;
```

- [ ] **Step 2: Route transport delivery through the new helper**

Replace the shared-attribute subscribe lambda in `pv_cleaning_robot/service/cloud_service.cc` with:

```cpp
    network_->subscribe(topics_.attributes,
        [this](const std::string& /*t*/, const std::string& p) {
            on_shared_attributes_message(p);
        });
```

Then implement the helper exactly like this:

```cpp
void CloudService::on_shared_attributes_message(const std::string& payload)
{
    AttrCallback cb;
    {
        std::lock_guard<std::mutex> lk(attr_cb_mtx_);
        cb = attr_cb_;
    }
    if (!cb) return;

    try {
        auto j = nlohmann::json::parse(payload);
        cb(j);
    } catch (...) {
    }
}
```

- [ ] **Step 3: Protect callback registration with the same lock**

Update `subscribe_shared_attributes()` to:

```cpp
void CloudService::subscribe_shared_attributes(AttrCallback cb)
{
    std::lock_guard<std::mutex> lk(attr_cb_mtx_);
    attr_cb_ = std::move(cb);
}
```

- [ ] **Step 4: Rebuild to confirm the implementation compiles**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The build succeeds.

- [ ] **Step 5: Commit the thread-safety fix**

Run:

```bash
git add include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc
git commit -m "fix: serialize cloud shared attribute callback access"
```

### Task 3: Remove Startup Ordering Dependence in `main.cc`

**Files:**
- Modify: `pv_cleaning_robot/main.cc`

- [ ] **Step 1: Move shared-attribute registration before network connect**

In `pv_cleaning_robot/main.cc`, change the startup order from:

```cpp
auto net_mgr =
    std::make_shared<robot::middleware::NetworkManager>(mqtt, lorawan_transport, net_mode);
net_mgr->connect();
...
auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
...
cloud->subscribe_shared_attributes([&cfg](const nlohmann::json& attrs) {
```

to:

```cpp
auto net_mgr =
    std::make_shared<robot::middleware::NetworkManager>(mqtt, lorawan_transport, net_mode);
...
auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
...
cloud->subscribe_shared_attributes([&cfg](const nlohmann::json& attrs) {
    if (attrs.contains("passes"))
        cfg.set("robot.passes", attrs["passes"].get<float>());
    if (attrs.contains("clean_speed_rpm"))
        cfg.set("robot.clean_speed_rpm", attrs["clean_speed_rpm"].get<float>());
    if (attrs.contains("return_speed_rpm"))
        cfg.set("robot.return_speed_rpm", attrs["return_speed_rpm"].get<float>());
    if (attrs.contains("brush_rpm"))
        cfg.set("robot.brush_rpm", attrs["brush_rpm"].get<int>());
    cfg.save();
});
net_mgr->connect();
```

- [ ] **Step 2: Rebuild the product target**

Run:

```bash
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected:
- The main executable still builds cleanly.

- [ ] **Step 3: Commit the startup-order cleanup**

Run:

```bash
git add pv_cleaning_robot/main.cc
git commit -m "refactor: register cloud attribute callback before connect"
```

### Task 4: Verify and Close Out the Cloud Service Changes

**Files:**
- Read: `test/service/cloud_service_test.cc`
- Read: `include/pv_cleaning_robot/service/cloud_service.h`
- Read: `pv_cleaning_robot/service/cloud_service.cc`

- [ ] **Step 1: Run the unit-test build again**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The test target compiles successfully with `cloud_service_test.cc` included.

- [ ] **Step 2: Run the executable build again**

Run:

```bash
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected:
- The executable build succeeds with no new source-level failures.

- [ ] **Step 3: Commit the final verification checkpoint**

Run:

```bash
git commit --allow-empty -m "chore: verify cloud service thread safety hardening"
```
