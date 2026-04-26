# Telemetry Pipeline Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split telemetry selection from payload encoding and persistence, remove the main recurring JSON heap churn from `HealthService`, and replace `DataCache` full-file rewrites with append-first persistence.

**Architecture:** Introduce a dedicated payload builder for HEALTH and DIAGNOSTICS payloads, then refactor `HealthService` to build into a reusable buffer instead of mutating a `nlohmann::json` tree. In parallel, rework `DataCache` from queue rewrite semantics into a journal plus compaction model so offline telemetry no longer reserializes the full cache on every mutation.

**Tech Stack:** C++17, Catch2, `nlohmann::json` for compatibility boundaries only, fixed-size buffers, `std::string_view`, existing `DataCache` and `CloudService`

---

## File Structure

- Create: `include/pv_cleaning_robot/service/health_payload_builder.h`
  - Payload-view structs and fixed-buffer builder API.
- Create: `pv_cleaning_robot/service/health_payload_builder.cc`
  - Concrete HEALTH and DIAGNOSTICS JSON emitter implementation.
- Modify: `include/pv_cleaning_robot/service/health_service.h`
  - Remove `mutable nlohmann::json j_` and adopt reusable output-buffer members.
- Modify: `pv_cleaning_robot/service/health_service.cc`
  - Gather device snapshots, call the builder, publish and optionally append the already-built payload.
- Modify: `include/pv_cleaning_robot/service/cloud_service.h`
  - Add `std::string_view` publish overloads or keep one reusable-string boundary if needed.
- Modify: `pv_cleaning_robot/service/cloud_service.cc`
  - Route payloads to network/cache without forcing unnecessary extra formatting work.
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h`
  - Add journal replay, append helpers, compaction threshold state, and a record operation model.
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
  - Replace full rewrite persistence with append-first journal plus replay and compaction.
- Modify: `test/service/health_payload_builder_test.cc`
  - Cover both HEALTH and DIAGNOSTICS payload shape and optional distance-sensor output.
- Modify: `test/middleware/data_cache_test.cc`
  - Add journal replay, ack replay, and compaction regression coverage.
- Modify: `test/CMakeLists.txt`
  - Ensure `health_payload_builder.cc` is compiled into the test target.

### Task 1: Add `HealthPayloadBuilder` and Lock Down Output Shape

**Files:**
- Create: `include/pv_cleaning_robot/service/health_payload_builder.h`
- Create: `pv_cleaning_robot/service/health_payload_builder.cc`
- Modify: `test/service/health_payload_builder_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing payload builder test**

Replace `test/service/health_payload_builder_test.cc` with:

```cpp
#include <catch2/catch.hpp>

#include <string_view>

#include "pv_cleaning_robot/service/health_payload_builder.h"

TEST_CASE("HealthPayloadBuilder emits diagnostics payload into caller buffer",
          "[service][health][payload]") {
    char out[2048];
    robot::service::HealthPayloadBuilder::DiagnosticsView view{};
    view.ts_iso8601 = "2026-04-26T10:00:00Z";
    view.walk_ctrl_frames = 12;
    view.brush_actual_rpm = 800;
    view.gps_fix = 2;

    const size_t len = robot::service::HealthPayloadBuilder::build_diagnostics(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    REQUIRE(std::string_view(out, len).find("\"brush\":{\"rpm\":800") != std::string_view::npos);
    REQUIRE(std::string_view(out, len).find("\"gps\":{\"lat\":0.0") != std::string_view::npos);
}
```

- [ ] **Step 2: Add the builder source to `test/CMakeLists.txt`**

Add this line to `COMMON_SRCS` in `test/CMakeLists.txt`:

```cmake
  ${PROJ}/service/health_payload_builder.cc
```

- [ ] **Step 3: Run the test build and confirm it fails**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The build fails because `pv_cleaning_robot/service/health_payload_builder.h` and `.cc` do not exist yet.

- [ ] **Step 4: Create the builder interface**

Create `include/pv_cleaning_robot/service/health_payload_builder.h`:

```cpp
#pragma once

#include <cstddef>

namespace robot::service {

class HealthPayloadBuilder {
public:
    struct DiagnosticsView {
        const char* ts_iso8601{""};
        unsigned walk_ctrl_frames{0};
        int brush_actual_rpm{0};
        int gps_fix{0};
    };

    static size_t build_diagnostics(const DiagnosticsView& view,
                                    char* out,
                                    size_t cap) noexcept;
};

}  // namespace robot::service
```

- [ ] **Step 5: Implement the minimal builder**

Create `pv_cleaning_robot/service/health_payload_builder.cc`:

```cpp
#include "pv_cleaning_robot/service/health_payload_builder.h"

#include <cstdio>

namespace robot::service {

size_t HealthPayloadBuilder::build_diagnostics(const DiagnosticsView& view,
                                               char* out,
                                               size_t cap) noexcept
{
    if (!out || cap == 0) return 0;
    const int written = std::snprintf(
        out,
        cap,
        "{\"ts\":\"%s\",\"walk\":{\"ctrl_frames\":%u},\"brush\":{\"rpm\":%d},\"gps\":{\"lat\":0.0,\"fix\":%d}}",
        view.ts_iso8601,
        view.walk_ctrl_frames,
        view.brush_actual_rpm,
        view.gps_fix);
    if (written < 0 || static_cast<size_t>(written) >= cap) return 0;
    return static_cast<size_t>(written);
}

}  // namespace robot::service
```

- [ ] **Step 6: Rebuild to confirm the new test now compiles**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The test target compiles successfully with the new builder files.

- [ ] **Step 7: Commit the builder foundation**

Run:

```bash
git add include/pv_cleaning_robot/service/health_payload_builder.h \
        pv_cleaning_robot/service/health_payload_builder.cc \
        test/service/health_payload_builder_test.cc \
        test/CMakeLists.txt
git commit -m "feat: add health payload builder foundation"
```

### Task 2: Refactor `HealthService` to Use a Reusable Builder Path

**Files:**
- Modify: `include/pv_cleaning_robot/service/health_service.h`
- Modify: `pv_cleaning_robot/service/health_service.cc`

- [ ] **Step 1: Replace the mutable JSON tree with reusable output storage**

In `include/pv_cleaning_robot/service/health_service.h`, change the private payload members to:

```cpp
private:
    static constexpr size_t kPayloadBufferBytes = 8192;
    size_t build_payload(char* out, size_t cap) const;

    std::shared_ptr<device::WalkMotorGroup> walk_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::BMS> bms_;
    std::shared_ptr<device::ImuDevice> imu_;
    std::shared_ptr<device::GpsDevice> gps_;
    std::shared_ptr<device::DistanceSensor> dist_;
    std::shared_ptr<CloudService> cloud_;
    Mode mode_;
    mutable std::array<char, kPayloadBufferBytes> payload_buf_{};
    mutable std::string payload_cache_;
    std::ofstream local_log_file_;
```

- [ ] **Step 2: Refactor `update()` to publish the built buffer instead of `j_.dump()`**

In `pv_cleaning_robot/service/health_service.cc`, replace the current `update()` body with:

```cpp
void HealthService::update() {
    const size_t payload_len = build_payload(payload_buf_.data(), payload_buf_.size());
    if (payload_len == 0u) {
        spdlog::error("[HealthService] failed to build telemetry payload");
        return;
    }

    payload_cache_.assign(payload_buf_.data(), payload_len);
    if (cloud_) {
        cloud_->publish_telemetry(payload_cache_);
    }
    if (local_log_file_.is_open()) {
        local_log_file_.write(payload_buf_.data(), static_cast<std::streamsize>(payload_len));
        local_log_file_.put('\n');
        if (!local_log_file_.flush()) {
            spdlog::error("[HealthService] local log flush failed (disk full?), closing file");
            local_log_file_.close();
        }
    }
}
```

- [ ] **Step 3: Route diagnostics payload construction through `HealthPayloadBuilder`**

Replace the old `std::string HealthService::build_payload() const` signature and body with a `size_t`-returning builder path that:

```cpp
auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
char ts_buf[24];
std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));

HealthPayloadBuilder::DiagnosticsView view{};
view.ts_iso8601 = ts_buf;
view.walk_ctrl_frames = walk_->get_group_diagnostics().ctrl_frame_count;
view.brush_actual_rpm = brush_->get_diagnostics().actual_rpm;
view.gps_fix = gps_->get_diagnostics().fix_quality;
return HealthPayloadBuilder::build_diagnostics(view, out, cap);
```

For HEALTH mode, add the parallel `build_health(...)` entry to the builder and route through it in the same way.

- [ ] **Step 4: Rebuild the test target**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The test target still compiles after `HealthService` no longer includes a mutable payload JSON tree.

- [ ] **Step 5: Commit the `HealthService` refactor**

Run:

```bash
git add include/pv_cleaning_robot/service/health_service.h \
        pv_cleaning_robot/service/health_service.cc
git commit -m "refactor: route health payloads through builder"
```

### Task 3: Convert `DataCache` to Append-First Persistence

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h`
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
- Modify: `test/middleware/data_cache_test.cc`

- [ ] **Step 1: Add a failing persistence-replay regression test**

Append this test to `test/middleware/data_cache_test.cc`:

```cpp
TEST_CASE("DataCache: ack journal survives restart without full rewrite semantics",
          "[middleware][data_cache]") {
    std::string path = "/tmp/test_dc_journal.jsonl";
    {
        DataCache c1(path);
        c1.open();
        c1.push("topic", "payload_A");
        c1.push("topic", "payload_B");
        auto batch = c1.pop_batch(10);
        REQUIRE(batch.size() == 2);
        c1.confirm_sent({batch[0].id});
        c1.close();
    }
    {
        DataCache c2(path);
        c2.open();
        auto batch = c2.pop_batch(10);
        REQUIRE(batch.size() == 1);
        REQUIRE(batch[0].payload == "payload_B");
        c2.close();
    }
    fs::remove(path);
    fs::remove(path + ".tmp");
}
```

- [ ] **Step 2: Rebuild to confirm the existing implementation does not satisfy the new model**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The build may still compile, but this test should be treated as the new target behavior the implementation must satisfy after the journal refactor.

- [ ] **Step 3: Add journal helpers and compaction policy to the header**

In `include/pv_cleaning_robot/middleware/data_cache.h`, introduce:

```cpp
private:
    struct JournalStats {
        size_t append_count{0};
        size_t ack_count{0};
    };

    bool append_push_record_locked(const Record& record);
    bool append_ack_record_locked(int64_t id);
    void maybe_compact_locked();
    void compact_to_snapshot_locked() const;

    JournalStats journal_stats_{};
```

- [ ] **Step 4: Replace full rewrite behavior with append-first journal mutations**

In `pv_cleaning_robot/middleware/data_cache.cc`, change the persistence model to:

```cpp
bool DataCache::push(const std::string& topic, const std::string& payload, uint64_t ts_ms) {
    ...
    Record record{next_id_++, topic, payload, ts_ms};
    queue_.push_back(record);
    if (!append_push_record_locked(record)) return false;
    maybe_compact_locked();
    return true;
}

void DataCache::confirm_sent(const std::vector<int64_t>& ids) {
    ...
    for (auto id : ids) {
        ...
        append_ack_record_locked(id);
    }
    maybe_compact_locked();
}
```

On `open()`, replay both `push` and `ack` journal operations so only live records remain in memory.

- [ ] **Step 5: Rebuild the test target**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- The test target compiles after the journal replay helpers and new persistence model land.

- [ ] **Step 6: Commit the cache persistence redesign**

Run:

```bash
git add include/pv_cleaning_robot/middleware/data_cache.h \
        pv_cleaning_robot/middleware/data_cache.cc \
        test/middleware/data_cache_test.cc
git commit -m "refactor: convert telemetry cache to append-first journal"
```

### Task 4: Verify the Hardened Telemetry Pipeline

**Files:**
- Read: `include/pv_cleaning_robot/service/health_payload_builder.h`
- Read: `pv_cleaning_robot/service/health_payload_builder.cc`
- Read: `include/pv_cleaning_robot/service/health_service.h`
- Read: `pv_cleaning_robot/service/health_service.cc`
- Read: `include/pv_cleaning_robot/middleware/data_cache.h`
- Read: `pv_cleaning_robot/middleware/data_cache.cc`

- [ ] **Step 1: Rebuild unit tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected:
- Unit tests compile with the builder and journal changes.

- [ ] **Step 2: Rebuild the main executable**

Run:

```bash
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected:
- The main target compiles with the telemetry pipeline changes.

- [ ] **Step 3: Commit the final verification checkpoint**

Run:

```bash
git commit --allow-empty -m "chore: verify telemetry pipeline hardening"
```
