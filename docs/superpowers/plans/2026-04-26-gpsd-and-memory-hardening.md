# GPSD And Memory Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move GPSD JSON parsing into the protocol layer and remove the main long-running heap-churn hotspots in GPSD ingest, health payload generation, and offline telemetry caching.

**Architecture:** Split GPSD support into a pure `protocol` parser plus a lean socket-owning device source. Replace `HealthService`'s mutable `nlohmann::json` tree with a fixed-buffer payload builder. Convert `DataCache` from full-file rewrite on every `push()` / `confirm_sent()` into an append-first journal with threshold-based compaction.

**Tech Stack:** C++17, Catch2, fixed-size buffers, `std::string_view`, existing `ThreadExecutor`, `CloudService`, `DataCache`, `GpsdGpsSource`

---

## File Structure

- Create: `include/pv_cleaning_robot/protocol/gpsd_json_parser.h`
  - Pure parser interface for one GPSD JSON line.
- Create: `pv_cleaning_robot/protocol/gpsd_json_parser.cc`
  - No-I/O parser implementation using `std::string_view` and fixed-scope helpers.
- Create: `test/protocol/gpsd_json_parser_test.cc`
  - Parser-focused tests for `TPV`, `SKY`, invalid JSON-like input, and timestamp parsing.
- Modify: `include/pv_cleaning_robot/device/gps_source.h`
  - Replace GPSD hot-path `std::string` interfaces/buffers with fixed-buffer members and `std::string_view` test hook.
- Modify: `pv_cleaning_robot/device/gpsd_gps_source.cc`
  - Keep TCP/session management only; route parsing through `protocol::GpsdJsonParser`.
- Modify: `test/device/gpsd_gps_source_test.cc`
  - Keep source tests focused on device behavior, not raw field decoding internals.
- Create: `include/pv_cleaning_robot/service/health_payload_builder.h`
  - Fixed-buffer JSON emitter for health/diagnostics payloads.
- Create: `pv_cleaning_robot/service/health_payload_builder.cc`
  - Minimal formatting helpers for telemetry JSON.
- Create: `test/service/health_payload_builder_test.cc`
  - Builder coverage for HEALTH and DIAGNOSTICS payload shape.
- Modify: `include/pv_cleaning_robot/service/health_service.h`
  - Remove mutable `nlohmann::json` payload tree and expose fixed-buffer build helper.
- Modify: `pv_cleaning_robot/service/health_service.cc`
  - Use builder output buffer instead of `j_.dump()`.
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h`
  - Introduce append-journal and compaction policy state.
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
  - Implement append-only `push()` / `confirm_sent()` plus startup replay and bounded compaction.
- Modify: `test/middleware/data_cache_test.cc`
  - Add replay/ack/compaction regression tests.
- Modify: `test/CMakeLists.txt`
  - Register the new protocol and service tests.

## Task 1: Split GPSD JSON Parsing Into `protocol` And Remove GPSD Hot-Path String Churn

**Files:**
- Create: `include/pv_cleaning_robot/protocol/gpsd_json_parser.h`
- Create: `pv_cleaning_robot/protocol/gpsd_json_parser.cc`
- Create: `test/protocol/gpsd_json_parser_test.cc`
- Modify: `include/pv_cleaning_robot/device/gps_source.h`
- Modify: `pv_cleaning_robot/device/gpsd_gps_source.cc`
- Modify: `test/device/gpsd_gps_source_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing protocol parser test**

```cpp
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/protocol/gpsd_json_parser.h"

TEST_CASE("GpsdJsonParser parses TPV without device-layer socket state", "[protocol][gpsd]") {
    robot::protocol::GpsData data{};
    robot::protocol::GpsdJsonParseResult result{};

    const bool ok = robot::protocol::GpsdJsonParser::parse_line(
        R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":3,"time":"2026-04-25T12:34:56.789Z"})",
        data,
        result);

    REQUIRE(ok);
    REQUIRE(result.message_class == robot::protocol::GpsdMessageClass::TPV);
    REQUIRE(result.updated_fix);
    REQUIRE(data.valid);
    REQUIRE(data.fix_quality == 2);
    REQUIRE(data.utc_timestamp_ms == 1777120496789ULL);
}
```

- [ ] **Step 2: Run the test target to verify it fails**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile failure for missing `gpsd_json_parser.h` / `GpsdJsonParser`.

- [ ] **Step 3: Add the parser interface in the protocol layer**

```cpp
#pragma once

#include "pv_cleaning_robot/protocol/nmea_parser.h"

#include <cstdint>
#include <string_view>

namespace robot::protocol {

enum class GpsdMessageClass : uint8_t {
    NONE = 0,
    TPV,
    SKY,
    VERSION,
    WATCH,
    ERROR_MESSAGE,
    UNKNOWN,
};

struct GpsdJsonParseResult {
    GpsdMessageClass message_class{GpsdMessageClass::NONE};
    bool updated_fix{false};
    bool updated_satellites{false};
    bool parse_error{false};
    bool time_parse_error{false};
};

class GpsdJsonParser {
public:
    static bool parse_line(std::string_view line,
                           GpsData& data,
                           GpsdJsonParseResult& result) noexcept;
};

}  // namespace robot::protocol
```

- [ ] **Step 4: Implement a string-view parser instead of `nlohmann::json::parse()`**

```cpp
namespace robot::protocol {

namespace {

bool find_number(std::string_view json, std::string_view key, double& out) noexcept;
bool find_int(std::string_view json, std::string_view key, int& out) noexcept;
bool find_bool(std::string_view json, std::string_view key, bool& out) noexcept;
bool find_string(std::string_view json, std::string_view key, std::string_view& out) noexcept;
bool parse_tpv(std::string_view json, GpsData& data, GpsdJsonParseResult& result) noexcept;
bool parse_sky(std::string_view json, GpsData& data, GpsdJsonParseResult& result) noexcept;

}  // namespace

bool GpsdJsonParser::parse_line(std::string_view line,
                                GpsData& data,
                                GpsdJsonParseResult& result) noexcept {
    result = {};

    std::string_view cls;
    if (!find_string(line, "\"class\"", cls)) {
        result.parse_error = true;
        return false;
    }
    if (cls == "TPV") {
        result.message_class = GpsdMessageClass::TPV;
        return parse_tpv(line, data, result);
    }
    if (cls == "SKY") {
        result.message_class = GpsdMessageClass::SKY;
        return parse_sky(line, data, result);
    }
    if (cls == "VERSION") {
        result.message_class = GpsdMessageClass::VERSION;
        return true;
    }
    if (cls == "WATCH") {
        result.message_class = GpsdMessageClass::WATCH;
        return true;
    }
    if (cls == "ERROR") {
        result.message_class = GpsdMessageClass::ERROR_MESSAGE;
        result.parse_error = true;
        return false;
    }
    result.message_class = GpsdMessageClass::UNKNOWN;
    result.parse_error = true;
    return false;
}

}  // namespace robot::protocol
```

- [ ] **Step 5: Refactor `GpsdGpsSource` to use fixed receive buffers and the protocol parser**

```cpp
class GpsdGpsSource final : public IGpsSource {
public:
    void ingest_json_line_for_test(std::string_view line);

private:
    static constexpr size_t kRxBufCap = 2048;
    void handle_json_line(std::string_view line);

    std::array<char, kRxBufCap> rx_buf_{};
    size_t rx_len_{0};
};
```

```cpp
void GpsdGpsSource::read_loop()
{
    char buf[512];
    while (running_.load()) {
        const int n = ::recv(sock_fd_, buf, sizeof(buf), 0);
        if (n <= 0) { /* reconnect branch unchanged */ }

        for (int i = 0; i < n; ++i) {
            const char c = buf[i];
            if (c == '\n') {
                std::string_view line(rx_buf_.data(), rx_len_);
                if (!line.empty() && line.back() == '\r') {
                    line.remove_suffix(1);
                }
                if (!line.empty()) {
                    if (on_message_) on_message_();
                    handle_json_line(line);
                }
                rx_len_ = 0;
                continue;
            }
            if (rx_len_ + 1 < rx_buf_.size()) {
                rx_buf_[rx_len_++] = c;
            } else {
                rx_len_ = 0;
                if (on_parse_error_) on_parse_error_();
            }
        }
    }
}

void GpsdGpsSource::handle_json_line(std::string_view line)
{
    protocol::GpsdJsonParseResult result{};
    if (!protocol::GpsdJsonParser::parse_line(line, data_, result)) {
        if (on_parse_error_) on_parse_error_();
        return;
    }
    if ((result.message_class == protocol::GpsdMessageClass::TPV ||
         result.message_class == protocol::GpsdMessageClass::SKY) &&
        on_data_) {
        on_data_(data_);
    }
    if (result.time_parse_error && on_parse_error_) {
        on_parse_error_();
    }
}
```

- [ ] **Step 6: Build and verify the protocol/device tests compile**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build succeeds and `gpsd_json_parser_test.cc` is compiled into `unit_tests`.

- [ ] **Step 7: Commit the GPSD protocol split**

```bash
git add \
  include/pv_cleaning_robot/protocol/gpsd_json_parser.h \
  pv_cleaning_robot/protocol/gpsd_json_parser.cc \
  include/pv_cleaning_robot/device/gps_source.h \
  pv_cleaning_robot/device/gpsd_gps_source.cc \
  test/protocol/gpsd_json_parser_test.cc \
  test/device/gpsd_gps_source_test.cc \
  test/CMakeLists.txt
git commit -m "refactor: move gpsd parsing into protocol layer"
```

## Task 2: Replace `HealthService` JSON-Tree Payload Building With A Fixed-Buffer Builder

**Files:**
- Create: `include/pv_cleaning_robot/service/health_payload_builder.h`
- Create: `pv_cleaning_robot/service/health_payload_builder.cc`
- Create: `test/service/health_payload_builder_test.cc`
- Modify: `include/pv_cleaning_robot/service/health_service.h`
- Modify: `pv_cleaning_robot/service/health_service.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing fixed-buffer payload builder test**

```cpp
#include <catch2/catch.hpp>

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

- [ ] **Step 2: Run the build to verify it fails before the builder exists**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: compile failure for missing `health_payload_builder.h`.

- [ ] **Step 3: Add a fixed-buffer payload builder interface**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace robot::service {

class HealthPayloadBuilder {
public:
    struct DiagnosticsView {
        std::string_view ts_iso8601;
        float walk_avg_rpm{0.0f};
        float walk_avg_torque_a{0.0f};
        uint32_t walk_ctrl_frames{0};
        int brush_actual_rpm{0};
        int brush_target_rpm{0};
        float brush_current_a{0.0f};
        float brush_voltage_v{0.0f};
        float brush_temp_c{0.0f};
        double gps_lat{0.0};
        double gps_lon{0.0};
        int gps_fix{0};
    };

    struct HealthView {
        std::string_view ts_iso8601;
        float walk_avg_rpm{0.0f};
        bool brush_running{false};
        bool brush_fault{false};
        float battery_soc{0.0f};
        double gps_lat{0.0};
        double gps_lon{0.0};
        bool gps_valid{false};
    };

    static size_t build_diagnostics(const DiagnosticsView& view, char* out, size_t cap) noexcept;
    static size_t build_health(const HealthView& view, char* out, size_t cap) noexcept;
};

}  // namespace robot::service
```

- [ ] **Step 4: Implement `snprintf`-based JSON emission**

```cpp
size_t HealthPayloadBuilder::build_health(const HealthView& view,
                                          char* out,
                                          size_t cap) noexcept {
    const int n = std::snprintf(
        out,
        cap,
        "{\"ts\":\"%.*s\",\"walk\":{\"rpm\":%.2f},\"brush\":{\"running\":%s,\"fault\":%s},"
        "\"battery\":{\"soc\":%.2f},\"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"valid\":%s}}",
        static_cast<int>(view.ts_iso8601.size()), view.ts_iso8601.data(),
        view.walk_avg_rpm,
        view.brush_running ? "true" : "false",
        view.brush_fault ? "true" : "false",
        view.battery_soc,
        view.gps_lat,
        view.gps_lon,
        view.gps_valid ? "true" : "false");
    if (n < 0 || static_cast<size_t>(n) >= cap) {
        return 0;
    }
    return static_cast<size_t>(n);
}
```

- [ ] **Step 5: Refactor `HealthService` to build into a stack/member buffer instead of `j_.dump()`**

```cpp
class HealthService : public middleware::IRunnable {
private:
    static constexpr size_t kPayloadCap = 4096;
    size_t build_payload(char* out, size_t cap) const;
    std::ofstream local_log_file_;
};
```

```cpp
void HealthService::update() {
    char payload[kPayloadCap];
    const size_t len = build_payload(payload, sizeof(payload));
    if (len == 0) {
        spdlog::warn("[HealthService] payload truncated");
        return;
    }

    const std::string payload_str(payload, len);
    if (cloud_) {
        cloud_->publish_telemetry(payload_str);
    }
    if (local_log_file_.is_open()) {
        local_log_file_.write(payload, static_cast<std::streamsize>(len));
        local_log_file_.put('\n');
        local_log_file_.flush();
    }
}
```

- [ ] **Step 6: Build and verify the new service test compiles**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build succeeds and includes `health_payload_builder_test.cc`.

- [ ] **Step 7: Commit the health payload hardening**

```bash
git add \
  include/pv_cleaning_robot/service/health_payload_builder.h \
  pv_cleaning_robot/service/health_payload_builder.cc \
  include/pv_cleaning_robot/service/health_service.h \
  pv_cleaning_robot/service/health_service.cc \
  test/service/health_payload_builder_test.cc \
  test/CMakeLists.txt
git commit -m "refactor: build health payloads without json tree churn"
```

## Task 3: Convert `DataCache` To Append-First Journal With Bounded Compaction

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h`
- Modify: `pv_cleaning_robot/middleware/data_cache.cc`
- Modify: `test/middleware/data_cache_test.cc`

- [ ] **Step 1: Write the failing journal replay test**

```cpp
TEST_CASE("DataCache replays push and ack journal entries", "[middleware][data_cache]") {
    const std::string path = "/tmp/test_dc_journal.jsonl";
    {
        std::ofstream out(path, std::ios::trunc);
        out << "{\"op\":\"push\",\"id\":1,\"topic\":\"t\",\"payload\":\"a\",\"ts_ms\":1}\n";
        out << "{\"op\":\"push\",\"id\":2,\"topic\":\"t\",\"payload\":\"b\",\"ts_ms\":2}\n";
        out << "{\"op\":\"ack\",\"id\":1}\n";
    }

    robot::middleware::DataCache cache(path, 10);
    REQUIRE(cache.open());

    auto batch = cache.pop_batch(10);
    REQUIRE(batch.size() == 1);
    REQUIRE(batch[0].id == 2);
    REQUIRE(batch[0].payload == "b");
}
```

- [ ] **Step 2: Build to verify the current implementation does not support journal replay**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: test compiles but fails at runtime once the suite is run later, because `open()` currently expects snapshot-only records. The important precondition is that the test is present before implementation.

- [ ] **Step 3: Extend the cache state to track journal growth and compaction**

```cpp
class DataCache {
private:
    static constexpr size_t kCompactThresholdRows = 256;

    bool append_push_record_locked(const Record& rec);
    bool append_ack_record_locked(int64_t id);
    bool compact_locked();

    size_t journal_rows_{0};
    size_t ack_rows_{0};
};
```

- [ ] **Step 4: Change `push()` and `confirm_sent()` to append journal entries instead of full-file rewrite**

```cpp
bool DataCache::push(const std::string& topic, const std::string& payload, uint64_t ts_ms) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (queue_.size() >= max_rows_) {
        const int64_t dropped_id = queue_.front().id;
        queue_.pop_front();
        append_ack_record_locked(dropped_id);
    }
    Record rec{next_id_++, topic, payload, ts_ms};
    queue_.push_back(rec);
    if (!append_push_record_locked(rec)) {
        return false;
    }
    if (journal_rows_ >= kCompactThresholdRows || ack_rows_ * 2 >= journal_rows_) {
        return compact_locked();
    }
    return true;
}

void DataCache::confirm_sent(const std::vector<int64_t>& ids) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int64_t id : ids) {
        auto it = std::find_if(queue_.begin(), queue_.end(),
                               [id](const Record& r) { return r.id == id; });
        if (it != queue_.end()) {
            queue_.erase(it);
            append_ack_record_locked(id);
        }
    }
    if (journal_rows_ >= kCompactThresholdRows || ack_rows_ * 2 >= journal_rows_) {
        compact_locked();
    }
}
```

- [ ] **Step 5: Update `open()` to replay `push` / `ack` operations**

```cpp
while (std::getline(in, line)) {
    auto j = nlohmann::json::parse(line);
    const std::string op = j.value("op", "push");
    if (op == "push") {
        Record r;
        r.id = j.at("id").get<int64_t>();
        r.topic = j.at("topic").get<std::string>();
        r.payload = j.at("payload").get<std::string>();
        r.ts_ms = j.at("ts_ms").get<uint64_t>();
        queue_.push_back(std::move(r));
        ++journal_rows_;
    } else if (op == "ack") {
        const int64_t id = j.at("id").get<int64_t>();
        auto it = std::find_if(queue_.begin(), queue_.end(),
                               [id](const Record& r) { return r.id == id; });
        if (it != queue_.end()) {
            queue_.erase(it);
        }
        ++journal_rows_;
        ++ack_rows_;
    }
}
```

- [ ] **Step 6: Add a compaction regression test**

```cpp
TEST_CASE("DataCache compacts journal after many acknowledgements", "[middleware][data_cache]") {
    const std::string path = "/tmp/test_dc_compact.jsonl";
    robot::middleware::DataCache cache(path, 32);
    REQUIRE(cache.open());

    for (int i = 0; i < 20; ++i) {
        REQUIRE(cache.push("t", "payload-" + std::to_string(i), 100 + i));
    }
    std::vector<int64_t> ids;
    for (int i = 1; i <= 18; ++i) {
        ids.push_back(i);
    }
    cache.confirm_sent(ids);

    std::ifstream in(path);
    std::string line;
    size_t line_count = 0;
    while (std::getline(in, line)) {
        if (!line.empty()) {
            ++line_count;
        }
    }
    REQUIRE(line_count < 20);
}
```

- [ ] **Step 7: Build and verify the cache tests compile**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build succeeds with updated `data_cache_test.cc`.

- [ ] **Step 8: Commit the cache journaling change**

```bash
git add \
  include/pv_cleaning_robot/middleware/data_cache.h \
  pv_cleaning_robot/middleware/data_cache.cc \
  test/middleware/data_cache_test.cc
git commit -m "refactor: journal telemetry cache instead of full rewrites"
```

## Task 4: Final Cross-Build Verification And Documentation Notes

**Files:**
- Modify: `docs/superpowers/plans/2026-04-26-gpsd-and-memory-hardening.md`

- [ ] **Step 1: Run the full cross-build verification set**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
cmake --build --preset rk3576-build --target pv_cleaning_robot
cmake --build --preset rk3576-build --target hw_tests
```

Expected: all three targets build successfully. No test binaries need to be executed in the cross environment.

- [ ] **Step 2: Capture residual risk notes directly in the plan before closing**

```md
Residual risks after implementation:
- `ConfigService`, `CloudService` RPC parsing, and startup-time JSON file replay still use dynamic JSON parsing by design; they are not part of this hardening pass.
- `SerialGpsSource` still uses a bounded `std::string line_buf_`; acceptable for now because capacity stabilizes and the path is much lighter than GPSD JSON.
- If telemetry payload size grows past the fixed health buffer cap, builder tests must be updated before adding new fields.
```

- [ ] **Step 3: Commit any follow-up note edits if the plan was updated during execution**

```bash
git add docs/superpowers/plans/2026-04-26-gpsd-and-memory-hardening.md
git commit -m "docs: note residual risks for gpsd and memory hardening"
```
