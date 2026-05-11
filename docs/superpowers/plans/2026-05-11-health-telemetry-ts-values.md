# Health Telemetry Ts Values Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `HealthService` 的 `health` / `diagnostics` 遥测统一改成 ThingsBoard 推荐的 `{"ts":<ms>,"values":{...}}` 格式，改用扁平可读缩写键，并在完成后按典型业务流程估算月上行 payload 流量。

**Architecture:** 只修改 `HealthService` / `HealthPayloadBuilder` 这一条遥测链，不碰 `ThingsBoardControlPlane` 的 business telemetry / status event / command event。测试先锁定新 JSON 合同，再最小修改构造逻辑和本地 JSONL 断言，最后基于真实 payload 大小给出月流量估算。

**Tech Stack:** C++17、现有 `HealthService` / `CloudService`、Catch2 单元测试、硬件集成测试 JSONL 校验

---

## File Map

**Modify:**
- `include/pv_cleaning_robot/service/health_service.h`
- `pv_cleaning_robot/service/health_service.cc`
- `test/service/health_payload_builder_test.cc`
- `test/integration/hardware/system_hw_test.cc`

**Build verification only:**
- `cmake --build build --target unit_tests -j4`

## Task 1: 锁定新的遥测 JSON 合同

**Files:**
- Modify: `test/service/health_payload_builder_test.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`

- [ ] **Step 1: 先把 payload builder 单测改成新格式**

把 `test/service/health_payload_builder_test.cc` 从旧的顶层嵌套断言，改成 `ts + values` 和扁平键断言。

目标测试形状：

```cpp
TEST_CASE("HealthPayloadBuilder emits diagnostics payload into caller buffer",
          "[service][health][payload]") {
    char out[4096];
    robot::service::HealthPayloadBuilder::DiagnosticsView view{};
    view.ts_ms = 1714202400123ULL;
    view.walk.wheel[0].speed_rpm = 11.0f;
    view.walk.wheel[0].torque_a = 1.5f;
    view.walk.wheel[0].fault = robot::protocol::WalkMotorFault::OVER_CURRENT;
    view.brush.actual_rpm = 800;
    view.gps.fix_quality = 2;
    view.imu.pitch_deg = 1.2f;
    view.imu.roll_deg = 2.3f;
    view.imu.yaw_deg = 3.4f;

    const size_t len = robot::service::HealthPayloadBuilder::build_diagnostics(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc.HasMember("ts"));
    REQUIRE(doc["ts"].IsUint64());
    REQUIRE(doc["ts"].GetUint64() == 1714202400123ULL);
    REQUIRE(doc.HasMember("values"));
    REQUIRE(doc["values"].IsObject());
    REQUIRE(doc["values"].HasMember("lt_rpm"));
    REQUIRE(doc["values"].HasMember("lt_cur"));
    REQUIRE(doc["values"].HasMember("lt_err"));
    REQUIRE(doc["values"].HasMember("imu_p"));
    REQUIRE(doc["values"].HasMember("imu_r"));
    REQUIRE(doc["values"].HasMember("imu_y"));
    REQUIRE(doc["values"].HasMember("br_rpm"));
    REQUIRE(doc["values"].HasMember("gps_fix"));
}
```

- [ ] **Step 2: 为 health payload 增加字段合同测试**

在同一个测试文件追加一个 `build_health(...)` 用例，锁定：

```cpp
TEST_CASE("HealthPayloadBuilder emits health payload with per-wheel walk values",
          "[service][health][payload]") {
    char out[4096];
    robot::service::HealthPayloadBuilder::HealthView view{};
    view.ts_ms = 1714202400456ULL;
    view.walk.wheel[0].speed_rpm = 10.0f;
    view.walk.wheel[1].speed_rpm = 20.0f;
    view.walk.wheel[2].speed_rpm = 30.0f;
    view.walk.wheel[3].speed_rpm = 40.0f;
    view.walk.wheel[0].torque_a = 1.0f;
    view.walk.wheel[1].torque_a = 2.0f;
    view.walk.wheel[2].torque_a = 3.0f;
    view.walk.wheel[3].torque_a = 4.0f;
    view.imu.pitch_deg = 5.0f;
    view.imu.roll_deg = 6.0f;
    view.imu.yaw_deg = 7.0f;

    const size_t len = robot::service::HealthPayloadBuilder::build_health(
        view, out, sizeof(out));

    REQUIRE(len > 0);
    rapidjson::Document doc;
    doc.Parse(out, len);
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc["values"].HasMember("lt_rpm"));
    REQUIRE(doc["values"].HasMember("rt_rpm"));
    REQUIRE(doc["values"].HasMember("lb_rpm"));
    REQUIRE(doc["values"].HasMember("rb_rpm"));
    REQUIRE(doc["values"].HasMember("lt_cur"));
    REQUIRE(doc["values"].HasMember("rt_cur"));
    REQUIRE(doc["values"].HasMember("lb_cur"));
    REQUIRE(doc["values"].HasMember("rb_cur"));
    REQUIRE(doc["values"].HasMember("imu_p"));
    REQUIRE(doc["values"].HasMember("imu_r"));
    REQUIRE(doc["values"].HasMember("imu_y"));
}
```

- [ ] **Step 3: 改硬件系统测试里的本地 JSONL 断言**

把 `test/integration/hardware/system_hw_test.cc` 里对健康日志的断言从：

```cpp
REQUIRE(j.HasMember("walk"));
REQUIRE(j.HasMember("bms"));
REQUIRE(j.HasMember("imu"));
CHECK(!std::string(j["ts"].GetString()).empty());
```

改成：

```cpp
REQUIRE(j.HasMember("ts"));
REQUIRE(j["ts"].IsUint64());
REQUIRE(j.HasMember("values"));
REQUIRE(j["values"].IsObject());
REQUIRE(j["values"].HasMember("lt_rpm"));
REQUIRE(j["values"].HasMember("bat_soc"));
REQUIRE(j["values"].HasMember("imu_p"));
```

- [ ] **Step 4: 运行构建，确认测试先失败**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- 因 `ts_iso8601` / 旧 JSON 结构与新断言不匹配而失败

- [ ] **Step 5: Commit**

```bash
git add test/service/health_payload_builder_test.cc test/integration/hardware/system_hw_test.cc
git commit -m "test: define ts values health telemetry contract"
```

## Task 2: 把 HealthPayloadBuilder 改成 `ts + values` 扁平格式

**Files:**
- Modify: `include/pv_cleaning_robot/service/health_service.h`
- Modify: `pv_cleaning_robot/service/health_service.cc`

- [ ] **Step 1: 先把时间字段从字符串换成毫秒整数**

在 `include/pv_cleaning_robot/service/health_service.h` 里，把 `HealthView` / `DiagnosticsView` 的时间字段从字符串改成整数：

```cpp
uint64_t ts_ms{0};
```

删除旧的：

```cpp
const char* ts_iso8601{""};
```

- [ ] **Step 2: 在 `build_payload()` 里统一生成毫秒时间戳**

在 `pv_cleaning_robot/service/health_service.cc` 的 `HealthService::build_payload(...)` 中，把：

```cpp
auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
char ts_buf[24];
std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));
```

替换成：

```cpp
const auto now = std::chrono::system_clock::now();
const auto ts_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
```

并把 `view.ts_ms = ts_ms;` 传给 health / diagnostics 视图。

- [ ] **Step 3: 重写 `build_health(...)` 输出结构**

把 `build_health(...)` 改成：

```cpp
{"ts":1714202400456,"values":{...}}
```

`values` 中至少输出这些键：

```cpp
"lt_rpm", "rt_rpm", "lb_rpm", "rb_rpm",
"lt_cur", "rt_cur", "lb_cur", "rb_cur",
"lt_err", "rt_err", "lb_err", "rb_err",
"br_run", "br_err",
"bat_soc", "bat_vol", "bat_chg", "bat_alm",
"imu_p", "imu_r", "imu_y",
"gps_lat", "gps_lon", "gps_fix"
```

实现约束：

- 不再输出平均 `walk.rpm`
- 不再输出嵌套对象
- 错误字段使用布尔

- [ ] **Step 4: 重写 `build_diagnostics(...)` 输出结构**

也统一改成：

```cpp
{"ts":1714202400123,"values":{...}}
```

`values` 中保留详细缩写键，例如：

```cpp
"lt_rpm","rt_rpm","lb_rpm","rb_rpm",
"lt_tgt","rt_tgt","lb_tgt","rb_tgt",
"lt_cur","rt_cur","lb_cur","rb_cur",
"lt_err","rt_err","lb_err","rb_err",
"lt_ec","rt_ec","lb_ec","rb_ec",
"lt_on","rt_on","lb_on","rb_on",
"walk_cf","walk_ce",
"br_rpm","br_tgt","br_cur","br_vol","br_tmp","br_stl","br_ce",
"bat_soc","bat_vol","bat_cur","bat_tmp","bat_cmax","bat_cmin","bat_rah","bat_cyc","bat_alm",
"imu_ax","imu_ay","imu_az","imu_gx","imu_gy","imu_gz","imu_p","imu_r","imu_y","imu_fr","imu_pe",
"gps_lat","gps_lon","gps_alt","gps_spd","gps_sat","gps_hdp","gps_fix","gps_sent"
```

- [ ] **Step 5: 保持本地 JSONL 与上云 payload 完全一致**

不要额外单独格式化本地日志。继续沿用：

```cpp
payload_cache_.assign(payload_buf_.data(), payload_len);
cloud_->publish_telemetry(payload_cache_);
local_log_->log(... payload_cache_ ...)
```

这样网络和本地文件看到的是同一帧 JSON。

- [ ] **Step 6: 运行构建验证**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `unit_tests` 构建通过

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/health_service.h pv_cleaning_robot/service/health_service.cc
git commit -m "feat: switch health telemetry to ts values format"
```

## Task 3: 回归并统计典型月上行 payload 流量

**Files:**
- Verify only: `test/service/health_payload_builder_test.cc`
- Verify only: `test/integration/hardware/system_hw_test.cc`
- Optional scratch only in shell, no repo file required

- [ ] **Step 1: 重新构建单测目标**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `unit_tests` 目标构建成功

- [ ] **Step 2: 统计单帧 payload 大小**

用 `health_payload_builder_test` 中的代表性样本，记录：

- 一帧 `health` JSON 字节数
- 一帧 `diagnostics` JSON 字节数

至少给出这两个量：

```text
health_payload_bytes = <N>
diagnostics_payload_bytes = <M>
```

- [ ] **Step 3: 按业务口径估算月上行 payload 流量**

使用固定口径：

- 每天 1 次调度启动
- 每次任务 `passes = 1`
- 单次运行 `20` 分钟
- 运行态 `1s` 1 帧
- 空闲态 `5min` 1 帧

计算步骤：

```text
running_frames_per_day = 20 * 60 = 1200
idle_minutes_per_day = 24 * 60 - 20 = 1420
idle_frames_per_day = ceil(1420 / 5) = 284
total_frames_per_day = 1484
monthly_frames = 1484 * 30 = 44520
```

然后基于当前默认模式判断到底是按 `health` 还是 `diagnostics` 上报：

- `diagnostics.mode == development` 时用 diagnostics 单帧大小
- `diagnostics.mode == production` 时用 health 单帧大小

给出：

```text
daily_payload_bytes = frame_bytes * 1484
monthly_payload_bytes = daily_payload_bytes * 30
monthly_payload_mb = monthly_payload_bytes / 1024 / 1024
```

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-05-11-health-telemetry-ts-values-design.md docs/superpowers/plans/2026-05-11-health-telemetry-ts-values.md
git commit -m "docs: add health telemetry ts values plan"
```

## Self-Review

- Spec coverage:
  - `health` / `diagnostics` 都改成 `ts + values`：Task 1 / Task 2
  - `health.walk` 改成四轮 rpm / 电流 / 错误：Task 2
  - `health.imu` 增加 `pitch / roll / yaw`：Task 1 / Task 2
  - 键名扁平可读缩写：Task 2
  - 月上行 payload 估算：Task 3
- Placeholder scan:
  - 无 `TODO` / `TBD`
  - 每个任务都有明确文件、命令、目标
- Type consistency:
  - 时间字段统一命名为 `ts_ms`
  - JSON 包裹统一为 `{"ts":...,"values":{...}}`
