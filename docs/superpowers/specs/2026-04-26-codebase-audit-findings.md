# Codebase Audit Findings

Date: 2026-04-26

## Scope

Included directories:

- `include/`
- `pv_cleaning_robot/`
- `test/`
- `config/`
- `docs/`
- `doc/`
- `tools/`
- `scripts/`
- `sample/`

Excluded from source-level audit conclusions:

- `3rdparty/`
- `build/`

These exclusions may still appear in repository hygiene observations if they create clutter, but they are not treated as first-party architecture or implementation findings.

## Evidence Levels

- `confirmed`: the issue is directly demonstrated by code structure, references, or checked-in files.
- `high-risk`: the issue is strongly suggested by static evidence, but final impact depends on runtime behavior or off-repo usage.

## Summary

The current repository state shows three kinds of issues clearly.

First, there are confirmed stale artifacts: an unused pre-ODrive brush mock, an unintegrated OTA module that still ships in the build, and raw experiment log files in `doc/`.

Second, the most urgent live-code correctness problem is a thread-safety bug in `CloudService`: the shared-attribute callback is written under a mutex but read from the network callback path without synchronization.

Third, the main long-lived heap churn is concentrated in the telemetry and offline-cache path. `HealthService` rebuilds JSON payloads every reporting cycle, and `DataCache` rewrites the entire JSONL queue on every `push()` and `confirm_sent()`. There is also a smaller but still plausible churn source in the serial NMEA parsing path.

## Findings by Dimension

### Redundant or Obsolete Code

#### AUDIT-001
- **Title:** `MockBrushMotor` is an unreferenced compatibility leftover from the pre-ODrive brush interface
- **Dimension:** Redundant or Obsolete Code
- **Evidence Level:** confirmed
- **Impact:** The repository carries a dead test double that still models `start()` and `start_count`, which no longer exist on `device::BrushMotor`. It increases maintenance cost and risks misleading later tests or reviewers about the current brush control contract.
- **Files:** `test/mock/mock_brush_motor.h`, `include/pv_cleaning_robot/device/brush_motor.h`
- **Summary:** `test/mock/mock_brush_motor.h` still exposes `start_result`, `start_count`, and `start()` even though the live `BrushMotor` API has moved to ODrive-native `set_mode_*`, `set_rpm`, `set_torque`, `stop`, and `enter_idle`. A repository-wide search shows no references to `MockBrushMotor` outside its own header.
- **Recommended Action:** Delete the mock if no external consumer depends on it. If a brush test double is still needed, replace it with one that matches the current ODrive brush interface.
- **Needs Remediation Spec:** no
- **Priority:** P2

#### AUDIT-002
- **Title:** `OtaManager` is compiled into the product but has no first-party integration path
- **Dimension:** Redundant or Obsolete Code
- **Evidence Level:** confirmed
- **Impact:** The OTA implementation adds maintenance surface, binary size, OpenSSL-linked behavior, and API documentation weight without any active production call path in `main`, `service`, `app`, or tests.
- **Files:** `include/pv_cleaning_robot/middleware/ota_manager.h`, `pv_cleaning_robot/middleware/ota_manager.cc`, `pv_cleaning_robot/CMakeLists.txt`, `doc/API_REFERENCE.md`
- **Summary:** `OtaManager` is documented and compiled through `middleware/*.cc` globbing, but repository-wide search finds no usage from `main.cc`, no wiring through `CloudService`, and no tests or fixtures exercising it. The only references outside the class itself are documentation and historical specs/plans.
- **Recommended Action:** Decide explicitly whether OTA is in scope for this product. If not, remove the module from the build and documentation. If yes, integrate it through a real control path and test surface.
- **Needs Remediation Spec:** yes
- **Priority:** P1

### Thread Safety

#### AUDIT-005
- **Title:** `CloudService::attr_cb_` has a real cross-thread data race between registration and network delivery
- **Dimension:** Thread Safety
- **Evidence Level:** confirmed
- **Impact:** This is undefined behavior on a live asynchronous path. A shared-attribute message can race with callback registration and observe or invoke a partially written `std::function`.
- **Files:** `include/pv_cleaning_robot/service/cloud_service.h`, `pv_cleaning_robot/service/cloud_service.cc`, `pv_cleaning_robot/main.cc`
- **Summary:** `CloudService::subscribe_shared_attributes()` stores `attr_cb_` while holding `rpc_mtx_`, but the attribute subscription lambda reads `attr_cb_` and invokes it with no lock at all. `main.cc` calls `net_mgr->connect()` before `cloud->subscribe_shared_attributes(...)`, which creates a real window where network delivery and callback installation can overlap.
- **Recommended Action:** Protect both read and write access to `attr_cb_` with the same synchronization strategy, or swap to an immutable callback handle with atomic publication semantics.
- **Needs Remediation Spec:** yes
- **Priority:** P0

### Memory Fragmentation Risk

#### AUDIT-006
- **Title:** `HealthService` rebuilds telemetry JSON on every reporting cycle through mutable `nlohmann::json` state and `dump()`
- **Dimension:** Memory Fragmentation Risk
- **Evidence Level:** confirmed
- **Impact:** The telemetry reporting path runs continuously for the lifetime of the process. Repeated `nlohmann::json` mutation plus `dump()` string creation adds sustained heap churn in a long-running embedded service.
- **Files:** `include/pv_cleaning_robot/service/health_service.h`, `pv_cleaning_robot/service/health_service.cc`
- **Summary:** `HealthService::update()` calls `build_payload()` each cycle. `build_payload()` mutates a large `mutable nlohmann::json j_` tree and returns `j_.dump()` as a fresh `std::string`, which is then sent to `CloudService` and written again to the local JSONL file. This is a direct repeated-allocation path, not just a one-time setup cost.
- **Recommended Action:** Move payload formatting out of `HealthService` and replace the hot path with a fixed-buffer or reuse-oriented encoder strategy.
- **Needs Remediation Spec:** yes
- **Priority:** P1

#### AUDIT-007
- **Title:** `DataCache` rewrites the entire persisted queue on every enqueue and acknowledgement
- **Dimension:** Memory Fragmentation Risk
- **Evidence Level:** confirmed
- **Impact:** In offline or unstable-network conditions, each telemetry enqueue and each confirmation causes full queue traversal, repeated JSON object construction, and full-file rewrite churn. This compounds heap churn with unnecessary I/O.
- **Files:** `include/pv_cleaning_robot/middleware/data_cache.h`, `pv_cleaning_robot/middleware/data_cache.cc`, `pv_cleaning_robot/service/cloud_service.cc`
- **Summary:** `DataCache::push()` appends the record to `queue_` and immediately calls `flush_to_file()`. `confirm_sent()` erases acknowledged records and calls the same full rewrite path. `flush_to_file()` constructs a new `nlohmann::json` object for every queued record and `dump()`s each one during every rewrite cycle.
- **Recommended Action:** Replace full-rewrite persistence with an append-first journal and explicit compaction policy, or another persistence strategy that avoids reserializing the full queue on every mutation.
- **Needs Remediation Spec:** yes
- **Priority:** P1

#### AUDIT-008
- **Title:** The serial NMEA parsing path still allocates per sentence in a long-lived read loop
- **Dimension:** Memory Fragmentation Risk
- **Evidence Level:** high-risk
- **Impact:** This path is lighter than the telemetry/cache hotspots, but it still performs repeated string growth, split-vector construction, and substring extraction on a device reader loop that may run indefinitely.
- **Files:** `include/pv_cleaning_robot/device/gps_source.h`, `pv_cleaning_robot/device/serial_gps_source.cc`, `include/pv_cleaning_robot/protocol/nmea_parser.h`, `pv_cleaning_robot/protocol/nmea_parser.cc`
- **Summary:** `SerialGpsSource::read_loop()` accumulates bytes into `line_buf_`, and `NmeaParser` then uses `std::string`, `std::vector<std::string>`, `substr`, and `stod`/`stoi` style parsing helpers per sentence. Capacity reuse may reduce the effect, but the code shape still suggests repeated heap activity in a background loop.
- **Recommended Action:** Treat this as a secondary optimization target after the heavier telemetry/cache churn. If memory pressure remains a concern, convert NMEA parsing toward fixed-buffer tokenization.
- **Needs Remediation Spec:** no
- **Priority:** observe

### Layering Discipline

#### AUDIT-003
- **Title:** `HealthService` mixes business telemetry selection, JSON serialization, and local file persistence in one service class
- **Dimension:** Layering Discipline
- **Evidence Level:** confirmed
- **Impact:** The service layer is carrying both orchestration concerns and serialization/infrastructure concerns, which makes the telemetry path harder to test, harder to optimize, and harder to reuse independently of MQTT/JSONL behavior.
- **Files:** `include/pv_cleaning_robot/service/health_service.h`, `pv_cleaning_robot/service/health_service.cc`
- **Summary:** `HealthService` not only decides which device fields belong in HEALTH versus DIAGNOSTICS mode, it also owns a mutable `nlohmann::json` tree, performs concrete JSON serialization in `build_payload()`, and writes JSONL records directly through `std::ofstream`. That combines service policy, payload formatting, and local persistence in one layer.
- **Recommended Action:** Split telemetry field selection from payload encoding and from optional local log sinking. A service-level component should choose the data; formatting and persistence should live behind narrower helpers.
- **Needs Remediation Spec:** yes
- **Priority:** P1

### Garbage, Stale, or Old Files

#### AUDIT-004
- **Title:** Raw hardware run logs are checked into `doc/` as ad-hoc text artifacts
- **Dimension:** Garbage, Stale, or Old Files
- **Evidence Level:** confirmed
- **Impact:** These files add noise to the repository, blur the line between maintained documentation and disposable experiment output, and make `doc/` less trustworthy as a curated reference set.
- **Files:** `doc/no_pid.txt`, `doc/pid.txt`, `doc/pid_afterfix.txt`
- **Summary:** These three files contain timestamped runtime log output from hardware experiments rather than maintained documentation. A repository-wide search finds no references to them from README, API docs, tests, or active specs.
- **Recommended Action:** Remove them from the repository or relocate them into a dedicated archived-results area with an index explaining why they are retained.
- **Needs Remediation Spec:** no
- **Priority:** P2

## Priority Rollup

- `P0`
  - `AUDIT-005` `CloudService::attr_cb_` data race on a live asynchronous path
- `P1`
  - `AUDIT-002` `OtaManager` compiled and documented without first-party integration
  - `AUDIT-003` `HealthService` layer-boundary violation between service policy, formatting, and persistence
  - `AUDIT-006` `HealthService` telemetry JSON hot-path heap churn
  - `AUDIT-007` `DataCache` full-rewrite persistence churn
- `P2`
  - `AUDIT-001` stale `MockBrushMotor`
  - `AUDIT-004` raw hardware log artifacts under `doc/`
- `observe`
  - `AUDIT-008` serial NMEA parse churn is plausible but secondary compared with the main telemetry/cache path

## Recommended Remediation Spec Candidates

- `docs/superpowers/specs/2026-04-26-telemetry-pipeline-hardening-design.md`
  - Covers `AUDIT-003`, `AUDIT-006`, and `AUDIT-007`
- `docs/superpowers/specs/2026-04-26-cloud-service-thread-safety-design.md`
  - Covers `AUDIT-005`
- `docs/superpowers/specs/2026-04-26-ota-module-scope-cleanup-design.md`
  - Covers `AUDIT-002`

## Observations That Should Not Trigger Code Changes

- The `gpsd` receive path is no longer the primary memory-fragmentation concern. It already uses a fixed receive buffer and a `std::string_view`-based protocol parser instead of the earlier `nlohmann::json` hot path.
- `doc/API_REFERENCE.md` and `doc/dev-guide/CONCURRENCY.md` are large but actively referenced from `README.md`, code comments, and prior engineering plans. They should not be treated as garbage merely because they are broad or historical.
- The checked-in `.vscode` files are repository-hygiene candidates rather than architecture findings. Whether they should stay depends on team convention, not on code correctness.
