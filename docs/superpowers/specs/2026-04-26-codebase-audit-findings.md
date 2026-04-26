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

### Memory Fragmentation Risk

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

## Recommended Remediation Spec Candidates

## Observations That Should Not Trigger Code Changes
