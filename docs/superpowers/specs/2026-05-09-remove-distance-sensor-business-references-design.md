# Remove Distance Sensor Business References Design

Date: 2026-05-09

## Goal

Distance sensor is no longer part of product business behavior.

Keep distance-sensor code only in:
- protocol layer
- device layer
- hardware tests

Remove every other business/runtime reference so beginners can read the runtime chain without carrying an unused sensor through service, app, telemetry, and non-hardware tests.

## Approved Scope

Delete distance-sensor usage from:
- `HealthService`
- main runtime wiring
- non-hardware integration tests
- any business-facing payloads and comments that still describe `dist` as runtime truth

Keep distance-sensor code in:
- `include/pv_cleaning_robot/protocol/distance_sensor_protocol.h`
- `pv_cleaning_robot/protocol/distance_sensor_protocol.cc`
- `include/pv_cleaning_robot/device/distance_sensor.h`
- `pv_cleaning_robot/device/distance_sensor.cc`
- `test/protocol/distance_sensor_protocol_test.cc`
- `test/device/distance_sensor_device_test.cc`
- `test/integration/hardware/distance_sensor_hw_test.cc`
- hardware-only fixture support in `test/integration/hardware/hw_config.h`

## Design

### 1. Health payload no longer contains `dist`

`HealthService` is business/runtime output. Since distance sensor is not part of current business logic anymore, remove:
- `DistanceSensor` constructor dependency
- `dist_` member
- `dist` fields from health/diagnostics views
- `dist` JSON serialization from HEALTH and DIAGNOSTICS payloads

Result:
- runtime telemetry stops exposing unused distance data
- local health JSONL matches current product business model
- `HealthService` becomes smaller and easier to read

### 2. Non-hardware runtime tests stop wiring distance sensor

Any non-hardware test that currently validates business/runtime behavior must stop mentioning distance sensor. Tests should only assert currently supported business/runtime truth.

This includes:
- unit tests around health payload business output
- mock/system integration tests that currently inherit `dist` through `HealthService`

Hardware tests remain allowed to construct and validate `DistanceSensor`.

### 3. Hardware fixture support remains

`test/integration/hardware/hw_config.h` keeps:
- distance-sensor hardware parameters
- Modbus construction
- hardware-only polling thread support

Reason:
- user explicitly wants to preserve driver/device/hardware code
- hardware validation still needs a shared fixture

No runtime service or app layer should depend on it outside hardware tests.

## Files Expected To Change

Primary:
- `include/pv_cleaning_robot/service/health_service.h`
- `pv_cleaning_robot/service/health_service.cc`
- `test/service/health_payload_builder_test.cc`
- `test/integration/system_integration_test.cc`

Possible:
- `README.md` if runtime telemetry docs still mention distance sensor
- other non-hardware tests if they assert `dist`

No changes expected to:
- protocol/device distance-sensor code
- hardware distance-sensor tests

## Validation

Build:
- `cmake --build build --target unit_tests -j4`
- `cmake --build build --target hw_tests -j4`

Regression focus:
- health payload and system integration tests still compile
- hardware distance-sensor tests still compile
- no remaining `distance_sensor` references in service/app/main/non-hardware test runtime paths

## Non-Goals

- do not delete distance-sensor driver/device/protocol code
- do not redesign hardware fixtures beyond removing accidental business leakage
- do not add abstraction layers
