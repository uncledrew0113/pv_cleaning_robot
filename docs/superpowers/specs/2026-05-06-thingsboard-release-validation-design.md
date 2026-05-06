# ThingsBoard Release Validation Design

## Goal

Add a release-validation layer around the existing ThingsBoard implementation so the project can answer three practical questions before fast rollout:

1. Can a real board connect to the real ThingsBoard instance and exchange data correctly?
2. Does the current ThingsBoard feature set satisfy the minimum release scope already implemented in code?
3. Are the core ThingsBoard code paths documented clearly enough for field debugging and follow-on maintenance?

This design is intentionally limited to the currently implemented MQTT/ThingsBoard path. It explicitly excludes:

- OTA
- LoRaWAN
- `passes = 0.5`
- `parking_policy = both`
- any new runtime feature that is not already implemented

## Current Supported Scope

The existing code already supports the following ThingsBoard-facing functions:

### 1. MQTT/TLS connectivity

- MQTT 3.1.1 transport via `MqttTransport`
- TLS with:
  - `ca_cert_path`
  - `client_cert_path`
  - `client_key_path`
  - optional `username/password`
- temporary debug-only switch:
  - `network.mqtt.insecure_skip_server_name_check`

### 2. Shared-attribute configuration

Supported shared attributes:

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy`
- `charging_side`
- `schedules`

Current release restrictions:

- `passes` must be a positive integer
- `passes = 0.5` is rejected
- `parking_policy = both` is rejected
- schedules are daily time points only, not weekday rules

Configuration semantics:

- `schedules` apply immediately to `active`
- task parameters are written to `pending`
- `pending` is promoted to `active` before the next task

### 3. RPC control

Supported RPC methods:

- `start`
- `stop`
- `return`
- `terminate`
- `reset`

### 4. Telemetry and event reporting

Supported uplink payload classes:

- startup attributes
- shared-attribute result event
- command accepted/completed/rejected events
- periodic business telemetry

Current business telemetry includes at least:

- `device_state`
- `task_state`
- `target_half_passes`
- `completed_half_passes`
- `active_config`
- `pending_config`
- `active_config_version`
- `last_command`

## Release Readiness Assessment

## Conclusion

The current implementation is close to minimum fast-release readiness, but not yet complete as an acceptance package.

### What is already strong enough

- Real TLS connection has been proven on target hardware
- Startup attributes have real integration coverage
- One-shot business telemetry publish has real integration coverage
- Shared-attribute rules and RPC semantics are covered by local tests
- The ThingsBoard code path has already been structurally simplified compared to the earlier version

### What is still missing before claiming rollout readiness

The missing piece is not a large new feature. It is the lack of a full real-board validation loop covering all four release-critical behaviors together:

- telemetry
- RPC
- cloud configuration
- state transitions

Today, real integration coverage proves connectivity and basic publish paths. It does not yet prove that:

- a cloud-side shared attribute update changes local runtime/config files and is then reflected back in telemetry
- a cloud-side RPC causes real runtime state transitions on the board and produces the expected telemetry/event sequence

Therefore the next work should focus on validation completeness, not feature expansion.

## Design

## 1. Test Strategy

Use a layered real-integration test strategy instead of a single large end-to-end case.

### Layer 1: Real connection and publish smoke tests

Base file:

- `test/integration/thingsboard_real_integration_test.cc`

This layer remains the lowest-risk smoke gate and validates:

- real MQTT/TLS connection
- startup attributes publish
- one-shot business telemetry publish

This layer stays manually enabled and is used to quickly determine whether:

- board certificates are valid
- ThingsBoard reachability is good
- the broker-side authentication path still works

### Layer 2: Real cloud-configuration validation

Add real-board integration coverage for shared attributes.

Validation targets:

- the board receives a real ThingsBoard shared-attribute update
- the update follows current release rules
- `config.json`, `config.pending.json`, and scheduler state change according to the active/pending rules
- the board emits `shared_attr_update`
- subsequent business telemetry reflects the updated active/pending configuration

Only currently supported attributes are in scope:

- `passes` with integer values only
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy` with single-side values only
- `charging_side`
- `schedules`

Rejected cases must also be represented, especially:

- `passes = 0.5`
- `parking_policy = both`

### Layer 3: Real RPC validation

Add real-board integration coverage for RPC entry points.

Validation targets:

- real platform invokes `start`, `stop`, `return`, `terminate`, `reset`
- board replies on the correct RPC response path
- command tracker emits the expected accepted/completed/rejected events
- state changes are consistent with current runtime rules

This layer focuses on command correctness, not long motion scenarios.

### Layer 4: Real state-transition and telemetry validation

This is the release-critical acceptance layer.

Add real-board integration coverage that verifies state transitions can be observed through ThingsBoard-visible telemetry, not only through local state checks.

Target transitions:

- `Idle -> CleanFwd`
- `CleanFwd -> Paused`
- `Paused -> resumed task`
- `Running -> Returning`
- `Returning/Charging/Idle` transitions as supported by the current runtime path

The exact transition matrix must reflect the current code, not an ideal future model.

This layer should verify both:

- local runtime state on the board
- corresponding cloud-visible event/telemetry changes

## 2. Real-Hardware Test Structure

Do not place every validation concern into the existing pure-cloud smoke test.

Instead, keep two real integration groups:

### Group A: Cloud-path smoke

Purpose:

- prove board-to-cloud connectivity
- prove basic publish path

Candidate file:

- existing `test/integration/thingsboard_real_integration_test.cc`

### Group B: Cloud + hardware state integration

Purpose:

- prove real state changes are visible through ThingsBoard
- tie RPC/config inputs to runtime transitions and telemetry output

Candidate file:

- a new integration test file under `test/integration/`
- it should use the real board environment and real cloud path, but avoid unnecessary product-main boot dependencies

This second group should reuse the simplified runtime stack already in the repository rather than booting the full production `main`.

The test harness should construct only the runtime components needed to:

- connect to ThingsBoard
- observe or trigger the current state transitions
- publish/observe telemetry and command events

## 3. ThingsBoard Platform Work Required

The platform side must be treated as part of the validation design. The code-only side is not enough.

### Device authentication setup

Platform work:

- ensure the device exists in ThingsBoard
- ensure the current client certificate is bound to the device authentication path in the same way the board expects
- keep using the current board-side TLS settings until the certificate/hostname mismatch is resolved properly

Operational note:

- the current board-side `insecure_skip_server_name_check=true` is an acceptance-time workaround only
- the proper production fix is either:
  - use a broker hostname that matches the server certificate
  - or reissue the server certificate with the broker IP/domain in SAN

### Shared-attribute preparation

Platform operators must know which attributes are supported and which are intentionally rejected in this release.

Supported for validation:

- `passes` with integer values
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy` limited to single-side values
- `charging_side`
- `schedules`

Explicitly unsupported in this release:

- `passes = 0.5`
- `parking_policy = both`
- weekday-based schedules
- new protection attributes not yet implemented in code

### RPC preparation

Platform must be able to invoke:

- `start`
- `stop`
- `return`
- `terminate`
- `reset`

And operators must know what to check:

- RPC response payload
- command events
- business telemetry changes

### Dashboard/observation preparation

The platform must expose at least the following observables during validation:

- startup attributes
- `device_state`
- `task_state`
- `target_half_passes`
- `completed_half_passes`
- `active_config`
- `pending_config`
- `last_command`

Without this observation surface, acceptance execution becomes guesswork.

## 4. Code Comment Plan

The goal is not broad comment expansion. The goal is to make the real release path legible.

### Comment targets

#### `MqttTransport`

Explain:

- meaning of TLS fields
- distinction between CA verification and hostname/IP verification
- why `insecure_skip_server_name_check` exists
- that it is for temporary debugging/acceptance only

#### `CloudService`

Explain:

- ThingsBoard topic routing responsibilities
- shared-attribute callback contract
- RPC method dispatch and payload handling

#### `ThingsBoardConfigManager`

Explain:

- active vs pending config semantics
- immediate vs next-task application behavior
- release restrictions on `passes` and `parking_policy`

#### `ThingsBoardControlPlane`

Explain:

- responsibility boundary between:
  - config ingress
  - RPC ingress
  - command/status event publish
  - business telemetry publish

The comments should help a field maintainer understand where to look when:

- configuration is accepted but not applied
- RPC is accepted but state does not change
- telemetry is present but stale

## 5. Recommended Implementation Order

1. Strengthen the existing real ThingsBoard smoke tests
2. Add real shared-attribute validation
3. Add real RPC validation
4. Add real state-transition + telemetry validation
5. Add focused comments in the four core files
6. Write operator-facing ThingsBoard validation instructions

This order is intentional:

- first prove the transport path
- then prove configuration ingress
- then prove command ingress
- finally prove runtime-state visibility through telemetry

## 6. Non-Goals

This work does not attempt to:

- add new ThingsBoard product features
- redesign the release semantics for terminals or half-pass behavior
- migrate the system to hostname-correct TLS production setup in this same change set
- add OTA, LoRaWAN, or unsupported cloud configuration fields

## 7. Acceptance Criteria

This design is complete when the project can demonstrate all of the following on a real board against the real ThingsBoard deployment:

1. The board can connect over MQTT/TLS with the current deployed certificate chain.
2. Startup attributes and periodic business telemetry can be published successfully.
3. Supported shared attributes can be applied and reflected in local runtime/config state and subsequent telemetry.
4. Unsupported shared attributes are rejected in a controlled, observable way.
5. Supported RPCs produce the expected response and command events.
6. At least one real runtime state-change sequence is visible through ThingsBoard telemetry.
7. Core ThingsBoard runtime files contain enough comments to explain the release path and the current temporary TLS caveat.
