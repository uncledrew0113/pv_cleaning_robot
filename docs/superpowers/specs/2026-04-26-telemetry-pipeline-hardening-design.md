# Telemetry Pipeline Hardening Design

Date: 2026-04-26

## Problem

The audit found one connected cluster of issues in the telemetry pipeline:

- `HealthService` combines service policy, JSON encoding, and local file persistence
- `HealthService` rebuilds JSON payloads through mutable `nlohmann::json` state and `dump()` on every reporting cycle
- `DataCache` rewrites the full persisted queue on every enqueue and acknowledgement

These are not isolated issues. They are different symptoms of one telemetry path that lacks clear boundaries and churns memory and I/O under steady-state and offline conditions.

## Scope

In scope:

- telemetry payload construction for HEALTH and DIAGNOSTICS modes
- separation between telemetry field selection, payload encoding, and local sink behavior
- offline cache persistence format and mutation strategy
- interaction points among `HealthService`, `CloudService`, and `DataCache`

Out of scope:

- MQTT or LoRaWAN transport redesign
- changing the external telemetry JSON shape unless strictly necessary
- unrelated device-level parsing optimizations such as serial NMEA tokenization

## Proposed Changes

### 1. Split telemetry responsibilities into explicit units

Target boundary:

- `HealthService`: choose which device data belongs in the payload
- payload builder/encoder helper: serialize the selected view into JSON
- optional local sink helper: append local telemetry records
- `CloudService`: route already-built payloads to network or offline cache

This removes formatting and persistence details from `HealthService`.

### 2. Replace hot-path JSON tree mutation with a reuse-oriented encoder

Preferred direction:

- stop using a mutable `nlohmann::json` tree as the periodic hot path
- use a fixed-buffer or otherwise reuse-oriented builder for the recurring payload
- keep output shape stable so cloud-side consumers do not break

The key requirement is not “no `std::string` anywhere.” The requirement is to eliminate repeated large-object churn in the once-per-cycle path.

### 3. Decouple local file logging from the service object

`HealthService` should not directly own JSONL append semantics.

Instead:

- either a dedicated telemetry sink helper handles local append logging
- or local logging is pushed behind `CloudService` or another infrastructure boundary

The service should decide that a payload exists, not how files are appended or flushed.

### 4. Replace full-rewrite cache persistence with append-first mutation

Preferred cache model:

- append a small record on enqueue
- append an acknowledgement record or maintain a compact ack representation
- compact periodically based on size or stale-ack thresholds

This avoids:

- full queue reserialization on every push
- full file rewrite on every confirmation
- repeated temporary-file rename cycles during offline bursts

### 5. Keep the telemetry contract stable

The remediation should preserve:

- existing HEALTH vs DIAGNOSTICS mode selection
- current external payload fields unless a specific field bug is intentionally fixed
- current `CloudService` role as the routing point to network or cache

## Risks and Tradeoffs

- A fixed-buffer encoder improves churn behavior, but makes formatting code more explicit and less flexible than raw `nlohmann::json`.
- Splitting the telemetry path into helpers increases file count, but the boundary clarity is worth it.
- Journal-style cache persistence reduces rewrite churn, but requires recovery and compaction logic on startup.

## Testing and Verification

Required verification should cover:

- HEALTH and DIAGNOSTICS payload shape remains compatible
- payload generation still includes optional distance-sensor data correctly
- offline caching still survives restart and replay
- cache replay still confirms and removes sent records correctly
- local telemetry logging still writes valid JSONL records when enabled

Performance-oriented checks should also verify:

- no full queue rewrite on every cache mutation
- the periodic telemetry path no longer depends on `nlohmann::json::dump()` as its hot path

## Non-Goals

- redesigning ThingsBoard topics or RPC
- changing cloud-side schema conventions
- optimizing every protocol parser in the codebase during the same change
