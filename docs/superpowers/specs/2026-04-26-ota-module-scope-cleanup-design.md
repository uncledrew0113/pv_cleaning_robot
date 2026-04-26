# OTA Module Scope Cleanup Design

Date: 2026-04-26

## Problem

`OtaManager` is currently in an ambiguous state:

- it is compiled into the product build
- it is documented in API material
- it has no wiring from `main`, `service`, or `app`
- it has no tests exercising a real first-party integration path

This is dead-weight scope, not a half-finished feature in active use.

## Scope

In scope:

- deciding whether OTA is officially supported in this repository state
- aligning build, source tree, and documentation with that decision
- removing or explicitly parking unused OTA code if it is out of scope

Out of scope:

- designing a new OTA feature from scratch
- adding cloud OTA control flow during this cleanup
- implementing a future production OTA workflow

## Proposed Changes

Recommended remediation: de-scope OTA from the active product build unless the team explicitly recommits to finishing it now.

### 1. Treat OTA as unsupported by default

Unless there is an immediate product need, the repository should stop pretending OTA is available.

That means:

- remove `OtaManager` from the active build
- remove or clearly mark current API documentation as inactive/not integrated
- avoid leaving dormant code compiled just because `middleware/*.cc` is globbed

### 2. Make the build reflect product reality

Preferred direction:

- exclude `ota_manager.cc` from normal product composition
- if preservation is required, move OTA behind an explicit build option or archived module boundary

The important point is that unsupported functionality should not silently ship in the binary.

### 3. Align documentation with actual scope

Documentation should either:

- remove the OTA section, or
- mark it as not currently integrated into the production application flow

API reference material should not imply support for a feature with no entry point.

### 4. Keep a clean path for future revival if needed

If the team wants to keep the code for later:

- preserve it behind an explicit disabled-by-default switch
- document that it is dormant and unintegrated
- require a future integration spec before re-enabling it

## Risks and Tradeoffs

- Full deletion gives the cleanest repository state, but removes a starting point for future OTA work.
- Build-gating keeps future optionality, but still leaves dormant maintenance surface.
- Documentation-only marking is insufficient on its own, because it still ships dead code in the binary.

## Testing and Verification

Required verification should cover:

- product build still succeeds after OTA de-scoping
- no active code path references `OtaManager`
- documentation no longer advertises OTA as a currently supported application capability

If OTA is moved behind a build flag instead of deleted, verify:

- default build excludes it
- enabling the flag restores the intended source inclusion cleanly

## Non-Goals

- implementing a replacement OTA architecture
- adding tests for an inactive feature just to preserve it
- expanding cloud control logic during cleanup
