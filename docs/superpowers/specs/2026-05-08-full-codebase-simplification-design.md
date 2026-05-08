# PV Cleaning Robot Full Codebase Simplification Design

## 1. Goal

This design defines a project-wide simplification pass across implementation code and test code.

The target result is:

- fewer files where current splits do not buy clarity
- less duplicated logic across `app`, `service`, `middleware`, and tests
- fewer compatibility branches kept only for historical structure
- clearer ThingsBoard behavior and comments
- code that a beginner can follow without tracing too many indirections

This is a simplification and consolidation phase, not a feature expansion phase.

## 2. Non-Negotiable Rules

All implementation work in this phase must follow these rules:

1. Do not add new abstraction layers.
2. Prefer merging over splitting when current files are over-separated.
3. Prefer deleting compatibility paths when they only preserve old structure and no longer serve the current business model.
4. Prefer direct, readable code over reusable-but-indirect helper frameworks.
5. Keep behavior stable unless the current behavior is redundant, inconsistent, or already identified as debt.
6. Update tests together with runtime code so tests stop preserving obsolete structure.

## 3. Scope

This phase covers all project code, with priority on the layers the user called out:

1. `pv_cleaning_robot/app`
2. `pv_cleaning_robot/service`
3. `pv_cleaning_robot/middleware`
4. all related headers in `include/pv_cleaning_robot`
5. all related test code in `test/app`, `test/service`, `test/middleware`, `test/integration`, and `test/integration/hardware`

Drivers, protocols, and devices are not the first target, but obvious dead code, duplicate helpers, or readability defects found during the pass are in scope when the change remains bounded.

## 4. Main Simplification Targets

### 4.1 App Layer

Current problems:

- `main.cc` still carries too much runtime knowledge and business orchestration.
- startup position interpretation, callback wiring, cloud registration, and startup-status projection are too spread out inside the entrypoint
- `RobotSupervisor` repeats state admission and state-projection logic
- `RobotFsm` still contains state-handling code that is readable locally but duplicated conceptually with supervisor- and entrypoint-level gating

Planned simplification:

- reduce `main.cc` to composition, lifecycle, and a small amount of startup glue
- move repeated parking-side and start-gating usage into fewer implementation points
- keep `RobotSupervisor` as the single business admission point for `start`, `stop`, and `return`
- simplify `RobotSupervisor` helpers so state checks and runtime snapshot projection are easier to follow in one place
- trim local helper sprawl in `RobotFsm` and align comments with the actual task semantics now used by the project

### 4.2 Service Layer

Current problems:

- ThingsBoard behavior is split across multiple files even though the code paths are tightly coupled
- JSON field lists, RPC reply semantics, command-event publishing, and shared-attribute rules are spread across multiple implementation units
- configuration truth is read from multiple angles and some service code still exposes historical structural separation rather than the current runtime model
- some files are small enough alone, but together force too much jumping for one business flow

Planned simplification:

- consolidate the ThingsBoard runtime chain into fewer files
- centralize ThingsBoard field names, payload semantics, and shared-attribute rules
- keep comments at the business boundary instead of repeating small fragments in every call site
- simplify surrounding services only where it reduces duplication or confusion without adding another layer

### 4.3 Middleware Layer

Current problems:

- middleware files are less duplicated than app/service, but some boundaries are under-explained
- transport/cache/safety code still requires too much context switching for readers new to the project

Planned simplification:

- keep middleware changes targeted
- remove redundant code paths and low-value helper duplication where found
- improve comments on real responsibilities, especially where middleware is part of the runtime contract used by ThingsBoard or app logic

### 4.4 Test Code

Current problems:

- runtime config file setup is duplicated across many fixtures
- ThingsBoard runtime assembly is duplicated across `test/app`, `test/service`, and integration tests
- mock runtime fixtures recreate the same object graph with small variations
- hardware and real-ThingsBoard support already has a shared support header, but the shared support boundary is too narrow

Planned simplification:

- consolidate repeated runtime/TB fixture setup into shared support
- reuse existing support files where possible instead of creating a new helper hierarchy
- delete repeated local fixture code when a shared setup is enough
- keep test code direct and readable; no test framework wrapper abstraction should be introduced

## 5. ThingsBoard Consolidation Plan

The user explicitly requested that ThingsBoard code be merged where possible.

### 5.1 Current split

Today the runtime behavior is spread across:

- `thingsboard_control_plane.*`
- `thingsboard_config_manager.*`
- `thingsboard_event_payload_builder.*`
- related parts of `cloud_service.*`

This split creates avoidable navigation cost because one business flow typically touches all of them.

### 5.2 Target shape

The ThingsBoard-specific service code should be collapsed into fewer files, with the preferred direction being:

- one primary ThingsBoard service header/source pair containing:
  - shared-attribute config application
  - RPC registration and reply semantics
  - startup attributes
  - status/command/business telemetry payload construction and publishing
- `CloudService` remains the generic MQTT/topic router unless merging a very small ThingsBoard-only helper into the consolidated service clearly removes duplication

The consolidation is intentionally biased toward fewer files, even if that means one larger service file, as long as internal sections remain clearly commented and ordered.

### 5.3 Documentation priority

ThingsBoard comments must explain:

1. which data is immediate vs next-task
2. why schedules are treated differently
3. where RPC truth lives
4. what payload families exist
5. how cloud command/result/event projection maps to local runtime behavior

Comments should be written for readers who do not already know ThingsBoard topic conventions.

## 6. File-Level Change Strategy

### 6.1 File count reduction is preferred

If two files are only separated for historical neatness and now form one runtime concept, they should be merged.

Examples of preferred merges:

- multiple ThingsBoard service files into one primary service unit
- repeated local test fixtures into existing shared support headers
- small internal-only helper code moved back into the implementation file that owns the behavior

### 6.2 New files are allowed only when they reduce more than they add

A new file is acceptable only if:

- it replaces more repeated code than it introduces
- it does not create a new abstraction layer
- it improves beginner readability

This means “extract because the file is long” is not enough by itself.

## 7. Deletion Policy

Old code should be deleted when one of these is true:

1. it exists only for old state semantics or old cloud contract semantics
2. it duplicates logic that now has a clearer single owner
3. it preserves a split that no longer matches the runtime business flow
4. tests only exist to preserve obsolete structure rather than current behavior

Deletion is preferred over leaving dormant parallel paths in place.

## 8. Expected Concrete Outcomes

This phase should produce outcomes like these:

- fewer ThingsBoard source/header files
- smaller effective runtime knowledge surface in `main.cc`
- simpler supervisor/control flow for task admission and projection
- reduced duplicate runtime-config setup across tests
- stronger in-code comments around ThingsBoard behavior
- fewer jumps required to trace one complete cloud-driven runtime path

## 9. Verification Standard

Every consolidation change must be validated with the smallest relevant test set first, then broader regression coverage for touched areas.

Minimum expectation:

- build the changed targets
- run touched unit tests
- run touched integration tests where applicable
- explicitly note any test area that could not be run

No success claim should be made without command-based validation.

## 10. Implementation Order

The implementation phase should proceed in this order:

1. consolidate ThingsBoard service code and comments
2. simplify `app` runtime orchestration around the new service shape
3. clean up adjacent `service` duplication exposed by that consolidation
4. apply targeted middleware readability and redundancy cleanup
5. consolidate test fixtures and delete repeated setup code
6. run verification for each touched area, then a broader final pass

## 11. Success Criteria

This design is successful when:

1. the codebase has fewer unnecessary files than before
2. the ThingsBoard flow can be understood from substantially fewer files
3. `main.cc` no longer acts as a major business-policy container
4. duplicated runtime and fixture assembly code is meaningfully reduced
5. comments explain runtime and ThingsBoard behavior in beginner-readable terms
6. no new abstraction layer was introduced to achieve the cleanup
