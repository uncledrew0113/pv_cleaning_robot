# Codebase Audit and Remediation Design

Date: 2026-04-26

## Goal

Produce a repository-wide audit that answers five questions with concrete evidence:

1. Which modules or implementations are redundant, obsolete, or unlikely to be used going forward.
2. Which code paths accessed by multiple threads are not demonstrably thread-safe.
3. Which long-running hot paths have meaningful heap churn and memory fragmentation risk.
4. Which modules violate the intended layer boundaries.
5. Which files are generated clutter, stale artifacts, or likely garbage.

The audit must not stop at observations. It must also identify which findings should become follow-up remediation designs, while keeping pure observations separate from code-change candidates.

## Scope

Included for audit:

- `include/`
- `pv_cleaning_robot/`
- `test/`
- `config/`
- `docs/`
- `doc/`
- `tools/`
- `scripts/`
- `sample/`

Explicitly excluded from source-level audit conclusions:

- `3rdparty/`
- `build/`

These two excluded areas may still be mentioned in the repository hygiene section if they create clutter or should be ignored more strictly, but they are not part of the code-quality or architecture conclusions.

## Deliverables

This work is split into two layers.

### 1. Primary Audit Spec

Write one top-level audit document at:

`docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`

This document is the source of truth for the audit. It contains:

- audit scope and exclusions
- evidence standards
- findings grouped by the five audit dimensions
- priority summary
- recommended follow-up actions
- observations that should not trigger code changes

### 2. Follow-up Remediation Designs

Create separate remediation specs only for findings that meet all of these conditions:

- the problem is concrete enough to act on
- the likely fix crosses more than one file or module, or changes an interface or workflow
- the remediation has enough value to justify planned implementation

Do not create remediation specs for:

- purely informational observations
- low-confidence suspicions without enough evidence
- one-line or purely local fixes that can be handled directly later

## Evidence Standard

Every finding must be labeled with one of two evidence levels.

### `confirmed`

Use this only when the code directly demonstrates the issue. The finding must cite the relevant files and explain why the implementation is problematic without needing runtime speculation.

Examples:

- an apparently dead module with no references
- a shared mutable object accessed from multiple threads without a common lock
- a hot path that repeatedly builds large `std::string` or JSON objects every cycle
- a device layer file containing protocol parsing or service policy logic that belongs elsewhere

### `high-risk`

Use this when the implementation shape strongly suggests a problem, but the final effect still depends on runtime patterns or deployment conditions.

Examples:

- a long-lived background path with repeated heap churn but unknown real-world payload size
- a thread-safety boundary enforced only by comments instead of API structure
- a stale file tree that appears unused but may still be referenced operationally outside the repository

`high-risk` findings must explicitly state what is known, what is inferred, and what would be required to confirm the issue.

## Audit Dimensions

### 1. Redundant or Obsolete Code

Check for:

- files, classes, or helpers with no meaningful references
- code paths superseded by newer implementations while old code remains wired in or partially wired in
- configuration, tests, or docs that still describe an older architecture
- interfaces retained only for historical compatibility but no longer needed by current behavior

Each finding must distinguish between:

- safe cleanup candidates
- compatibility-sensitive leftovers that may need staged removal

### 2. Thread Safety

Check all components that may be accessed from multiple threads, including:

- background reader loops
- periodic update threads
- callbacks from network, RPC, or transport code
- cached status and diagnostics objects
- shutdown and destructor paths

For each candidate, answer:

- what shared state exists
- which threads may read or write it
- what synchronization primitive or execution model protects it
- whether the protection is structural or only documented by comment

Priority goes to cases where:

- object lifetime races with background threads
- one lock protects some fields but not all logically related fields
- a component relies on “single-threaded caller discipline” without an API boundary that makes misuse hard

### 3. Memory Fragmentation Risk

This audit does not treat all use of `std::string`, `std::vector`, or JSON as a problem. It focuses on long-running churn in hot paths.

Inspect especially:

- periodic telemetry/reporting
- protocol parsing loops
- offline caching and replay
- repeated `dump()`, `append()`, `substr()`, `erase()`, and container growth/shrink cycles
- background services that may run for days on embedded Linux

Each finding must explain:

- why the path is hot or long-lived
- where heap allocation likely repeats
- whether capacity reuse makes the risk modest or whether the path truly churns the heap

### 4. Layering Discipline

Use the repository’s intended layering as the reference model:

- `protocol`: stateless encoding, decoding, parsing, field mapping
- `driver`: raw device or OS I/O wrappers
- `device`: device semantics, state caching, lifecycle, command sequencing
- `middleware`: generic infrastructure and shared services
- `service`: business orchestration and cross-device logic
- `app`: state machines and top-level application flow

Flag violations where one layer takes on another layer’s role, such as:

- protocol knowledge embedded in service logic
- business policy embedded in driver or protocol code
- infrastructure concerns embedded deep inside device logic
- top-level orchestration leaking into lower-level modules

The audit must be strict here. “Works today” is not enough if the responsibility boundary is materially blurred.

### 5. Garbage, Stale, or Old Files

Split this into two buckets.

#### Repository hygiene

Examples:

- generated outputs that should not live in the repository
- local environment files
- stale caches
- logs, scratch data, or temporary artifacts

#### Potentially stale maintained content

Examples:

- legacy docs that contradict current code
- obsolete samples
- abandoned tests
- old plans/specs that are clearly superseded and no longer useful as history

Do not classify something as garbage merely because it is old. The audit must consider whether the file still serves as reference, documentation history, or hardware bring-up material.

## Output Format for Findings

Each audit finding in the final audit document must use a consistent structure:

- `ID`
- `Title`
- `Dimension`
- `Evidence Level`: `confirmed` or `high-risk`
- `Impact`
- `Files`
- `Summary`
- `Recommended Action`
- `Needs Remediation Spec`: `yes` or `no`
- `Priority`: `P0`, `P1`, `P2`, or `observe`

Priority meaning:

- `P0`: correctness or safety risk that can plausibly cause runtime failure or unsafe behavior
- `P1`: important design debt or performance risk worth scheduling soon
- `P2`: real issue, but localized or low urgency
- `observe`: document it, but do not plan code changes yet

## Remediation Design Selection Rules

After the audit findings document is complete and reviewed, follow-up remediation specs should be created only for the subset of findings that are both real and worth planned work.

Expected categories, if justified by the findings, include:

- telemetry, `gpsd`, or cache-related hot-path memory hardening
- thread-safety hardening around lifecycle and shared-state boundaries
- layer-boundary cleanup where responsibilities are materially mixed
- repository hygiene or stale-file cleanup when it affects maintainability

This list is not a commitment. The actual remediation specs must be driven by audit evidence, not by preselected themes.

## Review and Execution Flow

The work proceeds in this order:

1. Perform the repository audit.
2. Write the audit findings spec.
3. Review the written audit spec for ambiguity, contradictions, and overreach.
4. Have the user review the audit spec.
5. Based on approved findings, write only the necessary remediation specs.
6. Have the user review those remediation specs.
7. Invoke `writing-plans` for the approved remediation work.

This keeps audit facts separate from implementation planning and prevents speculative cleanup from turning directly into code changes.

## Non-Goals

This work does not:

- implement fixes
- refactor code during the audit phase
- treat third-party code as part of the architecture review
- require dynamic instrumentation before producing the first audit report

Dynamic verification may be proposed later for `high-risk` findings, but the initial deliverable is a static evidence-based audit plus remediation design entry points.
