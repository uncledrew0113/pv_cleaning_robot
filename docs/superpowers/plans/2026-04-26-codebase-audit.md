# Codebase Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a repository-wide audit findings spec plus the necessary remediation design specs for confirmed or high-risk issues in redundancy, thread safety, memory fragmentation, layering, and file hygiene.

**Architecture:** Execute the work in two phases. First, perform a static audit across the included directories and consolidate the evidence into a single findings spec. Second, promote only the findings that justify planned work into separate remediation specs, keeping observations and implementation candidates clearly separated.

**Tech Stack:** Git, ripgrep, sed, C++ repository structure, Markdown specs under `docs/superpowers/specs/`

---

### Task 1: Establish Audit Workspace and Evidence Index

**Files:**
- Create: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`
- Modify: `docs/superpowers/plans/2026-04-26-codebase-audit.md`

- [ ] **Step 1: Capture current repository state**

Run:

```bash
git status --short
git log --oneline -12
find include pv_cleaning_robot test config docs doc tools scripts sample -type f | sort > /tmp/codebase_audit_filelist.txt
```

Expected:
- Working tree status is recorded for later isolation.
- Recent commit history is visible.
- `/tmp/codebase_audit_filelist.txt` contains the audit file inventory.

- [ ] **Step 2: Create the findings spec scaffold**

Write `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md` with these exact section headers:

```markdown
# Codebase Audit Findings

Date: 2026-04-26

## Scope

## Evidence Levels

## Summary

## Findings by Dimension

### Redundant or Obsolete Code

### Thread Safety

### Memory Fragmentation Risk

### Layering Discipline

### Garbage, Stale, or Old Files

## Priority Rollup

## Recommended Remediation Spec Candidates

## Observations That Should Not Trigger Code Changes
```

- [ ] **Step 3: Verify the findings scaffold exists**

Run:

```bash
sed -n '1,120p' docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
```

Expected:
- The scaffold renders with the exact top-level sections above.

- [ ] **Step 4: Commit the empty findings scaffold separately**

Run:

```bash
git add docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
git commit -m "docs: scaffold codebase audit findings"
```

Expected:
- A commit exists that isolates the findings document creation from the later audit content.

### Task 2: Audit Redundant Code, Layering, and Repository Hygiene

**Files:**
- Modify: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`
- Read: `include/pv_cleaning_robot/**/*.h`
- Read: `pv_cleaning_robot/**/*.cc`
- Read: `test/**/*.cc`
- Read: `docs/**/*.md`
- Read: `doc/**/*`

- [ ] **Step 1: Build a reference map for symbols and likely dead modules**

Run:

```bash
rg -n "class |struct |namespace robot::" include/pv_cleaning_robot pv_cleaning_robot > /tmp/codebase_audit_symbols.txt
rg -n "TODO|FIXME|deprecated|obsolete|legacy|compat|unused|dead" include pv_cleaning_robot test docs doc > /tmp/codebase_audit_keywords.txt
rg -n "start\\(|set_rpm\\(|set_mode_|enter_idle\\(|publish_telemetry\\(|flush_cache\\(" include pv_cleaning_robot test > /tmp/codebase_audit_callsites.txt
```

Expected:
- `/tmp/codebase_audit_symbols.txt` captures declared units.
- `/tmp/codebase_audit_keywords.txt` captures stale-code indicators.
- `/tmp/codebase_audit_callsites.txt` captures call-site evidence for recently changed interfaces.

- [ ] **Step 2: Inspect layer boundaries in representative hot modules**

Review these files directly:

```text
include/pv_cleaning_robot/protocol/gpsd_json_parser.h
pv_cleaning_robot/protocol/gpsd_json_parser.cc
include/pv_cleaning_robot/device/gps_source.h
pv_cleaning_robot/device/gpsd_gps_source.cc
pv_cleaning_robot/device/brush_motor.cc
pv_cleaning_robot/service/health_service.cc
pv_cleaning_robot/service/cloud_service.cc
pv_cleaning_robot/middleware/data_cache.cc
pv_cleaning_robot/main.cc
```

Expected:
- You can state whether protocol parsing, lifecycle policy, telemetry policy, and orchestration are in the correct layers.

- [ ] **Step 3: Inspect repository hygiene candidates**

Run:

```bash
find . -maxdepth 3 \( -path './3rdparty' -o -path './build' \) -prune -o -type f \( -name '*.log' -o -name '*.tmp' -o -name '*.bak' -o -name '.DS_Store' -o -path './.cache/*' -o -path './.vscode/*' \) -print
```

Expected:
- Potential garbage or local-environment files outside the excluded directories are listed.

- [ ] **Step 4: Write confirmed and high-risk findings for these three dimensions**

Add findings under:

```markdown
### Redundant or Obsolete Code
### Layering Discipline
### Garbage, Stale, or Old Files
```

Each finding must use this exact template:

```markdown
#### AUDIT-XXX
- **Title:**
- **Dimension:**
- **Evidence Level:** confirmed | high-risk
- **Impact:**
- **Files:**
- **Summary:**
- **Recommended Action:**
- **Needs Remediation Spec:** yes | no
- **Priority:** P0 | P1 | P2 | observe
```

- [ ] **Step 5: Commit the first populated findings chunk**

Run:

```bash
git add docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
git commit -m "docs: audit layering and stale code findings"
```

Expected:
- The findings doc now contains the first three completed dimensions in a dedicated commit.

### Task 3: Audit Thread Safety and Memory Fragmentation

**Files:**
- Modify: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`
- Read: `include/pv_cleaning_robot/device/*.h`
- Read: `pv_cleaning_robot/device/*.cc`
- Read: `include/pv_cleaning_robot/middleware/*.h`
- Read: `pv_cleaning_robot/middleware/*.cc`
- Read: `include/pv_cleaning_robot/service/*.h`
- Read: `pv_cleaning_robot/service/*.cc`

- [ ] **Step 1: Build the thread-sharing hotspot list**

Run:

```bash
rg -n "std::thread|atomic<|std::mutex|PiMutex|lock_guard|unique_lock|running_|read_loop|recv_loop|update\\(" include pv_cleaning_robot > /tmp/codebase_audit_threading.txt
```

Expected:
- `/tmp/codebase_audit_threading.txt` highlights files with background threads, locks, and lifecycle-sensitive state.

- [ ] **Step 2: Build the heap-churn hotspot list**

Run:

```bash
rg -n "std::string|std::vector|deque|json::parse|dump\\(|append\\(|substr\\(|erase\\(|push_back\\(|emplace_back\\(" include pv_cleaning_robot > /tmp/codebase_audit_heap.txt
```

Expected:
- `/tmp/codebase_audit_heap.txt` highlights likely fragmentation and repeated-allocation paths.

- [ ] **Step 3: Inspect the highest-signal concurrency and heap files**

Review these files directly:

```text
pv_cleaning_robot/device/imu_device.cc
pv_cleaning_robot/device/gpsd_gps_source.cc
pv_cleaning_robot/device/serial_gps_source.cc
pv_cleaning_robot/device/walk_motor_group.cc
pv_cleaning_robot/device/brush_motor.cc
pv_cleaning_robot/service/health_service.cc
pv_cleaning_robot/service/cloud_service.cc
pv_cleaning_robot/middleware/data_cache.cc
pv_cleaning_robot/middleware/network_manager.cc
pv_cleaning_robot/app/fault_handler.cc
pv_cleaning_robot/app/robot_fsm.cc
```

Expected:
- You can describe for each hotspot what shared state exists, which threads touch it, and whether the synchronization is structural or informal.

- [ ] **Step 4: Write thread-safety and memory-fragmentation findings**

Add findings under:

```markdown
### Thread Safety
### Memory Fragmentation Risk
```

Requirements:
- Distinguish `confirmed` from `high-risk`.
- For memory findings, explain why the path is hot or long-lived.
- For thread findings, explain which threads participate and what protects the state.

- [ ] **Step 5: Fill the summary and priority rollup sections**

Populate:

```markdown
## Summary
## Priority Rollup
## Recommended Remediation Spec Candidates
## Observations That Should Not Trigger Code Changes
```

Expected:
- The document now has complete findings coverage across all five dimensions.
- The remediation candidate section names only the issues that justify follow-up design work.

- [ ] **Step 6: Commit the completed audit findings**

Run:

```bash
git add docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
git commit -m "docs: complete codebase audit findings"
```

Expected:
- A commit exists with the full findings spec.

### Task 4: Self-Review the Findings Spec and Prepare User Review

**Files:**
- Modify: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`

- [ ] **Step 1: Check for placeholders and malformed finding entries**

Run:

```bash
rg -n "TBD|TODO|FIXME|XXX|\\*\\*Title:\\*\\* *$|\\*\\*Summary:\\*\\* *$|confirmed \\| high-risk|P0 \\| P1 \\| P2" docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
```

Expected:
- No placeholder matches remain.

- [ ] **Step 2: Check internal consistency between findings and remediation candidates**

Run:

```bash
sed -n '1,260p' docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
```

Expected:
- Every remediation candidate corresponds to one or more findings marked `Needs Remediation Spec: yes`.
- No `observe`-only item is accidentally promoted to a remediation candidate.

- [ ] **Step 3: Make inline corrections and commit the review pass**

Run:

```bash
git add docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
git commit -m "docs: polish codebase audit findings"
```

Expected:
- The findings spec is review-ready for the user.

### Task 5: Write Remediation Design Specs for Approved Candidate Areas

**Files:**
- Create: `docs/superpowers/specs/2026-04-26-<candidate>-design.md`
- Modify: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`

- [ ] **Step 1: Select only the candidate areas that meet the promotion rules**

Promotion rules:

```text
1. The finding is concrete enough to act on.
2. The fix spans multiple files or modules, or changes an interface or workflow.
3. The benefit justifies planned implementation.
```

Expected:
- A short list of candidate remediation specs is produced, with no purely observational items.

- [ ] **Step 2: Create one remediation spec per promoted candidate**

For each selected candidate, write a dedicated spec file with these sections:

```markdown
# <Candidate Title> Design

Date: 2026-04-26

## Problem

## Scope

## Proposed Changes

## Risks and Tradeoffs

## Testing and Verification

## Non-Goals
```

Expected:
- Each remediation area has an isolated design document with clear boundaries.

- [ ] **Step 3: Link the created remediation specs from the findings document**

Add a short list under `## Recommended Remediation Spec Candidates` in:

```text
docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
```

Expected:
- The findings document points to each created remediation spec path.

- [ ] **Step 4: Commit the remediation design documents**

Run:

```bash
git add docs/superpowers/specs/2026-04-26-codebase-audit-findings.md docs/superpowers/specs/2026-04-26-*-design.md
git commit -m "docs: add codebase audit remediation specs"
```

Expected:
- The remediation designs are isolated in a dedicated documentation commit.

### Task 6: Prepare for Planning Handoff

**Files:**
- Read: `docs/superpowers/specs/2026-04-26-codebase-audit-findings.md`
- Read: `docs/superpowers/specs/2026-04-26-*-design.md`

- [ ] **Step 1: Verify spec coverage against the audit design**

Run:

```bash
sed -n '1,260p' docs/superpowers/specs/2026-04-26-codebase-audit-design.md
sed -n '1,320p' docs/superpowers/specs/2026-04-26-codebase-audit-findings.md
```

Expected:
- All five audit dimensions appear in the findings spec.
- Remediation specs exist only for promoted findings.

- [ ] **Step 2: Confirm the user review gate is ready**

Checklist:

```text
- Findings spec is committed.
- Remediation specs, if any, are committed.
- No implementation changes were made as part of the audit phase.
```

Expected:
- The work is ready for the user to review before any implementation planning begins.

- [ ] **Step 3: Hand off for user review**

Use this exact message:

```text
Audit findings and remediation specs are written and committed. Please review the findings spec first, then the linked remediation specs, and let me know what you want to carry into implementation planning.
```

Expected:
- The user is prompted to review the audit deliverables before any implementation plan is created for code changes.
