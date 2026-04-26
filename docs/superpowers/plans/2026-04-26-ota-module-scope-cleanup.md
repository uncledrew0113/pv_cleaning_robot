# OTA Module Scope Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** De-scope the dormant OTA module from the default product build and align the build system and documentation with the fact that OTA is not currently integrated.

**Architecture:** Keep the OTA source available behind an explicit disabled-by-default build option instead of silently compiling it into every product build. Then update the active product documentation so it no longer advertises OTA as a current application capability.

**Tech Stack:** CMake, C++17 build layout, Markdown and API docs

---

## File Structure

- Modify: `CMakeLists.txt`
  - Add an explicit top-level build option for OTA support.
- Modify: `pv_cleaning_robot/CMakeLists.txt`
  - Exclude `middleware/ota_manager.cc` from default product sources when OTA is disabled.
- Modify: `README.md`
  - Remove or mark OTA references as inactive in the current product.
- Modify: `doc/API_REFERENCE.md`
  - Mark `OtaManager` as dormant / not integrated in the current product build.

### Task 1: Make OTA Exclusion Explicit in the Build

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `pv_cleaning_robot/CMakeLists.txt`

- [ ] **Step 1: Add a disabled-by-default OTA option at the top level**

In `CMakeLists.txt`, add this directly after the C++ standard configuration block:

```cmake
option(PV_ENABLE_OTA "Build dormant OTA support into the product binary" OFF)
message(STATUS "PV_ENABLE_OTA = ${PV_ENABLE_OTA}")
```

- [ ] **Step 2: Exclude `ota_manager.cc` from the default product sources**

In `pv_cleaning_robot/CMakeLists.txt`, keep the existing `file(GLOB_RECURSE ROBOT_SOURCES ...)`, then add:

```cmake
if(NOT PV_ENABLE_OTA)
  list(REMOVE_ITEM ROBOT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/middleware/ota_manager.cc")
endif()
```

- [ ] **Step 3: Reconfigure and rebuild the main target**

Run:

```bash
cmake --preset rk3576-build
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected:
- Configure output shows `PV_ENABLE_OTA = OFF`
- The main target still builds cleanly

- [ ] **Step 4: Commit the build-system change**

Run:

```bash
git add CMakeLists.txt pv_cleaning_robot/CMakeLists.txt
git commit -m "build: disable ota module in default product build"
```

### Task 2: Align Active Documentation With Product Reality

**Files:**
- Modify: `README.md`
- Modify: `doc/API_REFERENCE.md`

- [ ] **Step 1: Mark OTA as inactive in the API reference**

In `doc/API_REFERENCE.md`, change the `OtaManager` section header and intro from an active capability description to this wording:

```markdown
### 6.8 `OtaManager` — Dormant OTA Support (Not Integrated In Current Product Build)

This repository still contains an `OtaManager` implementation, but it is not wired into the
current production application flow and is excluded from the default product build unless
`PV_ENABLE_OTA=ON` is set intentionally.
```

- [ ] **Step 2: Update the README feature/documentation summary**

In `README.md`, if OTA is listed among active middleware or capabilities, replace the wording with:

```markdown
- OTA support is currently dormant and excluded from the default build.
- The repository still contains `OtaManager` as archived implementation material for future work.
```

- [ ] **Step 3: Verify no active doc wording still advertises OTA as current behavior**

Run:

```bash
rg -n "OTA|OtaManager" README.md doc/API_REFERENCE.md
```

Expected:
- The remaining matches describe OTA as dormant / not integrated rather than active runtime functionality.

- [ ] **Step 4: Commit the documentation cleanup**

Run:

```bash
git add README.md doc/API_REFERENCE.md
git commit -m "docs: mark ota support as dormant"
```

### Task 3: Verify the Cleanup Boundary

**Files:**
- Read: `CMakeLists.txt`
- Read: `pv_cleaning_robot/CMakeLists.txt`
- Read: `README.md`
- Read: `doc/API_REFERENCE.md`

- [ ] **Step 1: Verify the source tree still contains OTA only as dormant code**

Run:

```bash
rg -n "OtaManager|ota_manager" include pv_cleaning_robot README.md doc/API_REFERENCE.md
```

Expected:
- Source matches remain only in the OTA module itself and documentation that explicitly marks it as dormant.

- [ ] **Step 2: Rebuild the main target once more**

Run:

```bash
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected:
- The default product build still succeeds with OTA excluded.

- [ ] **Step 3: Commit the final verification checkpoint**

Run:

```bash
git commit --allow-empty -m "chore: verify ota scope cleanup"
```
