# Business Flow Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Audit the main-program business flow end-to-end against the newly confirmed simplified business model, classify deterministic defects vs. semantic design risks, and produce a bounded findings document that drives the next simplification phase.

**Architecture:** This plan treats the audit itself as the deliverable. It first creates a stable findings document skeleton, then walks the runtime flow in the same order as the real program entry path, but evaluates each segment against the user-confirmed target model: split configuration ownership, invalid-position start denial, reduced RPC contract, no pause/resume model, no low-battery return model, and threshold-driven charging. It finishes by extracting a must-fix list, a design-debt list, and a simplification-input section for the next phase.

**Tech Stack:** C++17 codebase, Catch2 tests, CMake build, markdown docs, `rg`, `sed`, `git`

---

## File Structure

**Create:**
- `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
  - Final audit output. Contains runtime flow map, per-flow findings, module attribution, must-fix defect list, design-debt list, and simplification inputs.

**Reference / Inspect Only:**
- `docs/superpowers/specs/2026-05-08-business-flow-audit-design.md`
  - Approved audit spec.
- `pv_cleaning_robot/main.cc`
  - Runtime wiring truth.
- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `include/pv_cleaning_robot/app/parking_side_runtime.h`
- `include/pv_cleaning_robot/device/limit_switch.h`
- `pv_cleaning_robot/device/limit_switch.cc`
- `include/pv_cleaning_robot/middleware/safety_monitor.h`
- `pv_cleaning_robot/middleware/safety_monitor.cc`
- `pv_cleaning_robot/service/motion_service.cc`
- `pv_cleaning_robot/service/nav_service.cc`
- `pv_cleaning_robot/service/fault_service.cc`
- `include/pv_cleaning_robot/app/fault_handler.h`
- `pv_cleaning_robot/app/fault_handler.cc`
- `pv_cleaning_robot/service/scheduler_service.cc`
- `include/pv_cleaning_robot/service/thingsboard_config_manager.h`
- `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- `include/pv_cleaning_robot/service/thingsboard_control_plane.h`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `include/pv_cleaning_robot/service/cloud_service.h`
- `pv_cleaning_robot/service/cloud_service.cc`
- `include/pv_cleaning_robot/middleware/mqtt_transport.h`
- `pv_cleaning_robot/middleware/mqtt_transport.cc`
- `include/pv_cleaning_robot/middleware/network_manager.h`
- `pv_cleaning_robot/middleware/network_manager.cc`
- `pv_cleaning_robot/middleware/data_cache.cc`
- `pv_cleaning_robot/middleware/event_bus.h`
- `include/pv_cleaning_robot/device/bms.h`
- `pv_cleaning_robot/device/bms.cc`
- `include/pv_cleaning_robot/middleware/ota_manager.h`
- `pv_cleaning_robot/middleware/ota_manager.cc`
- `pv_cleaning_robot/service/health_service.cc`
- `include/pv_cleaning_robot/app/watchdog_mgr.h`
- `pv_cleaning_robot/app/watchdog_mgr.cc`

**Validation References:**
- `test/integration/system_integration_test.cc`
- `test/integration/task_chain_test.cc`
- `test/service/thingsboard_control_plane_test.cc`
- `test/service/cloud_service_test.cc`
- `test/middleware/safety_monitor_test.cc`
- `test/app/robot_fsm_test.cc`
- `test/app/robot_supervisor_test.cc`

## Task 1: Create the Audit Findings Skeleton

**Files:**
- Create: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
- Modify: none
- Test: none

- [ ] **Step 1: Write the findings document skeleton**

```markdown
# PV Cleaning Robot Business Flow Audit Findings

## 1. Audit Scope

## 2. Runtime Flow Map

## 3. Flow Findings

### 3.1 Startup and Configuration Loading

### 3.2 Power-On Self-Check and Position Interpretation

### 3.3 Scheduler Start and RPC Start

### 3.4 Cleaning Round-Trip Main Flow

### 3.5 Stop and Task-Level Return

### 3.6 Reset and Reboot Behavior

### 3.7 Fault Reporting and Safe Halt

### 3.8 Charging and Battery Gate Behavior

### 3.9 Cloud Status Projection and Command Events

### 3.10 Thread, Event, Cache, and Runtime Support Layers

## 4. Must-Fix Deterministic Defects

## 5. High-Priority Semantic / Responsibility Risks

## 6. Test Divergence and Sync Targets

## 7. Simplification Inputs for the Next Phase
```

- [ ] **Step 2: Create the file with the skeleton**

Run:

```patch
*** Begin Patch
*** Add File: docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
+# PV Cleaning Robot Business Flow Audit Findings
+
+## 1. Audit Scope
+
+## 2. Runtime Flow Map
+
+## 3. Flow Findings
+
+### 3.1 Startup and Configuration Loading
+
+### 3.2 Power-On Self-Check and Position Interpretation
+
+### 3.3 Scheduler Start and RPC Start
+
+### 3.4 Cleaning Round-Trip Main Flow
+
+### 3.5 Stop and Task-Level Return
+
+### 3.6 Reset and Reboot Behavior
+
+### 3.7 Fault Reporting and Safe Halt
+
+### 3.8 Charging and Battery Gate Behavior
+
+### 3.9 Cloud Status Projection and Command Events
+
+### 3.10 Thread, Event, Cache, and Runtime Support Layers
+
+## 4. Must-Fix Deterministic Defects
+
+## 5. High-Priority Semantic / Responsibility Risks
+
+## 6. Test Divergence and Sync Targets
+
+## 7. Simplification Inputs for the Next Phase
*** End Patch
```

Expected: file created with the exact section structure above.

- [ ] **Step 3: Verify the skeleton file contents**

Run:

```bash
sed -n '1,120p' docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
```

Expected: the headings `## 1. Audit Scope` through `## 7. Simplification Inputs for the Next Phase` are present, with sections 3.5 through 3.10 matching the simplified business model.

- [ ] **Step 4: Commit the skeleton**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: add business flow audit findings skeleton"
```

## Task 2: Audit Startup, Position Truth, and Start Admission

**Files:**
- Modify: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
- Reference: `pv_cleaning_robot/main.cc`, `include/pv_cleaning_robot/app/parking_side_runtime.h`, `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/app/robot_fsm.cc`, `include/pv_cleaning_robot/app/robot_supervisor.h`, `pv_cleaning_robot/app/robot_supervisor.cc`, `include/pv_cleaning_robot/service/thingsboard_control_plane.h`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`, `pv_cleaning_robot/service/thingsboard_config_manager.cc`
- Test: `test/integration/system_integration_test.cc`, `test/integration/task_chain_test.cc`, `test/service/thingsboard_control_plane_test.cc`, `test/app/robot_fsm_test.cc`, `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: Collect runtime evidence for startup and start-admission flows**

Run:

```bash
sed -n '180,470p' pv_cleaning_robot/main.cc
sed -n '1,220p' include/pv_cleaning_robot/app/robot_fsm.h
sed -n '1,240p' pv_cleaning_robot/app/robot_fsm.cc
sed -n '1,220p' include/pv_cleaning_robot/app/robot_supervisor.h
sed -n '1,240p' pv_cleaning_robot/app/robot_supervisor.cc
sed -n '1,220p' include/pv_cleaning_robot/service/thingsboard_control_plane.h
sed -n '1,260p' pv_cleaning_robot/service/thingsboard_control_plane.cc
sed -n '1,260p' pv_cleaning_robot/service/thingsboard_config_manager.cc
```

Expected: enough context to reconstruct startup config loading, fixed-vs-cloud config ownership, abnormal-position handling, and scheduler/RPC start admission.

- [ ] **Step 2: Fill in sections 1, 2, 3.1, 3.2, and 3.3**

```markdown
## 1. Audit Scope

- Runtime truth source: `pv_cleaning_robot/main.cc` and modules wired directly by `main.cc`
- Tests used only as validation and divergence detectors
- README and historical comments treated as non-authoritative when code differs

## 2. Runtime Flow Map

`ConfigService -> main wiring -> LimitSwitch/SafetyMonitor -> main event bridge -> RobotFsm -> RobotSupervisor -> Motion/Nav/Fault/Scheduler -> CloudService/ThingsBoard -> periodic reporting/cache`

### 3.1 Startup and Configuration Loading

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `main.cc`
- ...

### 3.2 Power-On Self-Check and Position Interpretation

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `main.cc`
- `ParkingSideRuntime`
- `SafetyMonitor`

### 3.3 Scheduler Start and RPC Start

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `main.cc`
- `ThingsBoardControlPlane`
- `RobotSupervisor`
- `RobotFsm`
```

- [ ] **Step 3: Check test divergence for these flows**

Run:

```bash
rg -n "EvScheduleStart|EvFarEndLimitSettled|EvParkingSideLimitSettled|start_task|register_rpc_handlers|parking_side|start|stop|return|reset|terminate|pause|resume" \
  test/integration/system_integration_test.cc \
  test/integration/task_chain_test.cc \
  test/service/thingsboard_control_plane_test.cc \
  test/app/robot_fsm_test.cc \
  test/app/robot_supervisor_test.cc
```

Expected: a concrete list of tests that mirror or diverge from runtime start and position semantics.

- [ ] **Step 4: Record test sync targets in section 6**

```markdown
## 6. Test Divergence and Sync Targets

### Startup / Position / Start Admission

- `test/integration/system_integration_test.cc`
  - mirrors runtime startup wiring
  - must be kept in sync with `main.cc` position interpretation and invalid-position handling
- `test/integration/task_chain_test.cc`
  - exercises task-start path through FSM and SafetyMonitor bridge
- `test/service/thingsboard_control_plane_test.cc`
  - validates reduced RPC contract, start gating, and shared-attribute behavior
- `test/app/robot_fsm_test.cc`
  - validates self-check and pass-start semantics
- `test/app/robot_supervisor_test.cc`
  - validates admission, battery gate, and pending-to-active promotion
```

- [ ] **Step 5: Commit the startup/start-admission audit**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: audit startup and start admission flows"
```

## Task 3: Audit Main Task Execution, Stop/Return, Fault, and Charging/Battery Behavior

**Files:**
- Modify: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
- Reference: `include/pv_cleaning_robot/app/robot_fsm.h`, `pv_cleaning_robot/app/robot_fsm.cc`, `pv_cleaning_robot/service/motion_service.cc`, `pv_cleaning_robot/app/robot_supervisor.cc`, `pv_cleaning_robot/service/fault_service.cc`, `pv_cleaning_robot/app/fault_handler.cc`, `pv_cleaning_robot/device/bms.cc`, `pv_cleaning_robot/middleware/ota_manager.cc`
- Test: `test/app/robot_fsm_test.cc`, `test/app/robot_supervisor_test.cc`, `test/integration/task_chain_test.cc`, `test/integration/system_integration_test.cc`, `test/integration/hardware/clean_cycle_hw_test.cc`, `test/integration/hardware/system_hw_test.cc`

- [ ] **Step 1: Collect evidence for cleaning, stop, return, reset/reboot, fault, and charging/battery-gate flows**

Run:

```bash
sed -n '1,260p' include/pv_cleaning_robot/app/robot_fsm.h
sed -n '1,360p' pv_cleaning_robot/app/robot_fsm.cc
sed -n '1,260p' pv_cleaning_robot/app/robot_supervisor.cc
sed -n '1,260p' pv_cleaning_robot/service/motion_service.cc
sed -n '1,220p' pv_cleaning_robot/service/fault_service.cc
sed -n '1,220p' pv_cleaning_robot/app/fault_handler.cc
sed -n '1,260p' pv_cleaning_robot/device/bms.cc
sed -n '1,200p' pv_cleaning_robot/middleware/ota_manager.cc
```

Expected: enough context to reconstruct main cleaning progression, reduced stop/return/reset contract, fault behavior, and threshold-driven charging/battery gating.

- [ ] **Step 2: Fill in sections 3.4 through 3.8**

```markdown
### 3.4 Cleaning Round-Trip Main Flow

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `RobotFsm`
- `MotionService`
- `main.cc`

### 3.5 Stop and Task-Level Return

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

### 3.6 Reset and Reboot Behavior

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

### 3.7 Fault Reporting and Safe Halt

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

### 3.8 Charging and Battery Gate Behavior

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...
```

- [ ] **Step 3: Cross-check tests that encode these runtime branches and old complexity**

Run:

```bash
rg -n "Pause|Resume|ManualReturn|Terminate|Fault|LowBattery|Charging|BMS|reset|reboot|EvFarEndLimitSettled|EvParkingSideLimitSettled" \
  test/app/robot_fsm_test.cc \
  test/app/robot_supervisor_test.cc \
  test/integration/task_chain_test.cc \
  test/integration/system_integration_test.cc \
  test/integration/hardware/clean_cycle_hw_test.cc \
  test/integration/hardware/system_hw_test.cc
```

Expected: a concrete map of test cases that must follow any later runtime fix.

- [ ] **Step 4: Extend section 6 with task-branch sync targets**

```markdown
### Main Task / Branch Flows

- `test/app/robot_fsm_test.cc`
  - canonical unit coverage for task progression and legacy branch-state transitions to be simplified
- `test/app/robot_supervisor_test.cc`
  - admission, stop/return, and battery-gate behavior
- `test/integration/task_chain_test.cc`
  - multi-component task-chain behavior
- `test/integration/system_integration_test.cc`
  - main-like integration coverage with mock hardware
- `test/integration/hardware/clean_cycle_hw_test.cc`
  - hardware-oriented coverage for charging / low-battery legacy behavior that may need retirement or rewrite
- `test/integration/hardware/system_hw_test.cc`
  - hardware-system coverage for reset/reboot and command-flow contract changes
```

- [ ] **Step 5: Commit the branch-flow audit**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: audit task control and fault flows"
```

## Task 4: Audit Cloud Contract, Status Projection, Cache, Thread, and Support Layers

**Files:**
- Modify: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
- Reference: `include/pv_cleaning_robot/service/cloud_service.h`, `pv_cleaning_robot/service/cloud_service.cc`, `include/pv_cleaning_robot/service/thingsboard_control_plane.h`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`, `pv_cleaning_robot/service/thingsboard_config_manager.cc`, `include/pv_cleaning_robot/middleware/mqtt_transport.h`, `pv_cleaning_robot/middleware/mqtt_transport.cc`, `include/pv_cleaning_robot/middleware/network_manager.h`, `pv_cleaning_robot/middleware/network_manager.cc`, `pv_cleaning_robot/middleware/data_cache.cc`, `pv_cleaning_robot/service/health_service.cc`, `include/pv_cleaning_robot/app/watchdog_mgr.h`, `pv_cleaning_robot/app/watchdog_mgr.cc`
- Test: `test/service/cloud_service_test.cc`, `test/service/thingsboard_control_plane_test.cc`, `test/middleware/mqtt_transport_test.cc`, `test/middleware/network_manager_test.cc`

- [ ] **Step 1: Collect evidence for cloud/reporting/runtime-support behavior**

Run:

```bash
sed -n '1,360p' pv_cleaning_robot/service/cloud_service.cc
sed -n '1,340p' pv_cleaning_robot/service/thingsboard_control_plane.cc
sed -n '1,520p' pv_cleaning_robot/service/thingsboard_config_manager.cc
sed -n '1,360p' pv_cleaning_robot/middleware/mqtt_transport.cc
sed -n '1,220p' pv_cleaning_robot/middleware/network_manager.cc
sed -n '1,320p' pv_cleaning_robot/middleware/data_cache.cc
sed -n '1,220p' pv_cleaning_robot/service/health_service.cc
sed -n '1,220p' pv_cleaning_robot/app/watchdog_mgr.cc
```

Expected: enough evidence to describe cloud state projection, cache boundaries, reconnect behavior, and thread/event model.

- [ ] **Step 2: Fill in sections 3.9 and 3.10**

```markdown
### 3.9 Cloud Status Projection and Command Events

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `CloudService`
- `ThingsBoardControlPlane`
- `ThingsBoardConfigManager`

### 3.10 Thread, Event, Cache, and Runtime Support Layers

**Intended behavior**
- ...

**Actual behavior in current code**
- ...

**Deterministic defects**
- ...

**Semantic / responsibility risks**
- ...

**Involved modules**
- `MqttTransport`
- `NetworkManager`
- `DataCache`
- `HealthService`
- `WatchdogMgr`
```

- [ ] **Step 3: Record cloud and support-layer validation targets**

Run:

```bash
rg -n "shared attributes|RPC|telemetry|request_id|reject|reconnect|cache|watchdog|snapshot|start|stop|return|reset|terminate|pause|resume" \
  test/service/cloud_service_test.cc \
  test/service/thingsboard_control_plane_test.cc \
  test/middleware/mqtt_transport_test.cc \
  test/middleware/network_manager_test.cc
```

Expected: a concrete list of tests that validate current cloud/runtime-support behavior.

- [ ] **Step 4: Extend section 6 with cloud/support sync targets**

```markdown
### Cloud / Runtime Support Layers

- `test/service/cloud_service_test.cc`
  - shared attributes, reduced RPC parsing target, and explicit reject behavior
- `test/service/thingsboard_control_plane_test.cc`
  - control-plane mapping and telemetry/event expectations against the simplified command contract
- `test/middleware/mqtt_transport_test.cc`
  - topic routing and MQTT transport assumptions
- `test/middleware/network_manager_test.cc`
  - multi-transport routing assumptions
```

- [ ] **Step 5: Commit the cloud/runtime-support audit**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: audit cloud and runtime support flows"
```

## Task 5: Extract Final Defect Lists and Simplification Inputs

**Files:**
- Modify: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md`
- Reference: `docs/superpowers/specs/2026-05-08-business-flow-audit-design.md`
- Test: none

- [ ] **Step 1: Summarize deterministic defects into section 4**

```markdown
## 4. Must-Fix Deterministic Defects

1. [Short title]
   - Runtime evidence: ...
   - Impact: ...
   - Involved modules: ...

2. [Short title]
   - Runtime evidence: ...
   - Impact: ...
   - Involved modules: ...
```

- [ ] **Step 2: Summarize semantic risks into section 5**

```markdown
## 5. High-Priority Semantic / Responsibility Risks

1. [Short title]
   - Why it is risky: ...
   - How it creates future defects: ...
   - Involved modules: ...

2. [Short title]
   - Why it is risky: ...
   - How it creates future defects: ...
   - Involved modules: ...
```

- [ ] **Step 3: Write simplification inputs into section 7**

```markdown
## 7. Simplification Inputs for the Next Phase

1. Position-truth handling that must be centralized
   - ...

2. Configuration ownership that must be centralized
   - ...

3. `main.cc` responsibilities that should be thinned
   - ...

4. Admission logic and command contract that should collapse into one authority
   - ...

5. FSM state names and legacy task branches that should be removed or renamed
   - ...

6. Test areas that can only be simplified after runtime truth is fixed
   - ...
```

- [ ] **Step 4: Validate the findings document against the approved spec**

Run:

```bash
sed -n '1,260p' docs/superpowers/specs/2026-05-08-business-flow-audit-design.md
sed -n '1,320p' docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
```

Expected: every scoped flow from the spec appears in the findings doc, the user-confirmed target model is reflected in the findings, and sections 4, 5, 6, and 7 are all populated.

- [ ] **Step 5: Commit the final findings document**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: finalize business flow audit findings"
```

## Task 6: Final Verification and Handoff

**Files:**
- Modify: `docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md` if needed
- Test: none

- [ ] **Step 1: Run placeholder and scope checks on the findings doc**

Run:

```bash
rg -n "TBD|TODO|FIXME|XXX|placeholder" docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
```

Expected: no matches.

- [ ] **Step 2: Re-skim the key runtime files to ensure no audit section was skipped**

Run:

```bash
rg -n "int main\\(|EvScheduleStart|EvFarEndLimitSettled|EvParkingSideLimitSettled|EvLowBattery|EvManualReturn|EvTerminateTask|subscribe_shared_attributes|request_shared_attributes_snapshot|publish_telemetry|flush_cache|watchdog|reboot|charging|battery" \
  pv_cleaning_robot/main.cc \
  include/pv_cleaning_robot/app/robot_fsm.h \
  pv_cleaning_robot/service/cloud_service.cc \
  pv_cleaning_robot/service/thingsboard_control_plane.cc \
  pv_cleaning_robot/device/bms.cc \
  pv_cleaning_robot/middleware/data_cache.cc \
  pv_cleaning_robot/middleware/ota_manager.cc \
  pv_cleaning_robot/app/watchdog_mgr.cc
```

Expected: all major runtime entry points are visibly covered by at least one findings section.

- [ ] **Step 3: Add a short audit conclusion**

```markdown
## 8. Audit Conclusion

- The runtime truth is now documented by flow, not by scattered assumptions.
- Deterministic defects are separated from semantic design debt.
- The next simplification phase can now focus on deletion and boundary tightening instead of rediscovering runtime truth.
```

- [ ] **Step 4: Verify git diff is limited to the findings doc**

Run:

```bash
git diff -- docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git status --short
```

Expected: the audit execution only changes the findings doc, aside from unrelated pre-existing workspace changes.

- [ ] **Step 5: Commit the handoff conclusion**

```bash
git add docs/superpowers/specs/2026-05-08-business-flow-audit-findings.md
git commit -m "docs: conclude business flow audit"
```
