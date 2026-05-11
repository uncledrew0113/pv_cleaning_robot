# RPC Start Override Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保持普通业务 `start` 的停机位语义不变，同时让云端 RPC `start` 可以从非停机位启动完整的 `CleanFwd -> CleanReturn` 清扫任务。

**Architecture:** 不改现有 `EvScheduleStart` 的业务定义，新增一条只供 RPC 使用的特权启动路径。`RobotSupervisor` 和 `RobotFsm` 分别增加 RPC 专用入口；`ThingsBoardControlPlane` 的 RPC `start` 切到新入口；`stop` 和 `return` 语义保持不变。

**Tech Stack:** C++17、Boost.SML、现有 app/service 层、Catch2 单元测试与 ThingsBoard 集成测试

---

## File Map

**Modify:**
- `include/pv_cleaning_robot/app/robot_supervisor.h`
- `pv_cleaning_robot/app/robot_supervisor.cc`
- `include/pv_cleaning_robot/app/robot_fsm.h`
- `pv_cleaning_robot/app/robot_fsm.cc`
- `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- `test/app/robot_supervisor_test.cc`
- `test/app/robot_fsm_test.cc`
- `test/service/thingsboard_control_plane_test.cc`

**Build/Test verification:**
- `cmake --build build --target unit_tests -j4`

## Task 1: 为状态机增加 RPC 专用启动语义

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h`
- Modify: `pv_cleaning_robot/app/robot_fsm.cc`
- Test: `test/app/robot_fsm_test.cc`

- [ ] **Step 1: 先写状态机测试，定义 RPC 特权启动行为**

在 `test/app/robot_fsm_test.cc` 增加一个新的公开事件测试，覆盖：

```cpp
TEST_CASE("FSM RPC override start can begin cleaning away from parking side", "[app][fsm]") {
    FsmFixture f;

    EvRpcStartTask evt;
    evt.passes = 1.0f;
    f.fsm.dispatch(evt);
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fsm.dispatch(EvParkingSideLimitSettled{true});
    REQUIRE(f.fsm.current_state() == "Charging");
    REQUIRE(f.fsm.completed_passes() == 1);
}
```

并保留现有测试：

- `EvScheduleStart` 非停机位仍回 `Idle`

- [ ] **Step 2: 运行单测，确认新语义尚未实现**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `test/app/robot_fsm_test.cc` 因 `EvRpcStartTask` 未定义或未处理而失败

- [ ] **Step 3: 在状态机头文件增加 RPC 专用事件**

在 `include/pv_cleaning_robot/app/robot_fsm.h` 增加：

```cpp
struct EvRpcStartTask {
    float passes{1.0f};
};
```

并在注释中明确：

- 该事件仅用于云端 RPC 特权启动
- 不要求当前位于停机位

- [ ] **Step 4: 把 RPC 专用事件接入 SML 转换表**

在 `RobotFsm::Fsm` 转换表里增加：

```cpp
state<StateIdle>     + event<EvRpcStartTask> = state<StateCleanFwd>,
state<StateStopped>  + event<EvRpcStartTask> = state<StateCleanFwd>,
state<StateCharging> + event<EvRpcStartTask> = state<StateCleanFwd>,
```

不要修改现有 `EvScheduleStart` 相关转换。

- [ ] **Step 5: 在 `robot_fsm.cc` 实现 `dispatch<EvRpcStartTask>`**

最小实现目标：

```cpp
template <>
void RobotFsm::dispatch<EvRpcStartTask>(EvRpcStartTask e) {
    std::function<void()> action;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (!sm_->process_event(e)) {
            spdlog::warn("[FSM] 忽略 EvRpcStartTask (state={})", state_name_);
            return;
        }

        const float rounded_passes = std::round(e.passes);
        const bool passes_is_integer =
            std::fabs(e.passes - rounded_passes) < 1e-4f && rounded_passes >= 1.0f;
        if (!passes_is_integer) {
            state_name_ = "Idle";
            spdlog::error("[FSM] → Idle（RPC start 失败：仅支持整数趟，拒绝 passes={:.1f}）", e.passes);
            return;
        }

        target_passes_ = static_cast<int>(rounded_passes);
        completed_passes_ = 0;
        going_forward_ = true;
        state_name_ = "CleanFwd";
        spdlog::info("[FSM] → CleanFwd（RPC 特权启动，从当前位置开始清扫）");
        action = [this]() { motion_->start_cleaning(); };
    }
    if (action) action();
}
```

要求：

- 只放宽首段进入 `CleanFwd`
- 后续 `FarEnd -> CleanReturn -> ParkingSide` 继续复用现有逻辑

- [ ] **Step 6: 重新运行状态机相关编译验证**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `robot_fsm_test` 相关用例通过编译并可执行

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h pv_cleaning_robot/app/robot_fsm.cc test/app/robot_fsm_test.cc
git commit -m "feat: add rpc-only force start state transition"
```

## Task 2: 为 Supervisor 增加 RPC 特权启动入口

**Files:**
- Modify: `include/pv_cleaning_robot/app/robot_supervisor.h`
- Modify: `pv_cleaning_robot/app/robot_supervisor.cc`
- Test: `test/app/robot_supervisor_test.cc`

- [ ] **Step 1: 先写 Supervisor 测试，区分普通 start 和 RPC start**

在 `test/app/robot_supervisor_test.cc` 增加：

```cpp
TEST_CASE("RobotSupervisor rpc start can begin task away from parking side", "[app][robot_supervisor]") {
    SupervisorFixture f;
    REQUIRE(f.supervisor->start_task_from_current_position(true, 60.0f));
    REQUIRE(f.fsm->current_state() == "CleanFwd");
}
```

并保留现有断言：

```cpp
REQUIRE_FALSE(f.supervisor->start_task(false, true, 60.0f));
```

这样明确：

- 普通 `start_task` 仍要求停机位
- RPC 新入口不要求 `at_parking_side`

- [ ] **Step 2: 运行编译，确认新接口尚不存在**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `start_task_from_current_position` 未声明导致失败

- [ ] **Step 3: 在头文件声明 RPC 专用启动接口**

在 `include/pv_cleaning_robot/app/robot_supervisor.h` 增加：

```cpp
bool start_task_from_current_position(bool position_valid, float battery_soc);
```

要求：

- 不修改现有 `start_task(...)` 签名

- [ ] **Step 4: 在实现文件补最小逻辑**

在 `pv_cleaning_robot/app/robot_supervisor.cc` 增加新方法：

```cpp
bool RobotSupervisor::start_task_from_current_position(bool position_valid, float battery_soc) {
    const auto state = fsm_->current_state();
    if (!is_new_task_start_state(state) || !position_valid) {
        return false;
    }
    const auto runtime_cfg = start_runtime_config();
    if (battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)) {
        return false;
    }
    if (tb_cfg_->has_pending_config() && !tb_cfg_->promote_pending_to_active()) {
        return false;
    }
    fsm_->dispatch(EvRpcStartTask{static_cast<float>(tb_cfg_->active_config().passes)});
    return fsm_->current_state() == "CleanFwd" || fsm_->current_state() == "CleanReturn";
}
```

约束：

- 普通 `start_task(...)` 保持原样
- RPC 专用入口仍要求 `position_valid`
- 只移除“必须在停机位”的限制

- [ ] **Step 5: 运行 Supervisor 相关编译验证**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `robot_supervisor_test` 通过编译

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/app/robot_supervisor.h pv_cleaning_robot/app/robot_supervisor.cc test/app/robot_supervisor_test.cc
git commit -m "feat: add rpc override start in supervisor"
```

## Task 3: 切换 ThingsBoard RPC start 到特权入口

**Files:**
- Modify: `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- Test: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: 先改测试，固定新的 RPC 语义**

在 `test/service/thingsboard_control_plane_test.cc` 调整 `start RPC` 相关用例：

- 保留“`position_valid=false` 时拒绝”为 `robot_position_invalid`
- 把“`at_start_parking_side=false` 时拒绝”改成“仍可接受，只要 `position_valid=true`”

新增断言示例：

```cpp
TEST_CASE("ThingsBoardControlPlane start RPC can launch task away from parking side",
          "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers(true, false, false, 80.0f);

    f.mqtt->emit_rpc("43", R"({"method":"start","params":{}})");

    CHECK(f.fsm->current_state() == "CleanFwd");
    const auto response = f.last_published_json("rpc/response/43");
    CHECK(response["accepted"].GetBool() == true);
}
```

- [ ] **Step 2: 运行编译，确认当前控制平面仍按旧逻辑拒绝**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `thingsboard_control_plane_test` 中新用例失败

- [ ] **Step 3: 修改 RPC start 处理逻辑**

在 `pv_cleaning_robot/service/thingsboard_control_plane.cc` 中，把 RPC `start` 从：

```cpp
supervisor_->start_task(at_parking_side, position_valid, battery_soc)
```

改成：

```cpp
supervisor_->start_task_from_current_position(position_valid, battery_soc)
```

并同步调整拒绝原因：

- 仍保留 `robot_position_invalid`
- 去掉 RPC `start` 对 `robot_not_at_parking_side` 的拒绝
- 仍保留 `battery_below_start_threshold`
- 仍保留 `start_not_allowed_in_current_state`

日志也要明确这是 RPC 特权启动，例如：

```cpp
spdlog::info("[ThingsBoardControlPlane] RPC start received: state='{}' position_valid={} at_parking_side={} battery_soc={:.1f}",
             ...);
```

但不要再把 `at_parking_side=false` 当作直接拒绝条件。

- [ ] **Step 4: 重新运行控制平面相关编译验证**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `thingsboard_control_plane_test` 通过编译

- [ ] **Step 5: Commit**

```bash
git add pv_cleaning_robot/service/thingsboard_control_plane.cc test/service/thingsboard_control_plane_test.cc
git commit -m "feat: let rpc start override parking-side requirement"
```

## Task 4: 全量回归关键语义

**Files:**
- Verify only: `test/app/robot_fsm_test.cc`
- Verify only: `test/app/robot_supervisor_test.cc`
- Verify only: `test/service/thingsboard_control_plane_test.cc`

- [ ] **Step 1: 全量编译单测目标**

Run:

```bash
cmake --build build --target unit_tests -j4
```

Expected:

- `unit_tests` 目标构建成功

- [ ] **Step 2: 重点人工检查 3 条语义**

检查测试覆盖是否仍然证明：

- 普通业务 `start` 非停机位拒绝
- RPC `start` 非停机位允许，并走完整 `CleanFwd -> CleanReturn`
- RPC `return` 仍然只是 `Returning`，后续返程不带刷

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-05-11-rpc-start-override-design.md docs/superpowers/plans/2026-05-11-rpc-start-override.md
git commit -m "docs: add rpc start override plan"
```

## Self-Review

- Spec coverage:
  - 普通业务 `start` 保持不变：Task 2 / Task 4
  - RPC `start` 非停机位可启动：Task 1 / Task 2 / Task 3
  - RPC `start` 返程继续带刷：Task 1 复用现有 `CleanReturn` 路径验证
  - RPC `return` 保持不带刷：Task 4 人工回归确认
  - RPC `stop` 保持停机：现有测试保留，Task 4 回归
- Placeholder scan:
  - 无 `TBD` / `TODO`
  - 每个任务都给出具体文件和命令
- Type consistency:
  - 新增事件统一命名为 `EvRpcStartTask`
  - 新增 supervisor 入口统一命名为 `start_task_from_current_position(...)`
