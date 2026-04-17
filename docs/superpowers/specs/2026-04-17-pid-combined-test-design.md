# 设计文档：带 PID 控制的多趟联合测试

**日期**: 2026-04-17  
**作者**: Copilot  
**状态**: 待实施

---

## 一、背景与目标

### 问题

当前测试矩阵存在空缺：

| 测试 | 多趟 | PID | 健康数据 | PID 指标 |
|------|------|-----|---------|---------|
| `[hw_system][combined]` | ✅ | ❌ | ✅ | ❌ |
| `[hw_cycle][one_pass_with_pid]` | ❌ | ✅ | ❌ | 仅最大漂移 |
| **新增 `[hw_system][pid_combined]`** | ✅ | ✅ | ✅ | ✅ |

还存在一个遗留 bug：`[hw_system][combined]` 将 `at_home = true` 硬编码，未读取真实传感器状态，新测试应修复此问题。

### 目标

1. 验证 PID 航向控制在多趟（N 趟）连续运行中的稳定性与效果
2. 记录每 tick 的 yaw / yaw_err 到独立的 `pid_metrics.jsonl` 供离线分析
3. 将 PID 参数（kp/ki/kd/deadband 等）纳入 `hw_test_config.json`，方便调参
4. 在 `HeadingPidController` 中引入可配置死区（Deadband），减少微幅频繁修正

---

## 二、架构与变更范围

### 2.1 涉及文件

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `include/.../service/heading_pid_controller.h` | 修改 | `Params` 增加 `deadband_deg` 字段 |
| `pv_cleaning_robot/service/heading_pid_controller.cc` | 修改 | `compute()` 加死区逻辑 |
| `test/integration/hardware/hw_test_config.json` | 修改 | 增加 `pid` 配置节 |
| `test/integration/hardware/hw_config.h` | 修改 | `HwParams` 增加 pid 子结构，加载逻辑更新 |
| `test/integration/hardware/system_hw_test.cc` | 修改 | 新增 `[hw_system][pid_combined]` 测试 |
| `doc/API_REFERENCE.md` | 修改 | 同步 Params 字段、测试标签说明 |
| `test/service/heading_pid_test.cc` | 修改 | 增加死区单元测试 |

### 2.2 不涉及文件

- `WalkMotorGroup`：不需要新增暴露接口；yaw_err 在测试层计算
- `MotionService`：不变
- 其余测试文件：不变

---

## 三、功能设计

### 3.1 HeadingPidController 死区

在 `Params` 结构体中增加：
```cpp
float deadband_deg{0.0f};  ///< 死区（°），|err| < deadband_deg 时输出 0，默认关闭
```

`compute()` 中，在计算三项之前加入死区判断：
```cpp
if (std::abs(err) < params_.deadband_deg)
    return 0.0f;
```

**设计约束：**
- 死区仅抑制输出，积分照常累积（防止持续偏差被死区掩盖时积分不更新）
- `deadband_deg = 0.0f` 时行为与现有代码完全兼容（无死区）

### 3.2 配置文件扩展

`hw_test_config.json` 新增 `pid` 节：
```json
"pid": {
  "kp":            0.5,
  "ki":            0.05,
  "kd":            0.1,
  "max_output":    30.0,
  "integral_limit": 20.0,
  "deadband_deg":  0.0
}
```

`HwParams` 新增：
```cpp
struct PidParams {
    float kp{0.5f};
    float ki{0.05f};
    float kd{0.1f};
    float max_output{30.0f};
    float integral_limit{20.0f};
    float deadband_deg{0.0f};
} pid;
```

`load_hw_test_config()` 中按字段逐一读取，任意字段缺失时使用上述默认值。

`FullSystemFixture` 在构造 `MotionService::Config` 时，将 `HwParams::pid` 映射到 `HeadingPidController::Params`。

### 3.3 [hw_system][pid_combined] 测试

**测试流程：**
1. 构造 `FullSystemFixture(true /* pid_on */)`，从配置加载 PID 参数并通过 `set_heading_pid_params()` 设置
2. 读取真实传感器状态（修复 at_home 硬编码问题）：
   ```cpp
   const bool at_home  = !f.rear_sw->read_current_level();
   const bool at_front = !f.front_sw->read_current_level();
   f.fsm->dispatch(EvScheduleStart{at_home, at_front, f.p.combined_passes});
   ```
3. 在出发前记录 `target_yaw = imu->get_latest().yaw_deg`
4. `poll_once()` 增强：每次同时写一条 pid_metrics 记录
5. 逐段等待限位（与 `[combined]` 完全相同的循环结构）
6. 每段完成时追加一条 `segment_summary`
7. 结束时追加 `final_summary`，断言不产生 assert failure

**pid_metrics.jsonl 记录格式：**

每 tick 一条（约 500ms/条）：
```json
{"ts_ms": 1200, "seg": 2, "state": "CleanFwd", "yaw": 3.45, "target_yaw": 0.0, "yaw_err": -3.45}
```

每段结束一条摘要：
```json
{"type": "segment_summary", "seg": 2, "direction": "fwd", "max_drift_deg": 4.2, "duration_s": 38.5}
```

最终一条：
```json
{"type": "final_summary", "total_segs": 10, "max_drift_all_deg": 5.8, "kp": 0.5, "ki": 0.05, "kd": 0.1, "deadband_deg": 0.0}
```

**断言策略：**
- `REQUIRE` 级别：每段限位必须在超时内触发（与 `[combined]` 一致）
- `CHECK` 级别（不强制失败，仅警告）：`max_drift_all_deg < p.pid_max_drift_deg`（新配置项，默认 15°）
- 不强制要求 max_drift < 10°（多趟累计偏差可能比单趟大）

**数据持久化：**
- `pid_metrics.jsonl` 路径从配置读取（新增 `pid_jsonl_path` 字段，默认 `/tmp/hw_pid_test_metrics.jsonl`）
- 测试结束**不删除**文件（与 `[combined]` 保持一致的数据保留策略）

---

## 四、单元测试（heading_pid_test.cc）

新增两个测试用例：

1. **死区关闭时行为不变**：`deadband_deg = 0.0f`，`|err| = 5°` → `correction ≠ 0`
2. **死区内返回零**：`deadband_deg = 2.0f`，`|err| = 1.5°` → `correction == 0.0f`
3. **恰好在死区边界**：`|err| = 2.0f == deadband_deg` → `correction == 0.0f`（严格小于，边界为零）
4. **超出死区正常计算**：`deadband_deg = 2.0f`，`|err| = 5°` → `correction = kp * 5.0f`

---

## 五、实施顺序

1. `HeadingPidController` 死区功能 + 单元测试（独立，可先行）
2. `HwParams` + `hw_test_config.json` 扩展 pid 节
3. `FullSystemFixture` 加载并应用 pid 参数
4. 新增 `[hw_system][pid_combined]` 测试
5. 更新 `API_REFERENCE.md`
6. 完整构建验证

---

## 六、排除在外的内容

- **PID 方向验证**：yaw 符号约定需硬件实测确认，不在本次范围内
- **无 PID 对比运行**：已有 `[combined]` 可提供对比基准，不需要在同一测试内运行
- **PID 自动调参**：超出范围
- **距离传感器数据写入 pid_metrics**：保持在 health_jsonl 中，不合并

---

## 七、成功标准

- `cmake --build` 无错误无警告
- `heading_pid_test` 新增用例全部通过
- `[hw_system][pid_combined]` 在真实硬件上完成完整 N 趟无超时
- `pid_metrics.jsonl` 和 `health_jsonl` 均正常写入且文件保留

---

*本文档由 Copilot 生成，请在实施前审阅确认。*
