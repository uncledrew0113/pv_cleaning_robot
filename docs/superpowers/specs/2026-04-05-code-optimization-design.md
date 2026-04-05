# PV 清扫机器人 — 代码优化设计方案

**日期**：2026-04-05  
**目标平台**：RK3576 · aarch64-linux-gnu · PREEMPT_RT · C++17  
**实施策略**：方案 C（按优先级批次），共 4 批、19 项优化  

---

## 一、背景与范围

对 `pv_cleaning_robot` 固件的全面代码审查发现了横跨架构、安全、稳定性、可维护性 4 个维度的 19 项问题。本设计文档覆盖全部问题的修复方案，按 P0→P1→P2→P3 分 4 批独立实施，每批可独立测试和合并。

**修复原则**：
- 不改变现有公有接口签名（3.3 OTA 接口变更除外，已在设计中明确）
- 每批次完成后须通过 `unit_tests` 全量回归
- P2 的冗余删除在 P0/P1 修复后进行，避免干扰安全路径

---

## 二、Batch-1：P0 安全关键修复（3项）

### 2.1 PiMutex EOWNERDEAD 死锁修复

**文件**：`include/pv_cleaning_robot/hal/pi_mutex.h`

**根因**：当 `EOWNERDEAD` 发生时，`pthread_mutex_consistent()` 将锁标记为一致，内核已将锁所有权转交当前线程；但随后 `throw` 导致 `std::lock_guard` 构造失败，析构器不触发 `unlock()`，锁永久被当前线程持有，其他等待线程永久阻塞。

**修复**：`lock()` 在 `EOWNERDEAD` 路径上，先调用 `pthread_mutex_consistent()`，然后 **调用 `pthread_mutex_unlock()`** 释放已持有的锁，再 `throw`。确保抛出异常后锁已被释放，不会死锁。

```cpp
void lock() {
    int rc = pthread_mutex_lock(&mutex_);
    if (rc == 0) return;
    if (rc == EOWNERDEAD) {
        pthread_mutex_consistent(&mutex_);
        pthread_mutex_unlock(&mutex_);  // 先释放，再抛
        throw std::runtime_error(
            "[PiMutex] lock acquired but previous owner died. State inconsistent!");
    }
    throw std::runtime_error(
        std::string("[PiMutex] pthread_mutex_lock failed: ") + std::strerror(rc));
}
```

**影响文件**：`include/pv_cleaning_robot/hal/pi_mutex.h`

---

### 2.2 WatchdogMgr 持锁回调死锁修复

**文件**：`pv_cleaning_robot/app/watchdog_mgr.cc`

**根因**：`monitor_loop` 在持有 `tickets_mtx_` 锁期间调用 `on_timeout_` 回调。若回调内触发 `heartbeat()` 或 `register_thread()`（均需 `tickets_mtx_`），立即发生死锁。

**修复**：在锁内仅收集到期票据名称，出锁后再触发回调：

```cpp
void WatchdogMgr::monitor_loop() {
    // ... RT 设置 ...
    while (running_.load()) {
        feed_hw_watchdog();

        std::vector<std::string> expired_names;
        {
            std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
            auto now = std::chrono::steady_clock::now();
            for (auto& [id, ticket] : tickets_) {
                if (ticket.expired) continue;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - ticket.last_beat).count();
                if (elapsed > ticket.timeout_ms) {
                    ticket.expired = true;
                    expired_names.push_back(ticket.name);
                }
            }
        }
        // 锁外回调，安全
        for (auto& name : expired_names) {
            if (on_timeout_) on_timeout_(name);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
```

**影响文件**：`pv_cleaning_robot/app/watchdog_mgr.cc`

---

### 2.3 RPC 输入白名单验证

**文件**：`pv_cleaning_robot/service/cloud_service.cc`, `include/.../service/cloud_service.h`

**根因**：`on_rpc_message()` 对 `method` 和 `params` 无任何校验，云端可发送任意方法名调用未注册 handler，或发送超大 params 耗尽栈空间。

**修复**：
1. `CloudService` 构造参数增加可选白名单 `std::unordered_set<std::string> allowed_methods`
2. `on_rpc_message()` 增加双重防御：method 白名单 + params 大小限制（4096 bytes）

```cpp
// cloud_service.h 新增
static constexpr size_t kMaxRpcParamsBytes = 4096;

// cloud_service.cc on_rpc_message() 增加
if (!allowed_methods_.empty() &&
    allowed_methods_.find(method) == allowed_methods_.end()) {
    spdlog::warn("[CloudService] Rejected unknown RPC method: {}", method);
    return;
}
if (params.size() > kMaxRpcParamsBytes) {
    spdlog::warn("[CloudService] RPC params too large: {} bytes", params.size());
    return;
}
```

**默认白名单**（`main.cc` 中注入）：`{"setCleanMode", "startCleaning", "stopCleaning", "resetFault", "getStatus", "triggerOta"}`

**影响文件**：`include/.../service/cloud_service.h`, `pv_cleaning_robot/service/cloud_service.cc`

---

## 三、Batch-2：P1 稳定性 / 实时可靠性修复（6项）

### 3.1 SafetyMonitor 双限位并行防抖

**文件**：`pv_cleaning_robot/middleware/safety_monitor.cc`, `include/.../middleware/safety_monitor.h`

**根因**：`monitor_loop` 中两个 `nanosleep(180ms)` 串行执行，前后端同时触发时后端延迟 ~360ms，超出安全预期。

**修复**：用时间戳原子变量替代 `pending_front_/pending_rear_` bool，`monitor_loop` 非阻塞轮询：

```cpp
// header 变更
std::atomic<uint64_t> pending_front_ts_{0};  // 触发时刻 ms，0=未触发
std::atomic<uint64_t> pending_rear_ts_{0};

// on_limit_trigger() 记录时间戳
pending_front_ts_.store(now_ms(), std::memory_order_release);

// monitor_loop() 非阻塞检查（5ms 轮询间隔）
auto check_settled = [&](std::atomic<uint64_t>& ts, device::LimitSide side) {
    uint64_t t = ts.load(std::memory_order_acquire);
    if (t == 0) return;
    if (now_ms() - t >= 180u) {
        ts.store(0, std::memory_order_release);
        event_bus_.publish(LimitSettledEvent{side});
        if (side == device::LimitSide::REAR) reset_estop();
    }
};
// 两者并列调用，不互相阻塞
check_settled(pending_front_ts_, device::LimitSide::FRONT);
check_settled(pending_rear_ts_,  device::LimitSide::REAR);
```

**效果**：双端同时触发，均在 ~185ms（180ms + 5ms 轮询误差）内响应。

**影响文件**：`include/.../middleware/safety_monitor.h`, `pv_cleaning_robot/middleware/safety_monitor.cc`

---

### 3.2 FSM 状态查询使用 SML is<>()

**文件**：`pv_cleaning_robot/app/robot_fsm.cc`

**根因**：`dispatch<EvRearLimitSettled>` 使用 `if (state_name_ == "CleanReturn")` 字符串比较，拼写错误不会有编译期报错，且与 SML 内部状态可能漂移。

**修复**：使用 SML 类型安全查询：

```cpp
// 替换所有 state_name_ 业务判断
if (sm_->is(sml::state<StateCleanReturn>)) { ... }
if (sm_->is(sml::state<StateReturning>))  { ... }
```

`state_name_` 保留，但**仅用于日志输出**，不参与任何业务逻辑判断。在每次 `process_event()` 后用 `sm_->visit_current_states()` 同步更新 `state_name_`（可选，降低手动维护风险）。

**影响文件**：`pv_cleaning_robot/app/robot_fsm.cc`

---

### 3.3 OTA 流式写入（接口变更）

**文件**：`include/.../middleware/ota_manager.h`, `pv_cleaning_robot/middleware/ota_manager.cc`

**根因**：`start(const std::string& firmware_data, ...)` 要求调用方将全量固件（可能数十MB）载入内存的字符串，直接影响 RT 内存压力。

**修复**：新增流式接口，保留原接口兼容：

```cpp
// 新增接口（推荐）
bool start_stream(std::istream& firmware_stream,
                  uint64_t expected_bytes,
                  const std::string& expected_sha256);

// 原接口保留，内部包装为 istringstream 调用 start_stream（向后兼容）
bool start(const std::string& firmware_data, const std::string& expected_sha256);
```

`start_stream` 内部以 4KB chunk 写入，使用 `EVP_MD_CTX` 流式计算摘要，峰值内存占用 ~8KB。

**CloudService** 调用方由传入完整数据改为传入 `std::istringstream`（或直接走 HTTP 下载流）。

**影响文件**：`include/.../middleware/ota_manager.h`, `pv_cleaning_robot/middleware/ota_manager.cc`, `pv_cleaning_robot/service/cloud_service.cc`

---

### 3.4 OTA 校验升级为 SHA-256

**文件**：`pv_cleaning_robot/middleware/ota_manager.cc`

**根因**：`MD5()` 在 OpenSSL 3.x 已废弃，且 MD5 不满足固件完整性校验的安全要求。

**修复**：改用 OpenSSL EVP API（1.x/3.x 均兼容）：

```cpp
bool verify_sha256(std::istream& data, const std::string& expected_hex) {
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
    char buf[4096];
    while (data.read(buf, sizeof(buf)) || data.gcount() > 0)
        EVP_DigestUpdate(ctx, buf, static_cast<size_t>(data.gcount()));
    unsigned char digest[32];
    unsigned int len = 0;
    EVP_DigestFinal_ex(ctx, digest, &len);
    EVP_MD_CTX_free(ctx);
    // 比较 hex string ...
}
```

Cloud 端 OTA RPC payload 的 `expected_md5` 字段重命名为 `expected_sha256`。

---

### 3.5 HealthService 本地日志写失败告警

**文件**：`pv_cleaning_robot/service/health_service.cc`

**修复**：

```cpp
if (local_log_file_.is_open()) {
    local_log_file_ << payload << '\n';
    if (!local_log_file_.flush()) {
        spdlog::error("[HealthService] local log flush failed (disk full?), closing file");
        local_log_file_.close();  // 停止反复写失败，保护 I/O
        // 可选：通过 FaultService 上报 P3 故障
    }
}
```

---

### 3.6 OTA reboot 前安全状态检查

**文件**：`include/.../middleware/ota_manager.h`, `pv_cleaning_robot/middleware/ota_manager.cc`

**修复**：增加可注入的安全门卫回调：

```cpp
// ota_manager.h
using SafetyCheckFn = std::function<bool()>;  // true = 安全可重启
void set_safety_check(SafetyCheckFn fn);

// apply_and_reboot() 实现
bool OtaManager::apply_and_reboot() {
    if (safety_check_ && !safety_check_()) {
        spdlog::error("[OTA] Reboot blocked: robot not in safe state (Charging/Idle required)");
        set_state(State::FAILED, "blocked by safety check");
        return false;
    }
    // ... 原有逻辑
}
```

**main.cc** 注入安全检查：
```cpp
ota_manager->set_safety_check([&fsm]() {
    auto state = fsm->current_state();
    return state == "Idle" || state == "Charging";
});
```

---

## 四、Batch-3：P2 架构 / 可维护性改善（7项）

### 4.1 删除 WalkMotor 单轮冗余类

删除以下文件：
- `include/pv_cleaning_robot/device/walk_motor.h`
- `pv_cleaning_robot/device/walk_motor.cc`
- `test/device/walk_motor_test.cc`
- `test/walk_motor_test.cc`（遗留）

确认无外部调用方（`grep -r "WalkMotor[^G]"` 确认）后删除，并更新 `CMakeLists.txt`。

### 4.2 删除 DiagnosticsCollector 冗余类

删除：
- `include/pv_cleaning_robot/service/diagnostics_collector.h`
- `pv_cleaning_robot/service/diagnostics_collector.cc`

确认无调用方后删除，更新 `CMakeLists.txt`。

### 4.3 EventBus kMaxHandlers 注释修正

`event_bus.h` `publish()` 方法注释：将 "kMaxHandlers=16" 改为 "kMaxHandlers=32"（与代码一致）。

### 4.4 FSM state_name_ 降级为纯日志辅助

在 `robot_fsm.h` 的 `state_name_` 成员注释标注：

```cpp
/// 仅用于日志输出，不参与任何业务判断
/// 状态判断请使用 sm_->is(sml::state<StateXxx>)
std::string state_name_{"Init"};
```

### 4.5 清理遗留测试文件

删除 `test/` 根目录下以下遗留文件（功能已被 `test/device/` 和 `test/protocol/` 覆盖）：
- `test/bms_test.cc`
- `test/bms2_test.cc`
- `test/imu_test.cc`
- `test/walk_motor_test.cc`

更新 `test/CMakeLists.txt`。

### 4.6 HealthService mutable j_ 线程边界文档化

在 `health_service.h` 的 `j_` 成员注释：

```cpp
/// 预分配 JSON 键树，build_payload() 中复用以减少内存分配。
/// 线程约束：update()/build_payload() 必须由单一线程（ThreadExecutor）调用，
/// 不支持并发 build_payload()。若需多线程访问，须在调用方加锁。
mutable nlohmann::json j_;
```

### 4.7 DataCache std::mutex 调用约束文档化

在 `data_cache.h` 类注释增加：

```
/// 线程安全约束：内部使用 std::mutex（非 PiMutex）。
/// 禁止在 SCHED_FIFO RT 线程中调用本类的任何方法，
/// 否则可能引发优先级反转。本类仅供 SCHED_OTHER 线程（CloudService）使用。
```

---

## 五、Batch-4：P3 安全加固 / 工程质量（3项）

### 5.1 MQTT TLS 默认开启

`config/config.json` 变更：
```json
"mqtt": {
  "tls_enabled": true,
  "ca_cert_path": "/etc/pv_robot/certs/ca.crt"
}
```

`MqttTransport` 读取 `ca_cert_path`，TLS 握手失败时报 P2 故障（不直接下线，允许降级）。

开发环境可在本地 `config.json` 覆盖为 `false`（`.gitignore` 中本地覆盖文件）。

### 5.2 敏感配置支持环境变量覆盖

`ConfigService::get<T>(path, default)` 增加环境变量覆盖逻辑：
- 路径转换规则：`.` 替换为 `_`，全大写，加 `ROBOT_` 前缀
- 示例：`network.mqtt.broker_uri` → `ROBOT_NETWORK_MQTT_BROKER_URI`

优先级：环境变量 > config.json > 默认值

```cpp
template <typename T>
T ConfigService::get(const std::string& path, T default_val) {
    // 1. 尝试环境变量
    std::string env_key = to_env_key(path);  // "ROBOT_NETWORK_MQTT_BROKER_URI"
    if (const char* env_val = std::getenv(env_key.c_str())) {
        return parse_env_value<T>(env_val);
    }
    // 2. config.json
    // 3. default_val
}
```

### 5.3 CMakePresets 路径去硬编码

`CMakePresets.json` 改为从 `RK3576_SDK_PATH` 环境变量读取：

```json
"environment": {
  "PATH": "$env{RK3576_SDK_PATH}/sysroots/x86_64-linux/bin:$penv{PATH}",
  "SDK_QMAKE_SYSROOT": "$env{RK3576_SDK_PATH}/sysroots/armv8a-linux"
},
```

新增 `.env.example`：
```bash
export RK3576_SDK_PATH=/home/tronlong/RK3576
```

`README.md` 补充一节"构建前置条件"，说明需 `source .env.example`。

---

## 六、变更文件汇总

| Batch | 新增 | 修改 | 删除 |
|-------|------|------|------|
| Batch-1 | — | `pi_mutex.h`, `watchdog_mgr.cc`, `cloud_service.h/.cc` | — |
| Batch-2 | — | `safety_monitor.h/.cc`, `robot_fsm.cc`, `ota_manager.h/.cc`, `health_service.cc` | — |
| Batch-3 | — | `event_bus.h`, `robot_fsm.h`, `health_service.h`, `data_cache.h`, `test/CMakeLists.txt` | `walk_motor.h/.cc`, `diagnostics_collector.h/.cc`, `test/bms_test.cc`, `test/bms2_test.cc`, `test/imu_test.cc`, `test/walk_motor_test.cc`, `test/device/walk_motor_test.cc` |
| Batch-4 | `.env.example` | `config.json`, `mqtt_transport.cc`, `config_service.h/.cc`, `CMakePresets.json`, `README.md` | — |

---

## 七、验收标准

- Batch-1 后：`unit_tests` 全部通过；`PiMutex` EOWNERDEAD 场景测试通过；WatchdogMgr 并发 heartbeat 不死锁
- Batch-2 后：`unit_tests` 全部通过；SafetyMonitor 双限位测试 <200ms；OTA 流式写入内存峰值 <1MB
- Batch-3 后：`unit_tests` 全部通过；`grep -r "WalkMotor[^G]"` 无结果；`grep -r "DiagnosticsCollector"` 无结果
- Batch-4 后：环境变量可覆盖 broker_uri；新开发机器无需修改 CMakePresets.json 可正常构建
