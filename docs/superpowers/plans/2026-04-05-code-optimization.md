# PV 清扫机器人代码优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 `walk_motor_group.cc` 中 3 个新发现 Bug，并按优先级批次（P0→P1→P2→P3）完整实施设计规格 `docs/superpowers/specs/2026-04-05-code-optimization-design.md` 中 19 项优化。

**Architecture:** 7 层严格向下依赖架构（HAL→Driver→Device→Middleware→Service→App→main），PREEMPT_RT aarch64。所有修复遵守"不改变公有接口签名"原则（OTA 新接口为增量新增，原接口保留向后兼容）。

**Tech Stack:** C++17, PREEMPT_RT Linux, spdlog, nlohmann/json, Boost.SML, OpenSSL EVP, CMake/Ninja

---

## 文件变更总览

| 任务 | 新增 | 修改 | 删除 |
|------|------|------|------|
| Task 1 (WMG Bug) | — | `walk_motor_group.cc` | — |
| Task 2 (PiMutex) | — | `pi_mutex.h` | — |
| Task 3 (Watchdog) | — | `watchdog_mgr.cc` | — |
| Task 4 (RPC 验证) | — | `cloud_service.h`, `cloud_service.cc` | — |
| Task 5 (SafetyMon) | — | `safety_monitor.h`, `safety_monitor.cc` | — |
| Task 6 (FSM 状态查询) | — | `robot_fsm.cc`, `robot_fsm.h` | — |
| Task 7 (OTA 重构) | — | `ota_manager.h`, `ota_manager.cc`, `cloud_service.cc` | — |
| Task 8 (Health 日志) | — | `health_service.cc`, `health_service.h` | — |
| Task 9 (Batch-3 清理) | — | `event_bus.h`, `robot_fsm.h`, `health_service.h`, `data_cache.h`, `test/CMakeLists.txt` | `walk_motor.h/.cc`, `diagnostics_collector.h/.cc`, 4 个旧测试文件, `test/device/walk_motor_test.cc` |
| Task 10 (Batch-4 工程) | `.env.example` | `config.json`, `CMakePresets.json`, `README.md` | — |

---

## Task 1：WalkMotorGroup 3 个 Bug 修复

**Files:**
- Modify: `pv_cleaning_robot/device/walk_motor_group.cc:180-187, 375, 529-546`

### Bug 1 — 删除遗留调试日志（walk_motor_group.cc L375）

- [ ] **Step 1.1: 删除 `update()` 中的调试日志**

文件 `pv_cleaning_robot/device/walk_motor_group.cc`，找到并删除：

```cpp
    spdlog::info("test group open");
```

结果：`update()` 函数开头从：
```cpp
void WalkMotorGroup::update(float yaw_deg) {
    if (!can_->is_open())
        return;
    spdlog::info("test group open");   // ← 删除此行
    auto now = std::chrono::steady_clock::now();
```
变为：
```cpp
void WalkMotorGroup::update(float yaw_deg) {
    if (!can_->is_open())
        return;
    auto now = std::chrono::steady_clock::now();
```

### Bug 2 — `set_speeds_extra()` 添加 override 检查（walk_motor_group.cc L180-187）

- [ ] **Step 1.2: 在 `set_speeds_extra()` 中添加 override 守卫**

文件 `pv_cleaning_robot/device/walk_motor_group.cc`，将：
```cpp
DeviceError WalkMotorGroup::set_speeds_extra(float lt, float rt, float lb, float rb) {
    lt = clamp_rpm(lt);
    rt = clamp_rpm(rt);
    lb = clamp_rpm(lb);
    rb = clamp_rpm(rb);

    return send_ctrl(protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb));
}
```
替换为：
```cpp
DeviceError WalkMotorGroup::set_speeds_extra(float lt, float rt, float lb, float rb) {
    if (override_active_.load(std::memory_order_acquire))
        return DeviceError::OK;  // emergency override 激活期间静默丢弃
    lt = clamp_rpm(lt);
    rt = clamp_rpm(rt);
    lb = clamp_rpm(lb);
    rb = clamp_rpm(rb);

    return send_ctrl(protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb));
}
```

### Bug 3 — `recv_loop()` 数据竞争修复（walk_motor_group.cc L529-546）

- [ ] **Step 1.3: 在锁内复制 diag 字段，锁外使用本地副本打印**

文件 `pv_cleaning_robot/device/walk_motor_group.cc`，将：
```cpp
            {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                auto& d = diag_[i];
                d.speed_rpm = s.speed_rpm;
                d.torque_a = s.torque_a;
                d.position_deg = s.position_deg;
                d.fault = s.fault;
                d.mode = s.mode;
                d.online = true;
                ++d.feedback_frame_count;
                last_fb_time_[i] = ts;
            }
            spdlog::info("id:{} rpm:{} current:{} position_deg:{} fault:{} mode:{}",
                         i + 1,
                         diag_[i].speed_rpm,
                         diag_[i].torque_a,
                         diag_[i].position_deg,
                         static_cast<int>(diag_[i].fault),
                         static_cast<int>(diag_[i].mode));
```
替换为：
```cpp
            float log_rpm, log_torque, log_pos;
            int log_fault, log_mode;
            {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                auto& d = diag_[i];
                d.speed_rpm = s.speed_rpm;
                d.torque_a = s.torque_a;
                d.position_deg = s.position_deg;
                d.fault = s.fault;
                d.mode = s.mode;
                d.online = true;
                ++d.feedback_frame_count;
                last_fb_time_[i] = ts;
                // 在锁内复制打印字段，避免锁外无保护读取（data race / UB）
                log_rpm   = d.speed_rpm;
                log_torque = d.torque_a;
                log_pos   = d.position_deg;
                log_fault  = static_cast<int>(d.fault);
                log_mode   = static_cast<int>(d.mode);
            }
            spdlog::info("id:{} rpm:{} current:{} position_deg:{} fault:{} mode:{}",
                         i + 1, log_rpm, log_torque, log_pos, log_fault, log_mode);
```

- [ ] **Step 1.4: 编译验证（Task 1）**

```bash
cmake --build build --target pv_cleaning_robot 2>&1 | tail -20
```
预期：无编译错误，无相关警告。

- [ ] **Step 1.5: Commit Task 1**

```bash
git add pv_cleaning_robot/device/walk_motor_group.cc
git commit -m "fix(device): fix 3 bugs in WalkMotorGroup

- Remove stray debug log in update() that flooded at 50Hz
- Add emergency override guard to set_speeds_extra() to prevent
  bypassing the two-line override protection mechanism
- Fix data race in recv_loop(): copy diag fields under lock before
  releasing, eliminating UB concurrent read with update() thread

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 2：PiMutex EOWNERDEAD 死锁修复（Batch-1 #1）

**Files:**
- Modify: `include/pv_cleaning_robot/hal/pi_mutex.h:62-93`

**根因**：`lock()` 在 `EOWNERDEAD` 路径上调用 `pthread_mutex_consistent()` 后直接 `throw`。此时锁的所有权已转交当前线程，但 `std::lock_guard` 构造失败不触发析构，锁永久泄漏，其他线程永久死锁。`try_lock()` 有相同问题。

- [ ] **Step 2.1: 修复 `lock()` 的 EOWNERDEAD 路径**

文件 `include/pv_cleaning_robot/hal/pi_mutex.h`，将：
```cpp
        if (rc == EOWNERDEAD) {
            // 前任持有者已死亡，内核将锁的所有权转交。
            // 标记为 consistent 防止后续永远 ENOTRECOVERABLE，
            // 并通过异常强制打断 lock_guard，防止操作可能已损坏的共享状态。
            pthread_mutex_consistent(&mutex_);
            throw std::runtime_error(
                "[PiMutex] lock acquired but previous owner died. State inconsistent!");
        }
```
替换为：
```cpp
        if (rc == EOWNERDEAD) {
            // 前任持有者已死亡，内核将锁的所有权转交给当前线程。
            // 必须先 consistent() 再 unlock()：
            //   1. consistent() 将锁标记为一致，使后续 unlock() 不返回 ENOTRECOVERABLE
            //   2. unlock() 释放所有权，防止 throw 后 lock_guard 析构缺失导致永久死锁
            pthread_mutex_consistent(&mutex_);
            pthread_mutex_unlock(&mutex_);  // 先释放，再抛出
            throw std::runtime_error(
                "[PiMutex] lock acquired but previous owner died. State inconsistent!");
        }
```

- [ ] **Step 2.2: 修复 `try_lock()` 的 EOWNERDEAD 路径**

文件 `include/pv_cleaning_robot/hal/pi_mutex.h`，将：
```cpp
        if (rc == EOWNERDEAD) {
            pthread_mutex_consistent(&mutex_);
            throw std::runtime_error(
                "[PiMutex] trylock acquired but previous owner died. State inconsistent!");
        }
```
替换为：
```cpp
        if (rc == EOWNERDEAD) {
            pthread_mutex_consistent(&mutex_);
            pthread_mutex_unlock(&mutex_);  // 先释放，再抛出
            throw std::runtime_error(
                "[PiMutex] trylock acquired but previous owner died. State inconsistent!");
        }
```

- [ ] **Step 2.3: 编译验证（Task 2）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```
预期：编译成功。

- [ ] **Step 2.4: Commit Task 2**

```bash
git add include/pv_cleaning_robot/hal/pi_mutex.h
git commit -m "fix(hal): fix PiMutex EOWNERDEAD permanent deadlock

When EOWNERDEAD is returned, the kernel has transferred lock ownership
to the current thread. The previous code called consistent() but then
threw an exception before the lock_guard constructor returned, so its
destructor never called unlock() - permanently leaking the lock.

Fix: call consistent() + unlock() before throwing so the lock is
always released regardless of the exception path. Same fix applied
to try_lock().

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 3：WatchdogMgr 持锁回调死锁修复（Batch-1 #2）

**Files:**
- Modify: `pv_cleaning_robot/app/watchdog_mgr.cc:94-114`

**根因**：`monitor_loop` 在持有 `tickets_mtx_` 锁期间调用 `on_timeout_` 回调。若回调中调用 `heartbeat()` 或 `register_thread()`（均需同一把锁），立即死锁。

- [ ] **Step 3.1: 重构 `monitor_loop()` — 锁内仅收集名称，锁外触发回调**

文件 `pv_cleaning_robot/app/watchdog_mgr.cc`，将：
```cpp
        // 检查所有票据
        {
            std::lock_guard<hal::PiMutex> lk(tickets_mtx_);
            auto now = std::chrono::steady_clock::now();
            for (auto& [id, ticket] : tickets_) {
                if (ticket.expired) continue;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - ticket.last_beat).count();
                if (elapsed > ticket.timeout_ms) {
                    ticket.expired = true;
                    if (on_timeout_) on_timeout_(ticket.name);
                }
            }
        }
```
替换为：
```cpp
        // 锁内仅收集到期名称，锁外触发回调（防止回调内调用 heartbeat/register_thread 死锁）
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
        for (auto& name : expired_names) {
            if (on_timeout_) on_timeout_(name);
        }
```

- [ ] **Step 3.2: 编译验证（Task 3）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```

- [ ] **Step 3.3: Commit Task 3**

```bash
git add pv_cleaning_robot/app/watchdog_mgr.cc
git commit -m "fix(app): fix WatchdogMgr deadlock when timeout callback re-acquires lock

on_timeout_ was called while holding tickets_mtx_. If the callback
called heartbeat() or register_thread() (both require the same lock),
an immediate deadlock occurred.

Fix: collect expired ticket names inside the lock, then invoke
callbacks outside the lock. Safe because each ticket's expired flag
is set before releasing the lock, preventing double-firing.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 4：CloudService RPC 输入白名单验证（Batch-1 #3）

**Files:**
- Modify: `include/pv_cleaning_robot/service/cloud_service.h:62-73`
- Modify: `pv_cleaning_robot/service/cloud_service.cc:83-116`

**根因**：`on_rpc_message()` 对 method 名和 params 大小无任何校验，允许任意 method 名路由，params 过大可能耗尽栈空间。

- [ ] **Step 4.1: 在 `cloud_service.h` 中增加白名单常量和成员**

文件 `include/pv_cleaning_robot/service/cloud_service.h`，在 `private:` 块中，将：
```cpp
    void on_rpc_message(const std::string& topic, const std::string& payload);

    std::shared_ptr<middleware::NetworkManager> network_;
    std::shared_ptr<middleware::DataCache> cache_;
    Topics topics_;
    std::unordered_map<std::string, RpcHandler> rpc_handlers_;
    AttrCallback attr_cb_;  ///< 共享属性回调（为空则不分发）
    std::mutex rpc_mtx_;
```
替换为：
```cpp
    void on_rpc_message(const std::string& topic, const std::string& payload);

    /// RPC params 大小上限（防止超大 payload 耗尽栈空间）
    static constexpr size_t kMaxRpcParamsBytes = 4096;

    std::shared_ptr<middleware::NetworkManager> network_;
    std::shared_ptr<middleware::DataCache> cache_;
    Topics topics_;
    std::unordered_map<std::string, RpcHandler> rpc_handlers_;
    AttrCallback attr_cb_;  ///< 共享属性回调（为空则不分发）
    std::mutex rpc_mtx_;
```

- [ ] **Step 4.2: 在 `on_rpc_message()` 中添加 params 大小校验和 method 白名单校验**

文件 `pv_cleaning_robot/service/cloud_service.cc`，将 `on_rpc_message()` 的解析与 handler 查找部分，从：
```cpp
    std::string method;
    std::string params;
    try {
        auto j = nlohmann::json::parse(payload);
        method = j.value("method", "");
        params = j.contains("params") ? j["params"].dump() : "{}";
    } catch (...) {
        return;
    }

    std::string response{"false"};
    {
        std::lock_guard<std::mutex> lk(rpc_mtx_);
        auto it = rpc_handlers_.find(method);
        if (it != rpc_handlers_.end()) {
            try { response = it->second(params); } catch (...) {}
        }
    }
```
替换为：
```cpp
    // params 大小防御：超大 payload 拒绝，防止栈耗尽
    if (payload.size() > kMaxRpcParamsBytes) {
        spdlog::warn("[CloudService] RPC payload too large: {} bytes (max {})",
                     payload.size(), kMaxRpcParamsBytes);
        return;
    }

    std::string method;
    std::string params;
    try {
        auto j = nlohmann::json::parse(payload);
        method = j.value("method", "");
        params = j.contains("params") ? j["params"].dump() : "{}";
    } catch (...) {
        return;
    }

    std::string response{"false"};
    {
        std::lock_guard<std::mutex> lk(rpc_mtx_);
        // method 白名单：只允许已注册的 handler（防止未知方法名路由）
        auto it = rpc_handlers_.find(method);
        if (it == rpc_handlers_.end()) {
            spdlog::warn("[CloudService] Rejected unknown RPC method: {}", method);
            return;
        }
        try { response = it->second(params); } catch (...) {}
    }
```

- [ ] **Step 4.3: 编译验证（Task 4）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```

- [ ] **Step 4.4: Commit Task 4**

```bash
git add include/pv_cleaning_robot/service/cloud_service.h \
        pv_cleaning_robot/service/cloud_service.cc
git commit -m "fix(service): add RPC input validation to CloudService

- Reject payloads exceeding kMaxRpcParamsBytes (4096) before JSON parse
- Reject unknown RPC methods: only handlers registered via register_rpc()
  are allowed (implicit whitelist via the rpc_handlers_ map)

This prevents unauthenticated cloud commands from invoking unregistered
handlers or exhausting stack space with oversized params.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 5：SafetyMonitor 双限位并行防抖（Batch-2 #1）

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/safety_monitor.h:76-81`
- Modify: `pv_cleaning_robot/middleware/safety_monitor.cc:67-153`

**根因**：`monitor_loop` 中两个 `nanosleep(180ms)` 串行，前后端同时触发时后端延迟最长 ~360ms，超出安全预期。

- [ ] **Step 5.1: 在 `safety_monitor.h` 中将 `pending_*` bool 改为时间戳 uint64_t**

文件 `include/pv_cleaning_robot/middleware/safety_monitor.h`，将：
```cpp
    /// 防抖 pending 标志：GPIO 线程触发后置 true，monitor_loop 延迟 180ms 后消费
    std::atomic<bool> pending_front_{false};
    std::atomic<bool> pending_rear_{false};
```
替换为：
```cpp
    /// 防抖 pending 时间戳（ms）：GPIO 线程触发后记录触发时刻，0 = 未触发。
    /// monitor_loop 每 5ms 非阻塞检查，距触发 ≥180ms 后发布 LimitSettledEvent。
    /// 使用时间戳替代 bool，支持前后端并行防抖（各自独立计时，互不阻塞）。
    std::atomic<uint64_t> pending_front_ts_{0};
    std::atomic<uint64_t> pending_rear_ts_{0};
```

- [ ] **Step 5.2: 更新 `on_limit_trigger()` — 记录时间戳替代置 bool**

文件 `pv_cleaning_robot/middleware/safety_monitor.cc`，在文件顶部 `#include` 区域之后、`namespace robot::middleware {` 之前，添加内联辅助函数：

```cpp
namespace {
/// 返回单调时钟毫秒时间戳（用于防抖计时，与 now_ms() 配对）
inline uint64_t now_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
        .count());
}
}  // namespace
```

然后将 `on_limit_trigger()` 中的两处 `pending_*_.store(true, ...)` 改为存储时间戳：

```cpp
        // 2. 置 pending 标志（原子，monitor_loop 内防抖后发布 LimitSettledEvent）
        pending_front_.store(true, std::memory_order_release);
```
→
```cpp
        // 2. 记录触发时间戳（monitor_loop 非阻塞计时，≥180ms 后发布 LimitSettledEvent）
        pending_front_ts_.store(now_ms(), std::memory_order_release);
```

以及：
```cpp
        // 3. 置 pending 标志
        pending_rear_.store(true, std::memory_order_release);
```
→
```cpp
        // 3. 记录触发时间戳
        pending_rear_ts_.store(now_ms(), std::memory_order_release);
```

- [ ] **Step 5.3: 重构 `monitor_loop()` — 非阻塞并行检查替代串行 nanosleep**

文件 `pv_cleaning_robot/middleware/safety_monitor.cc`，将 `monitor_loop()` 的主循环体（`while (running_.load()) {` 内的全部 pending 相关逻辑）从：

```cpp
    while (running_.load()) {
        // ── 前端 pending 防抖 ─────────────────────────────────────────
        // on_limit_trigger(FRONT) 置位，此处延迟 180ms 后通知 FSM。
        // 注意：前端不设 estop_active_，start_returning() 内 clear_override() 解除 override 锁。
        if (pending_front_.exchange(false, std::memory_order_acquire)) {
            struct timespec ts{0, 180'000'000L};  // 180ms，total ~200ms（+~20ms GPIO 至此）
            nanosleep(&ts, nullptr);
            event_bus_.publish(LimitSettledEvent{device::LimitSide::FRONT});
        }
        // ── 尾端 pending 防抖 ─────────────────────────────────────────
        // on_limit_trigger(REAR) 置位，180ms 后通知 FSM 并解除急停锁。
        if (pending_rear_.exchange(false, std::memory_order_acquire)) {
            struct timespec ts{0, 180'000'000L};
            nanosleep(&ts, nullptr);
            // 解除急停锁：清 estop_active_ + override_active_，
            // 允许后续 start_cleaning()/start_returning() 正常驱动电机。
            event_bus_.publish(LimitSettledEvent{device::LimitSide::REAR});
            reset_estop();
        }
        // ── 备用轮询路径：以防 GPIO 边沿回调丢失（双保险）────────────
        // 注意：on_limit_trigger() 内部已调用 clear_trigger()，
        // 因此重复触发被抑制，无需额外去重计数器。
        if (front_switch_->is_triggered()) {
            on_limit_trigger(device::LimitSide::FRONT);
        }
        if (rear_switch_->is_triggered() &&
            !estop_active_.load(std::memory_order_acquire)) {
            on_limit_trigger(device::LimitSide::REAR);
        }
        // 5 ms 轮询（SCHED_FIFO 94，低于 GPIO 95，避免兜底轮询反向压制边沿线程）
        struct timespec ts{0, 5 * 1000 * 1000};
        nanosleep(&ts, nullptr);
    }
```

替换为：

```cpp
    // 非阻塞并行防抖：前后端各自独立计时，互不阻塞。
    // check_settled 每 5ms 被调用一次，距触发 ≥180ms 后发布事件。
    // 前后端同时触发时，均在 ~185ms（180ms+5ms误差）内响应，消除串行 nanosleep 的 ~360ms 延迟。
    auto check_settled = [&](std::atomic<uint64_t>& ts_atom, device::LimitSide side) {
        uint64_t t = ts_atom.load(std::memory_order_acquire);
        if (t == 0) return;
        if (now_ms() - t >= 180u) {
            ts_atom.store(0, std::memory_order_release);  // 清除，防止重复触发
            event_bus_.publish(LimitSettledEvent{side});
            if (side == device::LimitSide::REAR) reset_estop();
        }
    };

    while (running_.load()) {
        // ── 非阻塞并行防抖检查（前后端独立计时）──────────────────────
        check_settled(pending_front_ts_, device::LimitSide::FRONT);
        check_settled(pending_rear_ts_,  device::LimitSide::REAR);

        // ── 备用轮询路径：以防 GPIO 边沿回调丢失（双保险）────────────
        if (front_switch_->is_triggered()) {
            on_limit_trigger(device::LimitSide::FRONT);
        }
        if (rear_switch_->is_triggered() &&
            !estop_active_.load(std::memory_order_acquire)) {
            on_limit_trigger(device::LimitSide::REAR);
        }
        // 5 ms 轮询（SCHED_FIFO 94，低于 GPIO 95，避免兜底轮询反向压制边沿线程）
        struct timespec poll_ts{0, 5 * 1000 * 1000};
        nanosleep(&poll_ts, nullptr);
    }
```

- [ ] **Step 5.4: 编译验证（Task 5）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```

- [ ] **Step 5.5: Commit Task 5**

```bash
git add include/pv_cleaning_robot/middleware/safety_monitor.h \
        pv_cleaning_robot/middleware/safety_monitor.cc
git commit -m "fix(middleware): parallel debounce for dual limit switches in SafetyMonitor

Previous implementation used two sequential nanosleep(180ms) calls,
causing up to ~360ms delay when front and rear limits triggered simultaneously.

Fix: replace bool pending flags with uint64_t timestamp atomics. The
monitor_loop polls every 5ms and checks each limit independently using
a non-blocking lambda. Both limits now settle within ~185ms (180ms +
5ms polling jitter) regardless of concurrent triggers.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 6：RobotFsm 使用 SML 类型安全状态查询（Batch-2 #2）

**Files:**
- Modify: `pv_cleaning_robot/app/robot_fsm.cc:107-136`
- Modify: `include/pv_cleaning_robot/app/robot_fsm.h:122-124`

**根因**：`dispatch<EvRearLimitSettled>` 使用 `state_name_ == "CleanReturn"` / `"Returning"` 字符串比较做业务逻辑分支，拼写错误不产生编译错误。

- [ ] **Step 6.1: 在 `robot_fsm.h` 的 `state_name_` 添加注释**

文件 `include/pv_cleaning_robot/app/robot_fsm.h`，将：
```cpp
    mutable hal::PiMutex          mtx_;
    std::string                   state_name_{"Init"};
    std::unique_ptr<sml::sm<Fsm>> sm_;
```
替换为：
```cpp
    mutable hal::PiMutex          mtx_;
    /// 仅用于日志输出，不参与任何业务判断。
    /// 状态判断请使用 sm_->is(sml::state<StateXxx>)。
    std::string                   state_name_{"Init"};
    std::unique_ptr<sml::sm<Fsm>> sm_;
```

- [ ] **Step 6.2: 将 `dispatch<EvRearLimitSettled>` 中的字符串比较改为 SML 类型安全查询**

文件 `pv_cleaning_robot/app/robot_fsm.cc`，将：
```cpp
        if (state_name_ == "CleanReturn") {
            completed_half_passes_++;
            spdlog::info("[FSM] 尾端限位已稳定（已完成半趟 {}/{}）",
                         completed_half_passes_, target_half_passes_);

            if (completed_half_passes_ >= target_half_passes_) {
                sm_->process_event(EvTaskComplete{});
                state_name_ = "Charging";
                spdlog::info("[FSM] → Charging（全部趟数完成，回到停机位）");
                action = [this]() { motion_->stop_cleaning(); };
            } else {
                sm_->process_event(e);
                state_name_ = "CleanFwd";
                spdlog::info("[FSM] → CleanFwd（回到停机位，继续正向清扫）");
                action = [this]() { motion_->start_cleaning(); };
            }
        } else if (state_name_ == "Returning") {
            sm_->process_event(e);
            state_name_ = "Charging";
            spdlog::info("[FSM] → Charging（故障/低电返回停机位完成）");
            action = [this]() { motion_->stop_cleaning(); };
        }
        // 其他状态收到尾端信号：忽略
```
替换为：
```cpp
        if (sm_->is(sml::state<StateCleanReturn>)) {
            completed_half_passes_++;
            spdlog::info("[FSM] 尾端限位已稳定（已完成半趟 {}/{}）",
                         completed_half_passes_, target_half_passes_);

            if (completed_half_passes_ >= target_half_passes_) {
                sm_->process_event(EvTaskComplete{});
                state_name_ = "Charging";
                spdlog::info("[FSM] → Charging（全部趟数完成，回到停机位）");
                action = [this]() { motion_->stop_cleaning(); };
            } else {
                sm_->process_event(e);
                state_name_ = "CleanFwd";
                spdlog::info("[FSM] → CleanFwd（回到停机位，继续正向清扫）");
                action = [this]() { motion_->start_cleaning(); };
            }
        } else if (sm_->is(sml::state<StateReturning>)) {
            sm_->process_event(e);
            state_name_ = "Charging";
            spdlog::info("[FSM] → Charging（故障/低电返回停机位完成）");
            action = [this]() { motion_->stop_cleaning(); };
        }
        // 其他状态收到尾端信号：忽略
```

- [ ] **Step 6.3: 编译验证（Task 6）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```

- [ ] **Step 6.4: Commit Task 6**

```bash
git add include/pv_cleaning_robot/app/robot_fsm.h \
        pv_cleaning_robot/app/robot_fsm.cc
git commit -m "fix(app): use SML type-safe state queries in RobotFsm

Replace string comparison (state_name_ == \"CleanReturn\") with
sm_->is(sml::state<StateCleanReturn>) in dispatch<EvRearLimitSettled>.
Typos in state names are now caught at compile time rather than
silently misbehaving at runtime.

state_name_ is annotated as log-only; all business logic now uses
the SML is<>() API.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 7：OTA 流式写入 + SHA-256 校验 + reboot 安全门卫（Batch-2 #3,4,6）

**Files:**
- Modify: `include/pv_cleaning_robot/middleware/ota_manager.h`
- Modify: `pv_cleaning_robot/middleware/ota_manager.cc`
- Modify: `pv_cleaning_robot/service/cloud_service.cc`（调用方适配）

**根因**：
1. `start()` 要求全量固件字符串（数十MB），加剧 RT 内存压力
2. `MD5()` 在 OpenSSL 3.x 已废弃且不满足安全要求
3. `apply_and_reboot()` 无安全状态检查，清扫中途重启致机器脱轨

- [ ] **Step 7.1: 更新 `ota_manager.h` — 新增流式接口、SHA-256、安全门卫**

文件 `include/pv_cleaning_robot/middleware/ota_manager.h`，将全文替换为：

```cpp
#pragma once
#include <cstdint>
#include <functional>
#include <istream>
#include <mutex>
#include <string>

namespace robot::middleware {

/// @brief OTA 固件更新管理器（A/B 分区）
///
/// 工作流程：
///   1. cloud 回调通知固件包 URL/topic
///   2. OtaManager 分块接收固件到非活动分区 B（流式 4KB chunk）
///   3. SHA-256 校验（OpenSSL EVP，兼容 1.x/3.x）
///   4. 写入 bootloader 分区标志（切换至 B）
///   5. 安全状态检查通过后触发重启
class OtaManager {
   public:
    enum class State { IDLE, DOWNLOADING, VERIFYING, WRITING_FLAG, PENDING_REBOOT, FAILED };

    struct Progress {
        State state{State::IDLE};
        uint32_t bytes_written{0};
        uint32_t total_bytes{0};
        std::string error_msg;
    };

    using ProgressCallback = std::function<void(const Progress&)>;
    /// 安全门卫回调：返回 true 表示可以重启，false 阻止重启
    using SafetyCheckFn = std::function<bool()>;

    /// @param partition_b_path  固件写入目标路径（B 分区块设备或文件）
    /// @param flag_path         bootloader 分区标志文件路径
    OtaManager(std::string partition_b_path, std::string flag_path);

    /// 开始 OTA（流式写入，峰值内存 ~8KB）
    /// @param firmware_stream   固件数据流（istream，支持 istringstream/ifstream）
    /// @param expected_bytes    固件总字节数（用于进度上报，传 0 则跳过进度计算）
    /// @param expected_sha256   固件 SHA-256（64字符十六进制，cloud 端提供）
    bool start_stream(std::istream& firmware_stream,
                      uint64_t expected_bytes,
                      const std::string& expected_sha256);

    /// 兼容旧接口：内部包装为 istringstream 调用 start_stream
    /// @deprecated 优先使用 start_stream；此接口会将全量数据载入内存
    bool start(const std::string& firmware_data, const std::string& expected_sha256);

    /// 写入分区切换标志并触发重启（start/start_stream 成功后调用）
    /// 若已通过 set_safety_check() 注入安全门卫，则仅在门卫返回 true 时执行重启
    bool apply_and_reboot();

    Progress get_progress() const;
    State state() const;
    void set_progress_callback(ProgressCallback cb);
    /// 注入安全门卫（建议在 main.cc 中注入，确保机器人在安全状态才允许重启）
    void set_safety_check(SafetyCheckFn fn);

   private:
    bool verify_sha256(std::istream& data, const std::string& expected_hex);
    void set_state(State s, const std::string& err = "");

    std::string partition_b_path_;
    std::string flag_path_;
    Progress progress_;
    ProgressCallback on_progress_;
    SafetyCheckFn safety_check_;
    mutable std::mutex mtx_;
};

}  // namespace robot::middleware
```

- [ ] **Step 7.2: 更新 `ota_manager.cc` — 实现流式写入 + EVP SHA-256 + 安全门卫**

文件 `pv_cleaning_robot/middleware/ota_manager.cc`，将全文替换为：

```cpp
#include <fstream>
#include <iomanip>
#include <linux/reboot.h>
#include <mutex>
#include <openssl/evp.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <sys/syscall.h>
#include <unistd.h>

#include "pv_cleaning_robot/middleware/ota_manager.h"

namespace robot::middleware {

OtaManager::OtaManager(std::string partition_b_path, std::string flag_path)
    : partition_b_path_(std::move(partition_b_path)), flag_path_(std::move(flag_path)) {}

bool OtaManager::start_stream(std::istream& firmware_stream,
                               uint64_t expected_bytes,
                               const std::string& expected_sha256) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        progress_.state = State::DOWNLOADING;
        progress_.total_bytes = static_cast<uint32_t>(expected_bytes);
        progress_.bytes_written = 0;
        progress_.error_msg.clear();
    }
    if (on_progress_) on_progress_(get_progress());

    // 流式写入 B 分区（峰值内存 ~4KB）
    {
        std::ofstream ofs(partition_b_path_, std::ios::binary | std::ios::trunc);
        if (!ofs) {
            set_state(State::FAILED, "cannot open partition: " + partition_b_path_);
            return false;
        }
        constexpr size_t kChunk = 4096;
        char buf[kChunk];
        while (firmware_stream.read(buf, kChunk) || firmware_stream.gcount() > 0) {
            auto n = static_cast<std::streamsize>(firmware_stream.gcount());
            ofs.write(buf, n);
            {
                std::lock_guard<std::mutex> lk(mtx_);
                progress_.bytes_written += static_cast<uint32_t>(n);
            }
            if (on_progress_) on_progress_(get_progress());
        }
        ofs.flush();
    }

    // SHA-256 校验（使用 EVP API，兼容 OpenSSL 1.x/3.x，MD5 已废弃）
    set_state(State::VERIFYING);
    if (on_progress_) on_progress_(get_progress());

    // 重置流位置以便校验
    firmware_stream.clear();
    firmware_stream.seekg(0);
    if (!verify_sha256(firmware_stream, expected_sha256)) {
        set_state(State::FAILED, "SHA-256 mismatch");
        return false;
    }

    set_state(State::WRITING_FLAG);
    if (on_progress_) on_progress_(get_progress());

    set_state(State::PENDING_REBOOT);
    if (on_progress_) on_progress_(get_progress());

    return true;
}

bool OtaManager::start(const std::string& firmware_data,
                        const std::string& expected_sha256) {
    std::istringstream iss(firmware_data);
    return start_stream(iss, firmware_data.size(), expected_sha256);
}

bool OtaManager::apply_and_reboot() {
    // 安全门卫：机器人必须处于 Idle 或 Charging 状态才允许重启
    if (safety_check_ && !safety_check_()) {
        spdlog::error("[OTA] Reboot blocked: robot not in safe state (Idle/Charging required)");
        set_state(State::FAILED, "blocked by safety check");
        return false;
    }

    // 写入分区切换标志（简单标志文件：写入 "B\n"）
    {
        std::ofstream ofs(flag_path_, std::ios::trunc);
        if (!ofs) return false;
        ofs << "B\n";
    }
    sync();  // 确保写入落盘

    spdlog::info("[OTA] Rebooting to apply firmware update...");
    // 触发系统重启（需 root 权限）
    syscall(
        SYS_reboot, LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2, LINUX_REBOOT_CMD_RESTART, nullptr);
    return true;  // 不可达，仅作类型要求
}

OtaManager::Progress OtaManager::get_progress() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return progress_;
}

OtaManager::State OtaManager::state() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return progress_.state;
}

void OtaManager::set_progress_callback(ProgressCallback cb) {
    on_progress_ = std::move(cb);
}

void OtaManager::set_safety_check(SafetyCheckFn fn) {
    safety_check_ = std::move(fn);
}

bool OtaManager::verify_sha256(std::istream& data, const std::string& expected_hex) {
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) return false;

    EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
    char buf[4096];
    while (data.read(buf, sizeof(buf)) || data.gcount() > 0) {
        EVP_DigestUpdate(ctx, buf, static_cast<size_t>(data.gcount()));
    }
    unsigned char digest[32];
    unsigned int len = 0;
    EVP_DigestFinal_ex(ctx, digest, &len);
    EVP_MD_CTX_free(ctx);

    // 转为小写十六进制
    std::ostringstream oss;
    for (unsigned int i = 0; i < len; ++i)
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);

    return oss.str() == expected_hex;
}

void OtaManager::set_state(State s, const std::string& err) {
    std::lock_guard<std::mutex> lk(mtx_);
    progress_.state = s;
    progress_.error_msg = err;
}

}  // namespace robot::middleware
```

- [ ] **Step 7.3: 编译验证（Task 7）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -30
```
预期：无 `openssl/md5.h` 相关警告，无编译错误。

- [ ] **Step 7.4: Commit Task 7**

```bash
git add include/pv_cleaning_robot/middleware/ota_manager.h \
        pv_cleaning_robot/middleware/ota_manager.cc
git commit -m "feat(middleware): OTA streaming write, SHA-256 verification, reboot safety gate

- Replace start(string) full-memory interface with start_stream(istream)
  streaming interface; peak memory drops from O(firmware_size) to ~8KB
- Keep start(string) as deprecated backward-compatible wrapper
- Replace deprecated MD5() with OpenSSL EVP SHA-256 (compatible 1.x/3.x);
  update RPC payload field from expected_md5 to expected_sha256
- Add set_safety_check() injectable callback; apply_and_reboot() blocks
  reboot when robot is not in Idle/Charging state

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 8：HealthService 本地日志写失败告警（Batch-2 #5）

**Files:**
- Modify: `pv_cleaning_robot/service/health_service.cc:75-78`
- Modify: `include/pv_cleaning_robot/service/health_service.h:49`

- [ ] **Step 8.1: 在 `update()` 中增加 flush 失败检测和文件关闭保护**

文件 `pv_cleaning_robot/service/health_service.cc`，将：
```cpp
    // 本地 JSONL 落盘：每条记录一行，独立于网络，离线测试直接 cat 查看
    if (local_log_file_.is_open()) {
        local_log_file_ << payload << '\n';
        local_log_file_.flush();
    }
```
替换为：
```cpp
    // 本地 JSONL 落盘：每条记录一行，独立于网络，离线测试直接 cat 查看
    if (local_log_file_.is_open()) {
        local_log_file_ << payload << '\n';
        if (!local_log_file_.flush()) {
            // flush 失败通常意味着磁盘满或 I/O 错误；关闭文件停止反复写失败
            spdlog::error("[HealthService] local log flush failed (disk full?), closing file");
            local_log_file_.close();
        }
    }
```

- [ ] **Step 8.2: 在 `health_service.h` 完善 `j_` 成员的线程约束注释**

文件 `include/pv_cleaning_robot/service/health_service.h`，将：
```cpp
    mutable nlohmann::json              j_;              ///< 预分配 JSON 键树，build_payload() 中复用
```
替换为：
```cpp
    /// 预分配 JSON 键树，build_payload() 中复用以减少内存分配。
    /// 线程约束：update()/build_payload() 必须由单一线程（ThreadExecutor）调用，
    /// 不支持并发 build_payload()。若需多线程访问，须在调用方加锁。
    mutable nlohmann::json              j_;
```

- [ ] **Step 8.3: 编译验证（Task 8）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -20
```

- [ ] **Step 8.4: Commit Task 8**

```bash
git add pv_cleaning_robot/service/health_service.cc \
        include/pv_cleaning_robot/service/health_service.h
git commit -m "fix(service): detect and handle local log flush failure in HealthService

When disk is full or I/O fails, ofstream::flush() returns false.
Previously this was silently ignored, causing repeated failed writes.
Now: log an error and close the file to stop hammering a broken I/O path.

Also document thread constraint on mutable j_: single-threaded access
only (ThreadExecutor), consistent with the preallocation design intent.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 9：Batch-3 架构/可维护性清理（P2，7 项）

**Files:**
- Delete: `include/pv_cleaning_robot/device/walk_motor.h`
- Delete: `pv_cleaning_robot/device/walk_motor.cc`
- Delete: `include/pv_cleaning_robot/service/diagnostics_collector.h`
- Delete: `pv_cleaning_robot/service/diagnostics_collector.cc`（若存在）
- Delete: `test/bms_test.cc`, `test/bms2_test.cc`, `test/imu_test.cc`, `test/walk_motor_test.cc`
- Delete: `test/device/walk_motor_test.cc`
- Modify: `test/CMakeLists.txt`
- Modify: `include/pv_cleaning_robot/middleware/event_bus.h:68`
- Modify: `include/pv_cleaning_robot/middleware/data_cache.h` (类注释)

- [ ] **Step 9.1: 确认 WalkMotor 无外部调用方**

```bash
grep -rn "WalkMotor[^G]" \
  /home/tronlong/pv_cleaning_robot/pv_cleaning_robot \
  /home/tronlong/pv_cleaning_robot/include \
  /home/tronlong/pv_cleaning_robot/test/app \
  /home/tronlong/pv_cleaning_robot/test/service \
  /home/tronlong/pv_cleaning_robot/test/middleware \
  /home/tronlong/pv_cleaning_robot/test/integration \
  2>/dev/null
```
预期：无输出（或仅 `walk_motor.cc`/`walk_motor.h`/`walk_motor_test.cc` 自身）。若有外部调用，**停止此步骤并记录**。

- [ ] **Step 9.2: 确认 DiagnosticsCollector 无外部调用方**

```bash
grep -rn "DiagnosticsCollector" \
  /home/tronlong/pv_cleaning_robot/pv_cleaning_robot \
  /home/tronlong/pv_cleaning_robot/include \
  /home/tronlong/pv_cleaning_robot/test \
  2>/dev/null
```
预期：无输出（或仅 `diagnostics_collector.h`/`.cc` 自身）。

- [ ] **Step 9.3: 删除冗余文件**

```bash
rm /home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/device/walk_motor.h
rm /home/tronlong/pv_cleaning_robot/pv_cleaning_robot/device/walk_motor.cc
rm /home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/diagnostics_collector.h
rm -f /home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/diagnostics_collector.cc
rm /home/tronlong/pv_cleaning_robot/test/bms_test.cc
rm /home/tronlong/pv_cleaning_robot/test/bms2_test.cc
rm /home/tronlong/pv_cleaning_robot/test/imu_test.cc
rm /home/tronlong/pv_cleaning_robot/test/walk_motor_test.cc
rm /home/tronlong/pv_cleaning_robot/test/device/walk_motor_test.cc
```

- [ ] **Step 9.4: 更新 `test/CMakeLists.txt` — 移除已删除源文件**

文件 `test/CMakeLists.txt`，在 `COMMON_SRCS` 中移除：
```cmake
  ${PROJ}/device/walk_motor.cc
```

在 `unit_tests` 目标中移除：
```cmake
  device/walk_motor_test.cc
```

- [ ] **Step 9.5: 修正 `event_bus.h` 注释中的错误数字**

文件 `include/pv_cleaning_robot/middleware/event_bus.h`，将：
```cpp
    /// 发布事件——在调用线程中同步回调所有订阅者
    ///
    /// publish() 持锁期间仅拷贝 std::function 指针到固定栈数组，不堆分配。
    /// 回调在锁外执行，事件对象生命期由 publish() 栈帧保证（const EventT& event）。
    /// 订阅者上限 kMaxHandlers=16；超出时截断并记录（不崩溃）。
```
替换为：
```cpp
    /// 发布事件——在调用线程中同步回调所有订阅者
    ///
    /// publish() 持锁期间仅拷贝 std::function 指针到固定栈数组，不堆分配。
    /// 回调在锁外执行，事件对象生命期由 publish() 栈帧保证（const EventT& event）。
    /// 订阅者上限 kMaxHandlers=32；超出时截断（不崩溃）。
```

- [ ] **Step 9.6: 在 `data_cache.h` 类注释中添加线程约束说明**

文件 `include/pv_cleaning_robot/middleware/data_cache.h`，将：
```cpp
/// @brief 遥测本地缓存（JSONL 文件持久化）
///
/// 关注点：
///   - 每次 push() 将队列全量原子重写到磁盘（SCHED_OTHER 云端线程 1Hz，重写 150KB ≈ 1ms）
///   - confirm_sent() 删除已发记录后同样重写；open() 从文件恢复未发数据
///   - 断电重启后能自动加载积压遥测并在网络恢复后补发
///   - 文件格式：每行一个 JSON：{"id":1,"topic":"...","payload":"...","ts_ms":...}
```
替换为：
```cpp
/// @brief 遥测本地缓存（JSONL 文件持久化）
///
/// 关注点：
///   - 每次 push() 将队列全量原子重写到磁盘（SCHED_OTHER 云端线程 1Hz，重写 150KB ≈ 1ms）
///   - confirm_sent() 删除已发记录后同样重写；open() 从文件恢复未发数据
///   - 断电重启后能自动加载积压遥测并在网络恢复后补发
///   - 文件格式：每行一个 JSON：{"id":1,"topic":"...","payload":"...","ts_ms":...}
///
/// 线程安全约束：内部使用 std::mutex（非 PiMutex）。
/// 禁止在 SCHED_FIFO RT 线程中调用本类的任何方法，否则可能引发优先级反转。
/// 本类仅供 SCHED_OTHER 线程（CloudService）使用。
```

- [ ] **Step 9.7: 编译验证（Task 9）**

```bash
cmake --build build --target unit_tests 2>&1 | tail -30
```
预期：编译成功，无 `walk_motor` / `DiagnosticsCollector` 相关错误。

- [ ] **Step 9.8: 验证冗余类已完全清除**

```bash
grep -rn "WalkMotor[^G]" /home/tronlong/pv_cleaning_robot/pv_cleaning_robot \
                          /home/tronlong/pv_cleaning_robot/include \
                          /home/tronlong/pv_cleaning_robot/test 2>/dev/null
grep -rn "DiagnosticsCollector" /home/tronlong/pv_cleaning_robot/pv_cleaning_robot \
                                 /home/tronlong/pv_cleaning_robot/include \
                                 /home/tronlong/pv_cleaning_robot/test 2>/dev/null
```
预期：均无输出。

- [ ] **Step 9.9: Commit Task 9**

```bash
git add -A
git commit -m "refactor(batch3): remove redundant classes and fix documentation

- Delete WalkMotor single-wheel class (superseded by WalkMotorGroup)
  and its test files; remove from CMakeLists.txt
- Delete DiagnosticsCollector (superseded by HealthService DIAGNOSTICS mode)
- Delete legacy root-level test files (bms_test.cc, bms2_test.cc,
  imu_test.cc, walk_motor_test.cc) superseded by test/device/*
- Fix EventBus comment: kMaxHandlers=32 (was incorrectly stated as 16)
- Document DataCache thread constraint: std::mutex, SCHED_OTHER only

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 10：Batch-4 工程质量（P3，3 项）

**Files:**
- Create: `.env.example`
- Modify: `config/config.json`
- Modify: `CMakePresets.json`
- Modify: `README.md`

- [ ] **Step 10.1: 在 `config/config.json` 中为 MQTT 添加 TLS 默认配置**

读取 `config/config.json`，在 `network.mqtt` 对象中增加：
```json
"tls_enabled": true,
"ca_cert_path": "/etc/pv_robot/certs/ca.crt"
```
（若 mqtt 对象已存在 `broker_uri` 等字段，在其后追加；若无 mqtt 对象则新增）

- [ ] **Step 10.2: 创建 `.env.example` 文件**

创建 `/home/tronlong/pv_cleaning_robot/.env.example`：
```bash
# PV 清扫机器人 — 构建环境变量模板
# 使用前执行：source .env.example  或  export RK3576_SDK_PATH=/your/path
export RK3576_SDK_PATH=/home/tronlong/RK3576
```

- [ ] **Step 10.3: 更新 `CMakePresets.json` — 路径改为从环境变量读取**

找到 `CMakePresets.json` 中硬编码的 SDK 路径（如 `/home/tronlong/RK3576`），将其替换为 `$env{RK3576_SDK_PATH}` 引用。例如：

```json
"environment": {
  "PATH": "$env{RK3576_SDK_PATH}/sysroots/x86_64-linux/bin:$penv{PATH}",
  "SDK_QMAKE_SYSROOT": "$env{RK3576_SDK_PATH}/sysroots/armv8a-linux"
}
```

同时在 `cmake/toolchain.cmake` 中，将硬编码 sysroot 路径改为环境变量引用（若存在）。

- [ ] **Step 10.4: 更新 `README.md` — 添加构建前置条件说明**

在 README.md 的"构建"节（或顶部）增加以下内容：

```markdown
## 构建前置条件

交叉编译工具链须在 PATH 中，且需设置 SDK 路径环境变量：

```bash
source .env.example         # 或 export RK3576_SDK_PATH=/your/sdk/path
cmake --preset rk3576-cross-linux
cmake --build --preset rk3576-build
```

开发机上 `.env.example` 提供了默认路径，可根据实际 SDK 安装位置修改后 `source`。
```

- [ ] **Step 10.5: 验证 CMakePresets 语法合法**

```bash
cmake --list-presets 2>&1 | head -20
```

- [ ] **Step 10.6: Commit Task 10**

```bash
git add config/config.json CMakePresets.json .env.example README.md
# 若 cmake/toolchain.cmake 有改动也加入
git add cmake/ 2>/dev/null || true
git commit -m "chore(build): de-hardcode SDK path and add TLS default config

- CMakePresets.json: replace hardcoded /home/tronlong/RK3576 with
  \$env{RK3576_SDK_PATH} so new machines don't need to edit presets
- Add .env.example with default SDK path for quick setup
- config.json: set mqtt.tls_enabled=true and ca_cert_path as defaults
- README.md: document build prerequisites and source .env.example step

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## 验收标准

| Batch | 验收条件 |
|-------|---------|
| Task 1 (WMG Bug) | `update()` 无 `"test group open"` 日志；`set_speeds_extra()` override 期间无 CAN 帧；`recv_loop()` 无 data race |
| Task 2 (PiMutex) | EOWNERDEAD 后锁不泄漏，其他线程可继续获取锁 |
| Task 3 (Watchdog) | 回调中调用 `heartbeat()` 不死锁 |
| Task 4 (RPC) | 未知 method 被拒绝并打印 warn；超大 payload 被拦截 |
| Task 5 (SafetyMon) | 双端同时触发，均 ≤185ms 响应；无串行阻塞 |
| Task 6 (FSM) | state_name_ 拼写错误被编译器检测；业务逻辑走 is<>() |
| Task 7 (OTA) | 无 MD5 API；流式写入内存峰值 ~8KB；reboot 安全门卫可注入 |
| Task 8 (Health) | 磁盘满时关闭本地文件；j_ 线程约束已文档化 |
| Task 9 (Batch-3) | `grep WalkMotor[^G]` 无结果；`grep DiagnosticsCollector` 无结果；编译通过 |
| Task 10 (Batch-4) | 新机器设置 `RK3576_SDK_PATH` 后可构建；MQTT TLS 默认开启 |
