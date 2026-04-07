// test/integration/hardware/clean_cycle_hw_test.cc
/**
 * 全层栈清扫流程集成测试（与 main.cc 结构一致）
 *
 * 使用 FullSystemFixture，BrushMotor 用 MockModbusMaster 替代。
 *
 * 测试分组：
 *   [hw_cycle][startup]          - 全层栈初始化，FSM = "Idle"
 *   [hw_cycle][self_check_pass]  - EvScheduleStart{at_home=true} → FSM = "CleanFwd"
 *   [hw_cycle][one_pass_no_pid]  - 完整一趟（PID 关），前后限位驱动换向，≤120s
 *   [hw_cycle][one_pass_with_pid]- 完整一趟（PID 开），整趟 yaw 漂移 < 10°
 *   [hw_cycle][fault_p0_estop]   - 注入 P0 故障 → FSM = "Fault"，电机停转
 *   [hw_cycle][low_battery_return]- 注入低电 → FSM = "Returning" → "Charging"
 *   [hw_cycle][bms_valid]        - 清扫过程中 BMS 数据持续有效
 *   [hw_cycle][imu_valid]        - 清扫过程中 IMU 数据持续有效
 *   [hw_cycle][watchdog_alive]   - 正常运行 30s，watchdog 不触发 timeout
 *
 * 运行方法（目标板，机器人在停机位）：
 *   ./hw_tests "[hw_cycle]"
 *   ./hw_tests "[hw_cycle][one_pass_with_pid]"
 *
 * 安全：清扫速度 kTestSpeedRpm=30 RPM；FullSystemFixture 析构自动 emergency_stop()
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][startup] — 全层栈初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("全层栈初始化", "[hw_cycle][startup]") {
    hw::FullSystemFixture fx(false /* pid_off */);
    REQUIRE(fx.init());

    const std::string state = fx.fsm->current_state();
    spdlog::info("[hw_cycle][startup] FSM 状态={}", state);
    CHECK(state == "Idle");

    auto bms_data = fx.bms->get_data();
    spdlog::info("[hw_cycle][startup] BMS valid={} soc={:.1f}%",
                 bms_data.valid, bms_data.soc_pct);

    auto imu_data = fx.imu->get_latest();
    spdlog::info("[hw_cycle][startup] IMU valid={} yaw={:.2f}°",
                 imu_data.valid, imu_data.yaw_deg);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][self_check_pass] — 自检通过，进入 CleanFwd
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("自检流程（at_home=true → CleanFwd）", "[hw_cycle][self_check_pass]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 机器人在停机位，后限位触发
    const bool at_home  = !fx.rear_sw->read_current_level();   // 低=触发=在停机位
    const bool at_front = !fx.front_sw->read_current_level();  // 低=触发=在前端
    spdlog::info("[hw_cycle][self_check_pass] at_home={} at_front={}", at_home, at_front);

    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 自检 → CleanFwd（最多 5s 内）
    const bool in_clean_fwd = fx.wait_state("CleanFwd", 5s);
    spdlog::info("[hw_cycle][self_check_pass] FSM={}", fx.fsm->current_state());
    REQUIRE(in_clean_fwd);

    // 电机应已开始转动
    std::this_thread::sleep_for(1s);
    auto gd = fx.walk_group->get_group_diagnostics();
    spdlog::info("[hw_cycle][self_check_pass] ctrl_frame_count={}", gd.ctrl_frame_count);
    CHECK(gd.ctrl_frame_count > 0u);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][one_pass_no_pid] — 完整一趟（PID 关）
// 流程：Idle → SelfCheck → CleanFwd → (前限位) → CleanReturn → (后限位) → Charging
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("完整清扫一趟（PID 关闭）", "[hw_cycle][one_pass_no_pid]") {
    hw::FullSystemFixture fx(false /* pid_off */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    spdlog::warn("[hw_cycle][one_pass_no_pid] ★ 开始清扫（PID 关），预计 ≤120s ★");

    const auto t_start = std::chrono::steady_clock::now();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 阶段1：等待进入 CleanFwd
    REQUIRE(fx.wait_state("CleanFwd", 5s));
    const auto t_fwd_start = std::chrono::steady_clock::now();
    spdlog::info("[hw_cycle][one_pass_no_pid] → CleanFwd");

    // 阶段2：等待前限位触发（最多 kLimitTimeoutSec 秒）
    REQUIRE(fx.wait_state("CleanReturn",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    const auto t_fwd_end = std::chrono::steady_clock::now();
    spdlog::info("[hw_cycle][one_pass_no_pid] → CleanReturn (正向 {:.1f}s)",
                 std::chrono::duration<float>(t_fwd_end - t_fwd_start).count());

    // 阶段3：等待后限位触发 → Charging
    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    const auto t_total = std::chrono::steady_clock::now();
    const float elapsed = std::chrono::duration<float>(t_total - t_start).count();

    spdlog::info("[hw_cycle][one_pass_no_pid] → Charging，总耗时={:.1f}s", elapsed);
    CHECK(elapsed <= 120.0f);
    CHECK(fx.fsm->current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][one_pass_with_pid] — 完整一趟（PID 开），整趟 yaw 漂移 < 10°
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("完整清扫一趟（PID 开启，yaw 漂移 < 10°）", "[hw_cycle][one_pass_with_pid]") {
    hw::FullSystemFixture fx(true /* pid_on */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 记录初始 yaw
    auto d0 = fx.imu->get_latest();
    REQUIRE(d0.valid);
    const float yaw_start = d0.yaw_deg;
    float       yaw_max_drift = 0.0f;

    // 后台线程持续监测 yaw 漂移
    std::atomic<bool> monitor_running{true};
    std::thread monitor_thread([&] {
        while (monitor_running.load()) {
            auto d = fx.imu->get_latest();
            if (d.valid) {
                float drift = std::abs(d.yaw_deg - yaw_start);
                if (drift > yaw_max_drift) yaw_max_drift = drift;
            }
            std::this_thread::sleep_for(100ms);
        }
    });

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    spdlog::warn("[hw_cycle][one_pass_with_pid] ★ 开始清扫（PID 开），预计 ≤120s ★");

    const auto t_start = std::chrono::steady_clock::now();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    REQUIRE(fx.wait_state("CleanFwd", 5s));
    spdlog::info("[hw_cycle][one_pass_with_pid] → CleanFwd  初始 yaw={:.2f}°", yaw_start);

    REQUIRE(fx.wait_state("CleanReturn",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    spdlog::info("[hw_cycle][one_pass_with_pid] → CleanReturn");

    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));

    monitor_running.store(false);
    monitor_thread.join();

    const float elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - t_start).count();

    spdlog::info("[hw_cycle][one_pass_with_pid] ────────────────────────────");
    spdlog::info("[hw_cycle][one_pass_with_pid] 总耗时={:.1f}s", elapsed);
    spdlog::info("[hw_cycle][one_pass_with_pid] 整趟最大 yaw 漂移={:.2f}°", yaw_max_drift);
    spdlog::info("[hw_cycle][one_pass_with_pid] ────────────────────────────");

    CHECK(fx.fsm->current_state() == "Charging");
    CHECK(elapsed <= 120.0f);
    CHECK(yaw_max_drift < 10.0f);  // 整趟漂移 < 10°
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][fault_p0_estop] — P0 故障急停
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("P0 故障急停（FSM → Fault，电机停转）", "[hw_cycle][fault_p0_estop]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 启动清扫
    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 注入 P0 故障
    std::this_thread::sleep_for(2s);  // 让电机先转起来
    spdlog::warn("[hw_cycle][fault_p0_estop] 注入 P0 故障...");
    fx.fault->report(service::FaultService::FaultEvent::Level::P0,
                     0x9001u, "hw_test_inject_p0");

    // FSM 应进入 Fault
    REQUIRE(fx.wait_state("Fault", 3s));
    spdlog::info("[hw_cycle][fault_p0_estop] FSM → Fault ✓");

    // 等待电机停转（override 应已激活）
    std::this_thread::sleep_for(500ms);
    auto gd = fx.walk_group->get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_cycle][fault_p0_estop] wheel[{}] speed={:.2f}rpm online={}",
                     w, gd.wheel[w].speed_rpm, gd.wheel[w].online);

    CHECK(fx.fsm->current_state() == "Fault");
    // 电机速度应接近 0（允许 5 RPM 偏差，因惯性）
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        CHECK(std::abs(gd.wheel[w].speed_rpm) < 5.0f);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][low_battery_return] — 低电量触发返回
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("低电量触发安全返回", "[hw_cycle][low_battery_return]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 注入低电量事件
    std::this_thread::sleep_for(2s);
    spdlog::warn("[hw_cycle][low_battery_return] 注入低电量事件...");
    fx.fsm->dispatch(app::EvLowBattery{});

    // FSM → Returning
    REQUIRE(fx.wait_state("Returning", 3s));
    spdlog::info("[hw_cycle][low_battery_return] FSM → Returning ✓");

    // 等待后限位触发 → Charging（最多 kLimitTimeoutSec 秒）
    REQUIRE(fx.wait_state("Charging",
            std::chrono::seconds(hw::kLimitTimeoutSec)));
    spdlog::info("[hw_cycle][low_battery_return] FSM → Charging ✓ (已归停机位)");
    CHECK(fx.fsm->current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][bms_valid] — 清扫过程中 BMS 数据持续有效
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("清扫过程 BMS 数据持续有效", "[hw_cycle][bms_valid]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 启动 BMS 采集（500ms 轮询）
    std::atomic<bool> bms_running{true};
    int bms_valid_count = 0, bms_total_count = 0;
    std::thread bms_thread([&] {
        while (bms_running.load()) {
            fx.bms->update();
            auto d = fx.bms->get_data();
            ++bms_total_count;
            if (d.valid && d.soc_pct >= 0.0f && d.soc_pct <= 100.0f &&
                d.voltage_v > 0.0f)
                ++bms_valid_count;
            std::this_thread::sleep_for(500ms);
        }
    });

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});

    // 运行到 CleanFwd 后检查 BMS 持续有效 20s
    REQUIRE(fx.wait_state("CleanFwd", 5s));
    std::this_thread::sleep_for(20s);

    bms_running.store(false);
    bms_thread.join();

    spdlog::info("[hw_cycle][bms_valid] BMS valid={}/{} total polls",
                 bms_valid_count, bms_total_count);
    auto d = fx.bms->get_data();
    spdlog::info("[hw_cycle][bms_valid] 最终 soc={:.1f}% voltage={:.2f}V current={:.3f}A",
                 d.soc_pct, d.voltage_v, d.current_a);

    CHECK(d.valid);
    CHECK(d.soc_pct >= 0.0f);
    CHECK(d.soc_pct <= 100.0f);
    CHECK(d.voltage_v > 0.0f);
    // 至少 60% 的 BMS 轮询成功
    CHECK(bms_valid_count >= bms_total_count * 6 / 10);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][imu_valid] — 清扫过程中 IMU 数据持续有效
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("清扫过程 IMU 数据持续有效", "[hw_cycle][imu_valid]") {
    hw::FullSystemFixture fx(true /* pid_on，IMU 数据为 PID 必要条件 */);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 采样 IMU 20s
    int imu_valid_count = 0, imu_total = 0;
    float yaw_min = 999.0f, yaw_max = -999.0f;
    for (int i = 0; i < 40; ++i) {  // 40 × 500ms = 20s
        auto d = fx.imu->get_latest();
        ++imu_total;
        if (d.valid && !std::isnan(d.yaw_deg)) {
            ++imu_valid_count;
            yaw_min = std::min(yaw_min, d.yaw_deg);
            yaw_max = std::max(yaw_max, d.yaw_deg);
        }
        std::this_thread::sleep_for(500ms);
    }

    spdlog::info("[hw_cycle][imu_valid] IMU valid={}/{} yaw_range=[{:.2f},{:.2f}]°",
                 imu_valid_count, imu_total, yaw_min, yaw_max);

    // 至少 90% 的采样点 IMU 数据有效
    CHECK(imu_valid_count >= imu_total * 9 / 10);
    CHECK(yaw_max != -999.0f);  // 至少有一个有效值
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_cycle][watchdog_alive] — 正常运行 30s watchdog 不触发
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("正常运行 watchdog 不超时", "[hw_cycle][watchdog_alive]") {
    hw::FullSystemFixture fx(false);
    REQUIRE(fx.init());
    REQUIRE(fx.wait_state("Idle", 2s));

    // 注册 walk_ctrl 心跳（500ms 超时）
    bool watchdog_fired = false;
    fx.watchdog->set_timeout_callback([&](const std::string& name) {
        watchdog_fired = true;
        spdlog::error("[hw_cycle][watchdog_alive] watchdog 触发！name={}", name);
    });
    int wd_ticket = fx.watchdog->register_thread("walk_ctrl_test", 500);

    const bool at_home  = !fx.rear_sw->read_current_level();
    const bool at_front = !fx.front_sw->read_current_level();
    fx.fsm->dispatch(app::EvScheduleStart{at_home, at_front, 1.0f});
    REQUIRE(fx.wait_state("CleanFwd", 5s));

    // 正常运行 30s，每 200ms 汇报一次心跳
    for (int i = 0; i < 150; ++i) {  // 150 × 200ms = 30s
        fx.watchdog->heartbeat(wd_ticket);
        std::this_thread::sleep_for(200ms);
    }

    spdlog::info("[hw_cycle][watchdog_alive] 30s 正常运行，watchdog_fired={}",
                 watchdog_fired);
    CHECK(!watchdog_fired);
}
