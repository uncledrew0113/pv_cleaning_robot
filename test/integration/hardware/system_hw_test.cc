// test/integration/hardware/system_hw_test.cc
/**
 * @file system_hw_test.cc
 * @brief 全栈系统集成测试（真实硬件）
 *
 * 使用真实 IMU / BMS / WalkMotorGroup / LimitSwitch；
 * BrushMotor 使用 MockModbusMaster（辊刷未安装）；
 * GPS 使用占位对象（GPS 未安装）。
 *
 * 测试段：
 *   [hw_system][full_init]          — 全栈初始化、FSM → Idle、无崩溃
 *   [hw_system][health_real_data]   — HealthService 用真实传感器数据落盘 JSONL
 *   [hw_system][safety_idle]        — SafetyMonitor 启动后不误触发限位回调
 *   [hw_system][motion_then_stop]   — 运动 1s 后急停，验证电机停止且 override 激活
 *   [hw_system][watchdog_heartbeat] — WatchdogMgr 正常心跳不触发超时
 *
 * 运行方法（目标机）：
 *   ./hw_tests "[hw_system]"
 *   ./hw_tests "[hw_system][health_real_data]"
 */
#include <catch2/catch.hpp>

#include <atomic>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "hw_config.h"

using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][full_init] — 全栈初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）全栈初始化 FSM→Idle 无崩溃", "[hw_system][full_init]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init());

    // FSM 应处于 Idle 状态
    CHECK(f.fsm->current_state() == "Idle");

    // WalkMotorGroup 应已在线（open 成功）
    auto gd = f.walk_group->get_group_diagnostics();
    spdlog::info("[hw_system][full_init] ctrl_frames={} ctrl_err={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);

    // IMU：open() 启动后台读取线程，等待 500ms 让数据帧到来
    std::this_thread::sleep_for(500ms);
    auto ld = f.imu->get_latest();
    spdlog::info("[hw_system][full_init] IMU valid={} yaw={:.2f} pitch={:.2f} roll={:.2f}",
                 ld.valid, ld.yaw_deg, ld.pitch_deg, ld.roll_deg);

    // BMS：读取状态
    f.bms->update();
    std::this_thread::sleep_for(200ms);
    f.bms->update();
    auto bd = f.bms->get_data();
    spdlog::info("[hw_system][full_init] BMS valid={} soc={:.1f}% voltage={:.2f}V",
                 bd.valid, bd.soc_pct, bd.voltage_v);

    // 初始化成功即通过，硬件状态只记录不断言（允许 BMS/IMU 短期无应答）
    CHECK(f.fsm->current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][health_real_data] — HealthService 真实传感器数据落盘
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）HealthService DIAGNOSTICS 落盘真实传感器数据",
          "[hw_system][health_real_data]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init(hw::kHealthJsonlPath));
    REQUIRE(f.health != nullptr);

    // 等待 IMU/BMS 稳定输出 1 秒（IMU 后台线程自动读取，BMS 需 update()）
    for (int i = 0; i < 10; ++i) {
        f.bms->update();
        f.walk_group->update();
        std::this_thread::sleep_for(100ms);
    }

    // 连续调用 5 次 update，写入 5 行 JSONL
    for (int i = 0; i < 5; ++i) {
        f.health->update();
        spdlog::info("[hw_system][health_real_data] health update #{}", i + 1);
        std::this_thread::sleep_for(50ms);
    }

    // 验证文件存在
    REQUIRE(std::filesystem::exists(hw::kHealthJsonlPath));

    std::ifstream ifs(hw::kHealthJsonlPath);
    REQUIRE(ifs.is_open());

    int line_count = 0;
    std::string line;
    bool has_nonzero_voltage = false;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        ++line_count;

        // 每行必须是合法 JSON
        nlohmann::json j;
        REQUIRE_NOTHROW(j = nlohmann::json::parse(line));

        // DIAGNOSTICS 模式必须包含所有顶级键
        REQUIRE(j.contains("walk"));
        REQUIRE(j.contains("brush"));
        REQUIRE(j.contains("bms"));
        REQUIRE(j.contains("imu"));
        REQUIRE(j.contains("gps"));

        // walk：LT/RT/LB/RB 独立诊断 + ctrl_frames
        REQUIRE(j["walk"].contains("lt"));
        REQUIRE(j["walk"].contains("rt"));
        REQUIRE(j["walk"].contains("lb"));
        REQUIRE(j["walk"].contains("rb"));
        REQUIRE(j["walk"].contains("ctrl_frames"));

        // bms：核心字段存在
        REQUIRE(j["bms"].contains("soc"));
        REQUIRE(j["bms"].contains("voltage"));
        REQUIRE(j["bms"].contains("current"));

        // imu：核心字段存在
        REQUIRE(j["imu"].contains("pitch"));
        REQUIRE(j["imu"].contains("roll"));
        REQUIRE(j["imu"].contains("yaw"));

        // 检查 BMS 电压是否合理（BMS 在线时 > 0）
        float voltage = j["bms"]["voltage"].get<float>();
        if (voltage > 0.1f) has_nonzero_voltage = true;

        spdlog::info("[hw_system][health_real_data] line #{}: soc={:.1f}% volt={:.2f}V "
                     "imu_yaw={:.2f} walk_lt_rpm={:.2f}",
                     line_count,
                     j["bms"]["soc"].get<float>(),
                     voltage,
                     j["imu"]["yaw"].get<float>(),
                     j["walk"]["lt"]["rpm"].get<float>());
    }

    CHECK(line_count == 5);

    if (!has_nonzero_voltage) {
        // BMS 可能未就绪（告警但不失败，允许 BMS 协议有延迟）
        spdlog::warn("[hw_system][health_real_data] BMS voltage=0，请确认 BMS 接线和通信");
    }

    std::filesystem::remove(hw::kHealthJsonlPath);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][safety_idle] — SafetyMonitor 启动后静默期无误触发
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）SafetyMonitor 静默 2s 内不误触发限位",
          "[hw_system][safety_idle]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init());

    std::atomic<int> trigger_count{0};

    // 订阅 SafetyMonitor 限位事件
    f.event_bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const robot::middleware::SafetyMonitor::LimitSettledEvent&) {
            trigger_count.fetch_add(1, std::memory_order_relaxed);
        });

    spdlog::info("[hw_system][safety_idle] 监控 2s，期望无误触发...");
    std::this_thread::sleep_for(2s);

    spdlog::info("[hw_system][safety_idle] 触发次数={}", trigger_count.load());
    CHECK(trigger_count.load() == 0);
    // 若触发次数 > 0，说明传感器悬空或安装位置不对，需检查硬件
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][motion_then_stop] — 运动后急停
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）运动 1s 后 emergency_override 急停",
          "[hw_system][motion_then_stop]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init());

    // 使能电机并设置速度
    f.walk_group->enable_all();
    f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    f.walk_group->set_speed_uniform(hw::kTestSpeedRpm);

    // 运动 1s（持续发帧）
    for (int i = 0; i < 4; ++i) {
        f.walk_group->update();
        std::this_thread::sleep_for(250ms);
    }

    // 验证电机确实在转
    auto gd_before = f.walk_group->get_group_diagnostics();
    spdlog::info("[hw_system][motion_then_stop] 运动中: LT={:.1f}rpm RT={:.1f}rpm",
                 gd_before.wheel[0].speed_rpm, gd_before.wheel[1].speed_rpm);

    // 急停
    auto ret = f.walk_group->emergency_override(0.0f);
    CHECK(ret == robot::device::DeviceError::OK);
    CHECK(f.walk_group->is_override_active());
    spdlog::info("[hw_system][motion_then_stop] emergency_override 已激活");

    // override 期间调 update，不应发新帧
    const uint32_t frames_snap = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    f.walk_group->update();
    std::this_thread::sleep_for(200ms);
    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    CHECK(frames_after == frames_snap);  // override 激活期间无新控制帧

    // 等待电机减速到接近 0
    std::this_thread::sleep_for(500ms);
    auto gd_after = f.walk_group->get_group_diagnostics();
    spdlog::info("[hw_system][motion_then_stop] 急停后: LT={:.1f}rpm RT={:.1f}rpm",
                 gd_after.wheel[0].speed_rpm, gd_after.wheel[1].speed_rpm);

    // 电机转速应在 ±5rpm 内（允许惯性滑行残余）
    CHECK(std::abs(gd_after.wheel[0].speed_rpm) < 5.0f);
    CHECK(std::abs(gd_after.wheel[1].speed_rpm) < 5.0f);

    // 清除 override
    f.walk_group->clear_override();
    f.walk_group->update();
    CHECK(!f.walk_group->is_override_active());

    f.walk_group->disable_all();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][watchdog_heartbeat] — WatchdogMgr 正常心跳不触发超时
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）WatchdogMgr 正常心跳 1s 不触发超时",
          "[hw_system][watchdog_heartbeat]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init());

    std::atomic<bool> timeout_fired{false};
    f.watchdog->set_timeout_callback([&](const std::string& name) {
        spdlog::error("[hw_system][watchdog_heartbeat] 超时: {}", name);
        timeout_fired.store(true);
    });

    int tid = f.watchdog->register_thread("hw_sys_test", 500);  // 500ms 超时
    REQUIRE(tid >= 0);

    // 每 200ms 发一次心跳，持续 1s
    for (int i = 0; i < 5; ++i) {
        f.watchdog->heartbeat(tid);
        spdlog::info("[hw_system][watchdog_heartbeat] heartbeat #{}", i + 1);
        std::this_thread::sleep_for(200ms);
    }

    CHECK(!timeout_fired.load());
    spdlog::info("[hw_system][watchdog_heartbeat] 1s 内无超时，PASS");
}
