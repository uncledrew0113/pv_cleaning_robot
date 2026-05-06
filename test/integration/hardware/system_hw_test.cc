// test/integration/hardware/system_hw_test.cc
/**
 * @file system_hw_test.cc
 * @brief 全栈系统集成测试（真实硬件）
 *
 * 使用真实 IMU / BMS / WalkMotorGroup / LimitSwitch；
 * BrushMotor 使用 MockModbusMaster（滚刷未安装）；
 * GPS 使用真实 gpsd TCP 数据源（打开失败时仅记录 warning，不阻塞其余硬件用例）。
 *
 * 测试段：
 *   [hw_system][full_init]          — 全栈初始化、FSM → Idle、无崩溃
 *   [hw_system][health_real_data]   — HealthService 用真实传感器数据落盘 JSONL
 *   [hw_system][safety_idle]        — SafetyMonitor 启动后不误触发限位回调
 *   [hw_system][motion_then_stop]   — 运动 1s 后急停，验证电机停止且 override 激活
 *   [hw_system][watchdog_heartbeat] — WatchdogMgr 正常心跳不触发超时
 *   [hw_system][combined]           — N 趟完整任务链 + 全程持续采集健康数据
 *   [hw_system][pid_combined]       — N 趟完整任务链 + PID 控制 + yaw 指标采集到 pid_metrics.jsonl
 *   [hw_system][imu_gps_health_only]— 仅 IMU/GPS 持续采集，并由 HealthService 本地落盘
 *
 * 运行方法（目标机）：
 *   ./hw_tests "[hw_system]"
 *   ./hw_tests "[hw_system][health_real_data]"
 *   ./hw_tests "[hw_system][pid_combined]"
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

#include "hw_config.h"

using namespace std::chrono_literals;

namespace {

rapidjson::Document parse_json_line(const std::string& line)
{
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

std::vector<std::filesystem::path> collect_rotated_health_logs(const std::string& base_path)
{
    std::vector<std::filesystem::path> paths;
    const auto base = std::filesystem::path(base_path);
    const auto dir = base.parent_path().empty() ? std::filesystem::path(".") : base.parent_path();
    const auto filename = base.filename().string();

    if (!std::filesystem::exists(dir)) {
        return paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto candidate = entry.path().filename().string();
        if (candidate == filename || candidate.rfind(filename + ".", 0) == 0) {
            paths.push_back(entry.path());
        }
    }

    std::sort(paths.begin(), paths.end());
    return paths;
}

void remove_rotated_health_logs(const std::string& base_path)
{
    for (const auto& path : collect_rotated_health_logs(base_path)) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
}

std::string build_pid_sample_json(int64_t ts_ms,
                                  int seg,
                                  const std::string& state,
                                  float yaw,
                                  float omega_z)
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("ts_ms");
    writer.Int64(ts_ms);
    writer.Key("seg");
    writer.Int(seg);
    writer.Key("state");
    writer.String(state.c_str(), static_cast<rapidjson::SizeType>(state.size()));
    writer.Key("yaw");
    writer.Double(yaw);
    writer.Key("omega_z");
    writer.Double(omega_z);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_segment_summary_json(int seg,
                                       const char* direction,
                                       const std::string& from_state,
                                       const std::string& to_state,
                                       float max_drift_deg,
                                       float duration_s)
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("type");
    writer.String("segment_summary");
    writer.Key("seg");
    writer.Int(seg);
    writer.Key("direction");
    writer.String(direction);
    writer.Key("from_state");
    writer.String(from_state.c_str(), static_cast<rapidjson::SizeType>(from_state.size()));
    writer.Key("to_state");
    writer.String(to_state.c_str(), static_cast<rapidjson::SizeType>(to_state.size()));
    writer.Key("max_drift_deg");
    writer.Double(max_drift_deg);
    writer.Key("duration_s");
    writer.Double(duration_s);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_final_summary_json(int total_segs,
                                     int total_records,
                                     float max_drift_all_deg,
                                     float kp,
                                     float ki,
                                     float kd,
                                     float deadband_rate_dps)
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("type");
    writer.String("final_summary");
    writer.Key("total_segs");
    writer.Int(total_segs);
    writer.Key("total_records");
    writer.Int(total_records);
    writer.Key("max_drift_all_deg");
    writer.Double(max_drift_all_deg);
    writer.Key("kp");
    writer.Double(kp);
    writer.Key("ki");
    writer.Double(ki);
    writer.Key("kd");
    writer.Double(kd);
    writer.Key("deadband_rate_dps");
    writer.Double(deadband_rate_dps);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][imu_gps_health_only] — 仅 IMU/GPS 持续采集并由 HealthService 落盘
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）仅 IMU/GPS 持续采集并本地落盘",
          "[hw_system][imu_gps_health_only]") {
    hw::ImuGpsHealthFixture f;
    const std::string health_path = "/data/pv_robot/logs/hw_imu_gps_health_only.jsonl";
    remove_rotated_health_logs(health_path);

    REQUIRE(f.init(health_path));
    REQUIRE(f.health != nullptr);

    for (int i = 0; i < 20; ++i) {
        f.health->update();
        std::this_thread::sleep_for(100ms);
    }

    const auto paths = collect_rotated_health_logs(health_path);
    REQUIRE_FALSE(paths.empty());

    int line_count = 0;
    bool saw_imu = false;
    bool saw_gps = false;

    for (const auto& path : paths) {
        std::ifstream ifs(path);
        REQUIRE(ifs.is_open());

        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) {
                continue;
            }
            ++line_count;
            auto j = parse_json_line(line);
            REQUIRE(j.IsObject());
            REQUIRE(j.HasMember("imu"));
            REQUIRE(j.HasMember("gps"));
            saw_imu = true;
            saw_gps = true;
        }
    }

    CHECK(line_count >= 10);
    CHECK(saw_imu);
    CHECK(saw_gps);
}

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
                 gd.ctrl_frame_count,
                 gd.ctrl_err_count);

    // IMU：open() 启动后台读取线程，等待 500ms 让数据帧到来
    std::this_thread::sleep_for(500ms);
    auto ld = f.imu->get_latest();
    spdlog::info("[hw_system][full_init] IMU valid={} yaw={:.2f} pitch={:.2f} roll={:.2f}",
                 ld.valid,
                 ld.yaw_deg,
                 ld.pitch_deg,
                 ld.roll_deg);

    // BMS：读取状态
    f.bms->update();
    std::this_thread::sleep_for(200ms);
    f.bms->update();
    auto bd = f.bms->get_data();
    spdlog::info("[hw_system][full_init] BMS valid={} soc={:.1f}% voltage={:.2f}V",
                 bd.valid,
                 bd.soc_pct,
                 bd.voltage_v);

    // 初始化成功即通过，硬件状态只记录不断言（允许 BMS/IMU 短期无应答）
    CHECK(f.fsm->current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][health_real_data] — HealthService 真实传感器数据落盘
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）HealthService DIAGNOSTICS 落盘真实传感器数据",
          "[hw_system][health_real_data]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init(f.p.health_jsonl_path));
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
    REQUIRE(std::filesystem::exists(f.p.health_jsonl_path));

    std::ifstream ifs(f.p.health_jsonl_path);
    REQUIRE(ifs.is_open());

    int line_count = 0;
    std::string line;
    bool has_nonzero_voltage = false;

    while (std::getline(ifs, line)) {
        if (line.empty())
            continue;
        ++line_count;

        // 每行必须是合法 JSON
        auto j = parse_json_line(line);

        // DIAGNOSTICS 模式必须包含所有顶级键
        REQUIRE(j.HasMember("walk"));
        REQUIRE(j.HasMember("brush"));
        REQUIRE(j.HasMember("bms"));
        REQUIRE(j.HasMember("imu"));
        REQUIRE(j.HasMember("gps"));

        // walk：LT/RT/LB/RB 独立诊断 + ctrl_frames
        REQUIRE(j["walk"].HasMember("lt"));
        REQUIRE(j["walk"].HasMember("rt"));
        REQUIRE(j["walk"].HasMember("lb"));
        REQUIRE(j["walk"].HasMember("rb"));
        REQUIRE(j["walk"].HasMember("ctrl_frames"));

        // bms：核心字段存在
        REQUIRE(j["bms"].HasMember("soc"));
        REQUIRE(j["bms"].HasMember("voltage"));
        REQUIRE(j["bms"].HasMember("current"));

        // imu：核心字段存在
        REQUIRE(j["imu"].HasMember("pitch"));
        REQUIRE(j["imu"].HasMember("roll"));
        REQUIRE(j["imu"].HasMember("yaw"));

        // 检查 BMS 电压是否合理（BMS 在线时 > 0）
        float voltage = j["bms"]["voltage"].GetFloat();
        if (voltage > 0.1f)
            has_nonzero_voltage = true;

        spdlog::info(
            "[hw_system][health_real_data] line #{}: soc={:.1f}% volt={:.2f}V "
            "imu_yaw={:.2f} walk_lt_rpm={:.2f}",
            line_count,
            j["bms"]["soc"].GetFloat(),
            voltage,
            j["imu"]["yaw"].GetFloat(),
            j["walk"]["lt"]["rpm"].GetFloat());
    }

    CHECK(line_count == 5);

    if (!has_nonzero_voltage) {
        // BMS 可能未就绪（告警但不失败，允许 BMS 协议有延迟）
        spdlog::warn("[hw_system][health_real_data] BMS voltage=0，请确认 BMS 接线和通信");
    }

    std::filesystem::remove(f.p.health_jsonl_path);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][safety_idle] — SafetyMonitor 启动后静默期无误触发
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）SafetyMonitor 静默 2s 内不误触发限位", "[hw_system][safety_idle]") {
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
TEST_CASE("System（真实硬件）运动 1s 后 emergency_override 急停", "[hw_system][motion_then_stop]") {
    hw::FullSystemFixture f;
    REQUIRE(f.init());

    // 使能电机并设置速度
    f.walk_group->enable_all();
    f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    f.walk_group->set_speed_uniform(f.p.test_speed_rpm);

    // 运动 1s（持续发帧）
    for (int i = 0; i < 4; ++i) {
        f.walk_group->update();
        std::this_thread::sleep_for(250ms);
    }

    // 验证电机确实在转
    auto gd_before = f.walk_group->get_group_diagnostics();
    spdlog::info("[hw_system][motion_then_stop] 运动中: LT={:.1f}rpm RT={:.1f}rpm",
                 gd_before.wheel[0].speed_rpm,
                 gd_before.wheel[1].speed_rpm);

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
                 gd_after.wheel[0].speed_rpm,
                 gd_after.wheel[1].speed_rpm);

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
// [hw_system][watchdog_timeout] — WatchdogMgr 漏心跳超时（纯软件）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）WatchdogMgr 漏心跳后超时回调触发", "[hw_system][watchdog_timeout]") {
    robot::app::WatchdogMgr watchdog{""};  // 不操作 /dev/watchdog

    std::string fired_name;
    watchdog.set_timeout_callback([&](const std::string& name) { fired_name = name; });

    REQUIRE(watchdog.start());
    // 注册 50ms 超时，从不喂狗
    watchdog.register_thread("dead_thread", 50);

    // 等待 350ms（远超 50ms 超时 + 200ms 监控周期）
    std::this_thread::sleep_for(350ms);
    watchdog.stop();

    REQUIRE(fired_name == "dead_thread");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][p0_fault_chain] — P0 故障链（真实电机停止 + FSM Fault → Idle）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）P0 故障链：电机急停 + FSM Fault → 复位 → Idle",
          "[hw_system][p0_fault_chain]") {
    hw::FullSystemFixture f;
    spdlog::warn("[hw_system][p0_fault_chain] ⚠ 此测试将短暂启动行走电机（{:.0f}rpm），确保安全！",
                 f.p.test_speed_rpm);
    REQUIRE(f.init());
    REQUIRE(f.fsm->current_state() == "Idle");

    // 启动任务 → CleanFwd，电机开始运动
    f.fsm->dispatch(robot::app::EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    // 等待 300ms 让电机加速
    std::this_thread::sleep_for(300ms);
    {
        auto gd = f.walk_group->get_group_diagnostics();
        spdlog::info("[hw_system][p0_fault_chain] 运动中: LT={:.1f} RT={:.1f}rpm",
                     gd.wheel[0].speed_rpm,
                     gd.wheel[1].speed_rpm);
    }

    // 注入 P0 故障 → FaultHandler: emergency_stop + dispatch EvFaultP0 → Fault
    f.fault->report(
        robot::service::FaultService::FaultEvent::Level::P0, 0x1001, "CAN_comm_lost [hw_test]");

    REQUIRE(f.fsm->current_state() == "Fault");
    REQUIRE(!f.dispatched_faults.empty());
    CHECK(f.dispatched_faults[0].level == robot::service::FaultService::FaultEvent::Level::P0);
    CHECK(f.dispatched_faults[0].code == 0x1001u);

    // 等待电机减速到 0
    std::this_thread::sleep_for(600ms);
    auto gd = f.walk_group->get_group_diagnostics();
    spdlog::info("[hw_system][p0_fault_chain] 故障后: LT={:.1f} RT={:.1f}rpm",
                 gd.wheel[0].speed_rpm,
                 gd.wheel[1].speed_rpm);
    CHECK(std::abs(gd.wheel[0].speed_rpm) < 5.0f);
    CHECK(std::abs(gd.wheel[1].speed_rpm) < 5.0f);

    // 人工复位 → Idle
    f.fsm->dispatch(robot::app::EvFaultReset{});
    REQUIRE(f.fsm->current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][p1_fault_chain] — P1 故障链 → Returning → Charging
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）P1 故障链：FSM Returning → EvRearLimitSettled → Charging",
          "[hw_system][p1_fault_chain]") {
    spdlog::warn("[hw_system][p1_fault_chain] ⚠ 此测试将短暂启动电机，确保安全！");

    hw::FullSystemFixture f;
    REQUIRE(f.init());

    f.fsm->dispatch(robot::app::EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    // 等待 200ms 让电机建立速度
    std::this_thread::sleep_for(200ms);
    {
        auto gd = f.walk_group->get_group_diagnostics();
        spdlog::info("[hw_system][p1_fault_chain] 运动中: LT={:.1f} RT={:.1f}rpm",
                     gd.wheel[0].speed_rpm,
                     gd.wheel[1].speed_rpm);
    }

    // 注入 P1 → FaultHandler: stop_cleaning + start_returning + dispatch EvFaultP1
    f.fault->report(
        robot::service::FaultService::FaultEvent::Level::P1, 0x2001, "slope_too_steep [hw_test]");

    REQUIRE(f.fsm->current_state() == "Returning");
    REQUIRE(!f.dispatched_faults.empty());
    CHECK(f.dispatched_faults[0].level == robot::service::FaultService::FaultEvent::Level::P1);

    // P1 测试重点在故障链逻辑，直接 dispatch 尾端事件（不等真实物理运动）
    f.fsm->dispatch(robot::app::EvRearLimitSettled{});
    REQUIRE(f.fsm->current_state() == "Charging");

    spdlog::info("[hw_system][p1_fault_chain] PASS: CleanFwd→Returning→Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][low_battery] — EvLowBattery → Returning → Charging
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）EvLowBattery → Returning → Charging", "[hw_system][low_battery]") {
    spdlog::warn("[hw_system][low_battery] ⚠ 此测试将短暂启动电机，确保安全！");

    hw::FullSystemFixture f;
    REQUIRE(f.init());

    f.fsm->dispatch(robot::app::EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    std::this_thread::sleep_for(200ms);

    // 真实 BMS SOC 值（仅记录，不用于控制流）
    f.bms->update();
    auto bd = f.bms->get_data();
    spdlog::info(
        "[hw_system][low_battery] 真实 BMS SOC={:.1f}% voltage={:.2f}V", bd.soc_pct, bd.voltage_v);

    f.fsm->dispatch(robot::app::EvLowBattery{});
    REQUIRE(f.fsm->current_state() == "Returning");

    f.fsm->dispatch(robot::app::EvRearLimitSettled{});
    REQUIRE(f.fsm->current_state() == "Charging");

    spdlog::info("[hw_system][low_battery] PASS: CleanFwd→Returning→Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][n1_clean_cycle] — N=1 完整任务链（手动触发真实前后限位）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N=1 完整任务链（真实限位触发）", "[hw_system][n1_clean_cycle]") {
    spdlog::warn("[hw_system][n1_clean_cycle] ====================================");
    spdlog::warn("[hw_system][n1_clean_cycle] ⚠ 机器人将完整运动一个来回（N=1）！");
    spdlog::warn("[hw_system][n1_clean_cycle] 请确保：导轨就位，轨道无人员/障碍物");
    spdlog::warn("[hw_system][n1_clean_cycle] ====================================");

    hw::FullSystemFixture f;
    REQUIRE(f.init());

    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;

    // 启动 N=1 任务 → CleanFwd，电机开始正向运动
    f.fsm->dispatch(robot::app::EvScheduleStart{.at_home = true, .passes = 5.0f});
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    spdlog::warn(
        "[hw_system][n1_clean_cycle] ★ 机器人正在向前运动，等待【前端限位】触发（最多 {}s）...",
        f.p.limit_timeout_sec);

    // 等待 SafetyMonitor → EventBus → FSM CleanReturn（真实限位触发）
    const bool front_hit =
        f.wait_state("CleanReturn", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        INFO("前端限位等待超时，请检查导轨/传感器接线");
        REQUIRE(front_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ 前端限位触发，机器人开始返回");

    spdlog::warn(
        "[hw_system][n1_clean_cycle] ★ 机器人正在返回，等待【尾端限位】触发（最多 {}s）...",
        f.p.limit_timeout_sec);

    // 等待尾端限位触发 → Charging（任务完成）
    const bool rear_hit =
        f.wait_state("Charging", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        INFO("尾端限位等待超时，请检查导轨/传感器接线");
        REQUIRE(rear_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ 尾端限位触发，任务完成");

    // 验证任务期间产生了足够的 CAN 控制帧
    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_system][n1_clean_cycle] CAN 控制帧: {} → {} (增量={})",
                 frames_before,
                 frames_after,
                 frames_after - frames_before);
    CHECK(frames_after > frames_before + 20u);  // 两段运动至少 20 帧

    // 任务期间无异常故障
    CHECK(f.dispatched_faults.empty());
    REQUIRE(f.fsm->current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][combined] — 全系统联合启动（电机 + HealthService + WatchdogMgr）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N 趟完整任务链 + 全程持续采集健康数据", "[hw_system][combined]") {
    hw::FullSystemFixture f;
    const int passes_int = static_cast<int>(f.p.combined_passes);
    const double passes_half = f.p.combined_passes * 2.0;
    spdlog::warn("[hw_system][combined] ====================================");
    spdlog::warn("[hw_system][combined] ⚠ 机器人将运动 {:.1f} 趟（{} 段）！",
                 f.p.combined_passes,
                 static_cast<int>(passes_half));
    spdlog::warn("[hw_system][combined] 全程持续记录健康数据到: {}", f.p.health_jsonl_path);
    spdlog::warn("[hw_system][combined] 本地日志轮转: max_bytes={} max_files={}",
                 f.p.health_log_max_bytes,
                 f.p.health_log_max_files);
    spdlog::warn("[hw_system][combined] 每段限位超时: {}s（触发后重置）", f.p.limit_timeout_sec);
    spdlog::warn("[hw_system][combined] 请确保：导轨就位，轨道无人员/障碍物");
    spdlog::warn("[hw_system][combined] ====================================");
    (void)passes_int;
    REQUIRE(f.init(f.p.health_jsonl_path));
    REQUIRE(f.health != nullptr);

    // 注册看门狗（超时 = kLimitTimeoutSec * 2，每次 poll 喂狗）
    std::atomic<bool> wd_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& n) {
        spdlog::error("[hw_system][combined] 看门狗超时: {}", n);
        wd_timeout.store(true);
    });
    const int wd_tid = f.watchdog->register_thread("hw_combined", f.p.limit_timeout_sec * 2 * 1000);
    REQUIRE(wd_tid >= 0);

    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    int total_health_records = 0;

    // 每次调用：刷新 BMS + 写 JSONL + 喂狗 + 打印当前状态
    auto poll_once = [&]() {
        f.bms->update();
        f.health->update();
        f.watchdog->heartbeat(wd_tid);
        ++total_health_records;
        auto gd = f.walk_group->get_group_diagnostics();
        auto ld = f.imu->get_latest();
        spdlog::info(
            "[hw_system][combined] #{}: LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f} rpm "
            "yaw={:.2f}° state={}",
            total_health_records,
            gd.wheel[0].speed_rpm,
            gd.wheel[1].speed_rpm,
            gd.wheel[2].speed_rpm,
            gd.wheel[3].speed_rpm,
            ld.yaw_deg,
            f.fsm->current_state());
    };

    // ── 等待函数：从 from 状态等待切换到任意其他状态（每段独立 60s 超时）──
    // 返回切换后的新状态；若超时返回 from（未变）
    auto wait_transition = [&](const std::string& from) -> std::string {
        using clock = std::chrono::steady_clock;
        auto deadline = clock::now() + std::chrono::seconds(f.p.limit_timeout_sec);
        while (clock::now() < deadline) {
            std::string curr = f.fsm->current_state();
            if (curr != from)
                return curr;
            poll_once();
            std::this_thread::sleep_for(5ms);
        }
        return from;  // 超时：状态未变
    };

    // ── 启动任务，进入 CleanFwd ─────────────────────────────────────────────
    f.fsm->dispatch(robot::app::EvScheduleStart{.at_home = true, .passes = f.p.combined_passes});
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    // ── 逐段等待，每触发一次限位重置 60s 计时 ──────────────────────────────
    std::string state = "CleanFwd";
    int seg_idx = 0;

    while (state != "Charging") {
        ++seg_idx;
        const bool going_fwd = (state == "CleanFwd");
        spdlog::warn("[hw_system][combined] 段 {}: {} → 等待{}限位（最多 {}s）...",
                     seg_idx,
                     state,
                     going_fwd ? "【前端】" : "【尾端】",
                     f.p.limit_timeout_sec);

        std::string next = wait_transition(state);

        if (next == state) {
            // 超时，限位未触发
            spdlog::error("[hw_system][combined] 段 {} 超时 {}s，当前状态仍为 {}",
                          seg_idx,
                          f.p.limit_timeout_sec,
                          state);
            INFO("限位等待超时，请检查导轨/传感器接线");
            REQUIRE(next != state);  // 明确失败
            break;
        }

        spdlog::info("[hw_system][combined] ✓ 段 {} 完成：{} → {}（已采集 {} 条）",
                     seg_idx,
                     state,
                     next,
                     total_health_records);
        poll_once();  // 在状态切换点额外采集一条
        state = next;
    }

    // ── 任务完成断言 ────────────────────────────────────────────────────────
    {
        INFO("任务未能到达 Charging 状态");
        REQUIRE(state == "Charging");
    }
    spdlog::info(
        "[hw_system][combined] ✓ 全部 {} 段完成，总采集 {} 条", seg_idx, total_health_records);

    // 看门狗全程无超时
    CHECK(!wd_timeout.load());

    // CAN 控制帧增加（MotionService 全程在发帧）
    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_system][combined] CAN 帧: {} → {} (增量={})",
                 frames_before,
                 frames_after,
                 frames_after - frames_before);
    CHECK(frames_after > frames_before + 10u * static_cast<uint32_t>(passes_half));

    // 无故障
    CHECK(f.dispatched_faults.empty());

    // 本地健康日志存在；若启用了轮转，则允许出现 base + .1/.2...
    REQUIRE(std::filesystem::exists(f.p.health_jsonl_path));
    {
        const auto health_files = collect_rotated_health_logs(f.p.health_jsonl_path);
        REQUIRE_FALSE(health_files.empty());

        int total_retained_lines = 0;
        for (const auto& path : health_files) {
            std::ifstream ifs(path);
            REQUIRE(ifs.is_open());

            std::string line;
            while (std::getline(ifs, line)) {
                if (line.empty()) {
                    continue;
                }
                ++total_retained_lines;
                auto j = parse_json_line(line);
                REQUIRE(j.HasMember("ts"));  // 时间戳
                REQUIRE(j.HasMember("walk"));
                REQUIRE(j.HasMember("bms"));
                REQUIRE(j.HasMember("imu"));
                CHECK(!std::string(j["ts"].GetString()).empty());
            }
        }

        // 未发生轮转时，保留行数应等于本次采样条数。
        // 发生轮转时，只要求保留文件中仍有合法 JSONL 记录。
        CHECK(total_retained_lines > 0);
        CHECK(total_retained_lines <= total_health_records);
        spdlog::info(
            "[hw_system][combined] 保留健康日志 {} 个文件，共 {} 行，主路径: {}",
            health_files.size(),
            total_retained_lines,
            f.p.health_jsonl_path);
    }

    // IMU 末帧（仅记录）
    auto ld = f.imu->get_latest();
    spdlog::info("[hw_system][combined] 末帧 IMU: valid={} pitch={:.2f}° roll={:.2f}° yaw={:.2f}°",
                 ld.valid,
                 ld.pitch_deg,
                 ld.roll_deg,
                 ld.yaw_deg);

    // ⚠ 不删除 JSONL，供事后分析
    spdlog::info("[hw_system][combined] PASS — 健康数据已保存至 {}", f.p.health_jsonl_path);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][pid_combined] — N 趟完整任务链 + 航向 PID + 全程 yaw 指标采集
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N 趟完整任务链 + PID 控制 + yaw 指标采集",
          "[hw_system][pid_combined]") {
    hw::FullSystemFixture f(true /* pid_on */);
    const double passes_half = f.p.combined_passes * 2.0;
    spdlog::warn("[hw_system][pid_combined] ====================================");
    spdlog::warn("[hw_system][pid_combined] ⚠ 机器人将运动 {:.1f} 趟（{} 段）！",
                 f.p.combined_passes,
                 static_cast<int>(passes_half));
    spdlog::warn("[hw_system][pid_combined] PID kp={:.2f} ki={:.3f} kd={:.3f} deadband={:.1f}°/s",
                 f.p.pid.kp,
                 f.p.pid.ki,
                 f.p.pid.kd,
                 f.p.pid.deadband_rate_dps);
    spdlog::warn("[hw_system][pid_combined] 健康数据: {}", f.p.health_jsonl_path);
    spdlog::warn("[hw_system][pid_combined] PID 指标: {}", f.p.pid_jsonl_path);
    spdlog::warn("[hw_system][pid_combined] 最大漂移警告阈值: {:.1f}°", f.p.pid_max_drift_deg);
    spdlog::warn("[hw_system][pid_combined] 请确保：导轨就位，轨道无人员/障碍物");
    spdlog::warn("[hw_system][pid_combined] ====================================");

    REQUIRE(f.init(f.p.health_jsonl_path));
    REQUIRE(f.health != nullptr);

    // 打开 pid_metrics.jsonl（测试结束不删除，供离线分析）
    std::ofstream pid_ofs(f.p.pid_jsonl_path, std::ios::trunc);
    REQUIRE(pid_ofs.is_open());

    // 看门狗（超时 = 每段最大秒数 × 2）
    std::atomic<bool> wd_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& n) {
        spdlog::error("[hw_system][pid_combined] 看门狗超时: {}", n);
        wd_timeout.store(true);
    });
    const int wd_tid =
        f.watchdog->register_thread("hw_pid_combined", f.p.limit_timeout_sec * 2 * 1000);
    REQUIRE(wd_tid >= 0);

    // ── 读取真实传感器状态（修复 combined 测试 at_home 硬编码 bug）──────────
    const bool at_home = !f.rear_sw->read_current_level();
    const bool at_front = !f.front_sw->read_current_level();
    {
        INFO("设备不在已知端点（at_home=false, at_front=false），请将设备移至停机位或前端");
        REQUIRE((at_home || at_front));
    }
    spdlog::info("[hw_system][pid_combined] 传感器: at_home={} at_front={}", at_home, at_front);

    // 记录出发时的 target_yaw（PID 将锁定此航向）
    const float target_yaw = f.imu->get_latest().yaw_deg;
    spdlog::info("[hw_system][pid_combined] target_yaw={:.2f}°", target_yaw);

    using clock = std::chrono::steady_clock;
    const auto t_test_start = clock::now();
    int total_health_records = 0;
    float max_drift_all = 0.0f;  // 全程最大 yaw 漂移（绝对值）

    // norm_angle helper（−180 ~ +180）
    auto norm_angle = [](float deg) -> float {
        while (deg > 180.0f)
            deg -= 360.0f;
        while (deg < -180.0f)
            deg += 360.0f;
        return deg;
    };

    // 当前段号（写入每条 JSONL 记录）
    int cur_seg = 0;

    // 每次调用：刷新 BMS + 写健康 JSONL + 喂狗 + 写 pid_metrics JSONL
    auto poll_once = [&]() {
        f.bms->update();
        f.health->update();
        f.watchdog->heartbeat(wd_tid);
        ++total_health_records;

        auto gd = f.walk_group->get_group_diagnostics();
        auto imu = f.imu->get_latest();
        const float yaw = imu.yaw_deg;
        const float omega_z_dps = imu.gyro[2] * (180.0f / 3.14159265f);
        const float yaw_err = norm_angle(target_yaw - yaw);
        const float drift = std::abs(yaw_err);
        if (drift > max_drift_all)
            max_drift_all = drift;

        const int64_t ts_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t_test_start)
                .count();

        pid_ofs << build_pid_sample_json(
                       ts_ms,
                       cur_seg,
                       f.fsm->current_state(),
                       std::round(yaw * 100.0f) / 100.0f,
                       std::round(omega_z_dps * 100.0f) / 100.0f)
                << '\n';

        spdlog::info(
            "[hw_system][pid_combined] #{} seg={} yaw={:.2f}° omega_z={:.2f}°/s "
            "LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f}rpm state={}",
            total_health_records,
            cur_seg,
            yaw,
            omega_z_dps,
            gd.wheel[0].speed_rpm,
            gd.wheel[1].speed_rpm,
            gd.wheel[2].speed_rpm,
            gd.wheel[3].speed_rpm,
            f.fsm->current_state());
    };

    // 等待 FSM 从 from 状态切换到其他状态（每段独立超时）
    auto wait_transition = [&](const std::string& from) -> std::string {
        auto deadline = clock::now() + std::chrono::seconds(f.p.limit_timeout_sec);
        while (clock::now() < deadline) {
            std::string curr = f.fsm->current_state();
            if (curr != from)
                return curr;
            poll_once();
            std::this_thread::sleep_for(5ms);
        }
        return from;  // 超时：状态未变
    };

    // ── 启动任务 ────────────────────────────────────────────────────────────
    f.fsm->dispatch(robot::app::EvScheduleStart{
        .at_home = at_home, .at_front = at_front, .passes = f.p.combined_passes});
    {
        const std::string s = f.fsm->current_state();
        INFO("FSM 未能进入 CleanFwd/CleanReturn，请检查传感器与 FSM 逻辑");
        REQUIRE((s == "CleanFwd" || s == "CleanReturn"));
    }

    // ── 逐段等待限位，直到到达 Charging ─────────────────────────────────────
    std::string state = f.fsm->current_state();
    int seg_idx = 0;

    while (state != "Charging") {
        ++seg_idx;
        cur_seg = seg_idx;
        const bool going_fwd = (state == "CleanFwd");
        const auto seg_start = clock::now();
        spdlog::warn("[hw_system][pid_combined] 段 {}: {} → 等待{}限位（最多 {}s）...",
                     seg_idx,
                     state,
                     going_fwd ? "【前端】" : "【尾端】",
                     f.p.limit_timeout_sec);

        std::string next = wait_transition(state);

        if (next == state) {
            spdlog::error("[hw_system][pid_combined] 段 {} 超时 {}s，当前状态仍为 {}",
                          seg_idx,
                          f.p.limit_timeout_sec,
                          state);
            INFO("限位等待超时，请检查导轨/传感器接线");
            REQUIRE(next != state);
            break;
        }

        // 写段摘要到 pid_metrics.jsonl
        const float seg_dur = std::chrono::duration<float>(clock::now() - seg_start).count();
        pid_ofs << build_segment_summary_json(
                       seg_idx,
                       going_fwd ? "fwd" : "ret",
                       state,
                       next,
                       std::round(max_drift_all * 100.0f) / 100.0f,
                       std::round(seg_dur * 10.0f) / 10.0f)
                << '\n';

        spdlog::info(
            "[hw_system][pid_combined] ✓ 段 {} 完成：{} → {}（已采集 {} 条，耗时 {:.1f}s）",
            seg_idx,
            state,
            next,
            total_health_records,
            seg_dur);
        poll_once();
        state = next;
    }

    // ── 任务完成断言 ─────────────────────────────────────────────────────────
    {
        INFO("任务未能到达 Charging 状态");
        REQUIRE(state == "Charging");
    }

    // 写全局摘要到 pid_metrics.jsonl
    pid_ofs << build_final_summary_json(
                   seg_idx,
                   total_health_records,
                   std::round(max_drift_all * 100.0f) / 100.0f,
                   f.p.pid.kp,
                   f.p.pid.ki,
                   f.p.pid.kd,
                   f.p.pid.deadband_rate_dps)
            << '\n';
    pid_ofs.close();

    spdlog::info(
        "[hw_system][pid_combined] ✓ 全部 {} 段完成，总采集 {} 条", seg_idx, total_health_records);
    spdlog::info("[hw_system][pid_combined] 全程最大 yaw 漂移={:.2f}°", max_drift_all);

    // 看门狗全程无超时
    CHECK(!wd_timeout.load());
    // 无故障
    CHECK(f.dispatched_faults.empty());
    // yaw 漂移 CHECK（不强制 REQUIRE，允许 PID 参数不理想时继续记录）
    if (max_drift_all >= f.p.pid_max_drift_deg) {
        spdlog::warn(
            "[hw_system][pid_combined] ⚠ 最大漂移 {:.2f}° ≥ 阈值 {:.1f}°，建议调整 PID 参数",
            max_drift_all,
            f.p.pid_max_drift_deg);
    }
    CHECK(max_drift_all < f.p.pid_max_drift_deg);

    // pid_metrics.jsonl 存在且有内容
    REQUIRE(std::filesystem::exists(f.p.pid_jsonl_path));
    spdlog::info("[hw_system][pid_combined] PASS — PID 指标已保存至 {}", f.p.pid_jsonl_path);
    spdlog::info("[hw_system][pid_combined] PASS — 健康数据已保存至 {}", f.p.health_jsonl_path);
}

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
