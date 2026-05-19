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
 *   [hw_system][lower_pitch_peak]   — 上轮停止，下轮正反扫动寻找 pitch 最大点
 *   [hw_system][watchdog_heartbeat] — WatchdogMgr 正常心跳不触发超时
 *   [hw_system][combined]           — N 趟完整任务链 + 全程持续采集健康数据
 *   [hw_system][combined_nvm_real]  — N 趟完整任务链 + 全程打印融合里程计日志
 *   [hw_system][imu_gps_health_only]— 仅 IMU/GPS 持续采集，并由 HealthService 本地落盘
 *
 * 运行方法（目标机）：
 *   ./hw_tests "[hw_system]"
 *   ./hw_tests "[hw_system][health_real_data]"
 *   ./hw_tests "[hw_system][lower_pitch_peak]"
 */
#include <algorithm>
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>
#include <utility>

#include "hw_config.h"

using namespace std::chrono_literals;

namespace {

bool real_tb_test_enabled() {
    const char* value = std::getenv("TB_REAL_TEST");
    if (!value) {
        return false;
    }
    const std::string env_value(value);
    return !(env_value.empty() || env_value == "0" || env_value == "false" || env_value == "FALSE");
}

rapidjson::Document parse_json_line(const std::string& line) {
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

robot::app::EvScheduleStart make_schedule_start(bool at_parking_side,
                                                bool at_far_end,
                                                float passes) {
    robot::app::EvScheduleStart evt;
    evt.at_parking_side = at_parking_side;
    evt.at_far_end = at_far_end;
    evt.passes = passes;
    return evt;
}

const char* parking_side_name(robot::service::ParkingSide side) {
    return side == robot::service::ParkingSide::Left ? "left" : "right";
}

const char* front_limit_name(robot::service::ParkingSide side) {
    return side == robot::service::ParkingSide::Right ? "左侧限位" : "右侧限位";
}

const char* parking_limit_name(robot::service::ParkingSide side) {
    return side == robot::service::ParkingSide::Right ? "右侧限位" : "左侧限位";
}

const char* heading_pid_mode_name(robot::service::HeadingCorrector::Mode mode) {
    using Mode = robot::service::HeadingCorrector::Mode;
    switch (mode) {
        case Mode::UNINITIALIZED:
            return "UNINITIALIZED";
        case Mode::DISCONNECTED:
            return "DISCONNECTED";
        case Mode::STALE:
            return "STALE";
        case Mode::TRACK:
            return "TRACK";
    }
    return "UNKNOWN";
}

robot::app::EvScheduleStart start_from_configured_parking_side(const hw::FullSystemFixture& f,
                                                               float passes) {
    (void)f;
    return make_schedule_start(true, false, passes);
}

std::vector<std::filesystem::path> collect_rotated_health_logs(const std::string& base_path) {
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

void remove_rotated_health_logs(const std::string& base_path) {
    for (const auto& path : collect_rotated_health_logs(base_path)) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
}

struct ScopedActiveFixture {
    explicit ScopedActiveFixture(hw::IGracefulShutdown* fixture) : fixture_(fixture) {
        hw::HwExitGuard::instance().set_active(fixture_);
    }

    ~ScopedActiveFixture() {
        hw::HwExitGuard::instance().clear_active(fixture_);
    }

   private:
    hw::IGracefulShutdown* fixture_;
};

void fail_if_exit_requested(hw::FullSystemFixture& f) {
    if (!hw::HwExitGuard::instance().exit_requested()) {
        return;
    }
    f.shutdown();
    FAIL("hardware test interrupted by signal");
}

struct WalkStopGuard {
    explicit WalkStopGuard(std::shared_ptr<robot::device::WalkMotorGroup> group)
        : group_(std::move(group)) {}

    ~WalkStopGuard() {
        if (!group_) {
            return;
        }
        group_->emergency_override(0.0f);
        group_->disable_all();
    }

   private:
    std::shared_ptr<robot::device::WalkMotorGroup> group_;
};

bool wait_imu_valid(const std::shared_ptr<robot::device::ImuDevice>& imu,
                    std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu && imu->get_latest().valid) {
            return true;
        }
        std::this_thread::sleep_for(50ms);
    }
    return imu && imu->get_latest().valid;
}

void run_combined_system_test(hw::FullSystemFixture& f,
                              const char* tag,
                              bool expect_real_brush,
                              robot::app::EvScheduleStart start_evt,
                              bool log_fused_odometry = false,
                              bool log_heading_pid_debug = false) {
    hw::HwExitGuard::instance().install();
    ScopedActiveFixture active_fixture(&f);
    remove_rotated_health_logs(f.p.health_jsonl_path);

    const int passes_int = static_cast<int>(f.p.combined_passes);
    const double passes_half = f.p.combined_passes * 2.0;
    spdlog::warn("[{}] ====================================", tag);
    spdlog::warn("[{}] ⚠ 机器人将运动 {:.1f} 趟（{} 段）！",
                 tag,
                 f.p.combined_passes,
                 static_cast<int>(passes_half));
    spdlog::warn(
        "[{}] 当前测试配置 parking_side={}，前端={}",
        tag,
        parking_side_name(f.p.parking_side),
        front_limit_name(f.p.parking_side));
    spdlog::warn("[{}] 全程持续记录健康数据到: {}", tag, f.p.health_jsonl_path);
    spdlog::warn("[{}] 本地日志轮转: max_bytes={} max_files={}",
                 tag,
                 f.p.health_log_max_bytes,
                 f.p.health_log_max_files);
    spdlog::warn("[{}] 每段限位超时: {}s（触发后重置）", tag, f.p.limit_timeout_sec);
    if (expect_real_brush) {
        spdlog::warn("[{}] 真实滚刷已接入: port={} axis={} test_rpm={}",
                     tag,
                     f.p.brush_port,
                     static_cast<int>(f.p.brush_axis),
                     f.p.brush_test_rpm);
    }
    spdlog::warn("[{}] 请确保：导轨就位，轨道无人员/障碍物", tag);
    spdlog::warn("[{}] ====================================", tag);
    (void)passes_int;
    REQUIRE(f.init(f.p.health_jsonl_path));
    fail_if_exit_requested(f);
    REQUIRE(f.health != nullptr);
    robot::service::NavService::FusedOdometry odom_before{};
    if (log_fused_odometry) {
        odom_before = f.nav->get_fused_odometry();
        CHECK(odom_before.valid);
        CHECK(std::isfinite(odom_before.top_distance_m));
        CHECK(std::isfinite(odom_before.bottom_distance_m));
        CHECK(std::isfinite(odom_before.fused_distance_m));
        CHECK(std::isfinite(odom_before.distance_diff_m));
    }

    std::atomic<bool> wd_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& n) {
        spdlog::error("[{}] 看门狗超时: {}", tag, n);
        wd_timeout.store(true);
    });
    const int wd_tid = f.watchdog->register_thread(tag, f.p.limit_timeout_sec * 2 * 1000);
    REQUIRE(wd_tid >= 0);

    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    int total_health_records = 0;
    int max_abs_brush_rpm = 0;
    int brush_fault_samples = 0;

    auto poll_once = [&]() {
        fail_if_exit_requested(f);
        // f.bms->update();
        f.health->update();
        f.watchdog->heartbeat(wd_tid);
        ++total_health_records;
        auto gd = f.walk_group->get_group_diagnostics();
        auto ld = f.imu->get_latest();
        const auto brush_diag = f.brush->get_diagnostics();
        const auto pid_debug =
            log_heading_pid_debug ? f.motion->heading_pid_debug_state()
                                  : robot::service::HeadingCorrector::DebugState{};
        max_abs_brush_rpm = std::max(max_abs_brush_rpm, std::abs(brush_diag.actual_rpm));
        if (brush_diag.fault) {
            ++brush_fault_samples;
        }
        if (log_fused_odometry) {
            const auto odom = f.nav->get_fused_odometry();
            if (log_heading_pid_debug) {
                spdlog::info(
                    "[{}] #{}: LT={:.1f}/{:.1f} RT={:.1f}/{:.1f} LB={:.1f}/{:.1f} "
                    "RB={:.1f}/{:.1f} brush_rpm={} brush_fault={} yaw={:.2f}° state={} "
                    "pid(mode={} connected={} valid={} slope={:.4f} conf={:.3f} "
                    "filtered={:.4f} correction={:.3f} age_ms={}) fused_odom(valid={} "
                    "top={:.3f} bottom={:.3f} fused={:.3f} diff={:.3f} top_v={:.3f} "
                    "bottom_v={:.3f} fused_v={:.3f})",
                    tag,
                    total_health_records,
                    gd.wheel[0].speed_rpm,
                    gd.wheel[0].target_value,
                    gd.wheel[1].speed_rpm,
                    gd.wheel[1].target_value,
                    gd.wheel[2].speed_rpm,
                    gd.wheel[2].target_value,
                    gd.wheel[3].speed_rpm,
                    gd.wheel[3].target_value,
                    brush_diag.actual_rpm,
                    brush_diag.fault,
                    ld.yaw_deg,
                    f.fsm->current_state(),
                    heading_pid_mode_name(pid_debug.mode),
                    pid_debug.connected,
                    pid_debug.latest_valid,
                    pid_debug.latest_slope,
                    pid_debug.latest_confidence,
                    pid_debug.filtered_slope,
                    pid_debug.last_correction,
                    pid_debug.result_age_ms,
                    odom.valid,
                    odom.top_distance_m,
                    odom.bottom_distance_m,
                    odom.fused_distance_m,
                    odom.distance_diff_m,
                    odom.top_speed_mps,
                    odom.bottom_speed_mps,
                    odom.fused_speed_mps);
            } else {
                spdlog::info(
                    "[{}] #{}: LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f} walk_rpm brush_rpm={} "
                    "brush_fault={} yaw={:.2f}° state={} fused_odom(valid={} top={:.3f} "
                    "bottom={:.3f} fused={:.3f} diff={:.3f} top_v={:.3f} bottom_v={:.3f} "
                    "fused_v={:.3f})",
                    tag,
                    total_health_records,
                    gd.wheel[0].speed_rpm,
                    gd.wheel[1].speed_rpm,
                    gd.wheel[2].speed_rpm,
                    gd.wheel[3].speed_rpm,
                    brush_diag.actual_rpm,
                    brush_diag.fault,
                    ld.yaw_deg,
                    f.fsm->current_state(),
                    odom.valid,
                    odom.top_distance_m,
                    odom.bottom_distance_m,
                    odom.fused_distance_m,
                    odom.distance_diff_m,
                    odom.top_speed_mps,
                    odom.bottom_speed_mps,
                    odom.fused_speed_mps);
            }
        } else {
            if (log_heading_pid_debug) {
                spdlog::info(
                    "[{}] #{}: LT={:.1f}/{:.1f} RT={:.1f}/{:.1f} LB={:.1f}/{:.1f} "
                    "RB={:.1f}/{:.1f} brush_rpm={} brush_fault={} yaw={:.2f}° state={} "
                    "pid(mode={} connected={} valid={} slope={:.4f} conf={:.3f} "
                    "filtered={:.4f} correction={:.3f} age_ms={})",
                    tag,
                    total_health_records,
                    gd.wheel[0].speed_rpm,
                    gd.wheel[0].target_value,
                    gd.wheel[1].speed_rpm,
                    gd.wheel[1].target_value,
                    gd.wheel[2].speed_rpm,
                    gd.wheel[2].target_value,
                    gd.wheel[3].speed_rpm,
                    gd.wheel[3].target_value,
                    brush_diag.actual_rpm,
                    brush_diag.fault,
                    ld.yaw_deg,
                    f.fsm->current_state(),
                    heading_pid_mode_name(pid_debug.mode),
                    pid_debug.connected,
                    pid_debug.latest_valid,
                    pid_debug.latest_slope,
                    pid_debug.latest_confidence,
                    pid_debug.filtered_slope,
                    pid_debug.last_correction,
                    pid_debug.result_age_ms);
            } else {
                spdlog::info(
                    "[{}] #{}: LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f} walk_rpm brush_rpm={} "
                    "brush_fault={} yaw={:.2f}° state={}",
                    tag,
                    total_health_records,
                    gd.wheel[0].speed_rpm,
                    gd.wheel[1].speed_rpm,
                    gd.wheel[2].speed_rpm,
                    gd.wheel[3].speed_rpm,
                    brush_diag.actual_rpm,
                    brush_diag.fault,
                    ld.yaw_deg,
                    f.fsm->current_state());
            }
        }
    };

    auto wait_transition = [&](const std::string& from) -> std::string {
        using clock = std::chrono::steady_clock;
        auto deadline = clock::now() + std::chrono::seconds(f.p.limit_timeout_sec);
        while (clock::now() < deadline) {
            fail_if_exit_requested(f);
            std::string curr = f.fsm->current_state();
            if (curr != from)
                return curr;
            poll_once();
            std::this_thread::sleep_for(500ms);
        }
        fail_if_exit_requested(f);
        return from;
    };

    f.fsm->dispatch(start_evt);
    REQUIRE((f.fsm->current_state() == "CleanFwd" || f.fsm->current_state() == "Returning"));

    std::string state = f.fsm->current_state();
    int seg_idx = 0;

    while (state != "Charging") {
        ++seg_idx;
        const bool going_fwd = (state == "CleanFwd");
        spdlog::warn("[{}] 段 {}: {} → 等待{}限位（最多 {}s）...",
                     tag,
                     seg_idx,
                     state,
                     going_fwd ? "【前端】" : "【尾端】",
                     f.p.limit_timeout_sec);

        std::string next = wait_transition(state);

        if (next == state) {
            spdlog::error(
                "[{}] 段 {} 超时 {}s，当前状态仍为 {}", tag, seg_idx, f.p.limit_timeout_sec, state);
            INFO("限位等待超时，请检查导轨/传感器接线");
            REQUIRE(next != state);
            break;
        }

        spdlog::info("[{}] ✓ 段 {} 完成：{} → {}（已采集 {} 条）",
                     tag,
                     seg_idx,
                     state,
                     next,
                     total_health_records);
        poll_once();
        state = next;
    }

    {
        INFO("任务未能到达 Charging 状态");
        REQUIRE(state == "Charging");
    }
    spdlog::info("[{}] ✓ 全部 {} 段完成，总采集 {} 条", tag, seg_idx, total_health_records);

    CHECK(!wd_timeout.load());

    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[{}] CAN 帧: {} → {} (增量={})",
                 tag,
                 frames_before,
                 frames_after,
                 frames_after - frames_before);
    if (log_fused_odometry) {
        const auto odom_after = f.nav->get_fused_odometry();
        CHECK(odom_after.valid);
        CHECK(std::isfinite(odom_after.top_distance_m));
        CHECK(std::isfinite(odom_after.bottom_distance_m));
        CHECK(std::isfinite(odom_after.fused_distance_m));
        CHECK(std::isfinite(odom_after.distance_diff_m));
        CHECK(std::abs(odom_after.fused_distance_m - odom_before.fused_distance_m) > 0.02);
    }
    CHECK(frames_after > frames_before + 10u * static_cast<uint32_t>(passes_half));
    CHECK(f.dispatched_faults.empty());

    if (expect_real_brush) {
        spdlog::info("[{}] 真实滚刷统计: max_abs_brush_rpm={} fault_samples={}",
                     tag,
                     max_abs_brush_rpm,
                     brush_fault_samples);
        CHECK(max_abs_brush_rpm >= 50);
        CHECK(brush_fault_samples == 0);
    }

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
                REQUIRE(j.HasMember("ts"));
                REQUIRE(j["ts"].IsUint64());
                REQUIRE(j.HasMember("values"));
                REQUIRE(j["values"].IsObject());
                REQUIRE(j["values"].HasMember("lt_rpm"));
                REQUIRE(j["values"].HasMember("bat_soc"));
                REQUIRE(j["values"].HasMember("imu_p"));
            }
        }

        CHECK(total_retained_lines > 0);
        CHECK(total_retained_lines <= total_health_records);
        spdlog::info("[{}] 保留健康日志 {} 个文件，共 {} 行，主路径: {}",
                     tag,
                     health_files.size(),
                     total_retained_lines,
                     f.p.health_jsonl_path);
    }

    auto ld = f.imu->get_latest();
    spdlog::info("[{}] 末帧 IMU: valid={} pitch={:.2f}° roll={:.2f}° yaw={:.2f}°",
                 tag,
                 ld.valid,
                 ld.pitch_deg,
                 ld.roll_deg,
                 ld.yaw_deg);
    spdlog::info("[{}] PASS — 健康数据已保存至 {}", tag, f.p.health_jsonl_path);
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][tb_rpc_runtime] — 真实硬件 + 真实 ThingsBoard RPC 驱动状态变化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）ThingsBoard RPC start/stop/return 驱动运行态变化",
          "[hw_system][tb_rpc_runtime]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard hardware/runtime RPC test");
        return;
    }

    hw::ThingsBoardRuntimeFixture f;
    REQUIRE(f.init_thingsboard_runtime());
    REQUIRE(f.tb_control != nullptr);
    REQUIRE(f.supervisor != nullptr);
    REQUIRE(f.fsm->current_state() == "Idle");

    spdlog::warn("[hw_system][tb_rpc_runtime] ThingsBoard 平台准备：");
    spdlog::warn("[hw_system][tb_rpc_runtime] 1. 确认设备在线");
    spdlog::warn("[hw_system][tb_rpc_runtime] 2. 打开最新 telemetry，关注 device_state/task_state");
    spdlog::warn("[hw_system][tb_rpc_runtime] 3. 准备依次发送 RPC: start -> stop -> return");
    spdlog::warn(
        "[hw_system][tb_rpc_runtime] 4. return 之后，请人工触发【停机位一侧限位】让状态进入 "
        "Charging");
    spdlog::warn("[hw_system][tb_rpc_runtime] 当前 at_parking_side={} at_far_end={}",
                 f.is_at_parking_side(),
                 f.is_at_far_end());

    REQUIRE(f.is_at_parking_side());
    f.tb_control->publish_startup_attributes();
    f.tb_control->publish_business_telemetry();

    spdlog::warn(
        "[hw_system][tb_rpc_runtime] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `start`");
    REQUIRE(f.wait_state_with_thingsboard({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK((snap.device_state == "CleanFwd" || snap.device_state == "CleanReturn"));
        CHECK(snap.task_state == "RunningTask");
    }

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `stop`");
    REQUIRE(f.wait_state_with_thingsboard({"Stopped"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Stopped");
        CHECK(snap.task_state == "StoppedTask");
    }

    spdlog::warn(
        "[hw_system][tb_rpc_runtime] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `return`");
    REQUIRE(f.wait_state_with_thingsboard({"Returning"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Returning");
        CHECK(snap.task_state == "ReturningTask");
    }

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: 人工触发【停机位一侧限位】");
    REQUIRE(f.wait_state_with_thingsboard({"Charging"}, std::chrono::seconds(180)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Charging");
        CHECK(snap.task_state == "ChargingTask");
    }
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][imu_gps_health_only] — 仅 IMU/GPS 持续采集并由 HealthService 落盘
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）仅 IMU/GPS 持续采集并本地落盘", "[hw_system][imu_gps_health_only]") {
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

    auto odom = f.nav->get_fused_odometry();
    spdlog::info("[hw_system][full_init] fused_odom valid={} top={:.3f}m bottom={:.3f}m fused={:.3f}m diff={:.3f}m",
                 odom.valid,
                 odom.top_distance_m,
                 odom.bottom_distance_m,
                 odom.fused_distance_m,
                 odom.distance_diff_m);

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
enum class LowerPoseEvalMode {
    PitchStableGate,
    PitchRollRateScore,
};

void run_lower_pose_search_test(const char* tag, LowerPoseEvalMode mode) {
    hw::FullSystemFixture f(false /* pid_off */);
    hw::HwExitGuard::instance().install();
    ScopedActiveFixture active_fixture(&f);

    spdlog::warn("[{}] ====================================", tag);
    spdlog::warn("[{}] ⚠ 上轮 LT/RT 将保持 0rpm，下轮 LB/RB 将低速连续正反扫动", tag);
    spdlog::warn("[{}] 连续探测式：下一帧满足条件立即停", tag);
    spdlog::warn("[{}] 请确保机器人已放稳，轨道附近无人员/障碍物", tag);
    spdlog::warn("[{}] ====================================", tag);

    REQUIRE(f.init());
    REQUIRE(wait_imu_valid(f.imu, 3000ms));

    WalkStopGuard stop_guard(f.walk_group);
    f.walk_group->clear_override();
    f.walk_group->update();

    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);
    std::this_thread::sleep_for(300ms);

    constexpr float kLowerScanRpm = 3.0f;
    constexpr float kPitchAlpha = 1.0f;
    constexpr float kRollAlpha = 1.0f;
    constexpr float kRollRateAlpha = 0.25f;
    constexpr float kPitchImproveEpsDeg = 0.03f;
    constexpr float kPitchNearBestEpsDeg = 0.02f;
    constexpr float kScoreNearBestEps = 0.05f;
    constexpr float kStableRollRateGate = 0.2f;  // deg/s
    constexpr float kSafetyRollRateGate = 0.9f;  // deg/s
    constexpr float kPitchWeight = 1.0f;
    constexpr float kRollRateWeight = 6.0f;
    constexpr auto kSamplePeriod = 50ms;
    constexpr auto kMinPhaseRun = 500ms;
    constexpr auto kMaxPhaseRun = 15000ms;

    if (mode == LowerPoseEvalMode::PitchRollRateScore) {
        spdlog::info("[{}] score = {:.2f} * abs(pitch) - {:.2f} * abs(roll_rate)",
                     tag,
                     kPitchWeight,
                     kRollRateWeight);
    }

    auto imu0 = f.imu->get_latest();
    float filtered_pitch = imu0.pitch_deg;
    float filtered_roll = imu0.roll_deg;
    float prev_filtered_roll = filtered_roll;
    float filtered_roll_rate = 0.0f;
    bool roll_rate_inited = false;
    float global_best_abs_pitch = std::abs(filtered_pitch);
    float phase_best_abs_pitch = global_best_abs_pitch;
    float global_best_score = kPitchWeight * global_best_abs_pitch;
    float phase_best_score = global_best_score;
    const float start_pitch = filtered_pitch;
    const float start_roll = filtered_roll;
    int direction = 1;
    bool reversed = false;

    auto set_lower_scan = [&](int dir) {
        // 物理前进方向：下轮 LB/RB 为负转；反向则为正转。
        const float lower_rpm = (dir > 0) ? -kLowerScanRpm : kLowerScanRpm;
        return f.walk_group->set_speeds(0.0f, 0.0f, lower_rpm, lower_rpm);
    };

    auto stop_all = [&] {
        f.walk_group->set_speeds(0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(100ms);
    };

    REQUIRE(set_lower_scan(direction) == robot::device::DeviceError::OK);
    auto phase_started_at = std::chrono::steady_clock::now();
    auto last_sample_at = phase_started_at;
    int sample_count = 0;

    while (!hw::HwExitGuard::instance().exit_requested()) {
        std::this_thread::sleep_for(kSamplePeriod);
        const auto now = std::chrono::steady_clock::now();
        const float dt_s =
            std::max(1e-3f, std::chrono::duration<float>(now - last_sample_at).count());
        last_sample_at = now;

        const auto imu = f.imu->get_latest();
        if (!imu.valid) {
            continue;
        }

        filtered_pitch = (1.0f - kPitchAlpha) * filtered_pitch + kPitchAlpha * imu.pitch_deg;
        filtered_roll = (1.0f - kRollAlpha) * filtered_roll + kRollAlpha * imu.roll_deg;

        const float raw_roll_rate = (filtered_roll - prev_filtered_roll) / dt_s;
        prev_filtered_roll = filtered_roll;
        if (!roll_rate_inited) {
            filtered_roll_rate = raw_roll_rate;
            roll_rate_inited = true;
        } else {
            filtered_roll_rate =
                (1.0f - kRollRateAlpha) * filtered_roll_rate + kRollRateAlpha * raw_roll_rate;
        }

        const float abs_pitch = std::abs(filtered_pitch);
        const float abs_roll_rate = std::abs(filtered_roll_rate);
        const float pose_score = kPitchWeight * abs_pitch - kRollRateWeight * abs_roll_rate;

        if (abs_pitch > phase_best_abs_pitch + kPitchImproveEpsDeg) {
            phase_best_abs_pitch = abs_pitch;
        }
        if (abs_pitch > global_best_abs_pitch + kPitchImproveEpsDeg) {
            global_best_abs_pitch = abs_pitch;
        }
        if (pose_score > phase_best_score + kPitchImproveEpsDeg) {
            phase_best_score = pose_score;
        }
        if (pose_score > global_best_score + kPitchImproveEpsDeg) {
            global_best_score = pose_score;
        }

        if (++sample_count % 5 == 0) {
            spdlog::info(
                "[{}] dir={} raw_pitch={:.3f} filt_pitch={:.3f} raw_roll={:.3f} filt_roll={:.3f} "
                "roll_rate={:.3f} abs_pitch={:.3f} best_pitch={:.3f} score={:.3f} "
                "best_score={:.3f}",
                tag,
                direction > 0 ? "lower_forward" : "lower_reverse",
                imu.pitch_deg,
                filtered_pitch,
                imu.roll_deg,
                filtered_roll,
                filtered_roll_rate,
                abs_pitch,
                phase_best_abs_pitch,
                pose_score,
                phase_best_score);
        }

        const auto phase_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - phase_started_at);
        const bool safety_stop = abs_roll_rate >= kSafetyRollRateGate;
        const bool stable_zone = abs_roll_rate <= kStableRollRateGate;
        bool candidate_stop = false;
        if (phase_elapsed >= kMinPhaseRun && stable_zone) {
            if (mode == LowerPoseEvalMode::PitchStableGate) {
                candidate_stop = abs_pitch >= (phase_best_abs_pitch - kPitchNearBestEpsDeg);
            } else {
                candidate_stop = pose_score >= (phase_best_score - kScoreNearBestEps);
            }
        }
        const bool phase_timeout = phase_elapsed >= kMaxPhaseRun;

        if (!safety_stop && !candidate_stop && !phase_timeout) {
            continue;
        }

        stop_all();
        spdlog::info(
            "[{}] {}结束: reason={} elapsed={}ms pitch={:.3f} roll={:.3f} roll_rate={:.3f} "
            "abs_pitch={:.3f} phase_best_pitch={:.3f} score={:.3f} phase_best_score={:.3f}",
            tag,
            direction > 0 ? "正向" : "反向",
            safety_stop ? "safety_stop" : (candidate_stop ? "candidate_stop" : "timeout"),
            phase_elapsed.count(),
            filtered_pitch,
            filtered_roll,
            filtered_roll_rate,
            abs_pitch,
            phase_best_abs_pitch,
            pose_score,
            phase_best_score);

        if (safety_stop || reversed) {
            break;
        }

        reversed = true;
        direction = -direction;
        phase_best_abs_pitch = std::abs(filtered_pitch);
        phase_best_score = pose_score;
        phase_started_at = std::chrono::steady_clock::now();
        last_sample_at = phase_started_at;
        REQUIRE(set_lower_scan(direction) == robot::device::DeviceError::OK);
    }

    stop_all();
    const auto final_imu = f.imu->get_latest();
    spdlog::info(
        "[{}] start_pitch={:.3f} start_roll={:.3f} best_abs_pitch={:.3f} "
        "best_score={:.3f} final_pitch={:.3f} final_roll={:.3f}",
        tag,
        start_pitch,
        start_roll,
        global_best_abs_pitch,
        global_best_score,
        final_imu.pitch_deg,
        final_imu.roll_deg);

    f.walk_group->disable_all();
    fail_if_exit_requested(f);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][lower_pitch_peak] — 连续探测式：abs(pitch) + roll_rate 判稳/判险
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）连续探测式初始化找正（pitch 主判据）",
          "[hw_system][lower_pitch_peak]") {
    run_lower_pose_search_test("[hw_system][lower_pitch_peak]", LowerPoseEvalMode::PitchStableGate);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][lower_pitch_score] — 连续探测式：pitch + roll_rate 综合评分
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）连续探测式初始化找正（pitch + roll_rate 综合评分）",
          "[hw_system][lower_pitch_score]") {
    run_lower_pose_search_test("[hw_system][lower_pitch_score]",
                               LowerPoseEvalMode::PitchRollRateScore);
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
    f.fsm->dispatch(start_from_configured_parking_side(f, 1.0f));
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
TEST_CASE("System（真实硬件）P1 故障链：FSM Returning → EvParkingSideLimitSettled → Charging",
          "[hw_system][p1_fault_chain]") {
    spdlog::warn("[hw_system][p1_fault_chain] ⚠ 此测试将短暂启动电机，确保安全！");

    hw::FullSystemFixture f;
    REQUIRE(f.init());

    f.fsm->dispatch(start_from_configured_parking_side(f, 2.0f));
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
    f.fsm->dispatch(robot::app::EvParkingSideLimitSettled{});
    REQUIRE(f.fsm->current_state() == "Charging");

    spdlog::info("[hw_system][p1_fault_chain] PASS: CleanFwd→Returning→Charging");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][n1_clean_cycle] — N=1 完整任务链（按配置停机位切换前端/停机位限位）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N=1 完整任务链（真实限位触发）", "[hw_system][n1_clean_cycle]") {
    spdlog::warn("[hw_system][n1_clean_cycle] ====================================");
    spdlog::warn("[hw_system][n1_clean_cycle] ⚠ 机器人将完整运动一个来回（N=1）！");
    spdlog::warn("[hw_system][n1_clean_cycle] 请确保：导轨就位，轨道无人员/障碍物");
    spdlog::warn("[hw_system][n1_clean_cycle] ====================================");

    hw::FullSystemFixture f;
    REQUIRE(f.init());
    spdlog::warn("[hw_system][n1_clean_cycle] 当前测试配置 parking_side={}，前端={}，停机位端={}",
                 parking_side_name(f.p.parking_side),
                 front_limit_name(f.p.parking_side),
                 parking_limit_name(f.p.parking_side));

    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;

    // 启动 N=1 任务 → CleanFwd，电机开始正向运动
    f.fsm->dispatch(start_from_configured_parking_side(f, 5.0f));
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    spdlog::warn("[hw_system][n1_clean_cycle] ★ 机器人正在向前运动，等待【{}】触发（最多 {}s）...",
                 front_limit_name(f.p.parking_side),
                 f.p.limit_timeout_sec);

    // 等待 SafetyMonitor → EventBus → FSM CleanReturn（真实限位触发）
    const bool left_hit =
        f.wait_state("CleanReturn", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        if (!left_hit && hw::HwExitGuard::instance().exit_requested()) {
            FAIL("hardware test interrupted by signal");
        }
        INFO(std::string(front_limit_name(f.p.parking_side)) + "等待超时，请检查导轨/传感器接线");
        REQUIRE(left_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ {}触发，机器人开始返回",
                 front_limit_name(f.p.parking_side));

    spdlog::warn("[hw_system][n1_clean_cycle] ★ 机器人正在返回，等待【{}】触发（最多 {}s）...",
                 parking_limit_name(f.p.parking_side),
                 f.p.limit_timeout_sec);

    // 等待右侧限位触发 → Charging（任务完成）
    const bool right_hit =
        f.wait_state("Charging", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        if (!right_hit && hw::HwExitGuard::instance().exit_requested()) {
            FAIL("hardware test interrupted by signal");
        }
        INFO(std::string(parking_limit_name(f.p.parking_side)) +
             "等待超时，请检查导轨/传感器接线");
        REQUIRE(right_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ {}触发，任务完成",
                 parking_limit_name(f.p.parking_side));

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
    run_combined_system_test(
        f, "hw_system][combined", false, start_from_configured_parking_side(f, f.p.combined_passes));
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][combined_nvm_real] — 全系统联合启动 + 全程打印融合里程计
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）完整任务链 + 融合里程计日志", "[hw_system][combined_nvm_real]") {
    hw::FullSystemFixture f;
    const float passes = std::max(1.0f, f.p.combined_passes);
    run_combined_system_test(
        f,
        "hw_system][combined_nvm_real",
        false,
        start_from_configured_parking_side(f, passes),
        true);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][combined_brush_real] — 全系统联合启动 + 真实滚刷
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N 趟完整任务链 + 真实滚刷 + 全程持续采集健康数据",
          "[hw_system][combined_brush_real]") {
    hw::FullSystemFixture f(false, true);
    run_combined_system_test(f,
                             "hw_system][combined_brush_real",
                             true,
                             start_from_configured_parking_side(f, f.p.combined_passes));
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][pid_combined] — 视觉 PID + 真实滚刷，启动事件伪装停机位
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）视觉 PID 完整任务链 + 真实滚刷",
          "[hw_system][pid_combined]") {
    hw::FullSystemFixture f(true, true);
    const auto start_evt = start_from_configured_parking_side(f, f.p.combined_passes);
    spdlog::warn("[hw_system][pid_combined] parking_side={} start_evt: at_parking_side={} at_far_end={}",
                 parking_side_name(f.p.parking_side),
                 start_evt.at_parking_side,
                 start_evt.at_far_end);
    run_combined_system_test(f, "hw_system][pid_combined", true, start_evt, false, true);
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
