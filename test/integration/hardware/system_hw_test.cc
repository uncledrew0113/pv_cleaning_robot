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
#include <algorithm>
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

#include "hw_config.h"

using namespace std::chrono_literals;

namespace {

bool real_tb_test_enabled()
{
    const char* value = std::getenv("TB_REAL_TEST");
    if (!value) {
        return false;
    }
    const std::string env_value(value);
    return !(env_value.empty() || env_value == "0" || env_value == "false" ||
             env_value == "FALSE");
}

rapidjson::Document parse_json_line(const std::string& line)
{
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

robot::app::EvScheduleStart make_schedule_start(bool at_parking_side, bool at_far_end, float passes)
{
    robot::app::EvScheduleStart evt;
    evt.at_parking_side = at_parking_side;
    evt.at_far_end = at_far_end;
    evt.passes = passes;
    return evt;
}

robot::app::EvScheduleStart start_from_parking_side(float passes)
{
    return make_schedule_start(true, false, passes);
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

const char* pid_mode_name(robot::service::HeadingPidController::Mode mode)
{
    using Mode = robot::service::HeadingPidController::Mode;
    switch (mode) {
        case Mode::UNINITIALIZED:
            return "UNINITIALIZED";
        case Mode::LEARN:
            return "LEARN";
        case Mode::TRACK:
            return "TRACK";
        case Mode::FREEZE:
            return "FREEZE";
    }
    return "UNKNOWN";
}

float clamp_pid_target(float v)
{
    return std::max(-210.0f, std::min(210.0f, v));
}

bool open_pid_metrics_file(const std::filesystem::path& path, std::ofstream* out)
{
    if (!out) {
        return false;
    }

    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            spdlog::error("[hw_system][pid_combined] 创建 PID 指标目录失败: path={} err={}",
                          parent.string(),
                          ec.message());
            return false;
        }
    }

    out->open(path, std::ios::trunc);
    if (!out->is_open()) {
        spdlog::error("[hw_system][pid_combined] 打开 PID 指标文件失败: path={}", path.string());
        return false;
    }

    out->setf(std::ios::unitbuf);
    return true;
}

robot::service::HeadingPidController::Params make_pid_probe_params(const hw::HwParams::PidParams& pid)
{
    robot::service::HeadingPidController::Params params;
    params.pitch_alpha = pid.pitch_alpha;
    params.roll_alpha = pid.roll_alpha;
    params.gyro_alpha = pid.gyro_alpha;
    params.pitch_drop_threshold = pid.pitch_drop_threshold;
    params.roll_threshold = pid.roll_threshold;
    params.learn_improve_eps = pid.learn_improve_eps;
    params.best_decay_per_s = pid.best_decay_per_s;
    params.freeze_gyro_z_threshold = pid.freeze_gyro_z_threshold;
    params.freeze_pitch_rate_threshold = pid.freeze_pitch_rate_threshold;
    params.freeze_roll_rate_threshold = pid.freeze_roll_rate_threshold;
    params.max_output = pid.max_output;
    params.min_effective_output = pid.min_effective_output;
    params.warmup_ms = pid.warmup_ms;
    params.hold_ms = pid.hold_ms;
    params.freeze_release_ms = pid.freeze_release_ms;
    return params;
}

struct MotionPidProbeFilters {
    float filtered_pitch{0.0f};
    bool filtered_pitch_inited{false};
    float filtered_roll{0.0f};
    bool filtered_roll_inited{false};
    float filtered_omega_z{0.0f};
    bool filtered_omega_z_inited{false};

    void update(const robot::device::ImuDevice::ImuData& imu)
    {
        const float raw_pitch = imu.pitch_deg;
        const float raw_roll = imu.roll_deg;
        const float raw_omega_z = imu.gyro[2] * (180.0f / 3.14159265f);

        if (!filtered_pitch_inited) {
            filtered_pitch = raw_pitch;
            filtered_pitch_inited = true;
        } else {
            filtered_pitch = 0.8f * filtered_pitch + 0.2f * raw_pitch;
        }

        if (!filtered_roll_inited) {
            filtered_roll = raw_roll;
            filtered_roll_inited = true;
        } else {
            filtered_roll = 0.8f * filtered_roll + 0.2f * raw_roll;
        }

        if (!filtered_omega_z_inited) {
            filtered_omega_z = raw_omega_z;
            filtered_omega_z_inited = true;
        } else {
            filtered_omega_z = 0.8f * filtered_omega_z + 0.2f * raw_omega_z;
        }
    }
};

struct PidProbeSnapshot {
    robot::service::HeadingPidController::DebugState pid{};
    float correction{0.0f};
    float lt_target{0.0f};
    float rt_target{0.0f};
    float lb_target{0.0f};
    float rb_target{0.0f};
};

struct ProbeThreadGuard {
    std::atomic<bool>* stop{nullptr};
    std::thread* thread{nullptr};

    ~ProbeThreadGuard()
    {
        if (stop) {
            stop->store(true, std::memory_order_relaxed);
        }
        if (thread && thread->joinable()) {
            thread->join();
        }
    }
};

std::string build_pid_sample_json(int64_t ts_ms,
                                  int seg,
                                  const std::string& state,
                                  float yaw,
                                  float omega_z,
                                  const PidProbeSnapshot& pid_probe)
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
    writer.Key("pid_mode");
    writer.String(pid_mode_name(pid_probe.pid.mode));
    writer.Key("pid_filtered_pitch");
    writer.Double(pid_probe.pid.filtered_pitch);
    writer.Key("pid_filtered_roll");
    writer.Double(pid_probe.pid.filtered_roll);
    writer.Key("pid_filtered_gyro_z");
    writer.Double(pid_probe.pid.filtered_gyro_z);
    writer.Key("pid_pitch_best");
    writer.Double(pid_probe.pid.pitch_abs_best);
    writer.Key("pid_roll_best");
    writer.Double(pid_probe.pid.roll_at_best);
    writer.Key("pid_pitch_drop");
    writer.Double(pid_probe.pid.pitch_drop);
    writer.Key("pid_roll_delta");
    writer.Double(pid_probe.pid.roll_delta);
    writer.Key("pid_correction");
    writer.Double(pid_probe.correction);
    writer.Key("lt_tgt_final");
    writer.Double(pid_probe.lt_target);
    writer.Key("rt_tgt_final");
    writer.Double(pid_probe.rt_target);
    writer.Key("lb_tgt_final");
    writer.Double(pid_probe.lb_target);
    writer.Key("rb_tgt_final");
    writer.Double(pid_probe.rb_target);
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
                                     float pitch_drop_threshold,
                                     float roll_threshold,
                                     float max_output)
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
    writer.Key("pitch_drop_threshold");
    writer.Double(pitch_drop_threshold);
    writer.Key("roll_threshold");
    writer.Double(roll_threshold);
    writer.Key("max_output");
    writer.Double(max_output);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
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

void run_combined_system_test(hw::FullSystemFixture& f,
                              const char* tag,
                              bool expect_real_brush) {
    hw::HwExitGuard::instance().install();
    ScopedActiveFixture active_fixture(&f);

    const int passes_int = static_cast<int>(f.p.combined_passes);
    const double passes_half = f.p.combined_passes * 2.0;
    spdlog::warn("[{}] ====================================", tag);
    spdlog::warn("[{}] ⚠ 机器人将运动 {:.1f} 趟（{} 段）！",
                 tag,
                 f.p.combined_passes,
                 static_cast<int>(passes_half));
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
        f.bms->update();
        f.health->update();
        f.watchdog->heartbeat(wd_tid);
        ++total_health_records;
        auto gd = f.walk_group->get_group_diagnostics();
        auto ld = f.imu->get_latest();
        const auto brush_diag = f.brush->get_diagnostics();
        max_abs_brush_rpm = std::max(max_abs_brush_rpm, std::abs(brush_diag.actual_rpm));
        if (brush_diag.fault) {
            ++brush_fault_samples;
        }
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
            std::this_thread::sleep_for(5ms);
        }
        fail_if_exit_requested(f);
        return from;
    };

    f.fsm->dispatch(start_from_parking_side(f.p.combined_passes));
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    std::string state = "CleanFwd";
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
            spdlog::error("[{}] 段 {} 超时 {}s，当前状态仍为 {}",
                          tag,
                          seg_idx,
                          f.p.limit_timeout_sec,
                          state);
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
    spdlog::warn("[hw_system][tb_rpc_runtime] 4. return 之后，请人工触发【停机位一侧限位】让状态进入 Charging");
    spdlog::warn("[hw_system][tb_rpc_runtime] 当前 at_parking_side={} at_far_end={}",
                 f.is_at_parking_side(),
                 f.is_at_far_end());

    REQUIRE(f.is_at_parking_side());
    f.tb_control->publish_startup_attributes();
    f.tb_control->publish_business_telemetry();

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `start`");
    REQUIRE(f.wait_state_with_thingsboard({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK((snap.device_state == "CleanFwd" || snap.device_state == "CleanReturn"));
        CHECK(snap.task_state == "RunningTask");
    }

    spdlog::warn("[hw_system][tb_rpc_runtime] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `stop`");
    REQUIRE(f.wait_state_with_thingsboard({"Paused"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Paused");
        CHECK(snap.task_state == "PausedTask");
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
// [hw_system][tb_terminate_reset] — 真实硬件 + 真实 ThingsBoard terminate/reset
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）ThingsBoard RPC terminate/reset 驱动结束与复位",
          "[hw_system][tb_terminate_reset]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard terminate/reset test");
        return;
    }

    hw::ThingsBoardRuntimeFixture f;
    REQUIRE(f.init_thingsboard_runtime());
    REQUIRE(f.tb_control != nullptr);
    REQUIRE(f.supervisor != nullptr);
    REQUIRE(f.fsm->current_state() == "Idle");

    spdlog::warn("[hw_system][tb_terminate_reset] ThingsBoard 平台准备：");
    spdlog::warn("[hw_system][tb_terminate_reset] 1. 确认设备在线");
    spdlog::warn("[hw_system][tb_terminate_reset] 2. 准备依次发送 RPC: start -> stop -> terminate -> reset");
    spdlog::warn("[hw_system][tb_terminate_reset] 3. reset 之前，请确保机器人回到【停机位】或手动压住停机位一侧限位");
    spdlog::warn("[hw_system][tb_terminate_reset] 当前 at_parking_side={} at_far_end={}",
                 f.is_at_parking_side(),
                 f.is_at_far_end());

    REQUIRE(f.is_at_parking_side());
    f.tb_control->publish_startup_attributes();
    f.tb_control->publish_business_telemetry();

    spdlog::warn(
        "[hw_system][tb_terminate_reset] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `start`");
    REQUIRE(f.wait_state_with_thingsboard({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));

    spdlog::warn(
        "[hw_system][tb_terminate_reset] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `stop`");
    REQUIRE(f.wait_state_with_thingsboard({"Paused"}, std::chrono::seconds(120)));

    spdlog::warn(
        "[hw_system][tb_terminate_reset] ACTION REQUIRED: 在 ThingsBoard 平台发送 RPC `terminate`");
    REQUIRE(f.wait_state_with_thingsboard({"Terminated"}, std::chrono::seconds(120)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Terminated");
        CHECK(snap.task_state == "TerminatedTask");
    }

    spdlog::warn("[hw_system][tb_terminate_reset] ACTION REQUIRED: 确保 at_parking_side=true，然后在 ThingsBoard 平台发送 RPC `reset`");
    REQUIRE(f.wait_state_with_thingsboard({"Idle"}, std::chrono::seconds(180)));
    {
        const auto snap = f.supervisor->snapshot();
        CHECK(snap.device_state == "Idle");
        CHECK(snap.task_state == "IdleTask");
    }
}

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
    f.fsm->dispatch(start_from_parking_side(1.0f));
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

    f.fsm->dispatch(start_from_parking_side(2.0f));
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
// [hw_system][n1_clean_cycle] — N=1 完整任务链（手动触发真实前右限位）
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
    f.fsm->dispatch(start_from_parking_side(5.0f));
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    spdlog::warn(
        "[hw_system][n1_clean_cycle] ★ 机器人正在向前运动，等待【左侧限位】触发（最多 {}s）...",
        f.p.limit_timeout_sec);

    // 等待 SafetyMonitor → EventBus → FSM CleanReturn（真实限位触发）
    const bool left_hit =
        f.wait_state("CleanReturn", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        if (!left_hit && hw::HwExitGuard::instance().exit_requested()) {
            FAIL("hardware test interrupted by signal");
        }
        INFO("左侧限位等待超时，请检查导轨/传感器接线");
        REQUIRE(left_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ 左侧限位触发，机器人开始返回");

    spdlog::warn(
        "[hw_system][n1_clean_cycle] ★ 机器人正在返回，等待【右侧限位】触发（最多 {}s）...",
        f.p.limit_timeout_sec);

    // 等待右侧限位触发 → Charging（任务完成）
    const bool right_hit =
        f.wait_state("Charging", std::chrono::milliseconds(f.p.limit_timeout_sec * 1000));

    {
        if (!right_hit && hw::HwExitGuard::instance().exit_requested()) {
            FAIL("hardware test interrupted by signal");
        }
        INFO("右侧限位等待超时，请检查导轨/传感器接线");
        REQUIRE(right_hit);
    }
    spdlog::info("[hw_system][n1_clean_cycle] ✓ 右侧限位触发，任务完成");

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
    run_combined_system_test(f, "hw_system][combined", false);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][combined_brush_real] — 全系统联合启动 + 真实滚刷
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N 趟完整任务链 + 真实滚刷 + 全程持续采集健康数据",
          "[hw_system][combined_brush_real]") {
    hw::FullSystemFixture f(false, true);
    run_combined_system_test(f, "hw_system][combined_brush_real", true);
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_system][pid_combined] — N 趟完整任务链 + 姿态纠偏 + 真实滚刷 + 全程 yaw 指标采集
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("System（真实硬件）N 趟完整任务链 + PID 控制 + 真实滚刷 + yaw 指标采集",
          "[hw_system][pid_combined]") {
    hw::FullSystemFixture f(true /* pid_on */, true /* use_real_brush */);
    hw::HwExitGuard::instance().install();
    ScopedActiveFixture active_fixture(&f);
    const double passes_half = f.p.combined_passes * 2.0;
    spdlog::warn("[hw_system][pid_combined] ====================================");
    spdlog::warn("[hw_system][pid_combined] ⚠ 机器人将运动 {:.1f} 趟（{} 段）！",
                 f.p.combined_passes,
                 static_cast<int>(passes_half));
    spdlog::warn("[hw_system][pid_combined] attitude pitch_drop={:.2f} roll_threshold={:.2f}",
                 f.p.pid.pitch_drop_threshold,
                 f.p.pid.roll_threshold);
    spdlog::warn("[hw_system][pid_combined] 健康数据: {}", f.p.health_jsonl_path);
    spdlog::warn("[hw_system][pid_combined] PID 指标: {}", f.p.pid_jsonl_path);
    spdlog::warn("[hw_system][pid_combined] 最大漂移警告阈值: {:.1f}°", f.p.pid_max_drift_deg);
    spdlog::warn("[hw_system][pid_combined] 请确保：导轨就位，轨道无人员/障碍物");
    spdlog::warn("[hw_system][pid_combined] ====================================");

    REQUIRE(f.init(f.p.health_jsonl_path));
    fail_if_exit_requested(f);
    REQUIRE(f.health != nullptr);

    // 打开 pid_metrics.jsonl（测试结束不删除，供离线分析）
    std::ofstream pid_ofs;
    REQUIRE(open_pid_metrics_file(f.p.pid_jsonl_path, &pid_ofs));

    // 看门狗（超时 = 每段最大秒数 × 2）
    std::atomic<bool> wd_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& n) {
        spdlog::error("[hw_system][pid_combined] 看门狗超时: {}", n);
        wd_timeout.store(true);
    });
    const int wd_tid =
        f.watchdog->register_thread("hw_pid_combined", f.p.limit_timeout_sec * 2 * 1000);
    REQUIRE(wd_tid >= 0);

    // ── 读取真实左右限位状态，避免把起点硬编码成右侧/停机位 ────────────────────
    const bool right_limit_active = !f.right_sw->read_current_level();
    const bool left_limit_active = !f.left_sw->read_current_level();
    {
        INFO("设备不在已知端点（right_limit_active=false, left_limit_active=false），请将设备移至某一端点");
        REQUIRE((right_limit_active || left_limit_active));
    }
    spdlog::info("[hw_system][pid_combined] 传感器: right_limit_active={} left_limit_active={}",
                 right_limit_active,
                 left_limit_active);

    // 记录出发时的 target_yaw（PID 将锁定此航向）
    const float target_yaw = f.imu->get_latest().yaw_deg;
    spdlog::info("[hw_system][pid_combined] target_yaw={:.2f}°", target_yaw);

    using clock = std::chrono::steady_clock;
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
    auto last_pid_sample_at = clock::time_point{};
    constexpr auto kPidSampleInterval = 500ms;
    auto last_health_update_at = clock::time_point{};
    constexpr auto kHealthUpdateInterval = 2000ms;

    std::mutex pid_probe_mu;
    PidProbeSnapshot pid_probe_snapshot;
    std::atomic<bool> pid_probe_stop{false};
    robot::service::HeadingPidController pid_probe_ctrl(make_pid_probe_params(f.p.pid));
    pid_probe_ctrl.enable(true);

    std::thread pid_probe_thread([&]() {
        MotionPidProbeFilters probe_filters;
        auto last_tick = clock::now();

        while (!pid_probe_stop.load(std::memory_order_relaxed)) {
            const auto now = clock::now();
            float dt_s = std::chrono::duration<float>(now - last_tick).count();
            if (dt_s < 0.0f) {
                dt_s = 0.0f;
            }
            last_tick = now;

            const auto imu = f.imu->get_latest();
            probe_filters.update(imu);

            PidProbeSnapshot snap;
            snap.correction = pid_probe_ctrl.compute(probe_filters.filtered_pitch,
                                                     probe_filters.filtered_roll,
                                                     probe_filters.filtered_omega_z,
                                                     dt_s);
            snap.pid = pid_probe_ctrl.debug_state();

            const auto gd = f.walk_group->get_group_diagnostics();
            const std::string state = f.fsm->current_state();
            const bool moving = (state == "CleanFwd" || state == "CleanReturn");
            const float applied_correction = moving ? snap.correction : 0.0f;

            snap.lt_target = clamp_pid_target(gd.wheel[0].target_value + applied_correction);
            snap.rt_target = clamp_pid_target(gd.wheel[1].target_value + applied_correction);
            snap.lb_target = clamp_pid_target(gd.wheel[2].target_value + applied_correction);
            snap.rb_target = clamp_pid_target(gd.wheel[3].target_value + applied_correction);

            {
                std::lock_guard<std::mutex> lock(pid_probe_mu);
                pid_probe_snapshot = snap;
            }

            std::this_thread::sleep_until(now + 50ms);
        }
    });
    ProbeThreadGuard pid_probe_guard{&pid_probe_stop, &pid_probe_thread};

    // 每次调用：刷新 BMS + 写健康 JSONL + 喂狗 + 写 pid_metrics JSONL
    auto poll_once = [&]() {
        fail_if_exit_requested(f);
        f.watchdog->heartbeat(wd_tid);

        const auto now = clock::now();
        if (last_health_update_at == clock::time_point{} ||
            now - last_health_update_at >= kHealthUpdateInterval) {
            // 临时停用 BMS 轮询，避免串口阻塞影响 PID 采样频率。
            f.health->update();
            last_health_update_at = clock::now();
            ++total_health_records;
        }

        auto gd = f.walk_group->get_group_diagnostics();
        auto imu = f.imu->get_latest();
        const float yaw = imu.yaw_deg;
        const float omega_z_dps = imu.gyro[2] * (180.0f / 3.14159265f);
        const float yaw_err = norm_angle(target_yaw - yaw);
        const float drift = std::abs(yaw_err);
        if (drift > max_drift_all)
            max_drift_all = drift;

        if (last_pid_sample_at != clock::time_point{} &&
            now - last_pid_sample_at < kPidSampleInterval) {
            return;  // 只节流 PID 采样（健康采集独立周期）
        }
        last_pid_sample_at = now;

        const int64_t ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

        PidProbeSnapshot pid_probe;
        {
            std::lock_guard<std::mutex> lock(pid_probe_mu);
            pid_probe = pid_probe_snapshot;
        }

        pid_ofs << build_pid_sample_json(
                       ts_ms,
                       cur_seg,
                       f.fsm->current_state(),
                       std::round(yaw * 100.0f) / 100.0f,
                       std::round(omega_z_dps * 100.0f) / 100.0f,
                       pid_probe)
                << '\n';

        spdlog::info(
            "[hw_system][pid_combined] #{} seg={} yaw={:.2f}° omega_z={:.2f}°/s "
            "LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f}rpm state={} "
            "pid_mode={} corr={:.2f} pitch_drop={:.3f} roll_delta={:.3f} "
            "final_tgt=[{:.1f},{:.1f},{:.1f},{:.1f}]",
            total_health_records,
            cur_seg,
            yaw,
            omega_z_dps,
            gd.wheel[0].speed_rpm,
            gd.wheel[1].speed_rpm,
            gd.wheel[2].speed_rpm,
            gd.wheel[3].speed_rpm,
            f.fsm->current_state(),
            pid_mode_name(pid_probe.pid.mode),
            pid_probe.correction,
            pid_probe.pid.pitch_drop,
            pid_probe.pid.roll_delta,
            pid_probe.lt_target,
            pid_probe.rt_target,
            pid_probe.lb_target,
            pid_probe.rb_target);
    };

    // 等待 FSM 从 from 状态切换到其他状态（每段独立超时）
    auto wait_transition = [&](const std::string& from) -> std::string {
        auto deadline = clock::now() + std::chrono::seconds(f.p.limit_timeout_sec);
        while (clock::now() < deadline) {
            fail_if_exit_requested(f);
            std::string curr = f.fsm->current_state();
            if (curr != from)
                return curr;
            poll_once();
            std::this_thread::sleep_for(5ms);
        }
        fail_if_exit_requested(f);
        return from;  // 超时：状态未变
    };

    // ── 启动任务 ────────────────────────────────────────────────────────────
    f.fsm->dispatch(make_schedule_start(
        right_limit_active, left_limit_active, f.p.combined_passes));
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
                   f.p.pid.pitch_drop_threshold,
                   f.p.pid.roll_threshold,
                   f.p.pid.max_output)
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
