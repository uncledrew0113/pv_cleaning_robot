// test/integration/hardware/walk_motor_group_hw_test.cc
/**
 * 行走电机组硬件单元测试（4 轮 M1502E_111，CAN 总线）
 *
 * 测试分组：
 *   [hw_walk][open_close]              - CAN open/close
 *   [hw_walk][enable_disable]          - 模式控制（使能/失能）
 *   [hw_walk][all_online]              - 全轮 10ms 反馈上线
 *   [hw_walk][fwd_no_pid]              - PID 关闭前进（30 RPM，3s）
 *   [hw_walk][fwd_with_pid]            - PID 开启前进（30 RPM，5s，yaw 漂移 < 5°）
 *   [hw_walk][pid_straightness]        - PID 开/关直线对比（各 5s，打印漂移量）
 *   [hw_walk][rev_speed]               - 反转（-30 RPM，2s）
 *   [hw_walk][emergency_override]      - 急停：override 后下一 update 速度=0
 *   [hw_walk][override_blocks_set_speeds] - override 期间 set_speeds 不发帧
 *   [hw_walk][clear_override]          - clear_override 后恢复驱动
 *   [hw_walk][comm_timeout_self_stop]  - 通信超时后电机自保护（打印观察）
 *   [hw_walk][frame_stats_no_pid]      - 帧统计（PID 关，10s）
 *   [hw_walk][frame_stats_with_pid]    - 帧统计（PID 开，10s）
 *
 * 运行方法（目标板，can0 已 up 500kbps）：
 *   ./hw_tests "[hw_walk]"
 *   ./hw_tests "[hw_walk][fwd_no_pid]"
 *
 * 安全：所有测试速度 ≤ 30 RPM；每段结束 disable_all() + close()。
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][open_close]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup CAN 初始化与关闭", "[hw_walk][open_close]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    spdlog::info("[hw_walk][open_close] CAN open ✓");
    grp.close();
    spdlog::info("[hw_walk][open_close] CAN close ✓");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][enable_disable]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 使能与失能", "[hw_walk][enable_disable]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);

    CHECK(grp.enable_all()  == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);
    spdlog::info("[hw_walk][enable_disable] enable_all ✓");

    CHECK(grp.disable_all() == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);
    spdlog::info("[hw_walk][enable_disable] disable_all ✓");

    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][all_online]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 全轮联机", "[hw_walk][all_online]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    // 配置 10ms 主动上报（100Hz），使电机快速上线
    CHECK(grp.set_feedback_mode_all(10u) == device::DeviceError::OK);

    // 等待最多 kOnlineTimeoutMs ms，轮询直到全部上线
    bool all_online = false;
    auto deadline   = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(hw::kOnlineTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        auto gs = grp.get_group_status();
        all_online = true;
        for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
            if (!gs.wheel[w].online) { all_online = false; break; }
        if (all_online) break;
        std::this_thread::sleep_for(50ms);
    }

    // 打印各轮状态
    auto gd = grp.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_walk][all_online] wheel[{}] online={}", w, gd.wheel[w].online);

    REQUIRE(all_online);  // 600ms 内全部上线

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][fwd_no_pid] — PID 关闭前进
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 前进（PID 关闭）", "[hw_walk][fwd_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(false);  // PID 关闭
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);

    std::this_thread::sleep_for(300ms);  // 等待上线

    // 前进 3s
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 6; ++i) {  // 6 × 500ms = 3s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][fwd_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_walk][fwd_no_pid] wheel[{}] online={} speed={:.2f}rpm",
                     w, gd.wheel[w].online, gd.wheel[w].speed_rpm);

    // 所有轮在线且速度在预期范围内
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        CHECK(gd.wheel[w].online);
        CHECK(gd.wheel[w].speed_rpm >= 10.0f);   // 低限：目标 30，允许负载下降至 10
        CHECK(gd.wheel[w].speed_rpm <= 60.0f);   // 高限：30 + 100% 余量
    }
    CHECK(gd.ctrl_err_count == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][fwd_with_pid] — PID 开启前进，yaw 漂移 < 5°
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 前进（PID 开启，yaw 漂移 < 5°）", "[hw_walk][fwd_with_pid]") {
    // 需要 IMU 提供 yaw 数据
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);  // 等待 IMU 首帧

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(true);  // PID 开启
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 记录初始 yaw
    auto imu_data0 = imu->get_latest();
    REQUIRE(imu_data0.valid);
    const float yaw_start = imu_data0.yaw_deg;
    spdlog::info("[hw_walk][fwd_with_pid] 初始 yaw={:.2f}°", yaw_start);

    // 前进 5s（50ms update 周期）
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 100; ++i) {  // 100 × 50ms = 5s
        auto d = imu->get_latest();
        grp.update(d.valid ? d.yaw_deg : 0.0f);
        std::this_thread::sleep_for(50ms);
    }

    auto imu_data1 = imu->get_latest();
    REQUIRE(imu_data1.valid);
    const float yaw_end   = imu_data1.yaw_deg;
    const float yaw_drift = std::abs(yaw_end - yaw_start);
    spdlog::info("[hw_walk][fwd_with_pid] 结束 yaw={:.2f}°  漂移={:.2f}°",
                 yaw_end, yaw_drift);

    CHECK(yaw_drift < 5.0f);  // PID 开启下 5s 内漂移应 < 5°

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
    imu->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][pid_straightness] — PID 开关直线对比
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup PID 直线度量化对比", "[hw_walk][pid_straightness]") {
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);

    auto run_and_measure = [&](bool pid_on) -> float {
        auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
        device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);
        grp.open();
        grp.set_feedback_mode_all(10u);
        grp.enable_heading_control(pid_on);
        grp.enable_all();
        grp.set_mode_all(protocol::WalkMotorMode::SPEED);
        std::this_thread::sleep_for(300ms);

        auto d0 = imu->get_latest();
        const float yaw0 = d0.valid ? d0.yaw_deg : 0.0f;

        grp.set_speed_uniform(hw::kTestSpeedRpm);
        for (int i = 0; i < 100; ++i) {  // 5s
            auto d = imu->get_latest();
            grp.update(d.valid ? d.yaw_deg : 0.0f);
            std::this_thread::sleep_for(50ms);
        }

        auto d1 = imu->get_latest();
        const float yaw1  = d1.valid ? d1.yaw_deg : 0.0f;
        const float drift = std::abs(yaw1 - yaw0);

        grp.set_speed_uniform(0.0f);
        grp.update();
        std::this_thread::sleep_for(500ms);  // 停稳后下一次运行
        grp.disable_all();
        grp.close();
        return drift;
    };

    spdlog::warn("[hw_walk][pid_straightness] 开始 PID 关闭行走 5s...");
    const float drift_no_pid = run_and_measure(false);

    spdlog::warn("[hw_walk][pid_straightness] 请将机器人复位，等待 5s 后继续 PID 开启行走...");
    std::this_thread::sleep_for(5s);  // 给操作人员复位机器人

    spdlog::warn("[hw_walk][pid_straightness] 开始 PID 开启行走 5s...");
    const float drift_with_pid = run_and_measure(true);

    spdlog::info("[hw_walk][pid_straightness] ┌─────────────────────────────────┐");
    spdlog::info("[hw_walk][pid_straightness] │ PID 关闭 yaw 漂移: {:.2f}°", drift_no_pid);
    spdlog::info("[hw_walk][pid_straightness] │ PID 开启 yaw 漂移: {:.2f}°", drift_with_pid);
    spdlog::info("[hw_walk][pid_straightness] └─────────────────────────────────┘");

    // PID 开启后漂移应小于关闭时的漂移（或至少不更差）
    CHECK(drift_with_pid <= drift_no_pid + 1.0f);  // 容忍 1° 误差
    SUCCEED();  // 主要验收标准是打印可量化对比日志
    imu->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][rev_speed] — 反转
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 反转", "[hw_walk][rev_speed]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(-hw::kTestSpeedRpm);
    for (int i = 0; i < 4; ++i) {  // 4 × 500ms = 2s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        spdlog::info("[hw_walk][rev_speed] wheel[{}] speed={:.2f}rpm", w, gd.wheel[w].speed_rpm);
        CHECK(gd.wheel[w].speed_rpm < 0.0f);  // 反转时速度为负
    }

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][emergency_override] — 急停
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 急停（emergency_override）", "[hw_walk][emergency_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 先建立运动
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(500ms);

    // 急停
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    CHECK(grp.emergency_override(0.0f) == device::DeviceError::OK);
    CHECK(grp.is_override_active());

    // 调 update：override 激活时不应再发运动帧，帧计数不增加
    grp.update();
    std::this_thread::sleep_for(100ms);
    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][emergency_override] frames_before={} frames_after={}",
                 frames_before, frames_after);
    CHECK(frames_after == frames_before);  // override 期间无新控制帧

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][override_blocks_set_speeds] — override 期间 set_speeds 封锁
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup override 封锁 set_speeds", "[hw_walk][override_blocks_set_speeds]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 激活 override
    grp.emergency_override(0.0f);
    REQUIRE(grp.is_override_active());

    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;

    // 尝试发送速度帧（应被封锁）
    grp.set_speeds(hw::kTestSpeedRpm, hw::kTestSpeedRpm,
                   hw::kTestSpeedRpm, hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(200ms);

    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][override_blocks_set_speeds] frames_before={} frames_after={}",
                 frames_before, frames_after);
    CHECK(frames_after == frames_before);  // 没有新帧被发出

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][clear_override] — 解除急停后恢复驱动
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 解除急停后恢复驱动", "[hw_walk][clear_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.emergency_override(0.0f);
    REQUIRE(grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已激活");

    grp.clear_override();
    CHECK(!grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已解除");

    // 解除后可以重新驱动
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(200ms);
    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;

    CHECK(frames_after > frames_before);  // clear 后能发出新帧
    spdlog::info("[hw_walk][clear_override] 解除后 ctrl_frame_count 增加 {} 帧",
                 frames_after - frames_before);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][comm_timeout_self_stop] — 通信超时自保护（打印观察）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 通信超时自保护", "[hw_walk][comm_timeout_self_stop]") {
    constexpr uint16_t kShortTimeout = 300u;  // 300ms 超时（更短，便于测试）

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, kShortTimeout);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 短暂前进
    grp.set_speed_uniform(hw::kTestSpeedRpm);
    grp.update();
    std::this_thread::sleep_for(500ms);
    spdlog::info("[hw_walk][comm_timeout_self_stop] 电机运行中，即将关闭 CAN 总线...");

    // 关闭 CAN（停止发送心跳），等待超时
    grp.close();  // 停止 recv_loop，不再发帧

    spdlog::warn("[hw_walk][comm_timeout_self_stop] CAN 已关闭，电机应在 {}ms 内自保护停转",
                 kShortTimeout + 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(kShortTimeout + 100));

    // 此处电机应已超时自停（通过观察电机是否停止确认）
    spdlog::info("[hw_walk][comm_timeout_self_stop] ★ 请确认电机已停转 ★");
    SUCCEED();  // 结果由人工观察确认
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][frame_stats_no_pid] — 帧统计（PID 关）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 帧统计（PID 关，10s）", "[hw_walk][frame_stats_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(false);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 20; ++i) {  // 20 × 500ms = 10s
        grp.update();
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][frame_stats_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);

    CHECK(gd.ctrl_frame_count > 100u);  // 10s × ~20Hz = 200 帧
    CHECK(gd.ctrl_err_count   == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][frame_stats_with_pid] — 帧统计（PID 开）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 帧统计（PID 开，10s）", "[hw_walk][frame_stats_with_pid]") {
    auto imu_serial = std::make_shared<driver::LibSerialPort>(
        hw::kImuPort, hal::UartConfig{hw::kImuBaud});
    auto imu = std::make_shared<device::ImuDevice>(imu_serial);
    REQUIRE(imu->open());
    std::this_thread::sleep_for(500ms);

    auto can = std::make_shared<driver::LinuxCanSocket>(hw::kCanIface);
    device::WalkMotorGroup grp(can, hw::kMotorIdBase, hw::kCommTimeoutMs);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_heading_control(true);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(hw::kTestSpeedRpm);
    for (int i = 0; i < 20; ++i) {  // 20 × 500ms = 10s
        auto d = imu->get_latest();
        grp.update(d.valid ? d.yaw_deg : 0.0f);
        std::this_thread::sleep_for(500ms);
    }

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][frame_stats_with_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count, gd.ctrl_err_count);

    CHECK(gd.ctrl_frame_count > 100u);
    CHECK(gd.ctrl_err_count   == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
    imu->close();
}
