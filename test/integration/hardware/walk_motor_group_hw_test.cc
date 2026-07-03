// test/integration/hardware/walk_motor_group_hw_test.cc
/**
 * 行走电机组硬件单元测试（4 轮 M1502E_111，CAN 总线）
 *
 * 测试分组：
 *   [hw_walk][open_close]              - CAN open/close
 *   [hw_walk][enable_disable]          - 模式控制（使能/失能）
 *   [hw_walk][all_online]              - 全轮 10ms 反馈上线
 *   [hw_walk][fwd_no_pid]              - 纯 WalkMotorGroup 前进（20 RPM，3s）
 *   [hw_walk][rev_speed]               - 反转（-20 RPM，2s）
 *   [hw_walk][emergency_override]      - 急停：override 后下一 update 速度=0
 *   [hw_walk][override_blocks_set_speeds] - override 期间 set_speeds 不发帧
 *   [hw_walk][clear_override]          - clear_override 后恢复驱动
 *   [hw_walk][comm_timeout_self_stop]  - 通信超时后电机自保护（打印观察）
 *   [hw_walk][frame_stats_no_pid]      - 帧统计（纯执行器路径，10s）
 *
 * 运行方法（目标板，can0 已 up 500kbps）：
 *   ./hw_tests "[hw_walk]"
 *   ./hw_tests "[hw_walk][fwd_no_pid]"
 *
 * 安全：所有测试速度 ≤ 20 RPM；每段结束 disable_all() + close()。
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

static const hw::HwParams kp = hw::load_hw_test_config();

static device::WalkMotorGroup make_hw_group(const std::shared_ptr<driver::LinuxCanSocket>& can,
                                            uint16_t comm_timeout_ms = kp.comm_timeout_ms) {
    return device::WalkMotorGroup(can,
                                  kp.motor_id_base,
                                  comm_timeout_ms,
                                  kp.termination_init_enabled,
                                  kp.termination_init_retry_count,
                                  kp.termination_motor_id);
}

static void run_control_updates(device::WalkMotorGroup& grp, std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    const auto period = std::chrono::milliseconds(kp.loop_period_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        grp.update();
        std::this_thread::sleep_for(period);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][open_close]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup CAN 初始化与关闭", "[hw_walk][open_close]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    spdlog::info("[hw_walk][open_close] CAN open ✓");
    grp.close();
    spdlog::info("[hw_walk][open_close] CAN close ✓");
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][enable_disable]
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 使能与失能", "[hw_walk][enable_disable]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);

    CHECK(grp.enable_all() == device::DeviceError::OK);
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
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    // 配置 10ms 主动上报（100Hz），使电机快速上线
    CHECK(grp.set_feedback_mode_all(10u) == device::DeviceError::OK);

    // 等待最多 kOnlineTimeoutMs ms，轮询直到全部上线
    bool all_online = false;
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kp.online_timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        auto gs = grp.get_group_status();
        all_online = true;
        for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
            if (!gs.wheel[w].online) {
                all_online = false;
                break;
            }
        if (all_online)
            break;
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
// [hw_walk][fwd_no_pid] — 纯 WalkMotorGroup 前进
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 前进（纯执行器路径）", "[hw_walk][fwd_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);

    std::this_thread::sleep_for(300ms);  // 等待上线

    // 前进 3s
    grp.set_speed_uniform(kp.test_speed_rpm);
    run_control_updates(grp, 3s);

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][fwd_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count,
                 gd.ctrl_err_count);
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        spdlog::info("[hw_walk][fwd_no_pid] wheel[{}] online={} speed={:.2f}rpm",
                     w,
                     gd.wheel[w].online,
                     gd.wheel[w].speed_rpm);

    // 所有轮在线且速度在预期范围内。set_speed_uniform(rpm) 会下发
    // LT/RT=rpm, LB/RB=-rpm；下轮因安装方向相反，前进时反馈应为负值。
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        CHECK(gd.wheel[w].online);
        if (w < 2) {
            CHECK(gd.wheel[w].speed_rpm >= 10.0f);
            CHECK(gd.wheel[w].speed_rpm <= 60.0f);
        } else {
            CHECK(gd.wheel[w].speed_rpm <= -10.0f);
            CHECK(gd.wheel[w].speed_rpm >= -60.0f);
        }
    }
    CHECK(gd.ctrl_err_count == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][rev_speed] — 反转
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 反转", "[hw_walk][rev_speed]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(-kp.test_speed_rpm);
    run_control_updates(grp, 2s);

    auto gd = grp.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        spdlog::info("[hw_walk][rev_speed] wheel[{}] speed={:.2f}rpm", w, gd.wheel[w].speed_rpm);
        // set_speed_uniform(rpm) → set_speeds(rpm, rpm, -rpm, -rpm)
        // LT(0)/RT(1): 正转=前进，反转(rpm<0) → speed < 0
        // LB(2)/RB(3): 安装方向相反，负转=前进，反转(rpm>0) → speed > 0
        if (w < 2) {
            CHECK(gd.wheel[w].speed_rpm < 0.0f);
        } else {
            CHECK(gd.wheel[w].speed_rpm > 0.0f);
        }
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
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 先建立运动
    grp.set_speed_uniform(kp.test_speed_rpm);
    run_control_updates(grp, 500ms);

    // 急停：emergency_override 本身会发一帧 stop 帧（ctrl_frame_count+1），
    // frames_before 需在其之后抓取，以便验证后续 update() 不再发帧
    CHECK(grp.emergency_override(0.0f) == device::DeviceError::OK);
    CHECK(grp.is_override_active());
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;

    // 调 update：override 激活时不应再发运动帧，帧计数不增加
    grp.update();
    std::this_thread::sleep_for(100ms);
    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][emergency_override] frames_before={} frames_after={}",
                 frames_before,
                 frames_after);
    CHECK(frames_after == frames_before);  // override 期间无新控制帧

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][override_blocks_set_speeds] — override 期间 set_speeds 封锁
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup override 封锁 set_speeds", "[hw_walk][override_blocks_set_speeds]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

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
    grp.set_speeds(kp.test_speed_rpm, kp.test_speed_rpm, kp.test_speed_rpm, kp.test_speed_rpm);
    grp.update();
    std::this_thread::sleep_for(200ms);

    const uint32_t frames_after = grp.get_group_diagnostics().ctrl_frame_count;
    spdlog::info("[hw_walk][override_blocks_set_speeds] frames_before={} frames_after={}",
                 frames_before,
                 frames_after);
    CHECK(frames_after == frames_before);  // 没有新帧被发出

    grp.disable_all();
    grp.close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_walk][clear_override] — 解除急停后恢复驱动
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 解除急停后恢复驱动", "[hw_walk][clear_override]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.emergency_override(0.0f);
    REQUIRE(grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已激活");

    grp.clear_override();
    // clear_override() 在下一次 update() 中生效：
    // 生效当拍仅解除锁存，不会立即恢复上一条 normal 心跳
    grp.update();
    CHECK(!grp.is_override_active());
    spdlog::info("[hw_walk][clear_override] override 已解除");

    // 解除后可以重新驱动
    const uint32_t frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    grp.set_speed_uniform(kp.test_speed_rpm);
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

    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can, kShortTimeout);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    // 短暂前进
    grp.set_speed_uniform(kp.test_speed_rpm);
    run_control_updates(grp, 100ms);
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
// [hw_walk][frame_stats_no_pid] — 帧统计（纯执行器路径）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("WalkMotorGroup 帧统计（纯执行器路径，10s）", "[hw_walk][frame_stats_no_pid]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    grp.set_feedback_mode_all(10u);
    grp.enable_all();
    grp.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(300ms);

    grp.set_speed_uniform(kp.test_speed_rpm);
    const auto frames_before = grp.get_group_diagnostics().ctrl_frame_count;
    run_control_updates(grp, 5s);

    auto gd = grp.get_group_diagnostics();
    spdlog::info("[hw_walk][frame_stats_no_pid] ctrl_frames={} ctrl_errs={}",
                 gd.ctrl_frame_count,
                 gd.ctrl_err_count);

    CHECK(gd.ctrl_frame_count >= frames_before + 90u);  // 5s × 20Hz，允许少量调度抖动
    CHECK(gd.ctrl_err_count == 0u);

    grp.set_speed_uniform(0.0f);
    grp.update();
    std::this_thread::sleep_for(200ms);
    grp.disable_all();
    grp.close();
}

TEST_CASE("WalkMotorGroup 启动阶段终端电阻初始化后仍可正常进入控制流程",
          "[hw_walk][termination_init_startup]") {
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto grp = make_hw_group(can);

    REQUIRE(grp.open() == device::DeviceError::OK);
    CHECK(grp.set_feedback_mode_all(10u) == device::DeviceError::OK);
    CHECK(grp.enable_all() == device::DeviceError::OK);
    CHECK(grp.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);

    grp.disable_all();
    grp.close();
}
