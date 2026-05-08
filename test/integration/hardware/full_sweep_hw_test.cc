/*
 * 全流程硬件集成测试（全扫描序列 + 限位开关急停链路联动）
 *
 * 测试分组：
 *   [hw_sweep][with_pid]     - 带航向 PID 的清扫序列：验证 IMU 差速补偿 + 命令队列正确性
 *   [hw_sweep][no_pid]       - 纯速度环清扫序列：验证 4 轮在线状态与帧统计
 *   [hw_sweep][limit_stop]   - 手动触发左限位：验证 GPIO → 回调 → emergency_override 全链路
 *   [hw_sweep][limit_stop][right] - 手动触发右限位：同上，针对右限位（RIGHT）
 *
 * 硬件接线要求：
 *   CAN  : can0，行走电机组 M1502E_111（motor_id_base=1）
 *   IMU  : /dev/ttyS1，WIT Motion 9轴（9600 baud，与 config.json 对齐）
 *   GPIO : gpiochip5 line0 = 左限位（LEFT），line1 = 右限位（RIGHT）
 *   滚刷 : 无真实硬件，BrushMotor 层不参与本测试（扫描序列直接操作 WalkMotorGroup）
 *
 * 运行方法（交叉编译后在目标机上）：
 *   ./hw_tests "[hw_sweep][with_pid]"       # 带 PID 清扫序列（需接 IMU）
 *   ./hw_tests "[hw_sweep][no_pid]"         # 无 PID 清扫序列（仅需 CAN）
 *   ./hw_tests "[hw_sweep][limit_stop]"     # FSM 全链路限位急停（手动触发）
 *
 * 安全注意事项：
 *   - 运行清扫测试前确认机器人在轨道上，前方有足够的行走空间（≥ 3s × vmax）
 *   - 限位测试低速运行（30 RPM），触发后 emergency_override 立即停车
 *   - 所有测试结束时电机自动失能（disable_all），CAN 关闭后电机通信超时自保护
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

using namespace robot;
using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════════
//  硬件配置常量（从 hw_config.h 加载）
// ═══════════════════════════════════════════════════════════════════════════

static const hw::HwParams kp = hw::load_hw_test_config();
/// kSweepLoops 由 sweep_duration_ms / loop_period_ms 计算得出
static int sweep_loops() {
    return kp.sweep_duration_ms / kp.loop_period_ms;
}

static device::WalkMotorGroup make_hw_group(const std::shared_ptr<driver::LinuxCanSocket>& can) {
    return device::WalkMotorGroup(can,
                                  kp.motor_id_base,
                                  kp.comm_timeout_ms,
                                  kp.termination_init_enabled,
                                  kp.termination_init_retry_count,
                                  kp.termination_motor_id);
}

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具
// ═══════════════════════════════════════════════════════════════════════════

/// 轮询等待条件满足，每 10ms 轮询一次，最多等待 wait_ms 毫秒
template <typename Pred>
static bool wait_for(Pred pred, int wait_ms) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred())
            return true;
        std::this_thread::sleep_for(10ms);
    }
    return pred();
}

/// 安全停止序列：
///   1. 入队零速命令（不直接发送，等 update() 消费）
///   2. 禁用 heading PID
///   3. 调用 update() 循环排干命令队列并最终发送零速帧
///   4. disable_all（直接 CAN 发送失能帧，不走队列）
///
/// @note 若 override_active_ 为 true，update() 进入步骤 3 会直接 return（不发心跳）。
///       此时 disable_all 仍能直接发出失能帧；电机靠 comm_timeout 自保护。
static void safe_stop(device::WalkMotorGroup& group) {
    group.set_speeds(0.0f, 0.0f, 0.0f, 0.0f);
    group.enable_heading_control(false);
    for (int i = 0; i < 5; ++i) {
        group.update(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }
    group.disable_all();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TEST 1：带航向 PID 的清扫序列
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("[hw_sweep][with_pid] 带 PID 全扫描序列", "[hw_sweep][with_pid]") {
    // ── 1. 建立 WalkMotorGroup（真实 CAN）─────────────────────────────────
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto group = make_hw_group(can);
    REQUIRE(group.open() == device::DeviceError::OK);
    spdlog::info(
        "[hw_sweep][with_pid] CAN {} 打开成功，motor_id_base={}", kp.can_iface, kp.motor_id_base);

    // ── 2. 建立 ImuDevice（真实 UART）─────────────────────────────────────
    hal::UartConfig imu_cfg;
    imu_cfg.baudrate = kp.imu_baud;
    auto imu_serial = std::make_shared<driver::LibSerialPort>(kp.imu_port, imu_cfg);
    device::ImuDevice imu(imu_serial);
    REQUIRE(imu.open());
    spdlog::info(
        "[hw_sweep][with_pid] {} 打开成功 @{} baud，等待首帧...", kp.imu_port, kp.imu_baud);

    // 等待首帧有效 IMU 数据（最多 3 秒）
    bool imu_valid = wait_for([&] { return imu.get_latest().valid; }, 3000);
    REQUIRE(imu_valid);
    const float init_yaw = imu.get_latest().yaw_deg;
    spdlog::info("[hw_sweep][with_pid] IMU 首帧 yaw={:.2f}°", init_yaw);

    // ── 3. 初始化行走电机（先使能，再切速度模式）─────────────────────────
    // M1502E 硬件确认：ENABLE 帧覆盖模式位，必须先 ENABLE 再 SPEED（Q5 修复）
    REQUIRE(group.enable_all() == device::DeviceError::OK);
    REQUIRE(group.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);  // 等待电机切换完成

    // ── 4. 配置航向速率 PID（速率 PID，无需锁定航向目标）─────────────────────
    device::WalkMotorGroup::HeadingPidParams pid;
    pid.kp = 0.5f;
    pid.ki = 0.05f;
    pid.kd = 0.1f;
    pid.max_output = 30.0f;
    pid.integral_limit = 20.0f;
    pid.deadband_rate_dps = 2.0f;
    group.set_heading_pid_params(pid);
    group.enable_heading_control(true);

    // ── 5. 入队清扫速度（前进：LT=+spd, RT=+spd, LB=-spd, RB=-spd）────────
    // 物理安装：LT/RT 正转=前进，LB/RB 安装方向相反，负值=前进
    REQUIRE(group.set_speeds(kp.sweep_rpm, kp.sweep_rpm, -kp.sweep_rpm, -kp.sweep_rpm) ==
            device::DeviceError::OK);
    spdlog::info(
        "[hw_sweep][with_pid] 开始清扫：{:.1f} RPM，速率 PID 已使能（deadband={:.1f}°/s）",
        kp.sweep_rpm,
        pid.deadband_rate_dps);

    // ── 6. 运行 update() 循环（60 次 × 50ms = 3 秒）──────────────────────
    float filtered_yaw = init_yaw;
    float filtered_omega_z = 0.0f;
    bool filtered_omega_z_inited = false;
    for (int i = 0; i < sweep_loops(); ++i) {
        const float raw_yaw = imu.get_latest().yaw_deg;
        const float raw_omega_z = imu.get_latest().gyro[2] * (180.0f / 3.14159265f);
        // EMA α=0.8，τ≈100ms @50ms 周期，与生产 MotionService::update() 完全一致
        filtered_yaw = 0.8f * filtered_yaw + 0.2f * raw_yaw;
        if (!filtered_omega_z_inited) {
            filtered_omega_z = raw_omega_z;
            filtered_omega_z_inited = true;
        } else {
            filtered_omega_z = 0.8f * filtered_omega_z + 0.2f * raw_omega_z;
        }
        group.update(filtered_yaw, filtered_omega_z);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));

        // 约 1 秒后检查所有轮在线状态
        if (i == 19) {
            auto gs = group.get_group_status();
            for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
                spdlog::info("[hw_sweep][with_pid] 1s 检查 wheel[{}] online={} speed={:.2f}",
                             w,
                             gs.wheel[w].online,
                             gs.wheel[w].speed_rpm);
                CHECK(gs.wheel[w].online);
            }
        }
    }

    // ── 7. 记录最终诊断数据 ───────────────────────────────────────────────
    auto gd = group.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        spdlog::info("[hw_sweep][with_pid] wheel[{}] online={} speed={:.2f}rpm target={:.2f}",
                     w,
                     gd.wheel[w].online,
                     gd.wheel[w].speed_rpm,
                     gd.wheel[w].target_value);
    }
    spdlog::info(
        "[hw_sweep][with_pid] ctrl_frames={} ctrl_errs={}", gd.ctrl_frame_count, gd.ctrl_err_count);

    // 基础断言：3秒后所有轮必须在线，无 CAN 发送错误
    REQUIRE(gd.ctrl_err_count == 0u);
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        REQUIRE(gd.wheel[w].online);

    // ── 8. 紧急覆盖 / 解除链路验证（SECTION 各独立运行一次主循环）──────────

    SECTION("emergency_override 后 is_override_active 为 true") {
        group.emergency_override(0.0f);
        CHECK(group.is_override_active());
        spdlog::info("[hw_sweep][with_pid] emergency_override 已触发，override_active=true ✓");
    }

    SECTION("clear_override 入队后 update() 驱动解除 override_active") {
        group.emergency_override(0.0f);
        REQUIRE(group.is_override_active());

        group.clear_override();      // 投递 CLEAR_OVERRIDE 到命令队列
        group.update(filtered_yaw, filtered_omega_z);  // update() step2 消费 CLEAR_OVERRIDE → store(false)

        CHECK_FALSE(group.is_override_active());
        spdlog::info(
            "[hw_sweep][with_pid] clear_override + 1次 update() → override_active=false ✓");
    }

    // ── 9. 安全停止（每个 SECTION 均执行）───────────────────────────────────
    safe_stop(group);
    imu.close();
    group.close();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TEST 2：无 PID 纯速度环清扫序列
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("[hw_sweep][no_pid] 无 PID 纯速度环清扫序列", "[hw_sweep][no_pid]") {
    // ── 1. 建立 WalkMotorGroup（仅 CAN，无 IMU 依赖）─────────────────────
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto group = make_hw_group(can);
    REQUIRE(group.open() == device::DeviceError::OK);
    spdlog::info("[hw_sweep][no_pid] CAN {} 打开成功", kp.can_iface);

    // ── 2. 初始化：先使能，再切速度模式 ──────────────────────────────────
    REQUIRE(group.enable_all() == device::DeviceError::OK);
    REQUIRE(group.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    // ── 3. 入队清扫速度（不启用 PID）────────────────────────────────────
    REQUIRE(group.set_speeds(kp.sweep_rpm, kp.sweep_rpm, -kp.sweep_rpm, -kp.sweep_rpm) ==
            device::DeviceError::OK);
    spdlog::info("[hw_sweep][no_pid] 开始清扫：{:.1f} RPM，PID 禁用", kp.sweep_rpm);

    // ── 4. 运行 update() 循环（60 次 × 50ms = 3 秒）──────────────────────
    for (int i = 0; i < sweep_loops(); ++i) {
        group.update(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));

        // 约 1 秒后检查各轮状态和速度合理性
        if (i == 19) {
            for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
                auto d = group.get_wheel_diagnostics(static_cast<device::WalkMotorGroup::Wheel>(w));
                spdlog::info("[hw_sweep][no_pid] 1s 检查 wheel[{}] online={} speed={:.2f}rpm",
                             w,
                             d.online,
                             d.speed_rpm);
                CHECK(d.online);
                CHECK(d.speed_rpm >= -210.0f);
                CHECK(d.speed_rpm <= 210.0f);
            }
        }
    }

    // ── 5. 最终断言 + 诊断日志 ───────────────────────────────────────────
    auto gd = group.get_group_diagnostics();
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w) {
        spdlog::info("[hw_sweep][no_pid] wheel[{}] online={} speed={:.2f}rpm xferr={}",
                     w,
                     gd.wheel[w].online,
                     gd.wheel[w].speed_rpm,
                     gd.wheel[w].feedback_lost_count);
    }
    spdlog::info(
        "[hw_sweep][no_pid] ctrl_frames={} ctrl_errs={}", gd.ctrl_frame_count, gd.ctrl_err_count);

    REQUIRE(gd.ctrl_err_count == 0u);
    for (int w = 0; w < device::WalkMotorGroup::kWheelCount; ++w)
        REQUIRE(gd.wheel[w].online);

    // ── 6. 安全停止 ──────────────────────────────────────────────────────
    safe_stop(group);
    group.close();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TEST 3：手动触发左限位 → 急停全链路验证
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("[hw_sweep][limit_stop] 手动触发左限位急停链路（FSM 全链路）", "[hw_sweep][limit_stop]") {
    // ── 1. 建立 WalkMotorGroup（低速，安全演示）──────────────────────────
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto group = make_hw_group(can);
    REQUIRE(group.open() == device::DeviceError::OK);

    REQUIRE(group.enable_all() == device::DeviceError::OK);
    REQUIRE(group.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    // 前进低速：LT=+rpm, RT=+rpm, LB=-rpm, RB=-rpm
    REQUIRE(group.set_speeds(
                kp.limit_test_rpm, kp.limit_test_rpm, -kp.limit_test_rpm, -kp.limit_test_rpm) ==
            device::DeviceError::OK);
    spdlog::info("[hw_limit][manual_left] 行走电机已启动 {:.1f} RPM（前进）", kp.limit_test_rpm);

    // ── 2. 建立左限位开关（真实 gpiochip5 line0）─────────────────────────
    auto left_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line, "hw_test_left");
    device::LimitSwitch left_sw(left_gpio, device::LimitSide::LEFT);

    // 限位触发回调：复制 SafetyMonitor::on_limit_trigger() 的核心逻辑：
    //   → emergency_override(0.0f) 立即发送停车帧 + 置 override_active_=true
    // 注意：回调在 GPIO 监控线程（SCHED_FIFO 95，CPU4）中执行，必须极短（<100μs）
    left_sw.set_trigger_callback([&](device::LimitSide /*side*/) {
        group.emergency_override(0.0f);
        spdlog::warn("[hw_limit][manual_left] 左限位已触发！emergency_override(0) 发出");
    });

    // open: rt_priority=95（SCHED_FIFO，与生产对齐）, debounce_ms=2, cpu_affinity=0（不绑核）
    REQUIRE(left_sw.open(95, 2, 0));
    left_sw.start_monitoring();
    spdlog::info("[hw_limit][manual_left] 左限位 GPIO 监控已启动（{} line{}）",
                 kp.gpio_chip,
                 kp.left_limit_line);

    // ── 3. 启动 update 后台线程（walk_ctrl 路径仿真，50ms 心跳）─────────────
    // 注：先启动 update 线程，motors 才真正开始周期性发送控制帧
    std::atomic<bool> stop_loop{false};
    std::thread update_thread([&] {
        while (!stop_loop.load(std::memory_order_relaxed)) {
            group.update(0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
        }
    });

    // ── 4. 提示操作员手动触发 ─────────────────────────────────────────────
    spdlog::warn("[hw_limit][manual_left] ====================================================");
    spdlog::warn("[hw_limit][manual_left] 请在 {} 秒内手动触发左限位开关",
                 (kp.limit_timeout_sec * 1000) / 1000);
    spdlog::warn("[hw_limit][manual_left]   硬件：{} line{}（低有效，下降沿触发）",
                 kp.gpio_chip,
                 kp.left_limit_line);
    spdlog::warn("[hw_limit][manual_left]   方法：将遮挡物接近传感器感应面，等待低电平触发");
    spdlog::warn("[hw_limit][manual_left] ====================================================");

    // ── 5. 等待限位触发（最多 limit_timeout_sec 秒）─────────────────────────
    const bool triggered =
        wait_for([&] { return left_sw.is_triggered(); }, (kp.limit_timeout_sec * 1000));

    // ── 6. 先安全关停 update 线程，再执行断言（防止 thread::~thread 调用 terminate）──
    stop_loop.store(true, std::memory_order_relaxed);
    update_thread.join();

    // ── 7. 断言链路完整性 ─────────────────────────────────────────────────
    REQUIRE(triggered);
    REQUIRE(group.is_override_active());
    spdlog::info(
        "[hw_limit][manual_left] 急停链路验证通过："
        "is_triggered={}  override_active={}",
        left_sw.is_triggered(),
        group.is_override_active());

    // ── 8. 关闭（析构顺序：left_sw → group，safe_stop 走 disable_all 直接发帧）──
    left_sw.close();
    safe_stop(group);
    group.close();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TEST 4：手动触发右限位 → 急停全链路验证
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("[hw_sweep][limit_stop] 手动触发右限位急停链路（FSM 全链路）",
          "[hw_sweep][limit_stop][right]") {
    // ── 1. 建立 WalkMotorGroup（低速后退，模拟返程场景）──────────────────
    auto can = std::make_shared<driver::LinuxCanSocket>(kp.can_iface);
    auto group = make_hw_group(can);
    REQUIRE(group.open() == device::DeviceError::OK);

    REQUIRE(group.enable_all() == device::DeviceError::OK);
    REQUIRE(group.set_mode_all(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    // 后退低速：LT=-rpm, RT=-rpm, LB=+rpm, RB=+rpm（模拟 start_returning() 方向）
    REQUIRE(group.set_speeds(
                -kp.limit_test_rpm, -kp.limit_test_rpm, kp.limit_test_rpm, kp.limit_test_rpm) ==
            device::DeviceError::OK);
    spdlog::info("[hw_limit][manual_right] 行走电机已启动 {:.1f} RPM（后退）", kp.limit_test_rpm);

    // ── 2. 建立右限位开关（真实 gpiochip5 line1）─────────────────────────
    auto right_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_limit_line, "hw_test_right");
    device::LimitSwitch right_sw(right_gpio, device::LimitSide::RIGHT);

    right_sw.set_trigger_callback([&](device::LimitSide /*side*/) {
        group.emergency_override(0.0f);
        spdlog::warn("[hw_limit][manual_right] 右限位已触发！emergency_override(0) 发出");
    });

    REQUIRE(right_sw.open(95, 2, 0));
    right_sw.start_monitoring();
    spdlog::info("[hw_limit][manual_right] 右限位 GPIO 监控已启动（{} line{}）",
                 kp.gpio_chip,
                 kp.right_limit_line);

    // ── 3. 启动 update 后台线程 ───────────────────────────────────────────
    std::atomic<bool> stop_loop{false};
    std::thread update_thread([&] {
        while (!stop_loop.load(std::memory_order_relaxed)) {
            group.update(0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
        }
    });

    // ── 4. 提示操作员手动触发 ─────────────────────────────────────────────
    spdlog::warn("[hw_limit][manual_right] ====================================================");
    spdlog::warn("[hw_limit][manual_right] 请在 {} 秒内手动触发右限位开关",
                 (kp.limit_timeout_sec * 1000) / 1000);
    spdlog::warn("[hw_limit][manual_right]   硬件：{} line{}（低有效，下降沿触发）",
                 kp.gpio_chip,
                 kp.right_limit_line);
    spdlog::warn("[hw_limit][manual_right]   方法：将遮挡物接近传感器感应面，等待低电平触发");
    spdlog::warn("[hw_limit][manual_right] ====================================================");

    // ── 5. 等待限位触发 ───────────────────────────────────────────────────
    const bool triggered =
        wait_for([&] { return right_sw.is_triggered(); }, (kp.limit_timeout_sec * 1000));

    // ── 6. 先安全关停 update 线程，再断言 ────────────────────────────────
    stop_loop.store(true, std::memory_order_relaxed);
    update_thread.join();

    // ── 7. 断言链路完整性 ─────────────────────────────────────────────────
    REQUIRE(triggered);
    REQUIRE(group.is_override_active());
    spdlog::info(
        "[hw_limit][manual_right] 急停链路验证通过："
        "is_triggered={}  override_active={}",
        right_sw.is_triggered(),
        group.is_override_active());

    // ── 8. 关闭 ──────────────────────────────────────────────────────────
    right_sw.close();
    safe_stop(group);
    group.close();
}
