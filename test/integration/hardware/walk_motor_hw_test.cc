/*
 * 行走电机协议层与设备层单元测试（M1502E_111 CAN 协议）
 *
 * 测试分组：
 *   [protocol][walk_motor]      - WalkMotorCanCodec 帧编解码（纯内存，无 I/O）
 *   [device][walk_motor]        - WalkMotor 设备层（基于 MockCanBus，不依赖真实硬件）
 *   [device][walk_motor_group]  - WalkMotorGroup 设备层（MockCanBus，4轮批量控制）
 *   [integration][walk_motor]   - 真实 CAN 总线集成测试（须在目标机上运行）
 *
 * 运行方法（交叉编译后在目标机上）：
 *   ./unit_tests "[protocol][walk_motor]"     # 协议层单元测试
 *   ./unit_tests "[device][walk_motor]"       # WalkMotor 设备层 mock 测试
 *   ./unit_tests "[device][walk_motor_group]" # WalkMotorGroup mock 测试
 *   ./unit_tests "[integration][walk_motor]"  # 硬件集成测试
 *
 * 量化规则（来自 walk_motor_can_codec.cc）：
 *   速度：int16 = RPM × 100           → 分辨率 0.01 RPM
 *   电流：int16 = A × (32767/33)      → 分辨率 ~0.001 A
 *   位置：uint16 = deg × (32767/360)  → 分辨率 ~0.011°
 *
 * 帧格式（状态反馈 0x96+motor_id，8字节 big-endian）：
 *   DATA[0-1]: speed_rpm  (int16)
 *   DATA[2-3]: torque_a   (int16)
 *   DATA[4-5]: position   (uint16)
 *   DATA[6]:   fault_code
 *   DATA[7]:   mode
 */
#include <array>
#include <catch2/catch.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "mock/mock_can_bus.h"
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

using namespace robot;

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具
// ═══════════════════════════════════════════════════════════════════════════

// 量化精度：取 1.5 LSB 对应物理量作为比较 margin
static constexpr float kSpeedScale = 100.0f;  // int16 = RPM × 100
static constexpr float kCurrentScale = 32767.0f / 33.0f;
static constexpr float kPosScale = 32767.0f / 360.0f;
static constexpr float kSpeedEps = 1.5f / kSpeedScale;      // ≈ 0.015 RPM
static constexpr float kCurrentEps = 1.5f / kCurrentScale;  // ≈ 0.0015 A
static constexpr float kPosEps = 1.5f / kPosScale;          // ≈ 0.016°

/// 从 big-endian 两字节中恢复 int16
static int16_t be16s(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

/// 从 big-endian 两字节中恢复 uint16
static uint16_t be16u(uint8_t hi, uint8_t lo) {
    return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

/// 构造 M1502E_111 状态反馈帧（CAN ID = 0x96 + motor_id）
static robot::hal::CanFrame make_status_frame(
    uint8_t motor_id,
    float speed_rpm,
    float torque_a,
    float position_deg,
    protocol::WalkMotorFault fault = protocol::WalkMotorFault::NONE,
    protocol::WalkMotorMode mode = protocol::WalkMotorMode::SPEED) {
    robot::hal::CanFrame frame{};
    frame.id = protocol::kWalkMotorStatusBase + motor_id;
    frame.len = 8;

    auto speed_raw = static_cast<int16_t>(speed_rpm * kSpeedScale);
    auto torque_raw = static_cast<int16_t>(torque_a * kCurrentScale);
    auto pos_raw = static_cast<uint16_t>(position_deg * kPosScale);

    frame.data[0] = static_cast<uint8_t>((static_cast<uint16_t>(speed_raw) >> 8) & 0xFFu);
    frame.data[1] = static_cast<uint8_t>(static_cast<uint16_t>(speed_raw) & 0xFFu);
    frame.data[2] = static_cast<uint8_t>((static_cast<uint16_t>(torque_raw) >> 8) & 0xFFu);
    frame.data[3] = static_cast<uint8_t>(static_cast<uint16_t>(torque_raw) & 0xFFu);
    frame.data[4] = static_cast<uint8_t>((pos_raw >> 8) & 0xFFu);
    frame.data[5] = static_cast<uint8_t>(pos_raw & 0xFFu);
    frame.data[6] = static_cast<uint8_t>(fault);
    frame.data[7] = static_cast<uint8_t>(mode);
    return frame;
}

/// 等待条件满足（最多 wait_ms 毫秒），用于 recv_loop 联动测试
template <typename Pred>
static bool wait_for(Pred pred, int wait_ms = 200) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

// ═══════════════════════════════════════════════════════════════════════════
//  硬件测试配置常量（在目标机上修改以匹配实际接线）
// ═══════════════════════════════════════════════════════════════════════════
static constexpr char kCanIface[] = "can0";
static constexpr uint8_t kMotorId = 1u;

// ═══════════════════════════════════════════════════════════════════════════
//  Part 1：协议层 - WalkMotorCanCodec 构造与 CAN ID 映射
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("集成测试 WalkMotor - CAN总线打开与电机在线检测", "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotor motor(bus, kMotorId);

    SECTION("CAN 总线打开成功") {
        REQUIRE(motor.open() == device::DeviceError::OK);
        REQUIRE(bus->is_open());
        spdlog::info("[WalkMotor 集成] {} 打开成功，motor_id={}", kCanIface, kMotorId);
        motor.close();
    }

    SECTION("500ms 内收到状态反馈帧") {
        REQUIRE(motor.open() == device::DeviceError::OK);

        bool online = false;
        for (int i = 0; i < 50 && !online; ++i) {
            std::this_thread::sleep_for(10ms);
            online = motor.get_diagnostics().online;
        }

        auto d = motor.get_diagnostics();
        spdlog::info(
            "[WalkMotor 集成] online={}  frames={}  "
            "speed={:.2f}rpm  torque={:.3f}A  pos={:.2f}deg",
            d.online,
            d.feedback_frame_count,
            d.speed_rpm,
            d.torque_a,
            d.position_deg);

        REQUIRE(online);
        REQUIRE(d.feedback_frame_count > 0u);
        motor.close();
    }
}

// ── 集成测试 2：状态数据合理性验证 ──────────────────────────────────────────
TEST_CASE("集成测试 WalkMotor - 状态数据合理性验证", "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotor motor(bus, kMotorId);
    REQUIRE(motor.open() == device::DeviceError::OK);

    // 等待至少 10 帧（100 Hz → 约 100 ms）
    for (int i = 0; i < 20 && motor.get_diagnostics().feedback_frame_count < 10u; ++i)
        std::this_thread::sleep_for(10ms);

    auto d = motor.get_diagnostics();
    spdlog::info(
        "[WalkMotor 集成] speed={:.2f}rpm  torque={:.3f}A  pos={:.2f}deg  "
        "fault={}  mode={}  frames={}",
        d.speed_rpm,
        d.torque_a,
        d.position_deg,
        static_cast<int>(d.fault),
        static_cast<int>(d.mode),
        d.feedback_frame_count);

    CHECK(d.feedback_frame_count >= 5u);
    CHECK(d.speed_rpm >= -210.0f);
    CHECK(d.speed_rpm <= 210.0f);
    CHECK(d.torque_a >= -33.0f);
    CHECK(d.torque_a <= 33.0f);
    CHECK(d.position_deg >= 0.0f);
    CHECK(d.position_deg <= 360.0f);
    CHECK(d.can_err_count == 0u);

    motor.close();
}

// ── 集成测试 3：enable / disable 指令发送无错误 ──────────────────────────────
TEST_CASE("集成测试 WalkMotor - enable/disable 指令发送无错误", "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotor motor(bus, kMotorId);
    REQUIRE(motor.open() == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);  // 等待电机首帧

    SECTION("enable() 返回 OK，无发送错误") {
        auto err = motor.enable();
        spdlog::info("[WalkMotor 集成] enable() = {}", static_cast<int>(err));
        REQUIRE(err == device::DeviceError::OK);
        REQUIRE(motor.get_diagnostics().can_err_count == 0u);
    }

    SECTION("disable() 返回 OK，无发送错误") {
        motor.enable();
        std::this_thread::sleep_for(50ms);
        auto err = motor.disable();
        spdlog::info("[WalkMotor 集成] disable() = {}", static_cast<int>(err));
        REQUIRE(err == device::DeviceError::OK);
        REQUIRE(motor.get_diagnostics().can_err_count == 0u);
    }

    motor.disable();
    motor.close();
}

// ── 集成测试 4：低速（30 RPM）速度环运行验证 ────────────────────────────────
TEST_CASE("集成测试 WalkMotor - 低速(30RPM)速度环运行验证", "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotor motor(bus, kMotorId);
    REQUIRE(motor.open() == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    motor.enable();
    motor.set_mode(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(150ms);

    SECTION("set_speed(30.0) 返回 OK") {
        auto err = motor.set_speed(30.0f);
        REQUIRE(err == device::DeviceError::OK);
        spdlog::info("[WalkMotor 集成] set_speed(30.0) 已发送");
    }

    SECTION("速度给定后 500ms 内实测转速在合理范围（-10 ~ +110 RPM）") {
        REQUIRE(motor.set_speed(30.0f) == device::DeviceError::OK);
        std::this_thread::sleep_for(500ms);

        auto d = motor.get_diagnostics();
        spdlog::info(
            "[WalkMotor 集成] target={:.1f}rpm  actual={:.2f}rpm  "
            "torque={:.3f}A  can_err={}",
            d.target_value,
            d.speed_rpm,
            d.torque_a,
            d.can_err_count);

        CHECK(d.can_err_count == 0u);
        // 低速段：允许 ±80 RPM 的加速偏差
        CHECK(d.speed_rpm >= -10.0f);
        CHECK(d.speed_rpm <= 110.0f);
    }

    motor.disable();
    std::this_thread::sleep_for(100ms);
    motor.close();
}

// ── 集成测试 5：update() 心跳重发无通信错误 ─────────────────────────────────
TEST_CASE("集成测试 WalkMotor - update() 心跳重发无通信错误", "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotor motor(bus, kMotorId);
    REQUIRE(motor.open() == device::DeviceError::OK);
    std::this_thread::sleep_for(100ms);

    motor.enable();
    REQUIRE(motor.set_speed(0.0f) == device::DeviceError::OK);

    for (int i = 0; i < 5; ++i) {
        motor.update();
        std::this_thread::sleep_for(10ms);
    }

    auto d = motor.get_diagnostics();
    spdlog::info("[WalkMotor 集成] update ×5 完成  can_err={}  frames={}",
                 d.can_err_count,
                 d.feedback_frame_count);

    REQUIRE(d.can_err_count == 0u);

    motor.disable();
    motor.close();
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration test: 查询模式+读超时1000ms+速度环+心跳维持
// 步骤：
//  1) 设置反馈方式为查询（period_ms=0），通信超时1000ms
//  2) 切换速度环，设定目标速度
//  3) 每500ms调用 update() 并主动查询状态，持续2~3秒
//  4) 检查期间 online 始终为 true，未掉线
// 说明：需真实电机，TEST_CAN_IFACE 指定接口，motor_id=1
