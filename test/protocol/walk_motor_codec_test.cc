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
#include "pv_cleaning_robot/device/walk_motor_types.h"
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
//  Part 1：协议层 - WalkMotorCanCodec 构造与 CAN ID 映射
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - motor_id=1 → ctrl_id=0x32, status_id=0x97", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    REQUIRE(c.motor_id() == 1u);
    REQUIRE(c.ctrl_can_id() == 0x032u);
    REQUIRE(c.status_can_id() == 0x097u);
}

TEST_CASE("协议层 - motor_id=4 → ctrl_id=0x32, status_id=0x9A", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(4u);
    REQUIRE(c.ctrl_can_id() == 0x032u);
    REQUIRE(c.status_can_id() == 0x09Au);
}

TEST_CASE("协议层 - motor_id=5 → ctrl_id=0x33, status_id=0x9B", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(5u);
    REQUIRE(c.ctrl_can_id() == 0x033u);
    REQUIRE(c.status_can_id() == 0x09Bu);
}

TEST_CASE("协议层 - motor_id=8 → ctrl_id=0x33, status_id=0x9E", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(8u);
    REQUIRE(c.ctrl_can_id() == 0x033u);
    REQUIRE(c.status_can_id() == 0x09Eu);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 2：协议层 - 控制帧编码
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - encode_speed(-50) → big-endian(-5000) 在 slot[0-1]", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_speed(-50.0f);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    REQUIRE(raw == -5000);
}

TEST_CASE("协议层 - motor_id=6（组2）速度帧写在 slot[2-3]，CAN ID=0x33", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(6u);  // group2, slot = (6-5)*2 = 2
    auto frame = c.encode_speed(50.0f);

    REQUIRE(frame.id == 0x033u);
    int16_t raw = be16s(frame.data[2], frame.data[3]);
    REQUIRE(raw == 5000);
    REQUIRE(frame.data[0] == 0u);
    REQUIRE(frame.data[1] == 0u);
}

TEST_CASE("协议层 - encode_current(33) → 32767 in slot", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_current(33.0f);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    REQUIRE(raw == 32767);
}

TEST_CASE("协议层 - encode_current(-16.5) → 约 -16384 in slot", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_current(-16.5f);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    int16_t expected = static_cast<int16_t>(-16.5f * kCurrentScale);
    REQUIRE(raw == expected);
}

TEST_CASE("协议层 - encode_position(180) → uint16 ≈ 16384 in slot", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_position(180.0f);

    uint16_t raw = be16u(frame.data[0], frame.data[1]);
    REQUIRE(raw == Approx(16384).margin(2));  // 180 × 32767/360 = 16383.5
}

TEST_CASE("协议层 - encode_open_loop(1234) → 直接写入 slot", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_open_loop(1234);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    REQUIRE(raw == 1234);
}

// ── 模式帧 ──────────────────────────────────────────────────────────────────

TEST_CASE("协议层 - encode_query_firmware → ID=0x10B, 全8字节=0", "[protocol][walk_motor]") {
    auto frame = protocol::WalkMotorCanCodec::encode_query_firmware();

    REQUIRE(frame.id == 0x10Bu);
    REQUIRE(frame.len == 8u);
    for (int i = 0; i < 8; ++i)
        REQUIRE(frame.data[i] == 0u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 3：协议层 - 接收帧解码
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - decode_status 负速度正确解符号", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(2u);
    auto frame = make_status_frame(2u, -75.5f, -3.0f, 0.0f);
    auto result = c.decode_status(frame);

    REQUIRE(result.has_value());
    REQUIRE(result->speed_rpm == Approx(-75.5f).margin(kSpeedEps));
    REQUIRE(result->torque_a == Approx(-3.0f).margin(kCurrentEps));
}

TEST_CASE("协议层 - decode_status 帧 ID 不匹配 → nullopt", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);            // 期望 0x97
    auto frame = make_status_frame(2u, 0, 0, 0);  // ID=0x98

    REQUIRE_FALSE(c.decode_status(frame).has_value());
}

TEST_CASE("协议层 - decode_status 帧长度<8 → nullopt", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = make_status_frame(1u, 0, 0, 0);
    frame.len = 7;

    REQUIRE_FALSE(c.decode_status(frame).has_value());
}

TEST_CASE("协议层 - decode_status 故障码 OVER_CURRENT 正确解析", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = make_status_frame(
        1u, 0, 0, 0, protocol::WalkMotorFault::OVER_CURRENT, protocol::WalkMotorMode::SPEED);
    auto result = c.decode_status(frame);

    REQUIRE(result.has_value());
    REQUIRE(result->fault == protocol::WalkMotorFault::OVER_CURRENT);
}

TEST_CASE("协议层 - decode_mode_resp 正向：返回正确模式", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    robot::hal::CanFrame frame{};
    frame.id = protocol::kWalkMotorModeRespBase + 1u;  // 0x201
    frame.len = 8;
    frame.data[0] = static_cast<uint8_t>(protocol::WalkMotorMode::SPEED);

    auto result = c.decode_mode_resp(frame);
    REQUIRE(result.has_value());
    REQUIRE(*result == protocol::WalkMotorMode::SPEED);
}

TEST_CASE("协议层 - decode_mode_resp ID 不匹配 → nullopt", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    robot::hal::CanFrame frame{};
    frame.id = protocol::kWalkMotorModeRespBase + 2u;  // 0x202 ≠ 0x201
    frame.len = 8;

    REQUIRE_FALSE(c.decode_mode_resp(frame).has_value());
}

TEST_CASE("协议层 - decode_firmware 正向：版本信息全部正确", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    robot::hal::CanFrame frame{};
    frame.id = protocol::kWalkMotorFwRespBase + 1u;  // 0x32D
    frame.len = 8;
    frame.data[0] = 1u;   // motor_id
    frame.data[1] = 2u;   // sw_major
    frame.data[2] = 3u;   // sw_minor
    frame.data[3] = 4u;   // hw_major
    frame.data[4] = 5u;   // hw_minor
    frame.data[5] = 26u;  // year (2026)
    frame.data[6] = 3u;   // month
    frame.data[7] = 22u;  // day

    auto result = c.decode_firmware(frame);
    REQUIRE(result.has_value());
    REQUIRE(result->sw_major == 2u);
    REQUIRE(result->hw_minor == 5u);
    REQUIRE(result->year == 26u);
    REQUIRE(result->month == 3u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 4：协议层 - 4电机组批量编码
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - encode_group_speed id_base=5 → ID=0x33", "[protocol][walk_motor]") {
    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(5u, 50.0f, 50.0f, 50.0f, 50.0f);

    REQUIRE(frame.id == 0x033u);
    REQUIRE(be16s(frame.data[0], frame.data[1]) == 5000);
    REQUIRE(be16s(frame.data[2], frame.data[3]) == 5000);
}

TEST_CASE("协议层 - encode_group_speed 支持负速度", "[protocol][walk_motor]") {
    auto frame =
        protocol::WalkMotorCanCodec::encode_group_speed(1u, -100.0f, 100.0f, -50.0f, 50.0f);

    REQUIRE(be16s(frame.data[0], frame.data[1]) == -10000);
    REQUIRE(be16s(frame.data[2], frame.data[3]) == 10000);
    REQUIRE(be16s(frame.data[4], frame.data[5]) == -5000);
    REQUIRE(be16s(frame.data[6], frame.data[7]) == 5000);
}

TEST_CASE("协议层 - encode_group_current id_base=1 → 电流量化正确", "[protocol][walk_motor]") {
    auto frame = protocol::WalkMotorCanCodec::encode_group_current(1u, 33.0f, 0.0f, -33.0f, 16.5f);

    REQUIRE(be16s(frame.data[0], frame.data[1]) == 32767);
    REQUIRE(be16s(frame.data[2], frame.data[3]) == 0);
    REQUIRE(be16s(frame.data[4], frame.data[5]) == -32767);
    // 16.5A × (32767/33) ≈ 16384
    int16_t raw_165 = be16s(frame.data[6], frame.data[7]);
    REQUIRE(raw_165 == Approx(16.5f * kCurrentScale).margin(2));
}

TEST_CASE("协议层 - encode_group_open_loop → raw 值直接写入 4 个 slot", "[protocol][walk_motor]") {
    auto frame = protocol::WalkMotorCanCodec::encode_group_open_loop(1u, 1000, -2000, 3000, -4000);

    REQUIRE(frame.id == 0x032u);
    REQUIRE(be16s(frame.data[0], frame.data[1]) == 1000);
    REQUIRE(be16s(frame.data[2], frame.data[3]) == -2000);
    REQUIRE(be16s(frame.data[4], frame.data[5]) == 3000);
    REQUIRE(be16s(frame.data[6], frame.data[7]) == -4000);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 5：设备层 - WalkMotor
// ═══════════════════════════════════════════════════════════════════════════

