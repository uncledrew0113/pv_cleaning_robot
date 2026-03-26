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

TEST_CASE("协议层 - encode_speed(100) motor_id=1 → slot[0-1]=10000, 其余=0",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_speed(100.0f);

    REQUIRE(frame.id == 0x032u);
    REQUIRE(frame.len == 8u);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    REQUIRE(raw == 10000);  // 100 RPM × 100

    // slot 外字节必须全 0（不污染其他电机设定值）
    for (int i = 2; i < 8; ++i)
        REQUIRE(frame.data[i] == 0u);
}

TEST_CASE("协议层 - encode_speed(-50) → big-endian(-5000) 在 slot[0-1]", "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_speed(-50.0f);

    int16_t raw = be16s(frame.data[0], frame.data[1]);
    REQUIRE(raw == -5000);
}

TEST_CASE("协议层 - motor_id=3 的速度帧写在 slot[4-5]，slot[0-3]及[6-7]=0",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(3u);  // slot = (3-1)*2 = 4
    auto frame = c.encode_speed(80.0f);

    REQUIRE(frame.id == 0x032u);

    // slot[4-5] 有效
    int16_t raw = be16s(frame.data[4], frame.data[5]);
    REQUIRE(raw == 8000);  // 80 × 100

    // 其余字节必须为 0
    for (int i : {0, 1, 2, 3, 6, 7})
        REQUIRE(frame.data[i] == 0u);
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

TEST_CASE("协议层 - encode_set_mode(SPEED) → ID=0x105, DATA[motor_id-1]=0x02",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(2u);
    auto frame = c.encode_set_mode(protocol::WalkMotorMode::SPEED);

    REQUIRE(frame.id == 0x105u);
    REQUIRE(frame.len == 8u);
    REQUIRE(frame.data[1] == 0x02u);  // DATA[motor_id-1 = 1] = 速度环 0x02
    // 其他 slot 不应写入非零值
    REQUIRE(frame.data[0] == 0u);
    REQUIRE(frame.data[2] == 0u);
}

TEST_CASE("协议层 - encode_set_mode(ENABLE/DISABLE) → 0x0A/0x09 in DATA[motor_id-1]",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(3u);  // DATA[2]

    auto en = c.encode_set_mode(protocol::WalkMotorMode::ENABLE);
    REQUIRE(en.data[2] == 0x0Au);

    auto dis = c.encode_set_mode(protocol::WalkMotorMode::DISABLE);
    REQUIRE(dis.data[2] == 0x09u);
}

// ── 反馈方式帧 ──────────────────────────────────────────────────────────────

TEST_CASE("协议层 - encode_set_feedback(10) → 主动上报10ms, DATA[motor_id-1]=0x0A",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_set_feedback(10u);

    REQUIRE(frame.id == 0x106u);
    REQUIRE(frame.data[0] == 0x0Au);  // 10 & 0x7F = 10
}

TEST_CASE("协议层 - encode_set_feedback(0) → 查询方式, DATA[motor_id-1]=0x80",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = c.encode_set_feedback(0u);

    REQUIRE(frame.id == 0x106u);
    REQUIRE(frame.data[0] == 0x80u);  // 最高位=1 表示查询方式
}

// ── 查询帧 ──────────────────────────────────────────────────────────────────

TEST_CASE("协议层 - encode_query → ID=0x107, data[0]=motor_id, 目标内容正确",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(3u);
    auto frame = c.encode_query(protocol::WalkMotorQueryTarget::SPEED,
                                protocol::WalkMotorQueryTarget::TORQUE,
                                protocol::WalkMotorQueryTarget::FAULT);

    REQUIRE(frame.id == 0x107u);
    REQUIRE(frame.data[0] == 3u);     // motor_id
    REQUIRE(frame.data[1] == 0x01u);  // SPEED
    REQUIRE(frame.data[2] == 0x02u);  // TORQUE
    REQUIRE(frame.data[3] == 0x05u);  // FAULT
}

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

TEST_CASE("协议层 - decode_status 正向：速度/电流/位置/故障/模式全部正确",
          "[protocol][walk_motor]") {
    protocol::WalkMotorCanCodec c(1u);
    auto frame = make_status_frame(
        1u, 100.0f, 10.0f, 90.0f, protocol::WalkMotorFault::NONE, protocol::WalkMotorMode::SPEED);
    auto result = c.decode_status(frame);

    REQUIRE(result.has_value());
    REQUIRE(result->speed_rpm == Approx(100.0f).margin(kSpeedEps));
    REQUIRE(result->torque_a == Approx(10.0f).margin(kCurrentEps));
    REQUIRE(result->position_deg == Approx(90.0f).margin(kPosEps));
    REQUIRE(result->fault == protocol::WalkMotorFault::NONE);
    REQUIRE(result->mode == protocol::WalkMotorMode::SPEED);
}

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

TEST_CASE("协议层 - encode_group_speed id_base=1 → ID=0x32, 4个 slot 全部正确",
          "[protocol][walk_motor]") {
    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(1u, 10.0f, 20.0f, 30.0f, 40.0f);

    REQUIRE(frame.id == 0x032u);
    REQUIRE(frame.len == 8u);

    REQUIRE(be16s(frame.data[0], frame.data[1]) == 1000);  // 10 × 100
    REQUIRE(be16s(frame.data[2], frame.data[3]) == 2000);  // 20 × 100
    REQUIRE(be16s(frame.data[4], frame.data[5]) == 3000);  // 30 × 100
    REQUIRE(be16s(frame.data[6], frame.data[7]) == 4000);  // 40 × 100
}

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

TEST_CASE("设备层WalkMotor - open() 成功：CAN总线打开，设置过滤器", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);

    auto err = motor.open();

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(bus->opened);
    REQUIRE(bus->last_set_filters.size() == 1u);
    // 过滤器精确匹配 0x97（motor_id=1 的状态帧 ID）
    REQUIRE(bus->last_set_filters[0].id == 0x097u);
    REQUIRE(bus->last_set_filters[0].mask == 0x7FFu);
}

TEST_CASE("设备层WalkMotor - open() CAN 打开失败 → NOT_OPEN", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    bus->open_result = false;

    device::WalkMotor motor(bus, 1u);
    REQUIRE(motor.open() == device::DeviceError::NOT_OPEN);
    REQUIRE_FALSE(bus->opened);
}

TEST_CASE("设备层WalkMotor - close() 停止线程，关闭总线", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();

    motor.close();

    REQUIRE_FALSE(bus->opened);
    REQUIRE(bus->closed);
}

TEST_CASE("设备层WalkMotor - enable() 发送 SET_MODE(ENABLE) 帧", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    auto err = motor.enable();

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE_FALSE(bus->sent_frames.empty());
    // 使能帧：0x105，DATA[0]=0x0A（motor_id=1）
    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x105u);
    REQUIRE(f.data[0] == 0x0Au);

    motor.close();
}

TEST_CASE("设备层WalkMotor - disable() 发送 SET_MODE(DISABLE) 帧", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 2u);
    motor.open();
    bus->sent_frames.clear();

    motor.disable();

    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x105u);
    REQUIRE(f.data[1] == 0x09u);  // DATA[motor_id-1=1] = 0x09

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_speed() 发送正确控制帧", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    auto err = motor.set_speed(100.0f);

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE_FALSE(bus->sent_frames.empty());

    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x032u);
    int16_t raw = be16s(f.data[0], f.data[1]);
    REQUIRE(raw == 10000);  // 100 RPM × 100

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_speed() 超出 ±210 RPM 范围 → INVALID_PARAM",
          "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();

    REQUIRE(motor.set_speed(211.0f) == device::DeviceError::INVALID_PARAM);
    REQUIRE(motor.set_speed(-210.1f) == device::DeviceError::INVALID_PARAM);

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_speed() CAN 发送失败 → COMM_TIMEOUT + err_count++",
          "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    bus->send_result = false;
    device::WalkMotor motor(bus, 1u);
    motor.open();

    auto err = motor.set_speed(50.0f);

    REQUIRE(err == device::DeviceError::COMM_TIMEOUT);
    REQUIRE(motor.get_diagnostics().can_err_count == 1u);

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_current() 范围 ±33A 均可发送", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    REQUIRE(motor.set_current(16.5f) == device::DeviceError::OK);
    REQUIRE(motor.set_current(-33.0f) == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 2u);

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_position() 范围 0~360° 可发送", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    REQUIRE(motor.set_position(180.0f) == device::DeviceError::OK);
    REQUIRE_FALSE(bus->sent_frames.empty());

    motor.close();
}

TEST_CASE("设备层WalkMotor - set_open_loop() 直接发送 raw 值", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    REQUIRE(motor.set_open_loop(5000) == device::DeviceError::OK);

    const auto& f = bus->sent_frames.back();
    REQUIRE(be16s(f.data[0], f.data[1]) == 5000);

    motor.close();
}

TEST_CASE("设备层WalkMotor - update() 重发心跳帧（最后一次 set_speed 的帧）",
          "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    motor.set_speed(60.0f);
    bus->sent_frames.clear();

    motor.update();

    // update 应该重发最后一帧
    REQUIRE_FALSE(bus->sent_frames.empty());
    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x032u);
    REQUIRE(be16s(f.data[0], f.data[1]) == 6000);  // 60 × 100

    motor.close();
}

TEST_CASE("设备层WalkMotor - 未调用 set_speed 时 update() 不发送额外帧", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotor motor(bus, 1u);
    motor.open();
    bus->sent_frames.clear();

    motor.update();

    REQUIRE(bus->sent_frames.empty());

    motor.close();
}

TEST_CASE("设备层WalkMotor - recv_loop 解析状态帧并更新诊断数据", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    bus->recv_frame = make_status_frame(
        1u, 50.0f, 5.0f, 120.0f, protocol::WalkMotorFault::NONE, protocol::WalkMotorMode::SPEED);
    bus->recv_result = true;

    device::WalkMotor motor(bus, 1u);
    REQUIRE(motor.open() == device::DeviceError::OK);

    bool got_frame = wait_for([&] { return motor.get_diagnostics().feedback_frame_count > 0u; });

    bus->recv_result = false;
    motor.close();

    REQUIRE(got_frame);
    auto d = motor.get_diagnostics();
    REQUIRE(d.speed_rpm == Approx(50.0f).margin(kSpeedEps));
    REQUIRE(d.torque_a == Approx(5.0f).margin(kCurrentEps));
    REQUIRE(d.position_deg == Approx(120.0f).margin(kPosEps));
    REQUIRE(d.online);
    REQUIRE(d.mode == protocol::WalkMotorMode::SPEED);
    REQUIRE(d.fault == protocol::WalkMotorFault::NONE);
}

TEST_CASE("设备层WalkMotor - 收到错误 ID 的帧不更新状态", "[device][walk_motor]") {
    auto bus = std::make_shared<MockCanBus>();
    // motor_id=2 的状态帧（0x98），但设备实例是 motor_id=1（0x97）
    bus->recv_frame = make_status_frame(2u, 99.0f, 1.0f, 0.0f);
    bus->recv_result = true;

    device::WalkMotor motor(bus, 1u);
    motor.open();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bus->recv_result = false;
    motor.close();

    // 不应该更新（状态帧 ID 不匹配）
    REQUIRE(motor.get_diagnostics().feedback_frame_count == 0u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 6：设备层 - WalkMotorGroup
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("设备层WalkMotorGroup - open() 成功：设置4个精确过滤器", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);

    auto err = group.open();

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(bus->opened);
    // 4个过滤器：0x97, 0x98, 0x99, 0x9A
    REQUIRE(bus->last_set_filters.size() == 4u);
    REQUIRE(bus->last_set_filters[0].id == 0x097u);
    REQUIRE(bus->last_set_filters[1].id == 0x098u);
    REQUIRE(bus->last_set_filters[2].id == 0x099u);
    REQUIRE(bus->last_set_filters[3].id == 0x09Au);
    for (auto& f : bus->last_set_filters)
        REQUIRE(f.mask == 0x7FFu);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - open() id_base=5 过滤器 ID 为 0x9B~0x9E",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 5u);
    group.open();

    REQUIRE(bus->last_set_filters[0].id == 0x09Bu);
    REQUIRE(bus->last_set_filters[3].id == 0x09Eu);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - open() CAN 打开失败 → NOT_OPEN", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    bus->open_result = false;
    device::WalkMotorGroup group(bus, 1u);

    REQUIRE(group.open() == device::DeviceError::NOT_OPEN);
    REQUIRE_FALSE(bus->opened);
}

TEST_CASE("设备层WalkMotorGroup - set_mode_all() 发送4帧 SET_MODE", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    auto err = group.set_mode_all(protocol::WalkMotorMode::SPEED);

    REQUIRE(err == device::DeviceError::OK);
    // 模式帧每台独立，共4帧
    REQUIRE(bus->sent_frames.size() == 4u);
    for (const auto& f : bus->sent_frames)
        REQUIRE(f.id == 0x105u);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - enable_all() 等效于 set_mode_all(ENABLE)",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    group.enable_all();

    REQUIRE(bus->sent_frames.size() == 4u);
    REQUIRE(bus->sent_frames[0].data[0] == 0x0Au);  // motor_id=1, DATA[0]=ENABLE

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_speeds() 仅发送1帧，ID=0x32", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    auto err = group.set_speeds(10.0f, 20.0f, 30.0f, 40.0f);

    REQUIRE(err == device::DeviceError::OK);
    // 关键断言：4路速度仅发1帧
    REQUIRE(bus->sent_frames.size() == 1u);

    const auto& f = bus->sent_frames[0];
    REQUIRE(f.id == 0x032u);
    REQUIRE(be16s(f.data[0], f.data[1]) == 1000);  // LT: 10 RPM
    REQUIRE(be16s(f.data[2], f.data[3]) == 2000);  // RT: 20 RPM
    REQUIRE(be16s(f.data[4], f.data[5]) == 3000);  // LB: 30 RPM
    REQUIRE(be16s(f.data[6], f.data[7]) == 4000);  // RB: 40 RPM

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_speeds(SpeedCmd) 等效于4参数版本",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    device::WalkMotorGroup::SpeedCmd cmd;
    cmd.lt_rpm = 50.0f;
    cmd.rt_rpm = 50.0f;
    cmd.lb_rpm = -50.0f;
    cmd.rb_rpm = -50.0f;
    auto err = group.set_speeds(cmd);

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 1u);
    REQUIRE(be16s(bus->sent_frames[0].data[0], bus->sent_frames[0].data[1]) == 5000);
    REQUIRE(be16s(bus->sent_frames[0].data[4], bus->sent_frames[0].data[5]) == -5000);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_speed_uniform() 4路速度相同", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    group.set_speed_uniform(100.0f);

    REQUIRE(bus->sent_frames.size() == 1u);
    const auto& f = bus->sent_frames[0];
    for (int slot = 0; slot < 4; ++slot)
        REQUIRE(be16s(f.data[slot * 2], f.data[slot * 2 + 1]) == 10000);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_speeds() 超过 ±210 RPM 被钳位",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    group.set_speeds(300.0f, -300.0f, 210.0f, -210.0f);

    const auto& f = bus->sent_frames[0];
    REQUIRE(be16s(f.data[0], f.data[1]) == 21000);   // 300 → 钳位 210
    REQUIRE(be16s(f.data[2], f.data[3]) == -21000);  // -300 → 钳位 -210

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_currents() 发送1帧、ID=0x32", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    auto err = group.set_currents(5.0f, -5.0f, 10.0f, -10.0f);

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 1u);
    REQUIRE(bus->sent_frames[0].id == 0x032u);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - set_open_loops() 发送1帧", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->sent_frames.clear();

    auto err = group.set_open_loops(1000, -1000, 2000, -2000);

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(bus->sent_frames.size() == 1u);
    const auto& f = bus->sent_frames[0];
    REQUIRE(be16s(f.data[0], f.data[1]) == 1000);
    REQUIRE(be16s(f.data[2], f.data[3]) == -1000);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - get_group_diagnostics() 统计 ctrl_frame_count",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();

    group.set_speeds(10.0f, 10.0f, 10.0f, 10.0f);
    group.set_speeds(20.0f, 20.0f, 20.0f, 20.0f);
    group.set_speed_uniform(30.0f);

    auto gd = group.get_group_diagnostics();
    REQUIRE(gd.ctrl_frame_count == 3u);
    REQUIRE(gd.ctrl_err_count == 0u);

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - CAN发送失败时 ctrl_err_count 递增",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    bus->send_result = false;

    group.set_speed_uniform(50.0f);

    auto gd = group.get_group_diagnostics();
    REQUIRE(gd.ctrl_err_count == 1u);
    REQUIRE(gd.ctrl_frame_count == 0u);  // 发送失败不计入成功帧

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - update() 重发上次控制帧", "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();
    group.set_speed_uniform(80.0f);
    bus->sent_frames.clear();

    group.update();

    // update 应重发心跳帧（内容与上次 set_speed_uniform 相同）
    REQUIRE_FALSE(bus->sent_frames.empty());
    const auto& f = bus->sent_frames.back();
    REQUIRE(f.id == 0x032u);
    REQUIRE(be16s(f.data[0], f.data[1]) == 8000);  // 80 × 100

    group.close();
}

TEST_CASE("设备层WalkMotorGroup - recv_loop: motor_id=2 状态帧只更新 Wheel::RT",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    // 注入 motor_id=2（id_base=1 时 Wheel::RT = codec[1]）的状态帧
    bus->recv_frame = make_status_frame(
        2u, 75.0f, 4.0f, 45.0f, protocol::WalkMotorFault::NONE, protocol::WalkMotorMode::SPEED);
    bus->recv_result = true;

    device::WalkMotorGroup group(bus, 1u);
    REQUIRE(group.open() == device::DeviceError::OK);

    bool received = wait_for([&] {
        return group.get_wheel_diagnostics(device::WalkMotorGroup::Wheel::RT).feedback_frame_count >
               0u;
    });

    bus->recv_result = false;
    group.close();

    REQUIRE(received);

    // Wheel::RT 应有正确数据
    auto rt = group.get_wheel_status(device::WalkMotorGroup::Wheel::RT);
    REQUIRE(rt.speed_rpm == Approx(75.0f).margin(kSpeedEps));
    REQUIRE(rt.torque_a == Approx(4.0f).margin(kCurrentEps));
    REQUIRE(rt.position_deg == Approx(45.0f).margin(kPosEps));
    REQUIRE(rt.online);

    // Wheel::LT（motor_id=1，0x97）未收到帧，不应更新
    auto lt = group.get_wheel_status(device::WalkMotorGroup::Wheel::LT);
    REQUIRE(lt.speed_rpm == Approx(0.0f).margin(kSpeedEps));
    REQUIRE_FALSE(lt.online);
}

TEST_CASE("设备层WalkMotorGroup - recv_loop: 4个电机各自独立更新", "[device][walk_motor_group]") {
    // 每次测试只注入一台电机帧，验证其余轮初始值不变
    for (int w = 0; w < 4; ++w) {
        auto bus = std::make_shared<MockCanBus>();
        uint8_t mid = static_cast<uint8_t>(1u + w);  // motor_id 1~4
        bus->recv_frame = make_status_frame(mid,
                                            static_cast<float>(w * 10 + 10),  // 10,20,30,40 RPM
                                            0.0f,
                                            0.0f);
        bus->recv_result = true;

        device::WalkMotorGroup group(bus, 1u);
        group.open();

        auto wheel = static_cast<device::WalkMotorGroup::Wheel>(w);
        bool received =
            wait_for([&] { return group.get_wheel_diagnostics(wheel).feedback_frame_count > 0u; });

        bus->recv_result = false;
        group.close();

        REQUIRE(received);
        auto st = group.get_wheel_status(wheel);
        REQUIRE(st.speed_rpm == Approx(static_cast<float>(w * 10 + 10)).margin(kSpeedEps));
    }
}

TEST_CASE("设备层WalkMotorGroup - get_group_status() 返回4轮初始状态",
          "[device][walk_motor_group]") {
    auto bus = std::make_shared<MockCanBus>();
    device::WalkMotorGroup group(bus, 1u);
    group.open();

    auto gs = group.get_group_status();
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        REQUIRE(gs.wheel[i].speed_rpm == Approx(0.0f).margin(0.001f));
        REQUIRE_FALSE(gs.wheel[i].online);
    }

    group.close();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 7：硬件集成测试（仅在目标机上运行，需要 CAN0 在线且电机 ID=1 上电）
//
//  运行前准备（目标机）：
//    ip link set can0 up type can bitrate 500000
//  运行：
//    ./unit_tests "[integration][walk_motor]"
//  在 CI / 无硬件环境中跳过：
//    ./unit_tests "~[integration]"
// ═══════════════════════════════════════════════════════════════════════════

static constexpr char kCanIface[] = "can0";
static constexpr uint8_t kMotorId = 1u;

// ── 集成测试 1：CAN 总线打开与电机在线检测 ──────────────────────────────────
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
TEST_CASE("integration - WalkMotor query mode, 1000ms timeout, keepalive by polling",
          "[integration][walk_motor]") {
    using namespace std::chrono_literals;

    const char* iface_env = std::getenv("TEST_CAN_IFACE");
    std::string iface = iface_env ? iface_env : "can0";

    auto bus = std::make_shared<robot::driver::LinuxCanSocket>(iface);
    device::WalkMotor motor(bus, 1u);

    REQUIRE(bus->open() == true);
    REQUIRE(motor.open() == device::DeviceError::OK);

    // 1) 设置为查询模式，通信超时1000ms
    REQUIRE(motor.set_feedback_mode(0u) == device::DeviceError::OK);
    std::this_thread::sleep_for(150ms);
    REQUIRE(motor.set_comm_timeout(1000u) == device::DeviceError::OK);
    std::this_thread::sleep_for(150ms);
    // 2) 切换速度环，设定目标速度
    REQUIRE(motor.set_mode(protocol::WalkMotorMode::SPEED) == device::DeviceError::OK);
    std::this_thread::sleep_for(150ms);
    REQUIRE(motor.set_speed(50.0f) == device::DeviceError::OK);
    std::this_thread::sleep_for(150ms);
    // 3) 每500ms调用 update() 并主动查询状态，持续3秒
    bool always_online = true;
    for (int i = 0; i < 6; ++i) {
        motor.update();
        std::this_thread::sleep_for(150ms);  // 先等一会，避免刚切换模式时误判
        // 主动查询一次（模拟主机问答）
        REQUIRE(motor.read_comm_timeout() == device::DeviceError::OK);
        std::this_thread::sleep_for(450ms);
        always_online = always_online && motor.get_diagnostics().online;
    }

    REQUIRE(always_online);

    motor.close();
}