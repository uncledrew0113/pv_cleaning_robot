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

