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

