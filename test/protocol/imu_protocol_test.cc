/*
 * IMU 协议层与设备层单元测试（WIT Motion UART 协议）
 *
 * 测试分组：
 *   [protocol][imu]    - ImuProtocol 帧解析 / 命令编码（纯内存，无 I/O）
 *   [device][imu]      - ImuDevice 设备层（基于 MockSerialPort，不依赖真实硬件）
 *   [integration][imu] - ImuDevice 与真实 /dev/ttyS1 的硬件集成测试（须在目标机上运行）
 *
 * 运行方法（交叉编译后在目标机上）：
 *   ./unit_tests "[protocol][imu]"    # 只跑协议层单元测试
 *   ./unit_tests "[device][imu]"      # 只跑设备层 mock 测试
 *   ./unit_tests "[integration][imu]" # 只跑硬件集成测试（需接 IMU 设备）
 *
 * 帧格式：
 *   [0x55][type][d0L][d0H][d1L][d1H][d2L][d2H][d3L][d3H][SUM]  共 11 字节
 *   SUM = (0x55 + type + d0L + d0H + ... + d3L + d3H) & 0xFF
 *
 * 物理换算（来自 imu_protocol.cc 中的 scale 常量）：
 *   加速度: raw / 32768 * 16 * 9.8  (m/s²)
 *   角速度: raw / 32768 * 2000 * π/180  (rad/s)
 *   欧拉角: raw / 32768 * 180  (°)
 *   四元数: raw / 32768
 */
#include <array>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

#include "mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/protocol/imu_protocol.h"

using namespace robot;

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具
// ═══════════════════════════════════════════════════════════════════════════

/// 物理换算常量（与 imu_protocol.cc 保持一致）
static constexpr float kAccelScale = 16.0f * 9.8f / 32768.0f;
static constexpr float kGyroScale = 2000.0f / 32768.0f * (static_cast<float>(M_PI) / 180.0f);
static constexpr float kAngleScale = 180.0f / 32768.0f;
static constexpr float kQuatScale = 1.0f / 32768.0f;

/// 允许的浮点比较误差（量化误差 ≈ 1 LSB 对应的物理量）
static constexpr float kAccelEps = kAccelScale * 1.5f;
static constexpr float kGyroEps = kGyroScale * 1.5f;
static constexpr float kAngleEps = kAngleScale * 1.5f;
static constexpr float kQuatEps = kQuatScale * 1.5f;

/**
 * @brief 构造一帧 WIT Motion 11 字节标准帧
 *
 * @param type  帧类型（0x51/0x52/0x53/0x54/0x59 等）
 * @param d0~d3 四组 int16（小端序存入 bytes 2~9）
 * @return 11 字节帧（含正确校验和）
 */
static std::array<uint8_t, 11> make_wit_frame(uint8_t type,
                                              int16_t d0,
                                              int16_t d1,
                                              int16_t d2,
                                              int16_t d3) {
    std::array<uint8_t, 11> f{};
    f[0] = 0x55;
    f[1] = type;
    f[2] = static_cast<uint8_t>(static_cast<uint16_t>(d0) & 0xFF);
    f[3] = static_cast<uint8_t>(static_cast<uint16_t>(d0) >> 8);
    f[4] = static_cast<uint8_t>(static_cast<uint16_t>(d1) & 0xFF);
    f[5] = static_cast<uint8_t>(static_cast<uint16_t>(d1) >> 8);
    f[6] = static_cast<uint8_t>(static_cast<uint16_t>(d2) & 0xFF);
    f[7] = static_cast<uint8_t>(static_cast<uint16_t>(d2) >> 8);
    f[8] = static_cast<uint8_t>(static_cast<uint16_t>(d3) & 0xFF);
    f[9] = static_cast<uint8_t>(static_cast<uint16_t>(d3) >> 8);
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i)
        sum = static_cast<uint8_t>(sum + f[i]);
    f[10] = sum;
    return f;
}

/// 将帧拷贝到 vector（便于追加到 rx_data）
static std::vector<uint8_t> frame_to_vec(const std::array<uint8_t, 11>& f) {
    return {f.begin(), f.end()};
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 1：协议层 - 帧解析
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - 加速度帧(0x51)正确解析", "[protocol][imu]") {
    // d0=16384 → Ax = 16384/32768 * 16 * 9.8 = 78.4 m/s²
    // d1=8192   → Ay = 8192/32768 * 16 * 9.8 = 39.2 m/s²
    // d2=-8192  → Az = -8192/32768 * 16 * 9.8 = -39.2 m/s²
    // d3=2500   → Temp（不测）
    auto frame = make_wit_frame(0x51, 16384, 8192, -8192, 2500);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.valid);
    REQUIRE(data.accel[0] == Approx(16384 * kAccelScale).epsilon(kAccelEps));
    REQUIRE(data.accel[1] == Approx(8192 * kAccelScale).epsilon(kAccelEps));
    REQUIRE(data.accel[2] == Approx(-8192 * kAccelScale).epsilon(kAccelEps));
    // take_frame 后标志清除
    REQUIRE_FALSE(parser.frame_complete());
}

TEST_CASE("协议层 - 角速度帧(0x52)正确解析并转换为rad/s", "[protocol][imu]") {
    // d0=16384 → Wx = 16384/32768 * 2000 * π/180 = 1000 * π/180 ≈ 17.453 rad/s
    // d1=0     → Wy = 0
    // d2=-16384 → Wz = -1000 * π/180
    auto frame = make_wit_frame(0x52, 16384, 0, -16384, 0);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.valid);
    const float expected_rad = 1000.0f * static_cast<float>(M_PI) / 180.0f;
    REQUIRE(data.gyro[0] == Approx(expected_rad).epsilon(kGyroEps));
    REQUIRE(data.gyro[1] == Approx(0.0f).margin(kGyroEps));
    REQUIRE(data.gyro[2] == Approx(-expected_rad).epsilon(kGyroEps));
}

TEST_CASE("协议层 - 欧拉角帧(0x53)正确解析", "[protocol][imu]") {
    // d0=16384 → Roll  = 16384/32768 * 180 = 90.0°
    // d1=-8192 → Pitch = -8192/32768 * 180 = -45.0°
    // d2=32767 → Yaw   ≈ 179.995°
    auto frame = make_wit_frame(0x53, 16384, -8192, 32767, 0);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.valid);
    REQUIRE(data.roll_deg == Approx(90.0f).margin(kAngleEps));
    REQUIRE(data.pitch_deg == Approx(-45.0f).margin(kAngleEps));
    REQUIRE(data.yaw_deg == Approx(32767 * kAngleScale).epsilon(kAngleEps));
}

TEST_CASE("协议层 - 磁场帧(0x54)正确解析", "[protocol][imu]") {
    // 磁场 raw 即为毫高斯，MAG_SCALE=1.0
    auto frame = make_wit_frame(0x54, 100, -200, 300, 0);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.mag[0] == Approx(100.0f).margin(0.01f));
    REQUIRE(data.mag[1] == Approx(-200.0f).margin(0.01f));
    REQUIRE(data.mag[2] == Approx(300.0f).margin(0.01f));
    // 磁场帧不单独设置 valid（需要欧拉角/加速度帧配合），此处只验证值
}

TEST_CASE("协议层 - 四元数帧(0x59)正确解析", "[protocol][imu]") {
    // d0=32767 → q0 ≈ 1.0，d1=0，d2=0，d3=0
    auto frame = make_wit_frame(0x59, 32767, 0, 0, 0);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.quat[0] == Approx(32767 * kQuatScale).epsilon(kQuatEps));
    REQUIRE(data.quat[1] == Approx(0.0f).margin(kQuatEps));
    REQUIRE(data.quat[2] == Approx(0.0f).margin(kQuatEps));
    REQUIRE(data.quat[3] == Approx(0.0f).margin(kQuatEps));
}

TEST_CASE("协议层 - 校验和错误时丢弃帧", "[protocol][imu]") {
    auto frame = make_wit_frame(0x53, 16384, 0, 0, 0);
    frame[10] = static_cast<uint8_t>(frame[10] + 1);  // 故意破坏校验和

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE_FALSE(parser.frame_complete());
}

TEST_CASE("协议层 - 帧头同步：乱码字节被跳过", "[protocol][imu]") {
    auto frame = make_wit_frame(0x53, 1000, 2000, 3000, 0);

    // 在帧前插入 5 个乱码字节（非 0x55）
    std::vector<uint8_t> stream = {0x00, 0x11, 0xDD, 0xAA, 0x01};
    stream.insert(stream.end(), frame.begin(), frame.end());

    protocol::ImuProtocol parser;
    for (uint8_t b : stream)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.roll_deg == Approx(1000 * kAngleScale).epsilon(kAngleEps));
}

TEST_CASE("协议层 - 连续多帧顺序解析", "[protocol][imu]") {
    auto accel_frame = make_wit_frame(0x51, 1000, 2000, 3000, 0);
    auto euler_frame = make_wit_frame(0x53, 4000, -2000, 8000, 0);

    protocol::ImuProtocol parser;

    // 推入加速度帧
    for (uint8_t b : accel_frame)
        parser.push_byte(b);
    REQUIRE(parser.frame_complete());
    const auto& d1 = parser.take_frame();
    REQUIRE(d1.accel[0] == Approx(1000 * kAccelScale).epsilon(kAccelEps));

    // 紧接着推入欧拉角帧
    for (uint8_t b : euler_frame)
        parser.push_byte(b);
    REQUIRE(parser.frame_complete());
    const auto& d2 = parser.take_frame();
    REQUIRE(d2.roll_deg == Approx(4000 * kAngleScale).epsilon(kAngleEps));
}

TEST_CASE("协议层 - reset() 清除解析状态", "[protocol][imu]") {
    auto frame = make_wit_frame(0x53, 5000, 0, 0, 0);

    protocol::ImuProtocol parser;

    // 推入一帧使其 valid
    for (uint8_t b : frame)
        parser.push_byte(b);
    REQUIRE(parser.frame_complete());

    // reset 后状态应清除
    parser.reset();
    REQUIRE_FALSE(parser.frame_complete());
    // take_frame 返回初始化的空数据
    const auto& data = parser.take_frame();
    REQUIRE_FALSE(data.valid);
}

TEST_CASE("协议层 - 未知帧类型不触发 frame_complete", "[protocol][imu]") {
    auto frame = make_wit_frame(0x5E, 0, 0, 0, 0);  // 0x5E 不在已知类型中

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE_FALSE(parser.frame_complete());
}

TEST_CASE("协议层 - 帧中负数原始值正确翻转符号", "[protocol][imu]") {
    // 欧拉角 Roll=-90° → raw = -16384
    auto frame = make_wit_frame(0x53, -16384, 0, 0, 0);

    protocol::ImuProtocol parser;
    for (uint8_t b : frame)
        parser.push_byte(b);

    REQUIRE(parser.frame_complete());
    const auto& data = parser.take_frame();
    REQUIRE(data.roll_deg == Approx(-90.0f).margin(kAngleEps));
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 2：协议层 - 命令编码
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("协议层 - encode_unlock 生成正确解锁序列", "[protocol][imu]") {
    auto cmd = protocol::ImuProtocol::encode_unlock();
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x69);
    REQUIRE(cmd[3] == 0x88);
    REQUIRE(cmd[4] == 0xB5);
}

TEST_CASE("协议层 - encode_save_config 生成正确保存命令", "[protocol][imu]") {
    // 保存：FF AA 00 00 00（SAVE 寄存器=0x00，写入 0）
    auto cmd = protocol::ImuProtocol::encode_save_config();
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x00);
    REQUIRE(cmd[3] == 0x00);
    REQUIRE(cmd[4] == 0x00);
}

TEST_CASE("协议层 - encode_reset 生成正确软复位命令", "[protocol][imu]") {
    // 重启：FF AA 00 FF 00（写 SAVE 寄存器 = 0x00FF）
    auto cmd = protocol::ImuProtocol::encode_reset();
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x00);
    REQUIRE(cmd[3] == 0xFF);
    REQUIRE(cmd[4] == 0x00);
}

TEST_CASE("协议层 - encode_calibrate_gyro 生成正确校准命令", "[protocol][imu]") {
    // CALSW(0x01) = CALGYROACC(0x01)：FF AA 01 01 00
    auto cmd = protocol::ImuProtocol::encode_calibrate_gyro();
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x01);
    REQUIRE(cmd[3] == 0x01);
    REQUIRE(cmd[4] == 0x00);
}

TEST_CASE("协议层 - encode_set_rate 编码输出频率寄存器 RRATE(0x03)", "[protocol][imu]") {
    // 50 Hz → rate_code=0x06
    auto cmd = protocol::ImuProtocol::encode_set_rate(0x06);
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x03);  // RRATE 寄存器地址
    REQUIRE(cmd[3] == 0x06);  // data_lo
    REQUIRE(cmd[4] == 0x00);  // data_hi
}

TEST_CASE("协议层 - encode_set_baudrate 编码波特率寄存器 BAUD(0x04)", "[protocol][imu]") {
    // 921600 → baud_code=0x07
    auto cmd = protocol::ImuProtocol::encode_set_baudrate(0x07);
    REQUIRE(cmd[0] == 0xFF);
    REQUIRE(cmd[1] == 0xAA);
    REQUIRE(cmd[2] == 0x04);  // BAUD 寄存器地址
    REQUIRE(cmd[3] == 0x07);  // data_lo
    REQUIRE(cmd[4] == 0x00);  // data_hi
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 3：设备层 - ImuDevice（MockSerialPort，不依赖真实硬件）
// ═══════════════════════════════════════════════════════════════════════════

