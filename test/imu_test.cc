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

TEST_CASE("设备层 - open/close 基本生命周期", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    device::ImuDevice imu(serial);
    REQUIRE_FALSE(serial->opened);

    REQUIRE(imu.open());
    REQUIRE(serial->opened);

    imu.close();
    REQUIRE_FALSE(serial->opened);
}

TEST_CASE("设备层 - open() 串口打开失败时返回 false", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();
    serial->open_result = false;

    device::ImuDevice imu(serial);
    REQUIRE_FALSE(imu.open());
}

TEST_CASE("设备层 - 初始状态 get_latest().valid 为 false", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());
    // 尚未接收任何帧，数据应无效
    auto data = imu.get_latest();
    REQUIRE_FALSE(data.valid);
    imu.close();
}

TEST_CASE("设备层 - 读取线程解析欧拉角帧并更新 get_latest()", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    // Roll=90° → raw=16384；Pitch=-45° → raw=-8192；Yaw=0
    auto frame = make_wit_frame(0x53, 16384, -8192, 0, 0);
    serial->rx_data = frame_to_vec(frame);

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    // 等待读取线程处理完一帧（最多 200ms）
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu.get_latest().valid)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto data = imu.get_latest();
    imu.close();

    REQUIRE(data.valid);
    REQUIRE(data.roll_deg == Approx(90.0f).margin(kAngleEps));
    REQUIRE(data.pitch_deg == Approx(-45.0f).margin(kAngleEps));
    REQUIRE(data.yaw_deg == Approx(0.0f).margin(kAngleEps));
}

TEST_CASE("设备层 - 读取线程解析加速度帧后 accel 字段有效", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    // Ax=4.9 m/s² ← raw=1024（1024/32768*16*9.8=4.9）
    auto frame = make_wit_frame(0x51, 1024, 0, 0, 0);
    serial->rx_data = frame_to_vec(frame);

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu.get_latest().valid)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto data = imu.get_latest();
    imu.close();

    REQUIRE(data.valid);
    REQUIRE(data.accel[0] == Approx(1024 * kAccelScale).epsilon(kAccelEps));
    REQUIRE(data.accel[1] == Approx(0.0f).margin(kAccelEps));
    REQUIRE(data.accel[2] == Approx(0.0f).margin(kAccelEps));
}

TEST_CASE("设备层 - 连续多帧：后帧覆盖欧拉角字段", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    // 先推一帧 Roll=90°，再推一帧 Roll=-45°
    auto frame1 = make_wit_frame(0x53, 16384, 0, 0, 0);
    auto frame2 = make_wit_frame(0x53, -8192, 0, 0, 0);
    auto& rx = serial->rx_data;
    rx.insert(rx.end(), frame1.begin(), frame1.end());
    rx.insert(rx.end(), frame2.begin(), frame2.end());

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    // 等待两帧都被处理（frame_count >= 2）
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu.get_diagnostics().frame_count >= 2)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto data = imu.get_latest();
    imu.close();

    REQUIRE(data.valid);
    // 最后一帧 Roll=-45°
    REQUIRE(data.roll_deg == Approx(-45.0f).margin(kAngleEps));
}

TEST_CASE("设备层 - set_output_rate 发送解锁 + RRATE 命令", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    // set_output_rate(50) → rate_code=0x08 (50Hz，RRATE_50HZ)
    // 预期：先发 unlock(5字节)，再发 RRATE 命令(5字节)，共 10 字节
    auto err = imu.set_output_rate(50);

    imu.close();

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(serial->tx_captured.size() == 10u);

    // 验证解锁序列 FF AA 69 88 B5
    REQUIRE(serial->tx_captured[0] == 0xFF);
    REQUIRE(serial->tx_captured[1] == 0xAA);
    REQUIRE(serial->tx_captured[2] == 0x69);
    REQUIRE(serial->tx_captured[3] == 0x88);
    REQUIRE(serial->tx_captured[4] == 0xB5);

    // 验证 RRATE 命令 FF AA 03 08 00（0x08 = 50Hz）
    REQUIRE(serial->tx_captured[5] == 0xFF);
    REQUIRE(serial->tx_captured[6] == 0xAA);
    REQUIRE(serial->tx_captured[7] == 0x03);
    REQUIRE(serial->tx_captured[8] == 0x08);
    REQUIRE(serial->tx_captured[9] == 0x00);
}

TEST_CASE("设备层 - save_config 发送解锁 + SAVE 命令", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    auto err = imu.save_config();

    imu.close();

    REQUIRE(err == device::DeviceError::OK);
    REQUIRE(serial->tx_captured.size() == 10u);

    // unlock
    REQUIRE(serial->tx_captured[0] == 0xFF);
    REQUIRE(serial->tx_captured[2] == 0x69);

    // SAVE 命令 FF AA 00 00 00
    REQUIRE(serial->tx_captured[5] == 0xFF);
    REQUIRE(serial->tx_captured[7] == 0x00);  // SAVE 寄存器
    REQUIRE(serial->tx_captured[8] == 0x00);
    REQUIRE(serial->tx_captured[9] == 0x00);
}

TEST_CASE("设备层 - 串口写失败时 send_command 返回 COMM_TIMEOUT", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();
    serial->write_return = 0;  // write 返回 0 字节（模拟写失败）

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    // set_output_rate 内部发送的 unlock 帧写失败，应返回错误
    auto err = imu.set_output_rate(50);
    imu.close();

    REQUIRE(err == device::DeviceError::COMM_TIMEOUT);
}

TEST_CASE("设备层 - get_diagnostics frame_count 累计正确", "[device][imu]") {
    auto serial = std::make_shared<MockSerialPort>();

    // 注入 3 帧（2×欧拉角 + 1×加速度）
    auto euler = make_wit_frame(0x53, 1000, 0, 0, 0);
    auto accel = make_wit_frame(0x51, 2000, 0, 0, 0);
    auto& rx = serial->rx_data;
    rx.insert(rx.end(), euler.begin(), euler.end());
    rx.insert(rx.end(), euler.begin(), euler.end());
    rx.insert(rx.end(), accel.begin(), accel.end());

    device::ImuDevice imu(serial);
    REQUIRE(imu.open());

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu.get_diagnostics().frame_count >= 3)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    imu.close();

    REQUIRE(imu.get_diagnostics().frame_count >= 3u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 4：硬件集成测试（需在目标机上接 IMU，/dev/ttyS1，9600 bps）
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("集成测试 - /dev/ttyS1 读取真实 IMU 欧拉角", "[integration][imu]") {
    using namespace robot;

    hal::UartConfig cfg;
    cfg.baudrate = 9600;

    auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS1", cfg);
    device::ImuDevice imu(serial);

    REQUIRE(imu.open());
    INFO("已打开 /dev/ttyS1，等待 IMU 数据...");

    // 等待最多 2 秒收到有效帧
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        if (imu.get_latest().valid)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto data = imu.get_latest();
    imu.close();

    REQUIRE(data.valid);
    // 欧拉角范围检查：Roll/Pitch ∈ [-90, 90]，Yaw ∈ [-180, 180]
    REQUIRE(data.roll_deg >= -90.0f);
    REQUIRE(data.roll_deg <= 90.0f);
    REQUIRE(data.pitch_deg >= -90.0f);
    REQUIRE(data.pitch_deg <= 90.0f);
    REQUIRE(data.yaw_deg >= -180.0f);
    REQUIRE(data.yaw_deg <= 180.0f);

    auto diag = imu.get_diagnostics();
    INFO("frame_count=" << diag.frame_count << " frame_rate=" << diag.frame_rate_hz);
    REQUIRE(diag.frame_count >= 1u);
}

TEST_CASE("集成测试 - /dev/ttyS1 连续读取 60 秒并打印数据", "[integration][imu][long]") {
    using namespace std::chrono_literals;

    hal::UartConfig cfg;
    cfg.baudrate = 9600;

    auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS1", cfg);
    device::ImuDevice imu(serial);

    REQUIRE(imu.open());
    spdlog::info("[IMU 连续读取] 已打开 /dev/ttyS1，开始 60 秒数据采集...");
    spdlog::info(
        "[IMU 连续读取] 格式: Roll(°)  Pitch(°)  Yaw(°) | "
        "Ax  Ay  Az(m/s²) | Wx  Wy  Wz(rad/s) | frames err");

    // 等待首帧（最多 3 秒）
    auto first_deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < first_deadline) {
        if (imu.get_latest().valid)
            break;
        std::this_thread::sleep_for(50ms);
    }
    REQUIRE(imu.get_latest().valid);

    auto start = std::chrono::steady_clock::now();
    auto next_print = start;
    constexpr auto kPrintInterval = 200ms;  // 每 200ms 打印一次
    constexpr auto kDuration = 60s;

    while (std::chrono::steady_clock::now() - start < kDuration) {
        auto now = std::chrono::steady_clock::now();
        if (now >= next_print) {
            auto data = imu.get_latest();
            auto diag = imu.get_diagnostics();
            auto elapsed_s =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() /
                1000.0f;
            spdlog::info(
                "[IMU {:5.1f}s] Roll={:7.2f}°  Pitch={:7.2f}°  Yaw={:8.2f}° | "
                "Ax={:6.2f}  Ay={:6.2f}  Az={:6.2f} m/s² | "
                "Wx={:6.3f}  Wy={:6.3f}  Wz={:6.3f} rad/s | "
                "frames={} rate={:.1f}Hz err={}",
                elapsed_s,
                data.roll_deg,
                data.pitch_deg,
                data.yaw_deg,
                data.accel[0],
                data.accel[1],
                data.accel[2],
                data.gyro[0],
                data.gyro[1],
                data.gyro[2],
                diag.frame_count,
                diag.frame_rate_hz,
                diag.parse_error_count);
            next_print += kPrintInterval;
        }
        std::this_thread::sleep_for(10ms);
    }

    auto diag = imu.get_diagnostics();
    spdlog::info("[IMU 连续读取] 完成。总帧数={} 平均帧率={:.1f}Hz 错误帧={}",
                 diag.frame_count,
                 diag.frame_rate_hz,
                 diag.parse_error_count);
    imu.close();

    CHECK(diag.frame_count > 0u);
    CHECK(diag.frame_rate_hz > 1.0f);
}
