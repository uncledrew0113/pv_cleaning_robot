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

