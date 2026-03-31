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
