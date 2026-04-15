/**
 * 距离传感器硬件集成测试
 *
 * 测试分组：[hw_dist]
 *
 * 测试前提：
 *   - 汇控电子 HK-xAI 模块已接 /dev/ttyS4（RS485），9600-8-N-1
 *   - 站号（拨码）= 1，数据解析方式 = 可变小数点（出厂默认）
 *   - 传感器处于上电并处于正常工作状态
 *
 * 硬件接线常量请修改 hw_config.h 中的 kDistSensor* 系列常量。
 *
 * 运行方法（目标机）：
 *   ./hw_tests "[hw_dist][read_channels]"   # 5次轮询验证通信
 *   ./hw_tests "[hw_dist][sustained_read]"  # 10秒持续采集（波形稳定性）
 */
#include <catch2/catch.hpp>

#include "hw_config.h"
#include "pv_cleaning_robot/device/distance_sensor.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/protocol/distance_sensor_protocol.h"

#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

using namespace robot;
using namespace std::chrono_literals;

// ── 辅助：创建 Modbus 主站 ─────────────────────────────────────────────────
static std::shared_ptr<driver::LibModbusMaster> make_dist_modbus()
{
    hal::ModbusConfig mbus_cfg;
    mbus_cfg.baudrate  = hw::kDistSensorBaud;
    mbus_cfg.parity    = 'N';   // 8-N-1（协议默认）
    mbus_cfg.data_bits = 8;
    mbus_cfg.stop_bits = 1;
    return std::make_shared<driver::LibModbusMaster>(hw::kDistSensorPort, mbus_cfg);
}

// ──────────────────────────────────────────────────────────────────────────
// Test 1：5次轮询（验证通信正常 + 数据解码）
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("距离传感器：5次轮询验证通信与数据解码", "[hw_dist][read_channels]")
{
    auto modbus = make_dist_modbus();
    REQUIRE(modbus->open());

    device::DistanceSensorConfig cfg;
    cfg.slave_id      = hw::kDistSensorSlaveId;
    cfg.channel_count = hw::kDistSensorChannelCount;
    cfg.decimal_mode  = protocol::DistDecimalMode::VARIABLE;  // 出厂默认
    cfg.analog_type   = protocol::DistAnalogType::VOLTAGE;

    device::DistanceSensor sensor(modbus, cfg);

    spdlog::info("[hw_dist][read_channels] slave={} port={} ch_count={}",
                 cfg.slave_id, hw::kDistSensorPort,
                 static_cast<int>(cfg.channel_count));

    int valid_readings = 0;
    for (int poll = 0; poll < 5; ++poll) {
        sensor.update();
        auto data = sensor.get_data();

        for (uint8_t ch = 0; ch < data.channel_count; ++ch) {
            spdlog::info("[hw_dist] poll={} ch[{}]={:.4f}V valid={}",
                         poll, ch, data.channels[ch].value_v,
                         data.channels[ch].valid ? 1 : 0);
            if (data.channels[ch].valid) ++valid_readings;
        }
        std::this_thread::sleep_for(100ms);
    }

    auto diag = sensor.get_diagnostics();
    spdlog::info("[hw_dist] summary: total_updates={} comm_errors={}",
                 diag.total_update_count, diag.comm_error_count);

    CHECK(diag.comm_error_count == 0);
    CHECK(valid_readings > 0);
}

// ──────────────────────────────────────────────────────────────────────────
// Test 2：持续采集10秒（波形稳定性验证）
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("距离传感器：持续采集10秒（波形稳定性验证）", "[hw_dist][sustained_read]")
{
    auto modbus = make_dist_modbus();
    REQUIRE(modbus->open());

    device::DistanceSensorConfig cfg;
    cfg.slave_id      = hw::kDistSensorSlaveId;
    cfg.channel_count = hw::kDistSensorChannelCount;
    cfg.decimal_mode  = protocol::DistDecimalMode::VARIABLE;
    cfg.analog_type   = protocol::DistAnalogType::VOLTAGE;

    device::DistanceSensor sensor(modbus, cfg);

    spdlog::info("[hw_dist][sustained_read] 开始10秒持续采集（200ms/次）…");

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline) {
        sensor.update();
        auto data = sensor.get_data();
        for (uint8_t ch = 0; ch < data.channel_count; ++ch) {
            spdlog::info("[hw_dist][sustained] ch[{}]={:.4f}V",
                         ch, data.channels[ch].value_v);
        }
        std::this_thread::sleep_for(200ms);
    }

    auto diag = sensor.get_diagnostics();
    spdlog::info("[hw_dist][sustained] 完成: updates={} comm_errors={}",
                 diag.total_update_count, diag.comm_error_count);

    // 10s / 200ms ≈ 50次，至少 40次
    CHECK(diag.total_update_count >= 40);
    // 通信错误率低于 50%
    CHECK(diag.comm_error_count < diag.total_update_count / 2 + 1);
}
