/**
 * 距离传感器设备层单元测试
 *
 * 测试分组：[distance_sensor][device]
 *
 * 使用 MockModbusMaster（input_registers）模拟 FC=0x04 响应。
 * 不依赖真实硬件，可在 QEMU/host 上运行。
 */
#include <catch2/catch.hpp>
#include "pv_cleaning_robot/device/distance_sensor.h"
#include "mock/mock_modbus_master.h"

using namespace robot;
using namespace robot::device;
using namespace robot::protocol;

// ── 正常读取 ──────────────────────────────────────────────────────────────
TEST_CASE("DistanceSensor update - normal voltage read (variable decimal)",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened = true;

    // ch0: 31000 → 1.000 V（variable decimal）
    // ch1: 25000 → 50.00 V
    // ch2: 0     → 0.000 V
    // ch3: 39999 → 9.999 V
    mock->input_registers[0] = 31000;
    mock->input_registers[1] = 25000;
    mock->input_registers[2] = 0;
    mock->input_registers[3] = 39999;

    DistanceSensorConfig cfg;
    cfg.slave_id      = 1;
    cfg.channel_count = 4;
    cfg.decimal_mode  = DistDecimalMode::VARIABLE;
    cfg.analog_type   = DistAnalogType::VOLTAGE;

    DistanceSensor sensor(mock, cfg);
    sensor.update();

    auto data = sensor.get_data();
    REQUIRE(data.channel_count == 4);
    REQUIRE(data.error_count   == 0);

    REQUIRE(data.channels[0].valid);
    CHECK(data.channels[0].value_v  == Approx(1.0f).epsilon(0.001f));
    CHECK(data.channels[0].value_ma == Approx(0.0f));  // 电压型，value_ma 无效

    REQUIRE(data.channels[1].valid);
    CHECK(data.channels[1].value_v  == Approx(50.0f).epsilon(0.001f));

    REQUIRE(data.channels[2].valid);
    CHECK(data.channels[2].value_v  == Approx(0.0f));

    REQUIRE(data.channels[3].valid);
    CHECK(data.channels[3].value_v  == Approx(9.999f).epsilon(0.001f));
}

TEST_CASE("DistanceSensor update - fixed decimal FIXED_3",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened = true;
    mock->input_registers[0] = 12076;  // fixed-3 → 12.076 V

    DistanceSensorConfig cfg;
    cfg.channel_count = 1;
    cfg.decimal_mode  = DistDecimalMode::FIXED_3;
    cfg.analog_type   = DistAnalogType::VOLTAGE;

    DistanceSensor sensor(mock, cfg);
    sensor.update();

    auto data = sensor.get_data();
    REQUIRE(data.channels[0].valid);
    CHECK(data.channels[0].value_v == Approx(12.076f).epsilon(0.001f));
}

// ── 电流型换算 ────────────────────────────────────────────────────────────
TEST_CASE("DistanceSensor update - current type voltage_to_ma",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened = true;
    // 31000 → 1.000 V → 1.0 / 249 × 1000 ≈ 4.016 mA
    mock->input_registers[0] = 31000;

    DistanceSensorConfig cfg;
    cfg.channel_count = 1;
    cfg.decimal_mode  = DistDecimalMode::VARIABLE;
    cfg.analog_type   = DistAnalogType::CURRENT;

    DistanceSensor sensor(mock, cfg);
    sensor.update();

    auto data = sensor.get_data();
    REQUIRE(data.channels[0].valid);
    CHECK(data.channels[0].value_v  == Approx(1.0f).epsilon(0.001f));
    CHECK(data.channels[0].value_ma == Approx(4.016f).epsilon(0.01f));
}

// ── 通信失败 ──────────────────────────────────────────────────────────────
TEST_CASE("DistanceSensor update - comm error marks channels invalid",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened        = true;
    mock->read_reg_return = -1;  // 强制 FC=0x04 失败

    DistanceSensorConfig cfg;
    cfg.channel_count = 2;

    DistanceSensor sensor(mock, cfg);
    sensor.update();

    auto data = sensor.get_data();
    CHECK(!data.channels[0].valid);
    CHECK(!data.channels[1].valid);
    CHECK(data.error_count == 1);

    auto diag = sensor.get_diagnostics();
    CHECK(diag.comm_error_count    == 1);
    CHECK(diag.total_update_count  == 1);
}

TEST_CASE("DistanceSensor update - modbus not open",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened = false;  // 未打开

    DistanceSensor sensor(mock, DistanceSensorConfig{});
    sensor.update();

    auto diag = sensor.get_diagnostics();
    CHECK(diag.comm_error_count   == 1);
    CHECK(diag.total_update_count == 1);
    CHECK(!diag.channels[0].valid);
}

// ── 通道数截断 ────────────────────────────────────────────────────────────
TEST_CASE("DistanceSensor channel_count clamped to [1, 18]",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();

    // 超过最大值截断为 18
    DistanceSensorConfig cfg_over;
    cfg_over.channel_count = 255;
    DistanceSensor s_over(mock, cfg_over);
    CHECK(s_over.get_data().channel_count == 18);

    // 零值截断为 1
    DistanceSensorConfig cfg_zero;
    cfg_zero.channel_count = 0;
    DistanceSensor s_zero(mock, cfg_zero);
    CHECK(s_zero.get_data().channel_count == 1);
}

// ── 多次更新累计统计 ─────────────────────────────────────────────────────
TEST_CASE("DistanceSensor update - multiple updates accumulate correctly",
          "[distance_sensor][device]")
{
    auto mock = std::make_shared<MockModbusMaster>();
    mock->opened = true;
    mock->input_registers[0] = 31000;

    DistanceSensorConfig cfg;
    cfg.channel_count = 1;

    DistanceSensor sensor(mock, cfg);

    // 2次成功
    sensor.update();
    sensor.update();

    // 1次失败
    mock->read_reg_return = -1;
    sensor.update();

    auto diag = sensor.get_diagnostics();
    CHECK(diag.total_update_count == 3);
    CHECK(diag.comm_error_count   == 1);
    CHECK(diag.error_count        == 1);
    CHECK(!diag.channels[0].valid);  // 最后一次失败，valid=false
}
