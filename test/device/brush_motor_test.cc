/**
 * BrushMotor 设备层单元测试（依赖 MockModbusMaster）
 * [device][brush_motor]
 */
#include <catch2/catch.hpp>

#include "../mock/mock_modbus_master.h"
#include "pv_cleaning_robot/device/brush_motor.h"

using robot::device::BrushMotor;
using robot::device::DeviceError;

// ────────────────────────────────────────────────────────────────
// 构建辅助：BrushMotor + MockModbusMaster 组合
// ────────────────────────────────────────────────────────────────
struct BrushMotorFixture {
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    BrushMotor motor;

    BrushMotorFixture() : motor(modbus, 1) {
        modbus->open_result = true;
        modbus->write_reg_return = 0;  // 0=成功
        modbus->read_reg_return = 0;   // >=0=成功，返回读取数
        motor.open();
    }

    /// 注入指定寄存器的读取值
    void inject_reg(int addr, uint16_t val) {
        modbus->registers[addr] = val;
    }
};

// ────────────────────────────────────────────────────────────────
// open()
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: open() 调用 modbus open 并返回 true", "[device][brush_motor]") {
    auto modbus = std::make_shared<MockModbusMaster>();
    BrushMotor motor(modbus, 1);

    modbus->open_result = true;
    REQUIRE(motor.open());
    REQUIRE(modbus->opened);
}

TEST_CASE("BrushMotor: open() modbus 失败时返回 false", "[device][brush_motor]") {
    auto modbus = std::make_shared<MockModbusMaster>();
    BrushMotor motor(modbus, 1);

    modbus->open_result = false;
    REQUIRE_FALSE(motor.open());
}

// ────────────────────────────────────────────────────────────────
// start() / stop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: start() 写 REG_ENABLE=1", "[device][brush_motor]") {
    BrushMotorFixture f;
    auto err = f.motor.start();
    REQUIRE(err == DeviceError::OK);

    // 找到写 REG_ENABLE 的调用
    bool found = false;
    for (auto& c : f.modbus->write_calls) {
        if (c.addr == BrushMotor::REG_ENABLE && c.val == 1) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

TEST_CASE("BrushMotor: stop() 写 REG_ENABLE=0", "[device][brush_motor]") {
    BrushMotorFixture f;
    auto err = f.motor.stop();
    REQUIRE(err == DeviceError::OK);

    bool found = false;
    for (auto& c : f.modbus->write_calls) {
        if (c.addr == BrushMotor::REG_ENABLE && c.val == 0) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

// ────────────────────────────────────────────────────────────────
// set_rpm()
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: set_rpm(1200) 写 REG_TARGET_RPM=1200", "[device][brush_motor]") {
    BrushMotorFixture f;
    auto err = f.motor.set_rpm(1200);
    REQUIRE(err == DeviceError::OK);

    bool found = false;
    for (auto& c : f.modbus->write_calls) {
        if (c.addr == BrushMotor::REG_TARGET_RPM && static_cast<int16_t>(c.val) == 1200) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

TEST_CASE("BrushMotor: set_rpm() 负值（反转）写入的有符号值正确", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.motor.set_rpm(-600);

    bool found = false;
    for (auto& c : f.modbus->write_calls) {
        if (c.addr == BrushMotor::REG_TARGET_RPM && static_cast<int16_t>(c.val) == -600) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

// ────────────────────────────────────────────────────────────────
// clear_fault()
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: clear_fault() 写 REG_CLR_FAULT=1", "[device][brush_motor]") {
    BrushMotorFixture f;
    auto err = f.motor.clear_fault();
    REQUIRE(err == DeviceError::OK);

    bool found = false;
    for (auto& c : f.modbus->write_calls) {
        if (c.addr == BrushMotor::REG_CLR_FAULT && c.val == 1) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

// ────────────────────────────────────────────────────────────────
// update() 读取寄存器更新状态缓存
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: update() 读取实际转速并更新缓存", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.inject_reg(BrushMotor::REG_ACT_RPM, 1100);
    f.inject_reg(BrushMotor::REG_STATUS, BrushMotor::STATUS_RUNNING);

    f.motor.update();

    auto st = f.motor.get_status();
    REQUIRE(st.running);
    REQUIRE(st.actual_rpm == 1100);
    REQUIRE_FALSE(st.fault);
}

TEST_CASE("BrushMotor: update() 检测故障位 STATUS_FAULT", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.inject_reg(BrushMotor::REG_STATUS, BrushMotor::STATUS_FAULT);
    f.inject_reg(BrushMotor::REG_FAULT_CODE, 0x0010);

    f.motor.update();

    auto st = f.motor.get_status();
    REQUIRE(st.fault);
    REQUIRE_FALSE(st.running);
}

// ────────────────────────────────────────────────────────────────
// 通信失败降级
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: modbus 读失败时 update() 不崩溃", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.modbus->read_reg_return = -1;  // 模拟读取超时

    REQUIRE_NOTHROW(f.motor.update());
    // 上一次有效值保持（默认全零 = running=false）
    REQUIRE_FALSE(f.motor.get_status().running);
}

TEST_CASE("BrushMotor: modbus 写失败时 start() 返回错误", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.modbus->write_reg_return = -1;

    auto err = f.motor.start();
    REQUIRE(err != DeviceError::OK);
}

// ────────────────────────────────────────────────────────────────
// get_diagnostics() 包含温度/电压
// ────────────────────────────────────────────────────────────────
TEST_CASE("BrushMotor: get_diagnostics() 含温度和总线电压", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.inject_reg(BrushMotor::REG_TEMP, 250);     // 250 → 25.0°C（*10）
    f.inject_reg(BrushMotor::REG_VOLTAGE, 240);  // 240 → 24.0V（*10）
    f.inject_reg(BrushMotor::REG_STATUS, 0);

    f.motor.update();
    auto d = f.motor.get_diagnostics();
    REQUIRE(d.temperature_c == Approx(25.0f).epsilon(0.1f));
    REQUIRE(d.bus_voltage_v == Approx(24.0f).epsilon(0.1f));
}
