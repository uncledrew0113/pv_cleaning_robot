#include <catch2/catch.hpp>

#include <string>

#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/brush_motor.h"

using robot::device::BrushMotor;
using robot::device::DeviceError;

struct BrushMotorFixture {
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    BrushMotor motor{serial, 0};
};

TEST_CASE("BrushMotor opens backing serial port", "[device][brush_motor]") {
    BrushMotorFixture f;
    f.serial->open_result = true;
    REQUIRE(f.motor.open());
    REQUIRE(f.serial->opened);
}

TEST_CASE("BrushMotor set_rpm sends velocity command immediately", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.open());
    f.serial->clear_tx();

    REQUIRE(f.motor.set_rpm(600) == DeviceError::OK);

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("v 0 10.000 0\n") != std::string::npos);
    REQUIRE(f.motor.get_diagnostics().target_rpm == 600);
}

TEST_CASE("BrushMotor stop clears target and running state", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.open());
    REQUIRE(f.motor.set_rpm(300) == DeviceError::OK);
    f.serial->clear_tx();

    REQUIRE(f.motor.stop() == DeviceError::OK);

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("v 0 0.000 0\n") != std::string::npos);
    REQUIRE(f.motor.get_diagnostics().target_rpm == 0);
    REQUIRE_FALSE(f.motor.get_status().running);
}

TEST_CASE("BrushMotor update parses odrive feedback and diagnostics", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.open());
    REQUIRE(f.motor.set_rpm(600) == DeviceError::OK);

    f.serial->queue_rx_text("1.5 5.0\n");
    f.serial->queue_rx_text("24.5\n");
    f.serial->queue_rx_text("3.25\n");
    f.serial->queue_rx_text("46.0\n");
    f.serial->queue_rx_text("0\n");
    f.serial->queue_rx_text("0\n");
    f.serial->queue_rx_text("0\n");

    f.motor.update();

    const auto d = f.motor.get_diagnostics();
    REQUIRE(d.actual_rpm == 300);
    REQUIRE(d.bus_voltage_v == Approx(24.5f));
    REQUIRE(d.current_a == Approx(3.25f));
    REQUIRE(d.temperature_c == Approx(46.0f));
    REQUIRE_FALSE(d.fault);
    REQUIRE(d.running);
}

TEST_CASE("BrushMotor restart sends odrive reboot command", "[device][brush_motor]") {
    BrushMotorFixture f;
    REQUIRE(f.motor.open());
    REQUIRE(f.motor.set_rpm(100) == DeviceError::OK);
    f.serial->clear_tx();

    REQUIRE(f.motor.restart() == DeviceError::OK);

    REQUIRE(f.serial->take_tx_text().find("sr\n") != std::string::npos);
    REQUIRE(f.motor.get_diagnostics().target_rpm == 0);
}
