#include <catch2/catch.hpp>

#include "../mock/mock_gpio_pin.h"
#include "pv_cleaning_robot/device/lock_motor.h"

namespace {

robot::device::LockMotor::Config fast_config() {
    robot::device::LockMotor::Config cfg;
    cfg.pulse_ms = 0;
    cfg.settle_ms = 0;
    return cfg;
}

}  // namespace

TEST_CASE("LockMotor initializes open and close GPIO pins as outputs", "[device][lock_motor]") {
    auto open_pin = std::make_shared<MockGpioPin>();
    auto close_pin = std::make_shared<MockGpioPin>();
    robot::device::LockMotor motor(open_pin, close_pin, fast_config());

    REQUIRE(motor.initialize());

    CHECK(open_pin->opened);
    CHECK(close_pin->opened);
    CHECK(open_pin->last_open_config.direction == robot::hal::GpioDirection::OUTPUT);
    CHECK(close_pin->last_open_config.direction == robot::hal::GpioDirection::OUTPUT);
}

TEST_CASE("LockMotor open action pulses only open GPIO high then low", "[device][lock_motor]") {
    auto open_pin = std::make_shared<MockGpioPin>();
    auto close_pin = std::make_shared<MockGpioPin>();
    robot::device::LockMotor motor(open_pin, close_pin, fast_config());
    REQUIRE(motor.initialize());

    REQUIRE(motor.open_lock());

    REQUIRE(open_pin->write_history.size() >= 2);
    CHECK(open_pin->write_history[open_pin->write_history.size() - 2] == true);
    CHECK(open_pin->write_history.back() == false);
    CHECK(close_pin->write_history.size() == 1);
    CHECK(close_pin->write_history.back() == false);
}

TEST_CASE("LockMotor close action pulses only close GPIO high then low", "[device][lock_motor]") {
    auto open_pin = std::make_shared<MockGpioPin>();
    auto close_pin = std::make_shared<MockGpioPin>();
    robot::device::LockMotor motor(open_pin, close_pin, fast_config());
    REQUIRE(motor.initialize());

    REQUIRE(motor.close_lock());

    REQUIRE(close_pin->write_history.size() >= 2);
    CHECK(close_pin->write_history[close_pin->write_history.size() - 2] == true);
    CHECK(close_pin->write_history.back() == false);
    CHECK(open_pin->write_history.size() == 1);
    CHECK(open_pin->write_history.back() == false);
}

TEST_CASE("LockMotor reports failure when GPIO write fails", "[device][lock_motor]") {
    auto open_pin = std::make_shared<MockGpioPin>();
    auto close_pin = std::make_shared<MockGpioPin>();
    robot::device::LockMotor motor(open_pin, close_pin, fast_config());
    REQUIRE(motor.initialize());
    open_pin->write_result = false;

    CHECK_FALSE(motor.open_lock());
}
