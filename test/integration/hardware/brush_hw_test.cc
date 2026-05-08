/*
 * @Author: UncleDrew
 * @Date: 2026-05-08 19:35:51
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-05-08 20:02:49
 * @FilePath: /pv_cleaning_robot/test/integration/hardware/brush_hw_test.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */

#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"

namespace {

int env_int(const char* name, int default_value) {
    const char* v = std::getenv(name);
    if (!v || *v == '\0')
        return default_value;
    return std::atoi(v);
}

float env_float(const char* name, float default_value) {
    const char* v = std::getenv(name);
    if (!v || *v == '\0')
        return default_value;
    return std::atof(v);
}

std::string env_string(const char* name, const std::string& default_value) {
    const char* v = std::getenv(name);
    if (!v || *v == '\0')
        return default_value;
    return std::string(v);
}

class MotorSafeStopGuard {
   public:
    explicit MotorSafeStopGuard(robot::device::BrushMotor& motor) : motor_(motor) {}

    ~MotorSafeStopGuard() {
        motor_.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        motor_.enter_idle();
    }

   private:
    robot::device::BrushMotor& motor_;
};

}  // namespace

TEST_CASE("real BrushMotor can rotate and stop", "[hardware][brush_motor]") {
    const char* enable = std::getenv("PV_ENABLE_REAL_BRUSH_MOTOR_TEST");
    if (!enable || std::string(enable) != "1") {
        spdlog::info("Set PV_ENABLE_REAL_BRUSH_MOTOR_TEST=1 to run real brush motor test");
    }

    const std::string port = env_string("PV_BRUSH_SERIAL", "/dev/ttyS2");
    const int baudrate = env_int("PV_BRUSH_BAUD", 115200);
    const int axis = env_int("PV_BRUSH_AXIS", 0);
    const float counts_per_rev = env_float("PV_BRUSH_COUNTS_PER_REV", 100.0f);
    const int test_rpm = env_int("PV_BRUSH_TEST_RPM", 30);

    REQUIRE(axis >= 0);
    REQUIRE(axis <= 255);
    REQUIRE(counts_per_rev > 0.0f);
    REQUIRE(std::abs(test_rpm) > 0);
    REQUIRE(std::abs(test_rpm) <= 100);

    robot::hal::UartConfig cfg;
    cfg.baudrate = 115200;
    cfg.data_bits = 8;
    cfg.parity = 'N';
    cfg.stop_bits = 1;
    cfg.write_timeout_ms = 500;

    // auto serial = std::make_shared<driver::LibSerialPort>(kp.bms_port, cfg);

    auto serial = std::make_shared<robot::driver::LibSerialPort>(port, cfg);

    robot::device::BrushMotor motor(serial,
                                    static_cast<uint8_t>(axis),
                                    counts_per_rev,
                                    true,  // watchdog_enabled
                                    0.5f   // watchdog_timeout_s
    );

    REQUIRE(motor.open());

    MotorSafeStopGuard guard(motor);

    REQUIRE(motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(motor.set_mode_speed() == robot::device::DeviceError::OK);

    REQUIRE(motor.set_rpm(test_rpm) == robot::device::DeviceError::OK);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (int i = 0; i < 5; ++i) {
        motor.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const auto diag = motor.get_diagnostics();

    INFO("actual_rpm=" << diag.actual_rpm);
    INFO("bus_voltage_v=" << diag.bus_voltage_v);
    INFO("current_a=" << diag.current_a);
    INFO("temperature_c=" << diag.temperature_c);
    INFO("fault=" << diag.fault);
    INFO("fault_code=" << diag.fault_code);
    INFO("comm_error_count=" << diag.comm_error_count);

    CHECK_FALSE(diag.fault);
    CHECK(diag.bus_voltage_v > 5.0f);

    // 只验证“确实转起来了”，不要求速度完全准确。
    CHECK(std::abs(diag.actual_rpm) > 5);

    REQUIRE(motor.stop() == robot::device::DeviceError::OK);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    motor.update();
    const auto stopped = motor.get_diagnostics();

    INFO("stopped_actual_rpm=" << stopped.actual_rpm);

    CHECK(std::abs(stopped.actual_rpm) < std::abs(test_rpm));
}