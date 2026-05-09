/**
 * @file brush_hw_test.cc
 * @brief ODrive BrushMotor 真实硬件测试
 *
 * 测试分组：
 *   [hw_brush][open_close]       - 串口打开 + watchdog/basic 命令写入
 *   [hw_brush][speed_cycle]      - 速度模式主链
 *   [hw_brush][torque_cycle]     - 力矩模式主链
 *   [hw_brush][idle_and_stop]    - stop / enter_idle 收敛
 *   [hw_brush][diagnostics_poll] - update/status/diagnostics 轮询
 *
 * 运行方法（目标机）：
 *   ./hw_tests "[hw_brush]"
 *   HW_TEST_CONFIG=/path/to/hw_test_config.json ./hw_tests "[hw_brush][speed_cycle]"
 *
 * 安全说明：
 *   - 这些用例默认就是“真实硬件测试”，不再额外用环境变量门闩。
 *   - 运行前必须确认滚刷已处于安全工位，且 brush_test_rpm 为低速。
 */

#include <algorithm>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"

using namespace std::chrono_literals;

namespace {

static const hw::HwParams kp = hw::load_hw_test_config();

class MotorSafeStopGuard {
   public:
    explicit MotorSafeStopGuard(robot::device::BrushMotor& motor) : motor_(motor) {}

    ~MotorSafeStopGuard() {
        motor_.stop();
        std::this_thread::sleep_for(300ms);
        motor_.enter_idle();
    }

   private:
    robot::device::BrushMotor& motor_;
};

struct BrushFixture {
    robot::hal::UartConfig uart_cfg{};
    std::shared_ptr<robot::driver::LibSerialPort> serial;
    robot::device::BrushMotor motor;

    BrushFixture()
        : serial(std::make_shared<robot::driver::LibSerialPort>(kp.brush_port, build_uart_config()))
        , motor(serial,
                kp.brush_axis,
                kp.brush_counts_per_rev,
                kp.brush_watchdog_enabled,
                kp.brush_watchdog_timeout_s) {}

    static robot::hal::UartConfig build_uart_config() {
        robot::hal::UartConfig cfg;
        cfg.baudrate = kp.brush_baud;
        cfg.data_bits = 8;
        cfg.parity = 'N';
        cfg.stop_bits = 1;
        cfg.write_timeout_ms = 500;
        return cfg;
    }
};

void require_brush_hw_params() {
    INFO("brush_port=" << kp.brush_port);
    INFO("brush_baud=" << kp.brush_baud);
    INFO("brush_axis=" << static_cast<int>(kp.brush_axis));
    INFO("brush_counts_per_rev=" << kp.brush_counts_per_rev);
    INFO("brush_watchdog_enabled=" << kp.brush_watchdog_enabled);
    INFO("brush_watchdog_timeout_s=" << kp.brush_watchdog_timeout_s);
    INFO("brush_test_rpm=" << kp.brush_test_rpm);

    REQUIRE(!kp.brush_port.empty());
    REQUIRE(kp.brush_baud > 0);
    REQUIRE(kp.brush_counts_per_rev > 0.0f);
    REQUIRE(kp.brush_watchdog_timeout_s > 0.0f);
    REQUIRE(std::abs(kp.brush_test_rpm) >= 5.0f);
    REQUIRE(std::abs(kp.brush_test_rpm) <= 5000.0f);
}

robot::device::BrushMotor::Diagnostics stream_diagnostics(robot::device::BrushMotor& motor,
                                                          const char* label,
                                                          std::chrono::milliseconds duration,
                                                          std::chrono::milliseconds interval) {
    robot::device::BrushMotor::Diagnostics diag;
    const auto deadline = std::chrono::steady_clock::now() + duration;
    int sample_idx = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        motor.update();
        diag = motor.get_diagnostics();
        spdlog::info(
            "[hw_brush][{}] sample={} actual_rpm={} target_rpm={} target_torque_nm={:.3f} "
            "current_a={:.3f} bus_voltage_v={:.3f} temp_c={:.3f} running={} fault={} "
            "fault_code={} comm_error_count={}",
            label,
            sample_idx,
            diag.actual_rpm,
            diag.target_rpm,
            static_cast<double>(diag.target_torque_nm),
            static_cast<double>(diag.current_a),
            static_cast<double>(diag.bus_voltage_v),
            static_cast<double>(diag.temperature_c),
            diag.running,
            diag.fault,
            diag.fault_code,
            diag.comm_error_count);
        ++sample_idx;
        std::this_thread::sleep_for(interval);
    }
    return diag;
}

void log_diag(const char* label, const robot::device::BrushMotor::Diagnostics& diag) {
    INFO(label << "_actual_rpm=" << diag.actual_rpm);
    INFO(label << "_target_rpm=" << diag.target_rpm);
    INFO(label << "_target_torque_nm=" << diag.target_torque_nm);
    INFO(label << "_bus_voltage_v=" << diag.bus_voltage_v);
    INFO(label << "_current_a=" << diag.current_a);
    INFO(label << "_temperature_c=" << diag.temperature_c);
    INFO(label << "_running=" << diag.running);
    INFO(label << "_fault=" << diag.fault);
    INFO(label << "_fault_code=" << diag.fault_code);
    INFO(label << "_comm_error_count=" << diag.comm_error_count);
}

}  // namespace

TEST_CASE("BrushMotor（真实硬件）打开串口并下发基础初始化命令", "[hw_brush][open_close]") {
    require_brush_hw_params();

    BrushFixture f;
    REQUIRE(f.motor.open());
    MotorSafeStopGuard guard(f.motor);

    REQUIRE(f.motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.enter_idle() == robot::device::DeviceError::OK);
}

TEST_CASE("BrushMotor（真实硬件）速度模式主链", "[hw_brush][speed_cycle]") {
    require_brush_hw_params();

    BrushFixture f;
    REQUIRE(f.motor.open());
    MotorSafeStopGuard guard(f.motor);
    REQUIRE(f.motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.set_mode_speed() == robot::device::DeviceError::OK);

    const int target_rpm = static_cast<int>(std::lround(kp.brush_test_rpm));
    REQUIRE(f.motor.set_rpm(target_rpm) == robot::device::DeviceError::OK);

    spdlog::info("[hw_brush][speed_cycle] spin up: port={} axis={} target_rpm={}",
                 kp.brush_port,
                 static_cast<int>(kp.brush_axis),
                 target_rpm);
    const auto diag = stream_diagnostics(f.motor, "speed_cycle", 15000ms, 200ms);
    log_diag("speed", diag);

    CHECK_FALSE(diag.fault);
    CHECK(diag.bus_voltage_v > 5.0f);
    CHECK(std::abs(diag.actual_rpm) >= 5);
    CHECK(diag.target_rpm == target_rpm);
    CHECK((diag.actual_rpm > 0) == (target_rpm > 0));
}

TEST_CASE("BrushMotor（真实硬件）力矩模式主链", "[hw_brush][torque_cycle]") {
    require_brush_hw_params();

    BrushFixture f;
    REQUIRE(f.motor.open());
    MotorSafeStopGuard guard(f.motor);
    REQUIRE(f.motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.set_mode_torque() == robot::device::DeviceError::OK);

    constexpr float kTestTorqueNm = 0.15f;
    REQUIRE(f.motor.set_torque(kTestTorqueNm) == robot::device::DeviceError::OK);
    const auto diag = stream_diagnostics(f.motor, "torque_cycle", 12000ms, 200ms);
    log_diag("torque", diag);

    CHECK_FALSE(diag.fault);
    CHECK(diag.bus_voltage_v > 5.0f);
    CHECK(std::abs(diag.target_torque_nm - kTestTorqueNm) < 0.02f);
    CHECK(diag.comm_error_count == 0u);
}

TEST_CASE("BrushMotor（真实硬件）stop 和 enter_idle 能让滚刷收敛下来",
          "[hw_brush][idle_and_stop]") {
    require_brush_hw_params();

    BrushFixture f;
    REQUIRE(f.motor.open());
    MotorSafeStopGuard guard(f.motor);
    REQUIRE(f.motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.set_mode_speed() == robot::device::DeviceError::OK);

    const int target_rpm = static_cast<int>(std::lround(kp.brush_test_rpm));
    REQUIRE(f.motor.set_rpm(target_rpm) == robot::device::DeviceError::OK);
    const auto running = stream_diagnostics(f.motor, "idle_and_stop_running", 12000ms, 200ms);
    REQUIRE(std::abs(running.actual_rpm) >= 5);

    REQUIRE(f.motor.stop() == robot::device::DeviceError::OK);
    const auto stopped = stream_diagnostics(f.motor, "idle_and_stop_stopped", 4000ms, 200ms);
    log_diag("stopped", stopped);
    CHECK_FALSE(stopped.fault);
    CHECK(std::abs(stopped.actual_rpm) < std::max(5, std::abs(running.actual_rpm)));
    CHECK(stopped.target_rpm == 0);

    REQUIRE(f.motor.enter_idle() == robot::device::DeviceError::OK);
    const auto idled = stream_diagnostics(f.motor, "idle_and_stop_idled", 2000ms, 200ms);
    log_diag("idled", idled);
    CHECK_FALSE(idled.fault);
    CHECK(idled.target_rpm == 0);
    CHECK(std::abs(idled.target_torque_nm) < 0.001f);
}

TEST_CASE("BrushMotor（真实硬件）status 和 diagnostics 在轮询后保持一致",
          "[hw_brush][diagnostics_poll]") {
    require_brush_hw_params();

    BrushFixture f;
    REQUIRE(f.motor.open());
    MotorSafeStopGuard guard(f.motor);
    REQUIRE(f.motor.clear_fault() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.set_mode_speed() == robot::device::DeviceError::OK);
    REQUIRE(f.motor.set_rpm(static_cast<int>(std::lround(kp.brush_test_rpm))) ==
            robot::device::DeviceError::OK);

    const auto diag = stream_diagnostics(f.motor, "diagnostics_poll", 12000ms, 200ms);
    const auto status = f.motor.get_status();
    log_diag("poll", diag);

    CHECK(status.actual_rpm == diag.actual_rpm);
    CHECK(status.current_a == Approx(diag.current_a).epsilon(0.001f));
    CHECK(status.running == diag.running);
    CHECK(status.fault == diag.fault);
    CHECK(status.fault_code == diag.fault_code);
    CHECK(diag.bus_voltage_v > 5.0f);
    CHECK(diag.comm_error_count == 0u);
}
