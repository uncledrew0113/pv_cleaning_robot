/**
 * @file lock_motor_hw_test.cc
 * @brief 锁止电机真实硬件集成测试。
 *
 * 本文件会实际驱动锁止电机 GPIO 输出，运行前必须确认机构动作安全。
 */
#include <catch2/catch.hpp>

#include "hw_config.h"
#include "pv_cleaning_robot/device/lock_motor.h"
#include "pv_cleaning_robot/driver/libgpiod_pin.h"

namespace {

robot::device::LockMotor::Config make_lock_motor_config(const hw::HwParams& p) {
    robot::device::LockMotor::Config cfg;
    cfg.pulse_ms = p.lock_motor_pulse_ms;
    cfg.settle_ms = p.lock_motor_settle_ms;
    return cfg;
}

std::shared_ptr<robot::device::LockMotor> make_lock_motor(const hw::HwParams& p) {
    auto open_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        p.lock_motor_open_chip, p.lock_motor_open_line, "hw_lock_motor_open");
    auto close_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        p.lock_motor_close_chip, p.lock_motor_close_line, "hw_lock_motor_close");
    return std::make_shared<robot::device::LockMotor>(
        open_gpio, close_gpio, make_lock_motor_config(p));
}

}  // namespace

TEST_CASE("LockMotor 真实 GPIO 初始化", "[hw_lock][open]") {
    const auto p = hw::load_hw_test_config();
    auto lock_motor = make_lock_motor(p);

    REQUIRE(lock_motor->initialize());
}

TEST_CASE("LockMotor 真实 GPIO 单独执行开锁止电机动作", "[hw_lock][open_lock]") {
    const auto p = hw::load_hw_test_config();
    auto lock_motor = make_lock_motor(p);
    REQUIRE(lock_motor->initialize());

    // 单独验证开锁止电机的 DO 脉冲链路，现场执行前需确认机构动作方向安全。
    REQUIRE(lock_motor->open_lock());
}

TEST_CASE("LockMotor 真实 GPIO 单独执行关锁止电机动作", "[hw_lock][close_lock]") {
    const auto p = hw::load_hw_test_config();
    auto lock_motor = make_lock_motor(p);
    REQUIRE(lock_motor->initialize());

    // 单独验证关锁止电机的 DO 脉冲链路，避免依赖完整清扫任务才能覆盖该动作。
    REQUIRE(lock_motor->close_lock());
}

TEST_CASE("LockMotor 真实 GPIO 执行关锁再开锁动作", "[hw_lock][close_open]") {
    const auto p = hw::load_hw_test_config();
    auto lock_motor = make_lock_motor(p);
    REQUIRE(lock_motor->initialize());

    // 关锁对应主程序启动清扫前释放锁止机构；开锁对应回到停机位后的锁车动作。
    REQUIRE(lock_motor->close_lock());
    REQUIRE(lock_motor->open_lock());
}
