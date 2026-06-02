// test/integration/hardware/attitude_limit_switch_hw_test.cc
/**
 * 姿态偏移极限接近传感器硬件测试。
 *
 * 运行方法（目标板）：
 *   ./hw_tests "[hw_attitude_limit][open]"
 *   ./hw_tests "[hw_attitude_limit][read_status]"
 *   ./hw_tests "[hw_attitude_limit][callback_left_lower]"
 *   ./hw_tests "[hw_attitude_limit][callback_right_lower]"
 *
 * 说明：
 *   本文件使用真实 LibGpiodPin 访问 gpiochip5 line2/3，不接入业务控制。
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "hw_config.h"
#include "pv_cleaning_robot/device/attitude_limit_switch.h"

using namespace robot;
using namespace std::chrono_literals;

static const hw::HwParams kp = hw::load_hw_test_config();

namespace {

const char* side_name(device::AttitudeLimitSide side) {
    return side == device::AttitudeLimitSide::LEFT_LOWER ? "LEFT_LOWER" : "RIGHT_LOWER";
}

void log_status(const char* tag, const device::AttitudeLimitSwitch::Status& status) {
    spdlog::info("{} side={} level={} active_low={} triggered={}",
                 tag,
                 side_name(status.side),
                 status.level_high ? "high" : "low",
                 status.active_low_asserted,
                 status.triggered);
}

}  // namespace

TEST_CASE("姿态极限接近传感器 GPIO 初始化", "[hw_attitude_limit][open]") {
    auto left_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_attitude_limit_line);
    auto right_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_attitude_limit_line);
    device::AttitudeLimitSwitch left_sw(left_gpio, device::AttitudeLimitSide::LEFT_LOWER);
    device::AttitudeLimitSwitch right_sw(right_gpio, device::AttitudeLimitSide::RIGHT_LOWER);

    REQUIRE(left_sw.open(0, 2, 0, false));
    REQUIRE(right_sw.open(0, 2, 0, false));
    spdlog::info("[hw_attitude_limit][open] left line={} right line={}",
                 kp.left_attitude_limit_line,
                 kp.right_attitude_limit_line);
}

TEST_CASE("姿态极限接近传感器连续读取状态", "[hw_attitude_limit][read_status]") {
    auto left_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_attitude_limit_line);
    auto right_gpio =
        std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_attitude_limit_line);
    device::AttitudeLimitSwitch left_sw(left_gpio, device::AttitudeLimitSide::LEFT_LOWER);
    device::AttitudeLimitSwitch right_sw(right_gpio, device::AttitudeLimitSide::RIGHT_LOWER);

    REQUIRE(left_sw.open(0, 2, 0, false));
    REQUIRE(right_sw.open(0, 2, 0, false));

    for (int i = 0; i < 20000; ++i) {
        log_status("[hw_attitude_limit][read_status][left]", left_sw.read_status());
        log_status("[hw_attitude_limit][read_status][right]", right_sw.read_status());
        std::this_thread::sleep_for(100ms);
    }
}

TEST_CASE("左下姿态极限接近传感器回调链路（手动触发）",
          "[hw_attitude_limit][callback_left_lower]") {
    auto gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_attitude_limit_line);
    device::AttitudeLimitSwitch sw(gpio, device::AttitudeLimitSide::LEFT_LOWER);

    std::atomic<int> cb_count{0};
    std::atomic<device::AttitudeLimitSide> cb_side{device::AttitudeLimitSide::RIGHT_LOWER};
    sw.set_trigger_callback([&](device::AttitudeLimitSide side) {
        cb_side.store(side);
        cb_count.fetch_add(1);
        spdlog::info("[hw_attitude_limit][callback_left_lower] callback side={}", side_name(side));
    });

    REQUIRE(sw.open(0, 2, 0, false));
    sw.start_monitoring();
    spdlog::warn("[hw_attitude_limit][callback_left_lower] 请在 10 秒内手动触发左下接近传感器");

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
        log_status("[hw_attitude_limit][callback_left_lower]", sw.read_status());
        std::this_thread::sleep_for(100ms);
    }

    sw.stop_monitoring();
    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::AttitudeLimitSide::LEFT_LOWER);
    CHECK(sw.is_triggered());
}

TEST_CASE("右下姿态极限接近传感器回调链路（手动触发）",
          "[hw_attitude_limit][callback_right_lower]") {
    auto gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_attitude_limit_line);
    device::AttitudeLimitSwitch sw(gpio, device::AttitudeLimitSide::RIGHT_LOWER);

    std::atomic<int> cb_count{0};
    std::atomic<device::AttitudeLimitSide> cb_side{device::AttitudeLimitSide::LEFT_LOWER};
    sw.set_trigger_callback([&](device::AttitudeLimitSide side) {
        cb_side.store(side);
        cb_count.fetch_add(1);
        spdlog::info("[hw_attitude_limit][callback_right_lower] callback side={}", side_name(side));
    });

    REQUIRE(sw.open(0, 2, 0, false));
    sw.start_monitoring();
    spdlog::warn("[hw_attitude_limit][callback_right_lower] 请在 10 秒内手动触发右下接近传感器");

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
        log_status("[hw_attitude_limit][callback_right_lower]", sw.read_status());
        std::this_thread::sleep_for(100ms);
    }

    sw.stop_monitoring();
    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::AttitudeLimitSide::RIGHT_LOWER);
    CHECK(sw.is_triggered());
}
