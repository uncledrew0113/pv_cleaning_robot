// test/integration/hardware/limit_switch_hw_test.cc
/**
 * 限位传感器硬件单元测试
 *
 * 测试分组：
 *   [hw_limit][open]             - GPIO open 成功
 *   [hw_limit][read_level_primary_dock]  - 主停机端电平自检（B=低/触发，A=高/未触发）
 *   [hw_limit][callback_left]   - 左侧传感器回调链路（需手动触发）
 *   [hw_limit][callback_right]    - 右侧传感器回调链路（需手动触发）
 *   [hw_limit][is_triggered]     - is_triggered() 状态查询
 *   [hw_limit][clear_trigger]    - clear_trigger() 清除状态
 *   [hw_limit][side_enum]        - 回调参数 LimitSide 正确传递
 *   [hw_limit][repeated_trigger] - 多次触发各计一次（传感器稳定性）
 *
 * 运行方法（目标板）：
 *   ./hw_tests "[hw_limit]"                    # 全部测试
 *   ./hw_tests "[hw_limit][open]"              # 仅 open 测试（无需机器人在停机位）
 *   ./hw_tests "[hw_limit][callback_left]"    # 需手动触发左侧传感器
 *
 * 前提：机器人停在停机位（右限位已触发）；can0 不必配置（本文件不涉及 CAN）
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

#include "hw_config.h"

using namespace robot;
using namespace std::chrono_literals;

static const hw::HwParams kp = hw::load_hw_test_config();

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][open] — GPIO 初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器 GPIO 初始化", "[limit][limit.init]") {
    auto left_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line);
    auto right_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_limit_line);
    auto left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);
    auto right_sw = std::make_shared<device::LimitSwitch>(right_gpio, device::LimitSide::RIGHT);

    SECTION("左限位 GPIO open 成功") {
        // rt_priority=0(SCHED_OTHER), debounce_ms=2, cpu_affinity=0(不绑定)
        REQUIRE(left_sw->open(0, 2, 0, false));
        spdlog::info("[hw_limit][open] 左限位 GPIO open 成功");
        left_sw->close();
    }

    SECTION("右限位 GPIO open 成功") {
        REQUIRE(right_sw->open(0, 2, 0, false));
        spdlog::info("[hw_limit][open] 右限位 GPIO open 成功");
        right_sw->close();
    }

    SECTION("重复 open/close 不崩溃") {
        REQUIRE(left_sw->open(0, 2, 0, false));
        left_sw->close();
        REQUIRE(left_sw->open(0, 2, 0, false));  // 第二次 open 也应成功
        left_sw->close();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][read_level_primary_dock] — 主停机端电平自检
// 运行前提：机器人停在停机位
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("主停机端 GPIO 电平自检", "[limit][limit.primary-level]") {
    // read_current_level() 语义：true=高电平/未遮挡，false=低电平/已遮挡
    // 停机位期望：right=false（遮挡），left=true（未遮挡）
    auto left_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line);
    auto right_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_limit_line);
    auto left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);
    auto right_sw = std::make_shared<device::LimitSwitch>(right_gpio, device::LimitSide::RIGHT);

    REQUIRE(left_sw->open(0, 2, 0, false));
    REQUIRE(right_sw->open(0, 2, 0, false));

    const bool left_level = left_sw->read_current_level();
    const bool right_level = right_sw->read_current_level();

    spdlog::info("[hw_limit][read_level_primary_dock] left_level={} (期望 true=未遮挡)",
                 left_level);
    spdlog::info("[hw_limit][read_level_primary_dock] right_level={}  (期望 false=主停机端遮挡)",
                 right_level);

    CHECK(left_level == true);    // 前端：未遮挡 = 高电平
    CHECK(right_level == false);  // 尾端：停机位遮挡 = 低电平

    left_sw->close();
    right_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_left] — 左侧传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("左限位传感器回调链路（手动触发）", "[limit][limit.left][manual]") {
    auto left_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line);
    auto left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);

    std::atomic<int> cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::LEFT};

    left_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_left] 回调触发！side={}",
                     (side == device::LimitSide::LEFT ? "LEFT" : "RIGHT"));
    });

    REQUIRE(left_sw->open(0, 2, 0, false));
    left_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_left] ★ 请在 5 秒内手动触发【左限位】传感器 ★");

    // 等待最多 5 秒
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_left] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::LEFT);

    left_sw->stop_monitoring();
    left_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_right] — 右侧传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("右限位传感器回调链路（手动触发）", "[limit][limit.right][manual]") {
    auto right_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.right_limit_line);
    auto right_sw = std::make_shared<device::LimitSwitch>(right_gpio, device::LimitSide::RIGHT);

    std::atomic<int> cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::RIGHT};

    right_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_right] 回调触发！side={}",
                     (side == device::LimitSide::LEFT ? "LEFT" : "RIGHT"));
    });

    REQUIRE(right_sw->open(0, 2, 0, false));
    right_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_right] ★ 请在 5 秒内手动触发【右限位】传感器 ★");

    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_right] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::RIGHT);

    right_sw->stop_monitoring();
    right_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][repeated_trigger] — 传感器稳定性（多次触发各计一次）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器多次触发稳定性", "[limit][limit.repeat][manual]") {
    auto left_gpio = std::make_shared<driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line);
    auto left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);

    std::atomic<int> cb_count{0};
    left_sw->set_trigger_callback([&](device::LimitSide) { cb_count.fetch_add(1); });

    REQUIRE(left_sw->open(0, 2, 0, false));
    left_sw->start_monitoring();

    spdlog::warn(
        "[hw_limit][repeated_trigger] ★ 请在 15 秒内触发【左限位】3 次（每次间隔 >500ms）★");
    std::this_thread::sleep_for(15s);

    spdlog::info("[hw_limit][repeated_trigger] 触发次数={}", cb_count.load());
    CHECK(cb_count.load() >= 3);

    left_sw->stop_monitoring();
    left_sw->close();
}
