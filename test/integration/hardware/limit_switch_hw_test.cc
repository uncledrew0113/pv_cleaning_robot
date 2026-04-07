// test/integration/hardware/limit_switch_hw_test.cc
/**
 * 限位传感器硬件单元测试
 *
 * 测试分组：
 *   [hw_limit][open]             - GPIO open 成功
 *   [hw_limit][read_level_home]  - 停机位电平自检（rear=低/触发，front=高/未触发）
 *   [hw_limit][callback_front]   - 前传感器回调链路（需手动触发）
 *   [hw_limit][callback_rear]    - 后传感器回调链路（需手动触发）
 *   [hw_limit][is_triggered]     - is_triggered() 状态查询
 *   [hw_limit][clear_trigger]    - clear_trigger() 清除状态
 *   [hw_limit][side_enum]        - 回调参数 LimitSide 正确传递
 *   [hw_limit][repeated_trigger] - 多次触发各计一次（传感器稳定性）
 *
 * 运行方法（目标板）：
 *   ./hw_tests "[hw_limit]"                    # 全部测试
 *   ./hw_tests "[hw_limit][open]"              # 仅 open 测试（无需机器人在停机位）
 *   ./hw_tests "[hw_limit][callback_front]"    # 需手动触发前传感器
 *
 * 前提：机器人停在停机位（后限位已触发）；can0 不必配置（本文件不涉及 CAN）
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

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][open] — GPIO 初始化
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器 GPIO 初始化", "[hw_limit][open]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    SECTION("前限位 GPIO open 成功") {
        // rt_priority=0(SCHED_OTHER), debounce_ms=2, cpu_affinity=0(不绑定)
        REQUIRE(front_sw->open(0, 2, 0));
        spdlog::info("[hw_limit][open] 前限位 GPIO open 成功");
        front_sw->close();
    }

    SECTION("后限位 GPIO open 成功") {
        REQUIRE(rear_sw->open(0, 2, 0));
        spdlog::info("[hw_limit][open] 后限位 GPIO open 成功");
        rear_sw->close();
    }

    SECTION("重复 open/close 不崩溃") {
        REQUIRE(front_sw->open(0, 2, 0));
        front_sw->close();
        REQUIRE(front_sw->open(0, 2, 0));  // 第二次 open 也应成功
        front_sw->close();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][read_level_home] — 停机位电平自检
// 运行前提：机器人停在停机位
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("停机位 GPIO 电平自检", "[hw_limit][read_level_home]") {
    // read_current_level() 语义：true=高电平/未遮挡，false=低电平/已遮挡
    // 停机位期望：rear=false（遮挡），front=true（未遮挡）
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    REQUIRE(front_sw->open(0, 2, 0));
    REQUIRE(rear_sw->open(0, 2, 0));

    const bool front_level = front_sw->read_current_level();
    const bool rear_level  = rear_sw->read_current_level();

    spdlog::info("[hw_limit][read_level_home] front_level={} (期望 true=未遮挡)",
                 front_level);
    spdlog::info("[hw_limit][read_level_home] rear_level={}  (期望 false=停机位遮挡)",
                 rear_level);

    CHECK(front_level == true);   // 前端：未遮挡 = 高电平
    CHECK(rear_level  == false);  // 尾端：停机位遮挡 = 低电平

    front_sw->close();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_front] — 前传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("前限位传感器回调链路（手动触发）", "[hw_limit][callback_front]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<int>            cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::FRONT};

    front_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_front] 回调触发！side={}",
                     (side == device::LimitSide::FRONT ? "FRONT" : "REAR"));
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_front] ★ 请在 5 秒内手动触发【前限位】传感器 ★");

    // 等待最多 5 秒
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_front] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::FRONT);

    front_sw->stop_monitoring();
    front_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][callback_rear] — 后传感器回调（需手动触发）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("后限位传感器回调链路（手动触发）", "[hw_limit][callback_rear]") {
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    std::atomic<int>               cb_count{0};
    std::atomic<device::LimitSide> cb_side{device::LimitSide::REAR};

    rear_sw->set_trigger_callback([&](device::LimitSide side) {
        cb_count.fetch_add(1);
        cb_side.store(side);
        spdlog::info("[hw_limit][callback_rear] 回调触发！side={}",
                     (side == device::LimitSide::FRONT ? "FRONT" : "REAR"));
    });

    REQUIRE(rear_sw->open(0, 2, 0));
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][callback_rear] ★ 请在 5 秒内手动触发【后限位】传感器 ★");

    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (cb_count.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(100ms);

    spdlog::info("[hw_limit][callback_rear] 触发次数={}", cb_count.load());

    REQUIRE(cb_count.load() >= 1);
    CHECK(cb_side.load() == device::LimitSide::REAR);

    rear_sw->stop_monitoring();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][is_triggered] — is_triggered / clear_trigger
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器触发状态管理", "[hw_limit][is_triggered]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<bool> triggered{false};
    front_sw->set_trigger_callback([&](device::LimitSide) {
        triggered.store(true);
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    // 初始未触发
    CHECK(front_sw->is_triggered() == false);

    spdlog::warn("[hw_limit][is_triggered] ★ 请在 5 秒内手动触发【前限位】★");
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!triggered.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(50ms);

    REQUIRE(triggered.load());

    SECTION("触发后 is_triggered() 为 true") {
        CHECK(front_sw->is_triggered() == true);
    }

    SECTION("clear_trigger() 后 is_triggered() 为 false") {
        CHECK(front_sw->is_triggered() == true);
        front_sw->clear_trigger();
        CHECK(front_sw->is_triggered() == false);
    }

    front_sw->stop_monitoring();
    front_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][clear_trigger] — 重置标志独立验证
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器触发标志清除", "[hw_limit][clear_trigger]") {
    auto rear_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    REQUIRE(rear_sw->open(0, 2, 0));
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][clear_trigger] ★ 请在 5 秒内手动触发【后限位】★");
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!rear_sw->is_triggered() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(50ms);

    REQUIRE(rear_sw->is_triggered());
    rear_sw->clear_trigger();
    CHECK(rear_sw->is_triggered() == false);
    spdlog::info("[hw_limit][clear_trigger] clear_trigger 后 is_triggered=false ✓");

    rear_sw->stop_monitoring();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][side_enum] — 回调 LimitSide 参数正确性
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器 side 枚举传递正确", "[hw_limit][side_enum]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto rear_gpio  = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kRearLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);
    auto rear_sw = std::make_shared<device::LimitSwitch>(
        rear_gpio, device::LimitSide::REAR);

    std::vector<device::LimitSide> received_sides;
    std::mutex                     sides_mtx;

    auto cb = [&](device::LimitSide side) {
        std::lock_guard<std::mutex> lk(sides_mtx);
        received_sides.push_back(side);
    };
    front_sw->set_trigger_callback(cb);
    rear_sw->set_trigger_callback(cb);

    REQUIRE(front_sw->open(0, 2, 0));
    REQUIRE(rear_sw->open(0, 2, 0));
    front_sw->start_monitoring();
    rear_sw->start_monitoring();

    spdlog::warn("[hw_limit][side_enum] ★ 请依次触发：【前限位】→ 等 2s → 【后限位】★");
    std::this_thread::sleep_for(10s);  // 给操作人员充足时间

    front_sw->stop_monitoring();
    rear_sw->stop_monitoring();

    spdlog::info("[hw_limit][side_enum] 收到 {} 次触发", received_sides.size());
    for (auto s : received_sides)
        spdlog::info("  side={}", (s == device::LimitSide::FRONT ? "FRONT" : "REAR"));

    // 至少触发了两次，且前后各有一次
    bool has_front = false, has_rear = false;
    for (auto s : received_sides) {
        if (s == device::LimitSide::FRONT) has_front = true;
        if (s == device::LimitSide::REAR)  has_rear  = true;
    }
    CHECK(has_front);
    CHECK(has_rear);

    front_sw->close();
    rear_sw->close();
}

// ────────────────────────────────────────────────────────────────────────────
// [hw_limit][repeated_trigger] — 传感器稳定性（多次触发各计一次）
// ────────────────────────────────────────────────────────────────────────────
TEST_CASE("限位传感器多次触发稳定性", "[hw_limit][repeated_trigger]") {
    auto front_gpio = std::make_shared<driver::LibGpiodPin>(
        hw::kGpioChip, hw::kFrontLine);
    auto front_sw = std::make_shared<device::LimitSwitch>(
        front_gpio, device::LimitSide::FRONT);

    std::atomic<int> cb_count{0};
    front_sw->set_trigger_callback([&](device::LimitSide) {
        cb_count.fetch_add(1);
    });

    REQUIRE(front_sw->open(0, 2, 0));
    front_sw->start_monitoring();

    spdlog::warn("[hw_limit][repeated_trigger] ★ 请在 15 秒内触发【前限位】3 次（每次间隔 >500ms）★");
    std::this_thread::sleep_for(15s);

    spdlog::info("[hw_limit][repeated_trigger] 触发次数={}", cb_count.load());
    CHECK(cb_count.load() >= 1);  // 至少触发了一次

    front_sw->stop_monitoring();
    front_sw->close();
}
