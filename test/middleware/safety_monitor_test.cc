/**
 * SafetyMonitor 中间件单元测试（依赖 MockCanBus + MockGpioPin）
 * [middleware][safety_monitor]
 *
 * 测试策略：
 *   - SafetyMonitor 使用真实 WalkMotorGroup(MockCanBus) 构造
 *   - LimitSwitch 使用真实 LimitSwitch(MockGpioPin) 构造
 *   - 通过 MockGpioPin::simulate_edge() 触发边沿中断路径
 *   - SafetyMonitor::on_limit_trigger() 在 GPIO 监控线程中同步调用
 *     WalkMotorGroup::emergency_override()，验证 MockCanBus::sent_frames
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <thread>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_gpio_pin.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

using robot::device::LimitSide;
using robot::device::LimitSwitch;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::middleware::SafetyMonitor;

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct SafetyMonitorFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> walk{std::make_shared<WalkMotorGroup>(can)};

    std::shared_ptr<MockGpioPin> front_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin> rear_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<LimitSwitch> front_sw{
        std::make_shared<LimitSwitch>(front_pin, LimitSide::FRONT)};
    std::shared_ptr<LimitSwitch> rear_sw{std::make_shared<LimitSwitch>(rear_pin, LimitSide::REAR)};

    EventBus bus;
    SafetyMonitor monitor;

    SafetyMonitorFixture() : monitor(walk, front_sw, rear_sw, bus) {
        can->open_result = true;
        front_pin->open_result = true;
        rear_pin->open_result = true;
        // 不调用 walk->open()，避免后台线程
        front_sw->open();
        rear_sw->open();
    }
};

// ────────────────────────────────────────────────────────────────
// start() / stop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: start() 和 stop() 不崩溃", "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    REQUIRE(f.monitor.start());
    f.monitor.stop();
}

TEST_CASE("SafetyMonitor: 初始状态 is_estop_active() == false", "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    f.monitor.start();
    REQUIRE_FALSE(f.monitor.is_estop_active());
    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// 前端限位触发：立即急停
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: 前端 GPIO 触发后 is_estop_active() == true",
          "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    f.monitor.start();

    // 模拟前端 GPIO 边沿触发（在 GPIO 监控线程路径上调用
    // on_limit_trigger，同步调用 emergency_override）
    if (f.front_pin->registered_cb) {
        f.front_pin->simulate_edge();
    }

    // 急停应立即生效（无 debounce 延迟在 estop_active_ 路径上）
    REQUIRE(f.monitor.is_estop_active());
    f.monitor.stop();
}

TEST_CASE("SafetyMonitor: 尾端 GPIO 触发后 is_estop_active() == true",
          "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    f.monitor.start();

    if (f.rear_pin->registered_cb) {
        f.rear_pin->simulate_edge();
    }
    REQUIRE(f.monitor.is_estop_active());
    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// reset_estop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: reset_estop() 解除急停状态", "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    f.monitor.start();

    if (f.front_pin->registered_cb) {
        f.front_pin->simulate_edge();
    }
    REQUIRE(f.monitor.is_estop_active());

    f.monitor.reset_estop();
    REQUIRE_FALSE(f.monitor.is_estop_active());
    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// LimitSettledEvent 延迟发布（>180ms）
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: 触发后 >200ms 发布 LimitSettledEvent", "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;

    bool settled_front = false;
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::FRONT)
                settled_front = true;
        });

    f.monitor.start();

    if (f.front_pin->registered_cb) {
        f.front_pin->simulate_edge();
    }

    // SafetyMonitor 的 monitor_loop 延迟 180ms 后发布 LimitSettledEvent
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    REQUIRE(settled_front);
    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// 多次触发：只触发一次 LimitSettledEvent（消抖 pending 标志）
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: 短时间内多次触发只发一次 LimitSettledEvent",
          "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    int settled_count = 0;
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::REAR)
                ++settled_count;
        });

    f.monitor.start();

    // 快速触发3次（应只发1次 settled）
    for (int i = 0; i < 3; ++i) {
        if (f.rear_pin->registered_cb)
            f.rear_pin->simulate_edge();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    REQUIRE(settled_count == 1);
    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// Phase G 回归：pending 标志在循环内检查，第二次触发也能发布事件
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: 第二次触发（间隔 >180ms）也能发布 LimitSettledEvent",
          "[middleware][safety_monitor]") {
    // FRONT 触发不置 estop_active_，因此可多次触发
    SafetyMonitorFixture f;
    int settled_count = 0;
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::FRONT) ++settled_count;
        });

    f.monitor.start();

    // 第一次触发
    if (f.front_pin->registered_cb) f.front_pin->simulate_edge();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    REQUIRE(settled_count == 1);

    // 第二次触发（pending 已被 monitor_loop 消费，可再次置位）
    if (f.front_pin->registered_cb) f.front_pin->simulate_edge();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    REQUIRE(settled_count == 2);

    f.monitor.stop();
}

// ────────────────────────────────────────────────────────────────
// 同步路径验证：simulate_edge() 返回后立即有急停 CAN 帧
// ────────────────────────────────────────────────────────────────
TEST_CASE("SafetyMonitor: 限位触发后立即发出急停 CAN 帧（ID=0x032）",
          "[middleware][safety_monitor]") {
    SafetyMonitorFixture f;
    f.can->sent_frames.clear();
    f.monitor.start();

    // on_limit_trigger → emergency_override → can_->send()
    // 整条路径为同步调用，simulate_edge() 返回后帧已在 sent_frames
    if (f.front_pin->registered_cb) f.front_pin->simulate_edge();

    bool has_stop_frame = false;
    for (const auto& frm : f.can->sent_frames)
        if (frm.id == 0x032u) { has_stop_frame = true; break; }
    REQUIRE(has_stop_frame);

    f.monitor.stop();
}
