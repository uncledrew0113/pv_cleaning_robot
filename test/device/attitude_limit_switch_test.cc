/**
 * AttitudeLimitSwitch 设备层单元测试（依赖 MockGpioPin）
 * [device][attitude_limit_switch]
 */
#include <catch2/catch.hpp>

#include <sstream>
#include <vector>

#include "../mock/mock_gpio_pin.h"
#include "pv_cleaning_robot/device/attitude_limit_switch.h"

using robot::device::AttitudeLimitSide;
using robot::device::AttitudeLimitSwitch;

struct AttitudeLimitFixture {
    std::shared_ptr<MockGpioPin> pin{std::make_shared<MockGpioPin>()};
    AttitudeLimitSwitch sw;

    explicit AttitudeLimitFixture(AttitudeLimitSide side = AttitudeLimitSide::LEFT_LOWER)
        : sw(pin, side) {
        pin->open_result = true;
    }
};

TEST_CASE("AttitudeLimitSwitch: open() 成功并使用低有效输入配置",
          "[device][attitude_limit_switch]") {
    AttitudeLimitFixture f;

    REQUIRE(f.sw.open());
    CHECK(f.pin->last_open_config.direction == robot::hal::GpioDirection::INPUT);
    CHECK(f.pin->last_open_config.bias == robot::hal::GpioBias::PULL_UP);
    CHECK(f.pin->last_open_config.debounce_ms == 2);
}

TEST_CASE("AttitudeLimitSwitch: side() 返回构造时传入的姿态限位侧",
          "[device][attitude_limit_switch]") {
    AttitudeLimitFixture left(AttitudeLimitSide::LEFT_LOWER);
    AttitudeLimitFixture right(AttitudeLimitSide::RIGHT_LOWER);

    CHECK(left.sw.side() == AttitudeLimitSide::LEFT_LOWER);
    CHECK(right.sw.side() == AttitudeLimitSide::RIGHT_LOWER);
}

TEST_CASE("AttitudeLimitSwitch: GPIO 低边沿触发后置位并回调侧别",
          "[device][attitude_limit_switch]") {
    AttitudeLimitFixture f(AttitudeLimitSide::RIGHT_LOWER);
    REQUIRE(f.sw.open());

    bool cb_fired = false;
    AttitudeLimitSide cb_side{AttitudeLimitSide::LEFT_LOWER};
    f.sw.set_trigger_callback([&](AttitudeLimitSide side) {
        cb_fired = true;
        cb_side = side;
    });

    f.sw.start_monitoring();
    REQUIRE(f.pin->registered_edge == robot::hal::GpioEdge::FALLING);
    f.pin->simulate_edge();

    CHECK(f.sw.is_triggered());
    CHECK(cb_fired);
    CHECK(cb_side == AttitudeLimitSide::RIGHT_LOWER);

    f.sw.clear_trigger();
    CHECK_FALSE(f.sw.is_triggered());
}

TEST_CASE("AttitudeLimitSwitch: 连续测量输出当前电平和触发状态",
          "[device][attitude_limit_switch]") {
    AttitudeLimitFixture f;
    REQUIRE(f.sw.open());

    const std::vector<bool> levels{true, true, false, false, true};
    std::ostringstream state_line;

    for (bool level : levels) {
        f.pin->read_result = level;
        const auto status = f.sw.read_status();
        state_line << "[level=" << (status.level_high ? "high" : "low")
                   << ", active=" << (status.active_low_asserted ? "true" : "false")
                   << ", triggered=" << (status.triggered ? "true" : "false") << "]";
        CHECK(status.side == AttitudeLimitSide::LEFT_LOWER);
        CHECK(status.level_high == level);
        CHECK(status.active_low_asserted == !level);
    }

    WARN("AttitudeLimitSwitch continuous states: " << state_line.str());
}
