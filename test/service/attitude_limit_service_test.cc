#include <catch2/catch.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "../mock/mock_gpio_pin.h"
#include "pv_cleaning_robot/device/attitude_limit_switch.h"
#include "pv_cleaning_robot/service/attitude_limit_service.h"

using robot::device::AttitudeLimitSide;
using robot::device::AttitudeLimitSwitch;
using robot::service::AttitudeLimitService;
using namespace std::chrono_literals;

namespace {

struct Fixture {
    std::shared_ptr<MockGpioPin> left_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin> right_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<AttitudeLimitSwitch> left{
        std::make_shared<AttitudeLimitSwitch>(left_pin, AttitudeLimitSide::LEFT_LOWER)};
    std::shared_ptr<AttitudeLimitSwitch> right{
        std::make_shared<AttitudeLimitSwitch>(right_pin, AttitudeLimitSide::RIGHT_LOWER)};
    int emergency_stop_count{0};
    int prepare_count{0};
    int stop_count{0};
    bool command_lower_wheels_ok{true};
    std::vector<float> lower_commands;
    AttitudeLimitService::MotionPorts ports{
        [&] { ++emergency_stop_count; },
        [&] {
            ++prepare_count;
            return true;
        },
        [&](float rpm) {
            lower_commands.push_back(rpm);
            return command_lower_wheels_ok;
        },
        [&] {
            ++stop_count;
            return true;
        },
    };
    AttitudeLimitService service{left, right, ports};

    Fixture() {
        left_pin->opened = true;
        right_pin->opened = true;
    }
};

}  // namespace

TEST_CASE("AttitudeLimitService records single attitude limit event",
          "[service][attitude_limit]") {
    Fixture f;
    f.service.start_monitoring();

    f.left_pin->read_result = false;
    f.right_pin->read_result = true;
    f.left_pin->simulate_edge();

    auto event = f.service.consume_pending_event();
    REQUIRE(event.has_value());
    CHECK(event->type == AttitudeLimitService::EventType::AttitudeLimit);
    CHECK(event->side == AttitudeLimitSide::LEFT_LOWER);
    CHECK(f.emergency_stop_count == 1);
    CHECK_FALSE(f.service.consume_pending_event().has_value());
}

TEST_CASE("AttitudeLimitService records both attitude limits as hard event",
          "[service][attitude_limit]") {
    Fixture f;
    f.service.start_monitoring();

    f.left_pin->read_result = false;
    f.right_pin->read_result = false;
    f.right_pin->simulate_edge();

    auto event = f.service.consume_pending_event();
    REQUIRE(event.has_value());
    CHECK(event->type == AttitudeLimitService::EventType::AttitudeLimitBoth);
    CHECK(f.emergency_stop_count == 1);
}

TEST_CASE("AttitudeLimitService lower attitude center delegates motion to ports",
          "[service][attitude_limit]") {
    Fixture f;
    f.left_pin->read_result = false;
    f.right_pin->read_result = true;

    auto worker = std::thread([&] {
        std::this_thread::sleep_for(25ms);
        f.left_pin->read_result = true;
        std::this_thread::sleep_for(25ms);
        f.right_pin->read_result = false;
        f.right_pin->simulate_edge();
    });

    const auto result = f.service.lower_attitude_center(
        AttitudeLimitService::CenterConfig{
            2.0f,
            1,
            200ms,
            20ms,
        });
    worker.join();

    REQUIRE(result.outcome == AttitudeLimitService::CenterOutcome::Completed);
    CHECK(f.prepare_count == 1);
    CHECK(f.stop_count == 1);
    REQUIRE_FALSE(f.lower_commands.empty());
    CHECK(std::find(f.lower_commands.begin(), f.lower_commands.end(), 2.0f) !=
          f.lower_commands.end());
    CHECK(f.lower_commands.back() == Approx(0.0f));
}

TEST_CASE("AttitudeLimitService suppresses single limit event while centering",
          "[service][attitude_limit]") {
    Fixture f;
    f.service.start_monitoring();
    f.left_pin->read_result = false;
    f.right_pin->read_result = true;

    auto worker = std::thread([&] {
        std::this_thread::sleep_for(25ms);
        f.left_pin->read_result = true;
        std::this_thread::sleep_for(25ms);
        f.right_pin->read_result = false;
        f.right_pin->simulate_edge();
    });

    const auto result = f.service.lower_attitude_center(
        AttitudeLimitService::CenterConfig{
            2.0f,
            1,
            200ms,
            20ms,
        });
    worker.join();

    REQUIRE(result.outcome == AttitudeLimitService::CenterOutcome::Completed);
    CHECK(f.emergency_stop_count == 1);
    CHECK_FALSE(f.service.consume_pending_event().has_value());
}

TEST_CASE("AttitudeLimitService lower attitude center timeout is non-fatal strategy outcome",
          "[service][attitude_limit]") {
    Fixture f;
    f.left_pin->read_result = false;
    f.right_pin->read_result = true;

    const auto started = std::chrono::steady_clock::now();
    const auto result = f.service.lower_attitude_center(
        AttitudeLimitService::CenterConfig{2.0f, 1, 60ms, 10ms});
    const auto elapsed = std::chrono::steady_clock::now() - started;

    CHECK(result.outcome == AttitudeLimitService::CenterOutcome::TimedOut);
    CHECK(elapsed >= 50ms);
    CHECK(f.prepare_count == 1);
    CHECK(f.stop_count >= 1);
    CHECK(f.emergency_stop_count == 1);
}

TEST_CASE("AttitudeLimitService lower attitude center command failure waits out timeout",
          "[service][attitude_limit]") {
    Fixture f;
    f.left_pin->read_result = false;
    f.right_pin->read_result = true;
    f.command_lower_wheels_ok = false;

    const auto started = std::chrono::steady_clock::now();
    const auto result = f.service.lower_attitude_center(
        AttitudeLimitService::CenterConfig{2.0f, 1, 60ms, 10ms});
    const auto elapsed = std::chrono::steady_clock::now() - started;

    CHECK(result.outcome == AttitudeLimitService::CenterOutcome::TimedOut);
    CHECK(elapsed >= 50ms);
    CHECK(f.prepare_count == 1);
    CHECK(f.emergency_stop_count == 1);
}

TEST_CASE("AttitudeLimitService lower attitude center can start from no active side",
          "[service][attitude_limit]") {
    Fixture f;
    f.service.start_monitoring();
    f.left_pin->read_result = true;
    f.right_pin->read_result = true;

    auto worker = std::thread([&] {
        std::this_thread::sleep_for(25ms);
        f.right_pin->read_result = false;
        f.right_pin->simulate_edge();
        std::this_thread::sleep_for(25ms);
        f.right_pin->read_result = true;
        f.left_pin->read_result = false;
        f.left_pin->simulate_edge();
    });

    const auto result = f.service.lower_attitude_center(
        AttitudeLimitService::CenterConfig{2.0f, 1, 200ms, 20ms});
    worker.join();

    REQUIRE(result.outcome == AttitudeLimitService::CenterOutcome::Completed);
    CHECK(f.emergency_stop_count == 2);
    CHECK_FALSE(f.service.consume_pending_event().has_value());
    CHECK(std::find(f.lower_commands.begin(), f.lower_commands.end(), -2.0f) !=
          f.lower_commands.end());
    CHECK(std::find(f.lower_commands.begin(), f.lower_commands.end(), 2.0f) !=
          f.lower_commands.end());
}
