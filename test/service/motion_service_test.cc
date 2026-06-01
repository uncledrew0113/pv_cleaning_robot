/**
 * @file motion_service_test.cc
 * @brief MotionService 只通过任务段入口验证业务运动语义。
 */
#include <catch2/catch.hpp>

#include <cstring>
#include <memory>
#include <string>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"
#include "pv_cleaning_robot/service/motion_service.h"

using robot::device::BrushMotor;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::MotionService;

namespace {

MotionService::Config make_motion_config() {
    MotionService::Config cfg;
    cfg.clean_speed_rpm = 300.0f;
    cfg.return_speed_rpm = 500.0f;
    cfg.brush_rpm = 1200;
    cfg.heading_pid_en = false;
    return cfg;
}

robot::domain::MissionSegment segment(robot::domain::Endpoint target,
                                      robot::domain::SegmentMode mode) {
    return robot::domain::MissionSegment{target, mode};
}

bool contains_frame(const std::vector<robot::hal::CanFrame>& frames,
                    const robot::hal::CanFrame& expected) {
    for (const auto& frame : frames) {
        if (frame.id != expected.id || frame.len != expected.len ||
            frame.is_ext != expected.is_ext || frame.is_rtr != expected.is_rtr) {
            continue;
        }
        if (std::memcmp(frame.data, expected.data, expected.len) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

struct MotionFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(serial, 0)};
    EventBus bus;
    MotionService motion{group, brush, nullptr, bus, make_motion_config()};

    MotionFixture() {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        serial->open_result = true;
        brush->open();
    }
};

TEST_CASE("MotionService starts cleaning toward opposite endpoint A", "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::A,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();

    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, 210.0f, 210.0f, -210.0f, -210.0f)));
    REQUIRE(f.serial->take_tx_text().find("v 0 -20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService applies primary dock A direction model", "[service][motion]") {
    MotionFixture f;
    f.motion.set_primary_dock_query([] { return robot::domain::Endpoint::A; });
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::B,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();

    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, -210.0f, -210.0f, 210.0f, 210.0f)));
    REQUIRE(f.serial->take_tx_text().find("v 0 20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService starts cleaning toward primary dock endpoint", "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::B,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();

    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, -210.0f, -210.0f, 210.0f, 210.0f)));
    REQUIRE(f.serial->take_tx_text().find("v 0 20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService syncs runtime config before segment start", "[service][motion]") {
    MotionFixture f;
    f.motion.set_runtime_config_query([] {
        robot::service::RuntimeConfig cfg;
        cfg.clean_speed_rpm = -360.0;
        cfg.return_speed_rpm = -420.0;
        cfg.brush_rpm = -1500;
        cfg.primary_dock = robot::domain::Endpoint::B;
        return cfg;
    });
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::A,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();

    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(-210.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 -25.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService stop_cleaning stops brush and walk motors", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::A,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();
    f.serial->clear_tx();

    f.motion.stop_cleaning();

    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(0.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService emergency_stop clears walk targets and brush", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_segment(segment(robot::domain::Endpoint::A,
                                           robot::domain::SegmentMode::Cleaning)));
    f.motion.update();
    f.serial->clear_tx();

    f.motion.emergency_stop();

    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(0.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(0.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService reports segment start failure on CAN send error", "[service][motion]") {
    MotionFixture f;
    f.can->send_result = false;

    REQUIRE_FALSE(f.motion.start_segment(segment(robot::domain::Endpoint::A,
                                                robot::domain::SegmentMode::Cleaning)));
}
