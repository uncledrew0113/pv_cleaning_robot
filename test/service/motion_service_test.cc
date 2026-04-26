/**
 * MotionService 单元测试（依赖 MockCanBus + MockSerialPort）
 * [service][motion]
 */
#include <catch2/catch.hpp>

#include <string>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/motion_service.h"

using robot::device::BrushMotor;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::MotionService;

struct MotionFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(serial, 0, 8192.0f, true, 0.5f)};
    EventBus bus;
    MotionService::Config cfg{
        .clean_speed_rpm = 300.0f,
        .return_speed_rpm = 500.0f,
        .brush_rpm = 1200,
        .heading_pid_en = false
    };
    MotionService motion;

    MotionFixture() : motion(group, brush, nullptr, bus, cfg) {
        can->open_result = true;
        can->send_result = true;
        serial->open_result = true;
        brush->open();
    }
};

TEST_CASE("MotionService start_cleaning emits CAN frames", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService start_cleaning switches brush to speed mode and sets rpm",
          "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();

    f.motion.start_cleaning();

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("w axis0.controller.config.control_mode 2\n") != std::string::npos);
    REQUIRE(tx.find("v 0 163840.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService stop_cleaning commands brush stop", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.serial->clear_tx();

    f.motion.stop_cleaning();

    REQUIRE(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_returning reverses brush direction", "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();

    REQUIRE(f.motion.start_returning());

    REQUIRE(f.serial->take_tx_text().find("v 0 -163840.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_returning_no_brush stops brush before reversing walk motors",
          "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    REQUIRE(f.motion.start_returning_no_brush());

    REQUIRE(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService emergency_stop idles brush", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.serial->clear_tx();
    f.can->sent_frames.clear();

    f.motion.emergency_stop();

    REQUIRE(f.serial->take_tx_text().find("w axis0.requested_state 1\n") != std::string::npos);
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService CAN send failure causes start_cleaning failure", "[service][motion]") {
    MotionFixture f;
    f.can->send_result = false;
    REQUIRE_FALSE(f.motion.start_cleaning());
}
