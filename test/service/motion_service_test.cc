/**
 * MotionService 单元测试（依赖 MockCanBus + MockSerialPort）
 * [service][motion]
 */
#include <catch2/catch.hpp>

#include <cstring>
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

}  // namespace

struct MotionFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(serial, 0)};
    EventBus bus;
    MotionService::Config cfg{make_motion_config()};
    MotionService motion;

    MotionFixture() : motion(group, brush, nullptr, bus, cfg) {
        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        serial->open_result = true;
        brush->open();
    }
};

void require_same_frame(const robot::hal::CanFrame& actual, const robot::hal::CanFrame& expected) {
    REQUIRE(actual.id == expected.id);
    REQUIRE(actual.len == expected.len);
    REQUIRE(actual.is_ext == expected.is_ext);
    REQUIRE(actual.is_rtr == expected.is_rtr);
    REQUIRE(std::memcmp(actual.data, expected.data, expected.len) == 0);
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

TEST_CASE("MotionService start_cleaning emits CAN frames", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService start_cleaning keeps right parking side as motion baseline",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Right; });

    REQUIRE(f.motion.start_cleaning());
    f.motion.update();
    REQUIRE_FALSE(f.can->sent_frames.empty());
    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, 210.0f, 210.0f, -210.0f, -210.0f)));
}

TEST_CASE("MotionService start_cleaning flips walk direction for left parking side",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Left; });

    REQUIRE(f.motion.start_cleaning());
    f.motion.update();
    REQUIRE_FALSE(f.can->sent_frames.empty());
    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, -210.0f, -210.0f, 210.0f, 210.0f)));
}

TEST_CASE("MotionService start_cleaning commands brush rpm with current direction model",
          "[service][motion]") {
    MotionFixture f;
    f.serial->clear_tx();

    f.motion.start_cleaning();

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("v 0 -20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_cleaning flips brush direction for left parking side",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Left; });
    f.serial->clear_tx();

    f.motion.start_cleaning();

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("v 0 20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_cleaning normalizes signed runtime brush rpm to absolute value",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Right; });
    f.motion.set_runtime_config_query([] {
        robot::service::RuntimeConfig cfg;
        cfg.clean_speed_rpm = 300.0;
        cfg.return_speed_rpm = 300.0;
        cfg.brush_rpm = -1200;
        cfg.parking_side = robot::service::ParkingSide::Right;
        return cfg;
    });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_cleaning());

    const auto tx = f.serial->take_tx_text();
    REQUIRE(tx.find("v 0 -20.000 0\n") != std::string::npos);
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

    REQUIRE(f.serial->take_tx_text().find("v 0 20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_returning flips walk and brush direction for left parking side",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Left; });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_returning());
    f.motion.update();

    REQUIRE_FALSE(f.can->sent_frames.empty());
    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, 210.0f, 210.0f, -210.0f, -210.0f)));
    REQUIRE(f.serial->take_tx_text().find("v 0 -20.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_cleaning syncs runtime config before sending commands",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_runtime_config_query([] {
        robot::service::RuntimeConfig cfg;
        cfg.clean_speed_rpm = 360.0;
        cfg.return_speed_rpm = 420.0;
        cfg.brush_rpm = 1500;
        cfg.parking_side = robot::service::ParkingSide::Right;
        return cfg;
    });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_cleaning());
    f.motion.update();
    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(-210.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 -25.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService start_returning syncs runtime brush rpm before sending commands",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_runtime_config_query([] {
        robot::service::RuntimeConfig cfg;
        cfg.clean_speed_rpm = 300.0;
        cfg.return_speed_rpm = 420.0;
        cfg.brush_rpm = 1500;
        cfg.parking_side = robot::service::ParkingSide::Right;
        return cfg;
    });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_returning());
    f.motion.update();
    const auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(210.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 25.000 0\n") != std::string::npos);
}

TEST_CASE("MotionService runtime speed config uses absolute values for wheel direction model",
          "[service][motion]") {
    MotionFixture f;
    f.motion.set_parking_side_query([] { return robot::service::ParkingSide::Left; });
    f.motion.set_runtime_config_query([] {
        robot::service::RuntimeConfig cfg;
        cfg.clean_speed_rpm = -360.0;
        cfg.return_speed_rpm = -420.0;
        cfg.brush_rpm = -1500;
        cfg.parking_side = robot::service::ParkingSide::Left;
        return cfg;
    });
    f.serial->clear_tx();

    REQUIRE(f.motion.start_cleaning());
    f.motion.update();
    auto diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(210.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 25.000 0\n") != std::string::npos);

    f.serial->clear_tx();
    REQUIRE(f.motion.start_returning());
    f.motion.update();
    diag = f.group->get_group_diagnostics();
    REQUIRE(diag.wheel[0].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[1].target_value == Approx(210.0f));
    REQUIRE(diag.wheel[2].target_value == Approx(-210.0f));
    REQUIRE(diag.wheel[3].target_value == Approx(-210.0f));
    REQUIRE(f.serial->take_tx_text().find("v 0 -25.000 0\n") != std::string::npos);
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

    REQUIRE(f.serial->take_tx_text().find("v 0 0.000 0\n") != std::string::npos);
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService emergency_stop clears walk target values to zero", "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    f.motion.update();

    auto before = f.group->get_group_diagnostics();
    REQUIRE(before.wheel[0].target_value != Approx(0.0f));
    REQUIRE(before.wheel[1].target_value != Approx(0.0f));
    REQUIRE(before.wheel[2].target_value != Approx(0.0f));
    REQUIRE(before.wheel[3].target_value != Approx(0.0f));

    f.motion.emergency_stop();

    const auto after = f.group->get_group_diagnostics();
    REQUIRE(after.wheel[0].target_value == Approx(0.0f));
    REQUIRE(after.wheel[1].target_value == Approx(0.0f));
    REQUIRE(after.wheel[2].target_value == Approx(0.0f));
    REQUIRE(after.wheel[3].target_value == Approx(0.0f));
}

TEST_CASE("MotionService clear_override 生效后重置纠偏器并恢复基础速度槽位",
          "[service][motion]") {
    MotionFixture f;
    REQUIRE(f.motion.start_cleaning());
    f.motion.update();

    f.can->sent_frames.clear();
    REQUIRE(f.group->emergency_override(0.0f) == robot::device::DeviceError::OK);
    f.can->sent_frames.clear();

    f.group->clear_override();
    f.motion.update();
    REQUIRE(f.can->sent_frames.empty());

    f.motion.update();
    REQUIRE(contains_frame(
        f.can->sent_frames,
        robot::protocol::WalkMotorCanCodec::encode_group_speed(
            1u, 210.0f, 210.0f, -210.0f, -210.0f)));
}

TEST_CASE("MotionService CAN send failure causes start_cleaning failure", "[service][motion]") {
    MotionFixture f;
    f.can->send_result = false;
    REQUIRE_FALSE(f.motion.start_cleaning());
}
