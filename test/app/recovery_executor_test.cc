#include <catch2/catch.hpp>

#include <string>
#include <vector>

#include "pv_cleaning_robot/app/recovery_executor.h"

using namespace robot::app;

namespace {

RecoveryRequest request(RecoveryPlanId plan, ComponentKind kind) {
    return RecoveryRequest{plan, ComponentId{kind, 0}};
}

void expect_prefix(const std::vector<std::string>& calls,
                   std::initializer_list<const char*> expected) {
    REQUIRE(calls.size() >= expected.size());
    size_t i = 0;
    for (const auto* item : expected) {
        CHECK(calls[i++] == item);
    }
}

}  // namespace

TEST_CASE("RecoveryExecutor pauses GPS stuck before brush recovery",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_brush = [&] {
        calls.push_back("stop_brush");
        return true;
    };
    ports.stop_brush_executor = [&] {
        calls.push_back("stop_brush_executor");
        return true;
    };
    ports.restart_brush_driver = [&] {
        calls.push_back("restart_brush_driver");
        return true;
    };
    ports.start_brush_executor = [&] {
        calls.push_back("start_brush_executor");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(RecoveryRequest{
        RecoveryPlanId::RecoverBrushMotor,
        ComponentId{ComponentKind::BrushMotor, 0}});

    REQUIRE(result.ok);
    REQUIRE(calls.size() == 7);
    CHECK(calls[0] == "pause_gps_stuck");
    CHECK(calls[1] == "stop_walk");
    CHECK(calls[2] == "stop_brush");
    CHECK(calls[3] == "stop_brush_executor");
    CHECK(calls[4] == "restart_brush_driver");
    CHECK(calls[5] == "start_brush_executor");
    CHECK(calls[6] == "resume_gps_stuck");
}

TEST_CASE("RecoveryExecutor returns failure when executor stop fails",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_brush = [] { return true; };
    ports.stop_brush_executor = [] { return false; };
    ports.restart_brush_driver = [] { return true; };
    ports.start_brush_executor = [] { return true; };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(RecoveryRequest{
        RecoveryPlanId::RecoverBrushMotor,
        ComponentId{ComponentKind::BrushMotor, 0}});

    CHECK_FALSE(result.ok);
    CHECK(calls.size() == 3);
    CHECK(calls[0] == "pause_gps_stuck");
    CHECK(calls[1] == "stop_walk");
    CHECK(calls[2] == "resume_gps_stuck");
}

TEST_CASE("RecoveryExecutor recovers walk motor group in driver restart order",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_walk_executor = [&] {
        calls.push_back("stop_walk_executor");
        return true;
    };
    ports.restart_walk_driver = [&] {
        calls.push_back("restart_walk_driver");
        return true;
    };
    ports.start_walk_executor = [&] {
        calls.push_back("start_walk_executor");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(
        request(RecoveryPlanId::RecoverWalkMotorGroup, ComponentKind::WalkMotorGroup));

    REQUIRE(result.ok);
    expect_prefix(calls,
                  {"pause_gps_stuck",
                   "stop_walk",
                   "stop_walk_executor",
                   "restart_walk_driver",
                   "start_walk_executor"});
}

TEST_CASE("RecoveryExecutor recovers BMS through executor restart order",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_bms_executor = [&] {
        calls.push_back("stop_bms_executor");
        return true;
    };
    ports.restart_bms_driver = [&] {
        calls.push_back("restart_bms_driver");
        return true;
    };
    ports.start_bms_executor = [&] {
        calls.push_back("start_bms_executor");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(request(RecoveryPlanId::RecoverBms, ComponentKind::Bms));

    REQUIRE(result.ok);
    expect_prefix(calls,
                  {"pause_gps_stuck",
                   "stop_walk",
                   "stop_bms_executor",
                   "restart_bms_driver",
                   "start_bms_executor"});
}

TEST_CASE("RecoveryExecutor recovers GPS through executor restart order",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_gps_executor = [&] {
        calls.push_back("stop_gps_executor");
        return true;
    };
    ports.restart_gps_driver = [&] {
        calls.push_back("restart_gps_driver");
        return true;
    };
    ports.start_gps_executor = [&] {
        calls.push_back("start_gps_executor");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(request(RecoveryPlanId::RecoverGps, ComponentKind::Gps));

    REQUIRE(result.ok);
    expect_prefix(calls,
                  {"pause_gps_stuck",
                   "stop_walk",
                   "stop_gps_executor",
                   "restart_gps_driver",
                   "start_gps_executor"});
}

TEST_CASE("RecoveryExecutor recovers IMU through executor restart order",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.stop_imu_executor = [&] {
        calls.push_back("stop_imu_executor");
        return true;
    };
    ports.restart_imu_driver = [&] {
        calls.push_back("restart_imu_driver");
        return true;
    };
    ports.start_imu_executor = [&] {
        calls.push_back("start_imu_executor");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result = executor.execute(request(RecoveryPlanId::RecoverImu, ComponentKind::Imu));

    REQUIRE(result.ok);
    expect_prefix(calls,
                  {"pause_gps_stuck",
                   "stop_walk",
                   "stop_imu_executor",
                   "restart_imu_driver",
                   "start_imu_executor"});
}

TEST_CASE("RecoveryExecutor walk stall reverse runs recovery motion directly",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.reverse_walk_motion = [&] {
        calls.push_back("reverse_walk_motion");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result =
        executor.execute(request(RecoveryPlanId::RecoverWalkStall, ComponentKind::WalkMotorGroup));

    REQUIRE(result.ok);
    expect_prefix(calls, {"pause_gps_stuck", "stop_walk", "reverse_walk_motion", "resume_gps_stuck"});
}

TEST_CASE("RecoveryExecutor GPS stuck reverse uses same reverse recovery path",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.reverse_walk_motion = [&] {
        calls.push_back("reverse_walk_motion");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result =
        executor.execute(request(RecoveryPlanId::RecoverGpsStuckReverse,
                                 ComponentKind::GpsStuckService));

    REQUIRE(result.ok);
    expect_prefix(calls, {"pause_gps_stuck", "stop_walk", "reverse_walk_motion", "resume_gps_stuck"});
}

TEST_CASE("RecoveryExecutor attitude center runs center motion directly",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.lower_attitude_center = [&] {
        calls.push_back("lower_attitude_center");
        return true;
    };

    RecoveryExecutor executor(ports);
    auto result =
        executor.execute(request(RecoveryPlanId::RecoverAttitudeCenter,
                                 ComponentKind::AttitudeLimitSwitch));

    REQUIRE(result.ok);
    expect_prefix(calls, {"pause_gps_stuck", "stop_walk", "lower_attitude_center", "resume_gps_stuck"});
}
