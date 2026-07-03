#include <catch2/catch.hpp>

#include <string>
#include <vector>

#include "pv_cleaning_robot/app/recovery_executor.h"

using namespace robot::app;

namespace {

RecoveryRequest request(RecoveryPlanId plan) {
    return RecoveryRequest{plan, ComponentId{ComponentKind::AttitudeLimitSwitch, 0}};
}

void expect_calls(const std::vector<std::string>& calls,
                  std::initializer_list<const char*> expected) {
    REQUIRE(calls.size() == expected.size());
    size_t i = 0;
    for (const auto* item : expected) {
        CHECK(calls[i++] == item);
    }
}

RecoveryExecutor::Ports base_ports(std::vector<std::string>& calls) {
    RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&] { calls.push_back("pause_gps_stuck"); };
    ports.resume_gps_stuck = [&] { calls.push_back("resume_gps_stuck"); };
    ports.stop_walk = [&] {
        calls.push_back("stop_walk");
        return true;
    };
    ports.lower_attitude_center = [&] {
        calls.push_back("lower_attitude_center");
        return RecoveryStepResult{RecoveryStepOutcome::Completed};
    };
    ports.reverse_walk_motion = [&] {
        calls.push_back("reverse_walk_motion");
        return true;
    };
    return ports;
}

}  // namespace

TEST_CASE("RecoveryExecutor attitude center runs center motion directly",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor executor(base_ports(calls));

    auto result = executor.execute(request(RecoveryPlanId::RecoverAttitudeCenter));

    REQUIRE(result.ok);
    expect_calls(calls,
                 {"pause_gps_stuck",
                  "stop_walk",
                  "lower_attitude_center",
                  "resume_gps_stuck"});
}

TEST_CASE("RecoveryExecutor attitude center then reverse runs combined motion sequence",
          "[app][recovery_executor]") {
    std::vector<std::string> calls;
    RecoveryExecutor executor(base_ports(calls));

    auto result = executor.execute(request(RecoveryPlanId::RecoverAttitudeCenterThenReverse));

    REQUIRE(result.ok);
    expect_calls(calls,
                 {"pause_gps_stuck",
                  "stop_walk",
                  "lower_attitude_center",
                  "reverse_walk_motion",
                  "stop_walk",
                  "resume_gps_stuck"});
}
