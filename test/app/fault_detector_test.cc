#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/fault_detector.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::FaultDetector;

TEST_CASE("FaultDetector emits conflicting limit fault", "[app][fault_detector]") {
    FaultDetector detector;
    FaultDetector::Input input;
    input.left_limit_active = true;
    input.right_limit_active = true;

    auto facts = detector.detect(input);

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kConflictingLimitSides);
}

TEST_CASE("FaultDetector emits spin-free fault only while executing", "[app][fault_detector]") {
    FaultDetector detector;
    FaultDetector::Input input;
    input.executing_mission = true;
    input.spin_free_detected = true;

    auto facts = detector.detect(input);

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kWheelSpinFree);

    input.executing_mission = false;
    facts = detector.detect(input);
    REQUIRE(facts.empty());
}

TEST_CASE("FaultDetector emits transient attitude fault for recoverable tilt",
          "[app][fault_detector]") {
    FaultDetector detector;
    FaultDetector::Input input;
    input.executing_mission = true;
    input.imu_fresh = true;
    input.attitude_out_of_range = true;

    auto facts = detector.detect(input);

    REQUIRE(facts.size() == 1);
    REQUIRE(facts[0].code == robot::domain::FaultCode::kTransientAttitudeError);
}
