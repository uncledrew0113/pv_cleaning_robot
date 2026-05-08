#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/parking_side_runtime.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

using robot::service::ParkingSide;

TEST_CASE("Parking side runtime maps left parking side to left_limit",
          "[app][parking_side]") {
    const auto facts = robot::app::ParkingSideRuntime::from_physical_limits(
        ParkingSide::Left,
        true,
        false);

    CHECK(facts.at_parking_side);
    CHECK_FALSE(facts.at_far_end);
}

TEST_CASE("Parking side runtime maps right parking side to right_limit",
          "[app][parking_side]") {
    const auto facts = robot::app::ParkingSideRuntime::from_physical_limits(
        ParkingSide::Right,
        false,
        true);

    CHECK(facts.at_parking_side);
    CHECK_FALSE(facts.at_far_end);
}
