#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

using robot::service::ChargingSide;
using robot::service::ParkingPolicy;
using robot::service::TerminalSide;
using robot::service::allows_half_pass;
using robot::service::can_start_from_terminal;
using robot::service::supports_charging_at;

TEST_CASE("Terminal semantics: half-pass requires both parking policy",
          "[app][terminal]") {
    CHECK_FALSE(allows_half_pass(ParkingPolicy::TerminalAOnly));
    CHECK_FALSE(allows_half_pass(ParkingPolicy::TerminalBOnly));
    CHECK(allows_half_pass(ParkingPolicy::Both));
}

TEST_CASE("Terminal semantics: start admission respects parking policy",
          "[app][terminal]") {
    CHECK(can_start_from_terminal(ParkingPolicy::TerminalAOnly, TerminalSide::A));
    CHECK_FALSE(can_start_from_terminal(ParkingPolicy::TerminalAOnly, TerminalSide::B));
    CHECK(can_start_from_terminal(ParkingPolicy::Both, TerminalSide::A));
    CHECK(can_start_from_terminal(ParkingPolicy::Both, TerminalSide::B));
}

TEST_CASE("Terminal semantics: charging support is independent from parking policy",
          "[app][terminal]") {
    CHECK(supports_charging_at(ChargingSide::TerminalA, TerminalSide::A));
    CHECK_FALSE(supports_charging_at(ChargingSide::TerminalA, TerminalSide::B));
    CHECK(supports_charging_at(ChargingSide::Both, TerminalSide::A));
    CHECK(supports_charging_at(ChargingSide::Both, TerminalSide::B));
}
