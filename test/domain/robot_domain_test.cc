#include <catch2/catch.hpp>

#include "pv_cleaning_robot/domain/robot_domain.h"

using namespace robot::domain;

TEST_CASE("Domain position is estimated from endpoint A/B limit snapshot", "[domain]") {
    CHECK(estimate_position(LimitState{true, false}) == PositionState::AtA);
    CHECK(estimate_position(LimitState{false, true}) == PositionState::AtB);
    CHECK(estimate_position(LimitState{false, false}) == PositionState::OnSegment);
    CHECK(estimate_position(LimitState{true, true}) == PositionState::Inconsistent);
}

TEST_CASE("Domain position gates configured mission start", "[domain]") {
    CHECK(can_start_configured_mission(LaneConfig{DockMode::SingleDock, Endpoint::A},
                                       PositionState::AtA));
    CHECK_FALSE(can_start_configured_mission(LaneConfig{DockMode::SingleDock, Endpoint::A},
                                             PositionState::AtB));
    CHECK(can_start_configured_mission(LaneConfig{DockMode::DualDock, Endpoint::A},
                                       PositionState::AtB));
    CHECK_FALSE(can_start_configured_mission(LaneConfig{DockMode::DualDock, Endpoint::A},
                                             PositionState::OnSegment));
    CHECK_FALSE(can_start_configured_mission(LaneConfig{DockMode::DualDock, Endpoint::A},
                                             PositionState::Inconsistent));
}

TEST_CASE("Domain target uses physical endpoint A/B directly", "[domain]") {
    CHECK(opposite_endpoint(Endpoint::A) == Endpoint::B);
    CHECK(opposite_endpoint(Endpoint::B) == Endpoint::A);
    CHECK(is_at_target(PositionState::AtA, Endpoint::A));
    CHECK(is_at_target(PositionState::AtB, Endpoint::B));
    CHECK_FALSE(is_at_target(PositionState::OnSegment, Endpoint::B));
}

TEST_CASE("Domain directional missions build single cleaning segment", "[domain]") {
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    const auto to_opposite = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-1");
    REQUIRE(to_opposite.segments.size() == 1);
    CHECK(to_opposite.kind == MissionKind::CleanTowardOppositeEndpoint);
    CHECK(to_opposite.source == CommandSource::Rpc);
    CHECK(to_opposite.command_id == "rpc-1");
    CHECK(to_opposite.segments[0].target == Endpoint::B);
    CHECK(to_opposite.segments[0].mode == SegmentMode::Cleaning);

    const auto to_primary = build_directional_clean_context(
        MissionKind::CleanTowardPrimaryDock, lane, CommandSource::Rpc, "rpc-2");
    REQUIRE(to_primary.segments.size() == 1);
    CHECK(to_primary.segments[0].target == Endpoint::A);
    CHECK(to_primary.segments[0].mode == SegmentMode::Cleaning);
}

TEST_CASE("Domain configured mission builds single dock round trip", "[domain]") {
    LaneConfig lane{DockMode::SingleDock, Endpoint::A};

    const auto ctx = build_configured_mission_context(
        lane, PositionState::AtA, CommandSource::Scheduler, "schedule-1", 2);

    REQUIRE(ctx.segments.size() == 2);
    CHECK(ctx.kind == MissionKind::ConfiguredMission);
    CHECK(ctx.repeat_count == 2);
    CHECK(ctx.completed_cycles == 0);
    CHECK(ctx.segments[0].target == Endpoint::B);
    CHECK(ctx.segments[0].mode == SegmentMode::Cleaning);
    CHECK(ctx.segments[1].target == Endpoint::A);
    CHECK(ctx.segments[1].mode == SegmentMode::Cleaning);
}

TEST_CASE("Domain configured mission builds dual dock transfer", "[domain]") {
    LaneConfig lane{DockMode::DualDock, Endpoint::A};

    const auto ctx = build_configured_mission_context(
        lane, PositionState::AtB, CommandSource::Rpc, "rpc-3", 1);

    REQUIRE(ctx.segments.size() == 1);
    CHECK(ctx.segments[0].target == Endpoint::A);
    CHECK(ctx.segments[0].mode == SegmentMode::Cleaning);
}
