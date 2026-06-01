#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/robot_fsm.h"

using namespace robot::app;
using namespace robot::domain;

TEST_CASE("RobotFsm accepts mission then starts first segment after self-check", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-1");

    const auto start = fsm.start(mission);
    REQUIRE(start.accepted);
    CHECK(fsm.robot_state() == RobotState::SelfChecking);

    const auto ok = fsm.complete_self_check(true, false, {});
    REQUIRE(ok.accepted);
    CHECK(fsm.robot_state() == RobotState::ExecutingMission);
    REQUIRE(ok.actions.size() == 1);
    CHECK(ok.actions[0].kind == RobotActionKind::StartSegmentMotion);
    REQUIRE(ok.actions[0].segment.has_value());
    CHECK(ok.actions[0].segment->target == Endpoint::B);
}

TEST_CASE("RobotFsm completes directional mission at expected endpoint", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardPrimaryDock, lane, CommandSource::Rpc, "rpc-2");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    const auto settled = fsm.settle_endpoint(Endpoint::A);

    REQUIRE(settled.accepted);
    CHECK(settled.reason == "mission_done");
    CHECK(fsm.robot_state() == RobotState::Idle);
    CHECK(settled.has_action(RobotActionKind::StopMotion));
}

TEST_CASE("RobotFsm repeats configured mission cycles", "[app][fsm]") {
    RobotFsm fsm;
    LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_configured_mission_context(
        lane, PositionState::AtA, CommandSource::Scheduler, "schedule", 2);
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    REQUIRE(fsm.settle_endpoint(Endpoint::B).accepted);
    const auto first_cycle =
        fsm.settle_endpoint(Endpoint::A);

    REQUIRE(first_cycle.accepted);
    CHECK(first_cycle.reason == "next_cycle");
    CHECK(fsm.robot_state() == RobotState::ExecutingMission);
    CHECK(fsm.completed_cycles() == 1u);
    CHECK(first_cycle.has_action(RobotActionKind::StartSegmentMotion));
}

TEST_CASE("RobotFsm rejects wrong endpoint as business inconsistency", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-3");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    const auto result = fsm.settle_endpoint(Endpoint::A);

    REQUIRE(result.accepted);
    CHECK(result.reason == "unexpected_endpoint");
    CHECK(result.safety_fault);
    CHECK(fsm.robot_state() == RobotState::FaultStopped);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::EmergencyStopMotion));
}

TEST_CASE("RobotFsm stop is only allowed while executing or recovering", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-4");
    REQUIRE(fsm.start(mission).accepted);

    const auto self_check_stop = fsm.stop("stop-before-run");
    CHECK_FALSE(self_check_stop.accepted);
    CHECK(self_check_stop.reason == "stop_not_allowed");
    CHECK(fsm.robot_state() == RobotState::SelfChecking);

    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    const auto result = fsm.stop("stop-1");

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::Idle);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::StopMotion));
}

TEST_CASE("RobotFsm immediate fault enters FaultStopped", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-5");
    REQUIRE(fsm.start(mission).accepted);

    const auto result =
        fsm.apply_fault(FaultHandling{0x1001u, FaultResponse::Stop, {}, std::nullopt});

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::FaultStopped);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::EmergencyStopMotion));
}

TEST_CASE("RobotFsm brush-off return is executed as a mission segment", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    REQUIRE(fsm.start(build_directional_clean_context(
                MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-return"))
                .accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    const auto return_mission =
        build_brush_off_return_context(lane, CommandSource::FaultPolicy, "fault");
    const auto result = fsm.apply_fault(
        FaultHandling{0x2001u, FaultResponse::ReturnHome, "return_home", return_mission});

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::ExecutingMission);
    REQUIRE(fsm.mission().has_value());
    REQUIRE(fsm.mission()->current_segment() != nullptr);
    CHECK(fsm.mission()->current_segment()->target == Endpoint::A);
    CHECK(fsm.mission()->current_segment()->mode == SegmentMode::BrushOffReturn);
    CHECK(result.has_action(RobotActionKind::StartSegmentMotion));
}

TEST_CASE("RobotFsm protective stop starts recovery without clearing mission", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-6");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);

    const auto result = fsm.apply_fault(FaultHandling{
        FaultCode::kTransientAttitudeError,
        FaultResponse::Recover,
        "transient_attitude_error",
        std::nullopt});

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::Recovering);
    CHECK(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::StopMotion));
    CHECK(result.has_action(RobotActionKind::StartRecoveryMotion));
}

TEST_CASE("RobotFsm recovery success resumes current mission segment", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-7");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);
    REQUIRE(fsm.apply_fault(FaultHandling{
                    FaultCode::kTransientAttitudeError,
                    FaultResponse::Recover,
                    "pose",
                    std::nullopt})
                .accepted);

    const auto result = fsm.complete_recovery(true, {});

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::ExecutingMission);
    CHECK(result.has_action(RobotActionKind::StartSegmentMotion));
}

TEST_CASE("RobotFsm recovery failure enters FaultStopped and clears mission", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-8");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);
    REQUIRE(fsm.apply_fault(FaultHandling{
                    FaultCode::kTransientAttitudeError,
                    FaultResponse::Recover,
                    "pose",
                    std::nullopt})
                .accepted);

    const auto result =
        fsm.complete_recovery(false, "failed");

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::FaultStopped);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::StopMotion));
}

TEST_CASE("RobotFsm cloud fault reset clears FaultStopped directly to Idle", "[app][fsm]") {
    RobotFsm fsm;
    REQUIRE(fsm.apply_fault(FaultHandling{0x1001u, FaultResponse::Stop, {}, std::nullopt})
                .accepted);

    const auto result = fsm.reset_fault("reset-1");

    REQUIRE(result.accepted);
    CHECK(fsm.robot_state() == RobotState::Idle);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::ClearFault));
}

TEST_CASE("RobotFsm endpoint settled during recovery enters FaultStopped", "[app][fsm]") {
    RobotFsm fsm;
    const LaneConfig lane{DockMode::SingleDock, Endpoint::A};
    auto mission = build_directional_clean_context(
        MissionKind::CleanTowardOppositeEndpoint, lane, CommandSource::Rpc, "rpc-9");
    REQUIRE(fsm.start(mission).accepted);
    REQUIRE(fsm.complete_self_check(true, false, {}).accepted);
    REQUIRE(fsm.apply_fault(FaultHandling{
                    FaultCode::kTransientAttitudeError,
                    FaultResponse::Recover,
                    "pose",
                    std::nullopt})
                .accepted);

    const auto result = fsm.settle_endpoint(Endpoint::B);

    REQUIRE(result.accepted);
    CHECK(result.reason == "endpoint_during_recovery");
    CHECK(result.safety_fault);
    CHECK(fsm.robot_state() == RobotState::FaultStopped);
    CHECK_FALSE(fsm.mission().has_value());
    CHECK(result.has_action(RobotActionKind::EmergencyStopMotion));
}
