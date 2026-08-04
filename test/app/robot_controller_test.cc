#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/app/error_manager.h"
#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::RobotController;
using robot::domain::CommandSource;
using robot::domain::RobotCommand;
using robot::domain::RobotCommandKind;

struct RecordingRobotActions {
    int start_segment_count{0};
    int stop_count{0};
    int emergency_stop_count{0};
    int start_recovery_count{0};
    int clear_fault_count{0};
    int lock_open_count{0};
    int lock_close_count{0};
    int hold_at_endpoint_count{0};
    bool lock_open_result{true};
    bool lock_close_result{true};
    bool hold_at_endpoint_result{true};
    std::vector<std::string> action_order;

    robot::app::RobotController::ActionPorts ports() {
        robot::app::RobotController::ActionPorts result;
        result.start_segment = [this](const robot::domain::MissionSegment&) {
            ++start_segment_count;
            return true;
        };
        result.stop_motion = [this] {
            ++stop_count;
            action_order.push_back("stop");
        };
        result.emergency_stop = [this] { ++emergency_stop_count; };
        result.start_recovery = [this] { ++start_recovery_count; };
        result.clear_fault = [this] { ++clear_fault_count; };
        result.open_lock_motor = [this] {
            ++lock_open_count;
            action_order.push_back("lock");
            return lock_open_result;
        };
        result.close_lock_motor = [this] {
            ++lock_close_count;
            return lock_close_result;
        };
        result.hold_at_endpoint = [this] {
            ++hold_at_endpoint_count;
            action_order.push_back("hold");
            return hold_at_endpoint_result;
        };
        return result;
    }
};

robot::app::ErrorDecision make_recovery_decision(
    robot::app::RecoveryPlanId plan = robot::app::RecoveryPlanId::RecoverAttitudeCenter) {
    robot::app::ErrorDecision decision;
    decision.action = robot::app::ErrorAction::StartRecovery;
    decision.plan = plan;
    decision.requires_robot_recovering = true;
    return decision;
}

robot::app::ErrorDecision make_fault_stopped_decision(
    robot::app::ErrorCode code,
    robot::app::ComponentKind component = robot::app::ComponentKind::WalkMotorGroup) {
    robot::app::ErrorDecision decision;
    decision.action = robot::app::ErrorAction::FaultStopped;
    decision.latch_fault = true;
    decision.root_error.code = code;
    decision.root_error.component = robot::app::ComponentId{component, 0};
    return decision;
}

void finish_endpoint_settle(RobotController& controller) {
    controller.handle_tick_for_test(std::chrono::steady_clock::time_point::max());
}

TEST_CASE("RobotController starts configured mission through SelfChecking",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController RPC configured mission assumes primary dock as start",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    robot::domain::Endpoint first_target{robot::domain::Endpoint::B};
    auto ports = actions.ports();
    ports.start_segment = [&](const robot::domain::MissionSegment& segment) {
        first_target = segment.target;
        ++actions.start_segment_count;
        return true;
    };
    RobotController controller(ports);
    controller.set_config_ports(RobotController::ConfigPorts{
        [] {
            robot::domain::RuntimeConfig cfg;
            cfg.primary_dock = robot::domain::Endpoint::B;
            return cfg;
        },
        [] { return std::optional<robot::domain::RuntimeConfig>{}; },
        [](const robot::domain::RuntimeConfig&) { return 1ull; },
        [] { return true; },
        [] {
            robot::domain::LaneConfig lane;
            lane.dock_mode = robot::domain::DockMode::DualDock;
            lane.primary_dock = robot::domain::Endpoint::B;
            return lane;
        }});
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "rpc-maint"});
    REQUIRE(result.accepted);

    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "ExecutingMission");
    REQUIRE(actions.start_segment_count == 1);
    CHECK(first_target == robot::domain::Endpoint::A);
}

TEST_CASE("RobotController rejects start while busy", "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-2"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "busy");
}

TEST_CASE("RobotController stop is only accepted while mission is active",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    auto result =
        controller.submit_command(RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-idle"});
    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "not_running");

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-running"});
    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 0);
}

TEST_CASE("RobotController rejects stop outside ExecutingMission", "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-self-check"});
    REQUIRE_FALSE(result.accepted);
    CHECK(controller.snapshot().state == "SelfChecking");

    controller.complete_self_check_for_test(true);
    controller.apply_error_decision(make_recovery_decision());
    REQUIRE(controller.snapshot().state == "Recovering");

    result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-recovering"});
    REQUIRE_FALSE(result.accepted);
    CHECK(controller.snapshot().state == "Recovering");
}

TEST_CASE("RobotController serializes posted callbacks on controller thread",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.start();

    std::atomic<int> count{0};
    std::vector<std::thread> workers;
    for (int i = 0; i < 8; ++i) {
        workers.emplace_back([&controller, &count] {
            for (int j = 0; j < 25; ++j) {
                controller.post_for_test([&count] { count.fetch_add(1); });
            }
        });
    }
    for (auto& worker : workers) {
        worker.join();
    }

    controller.drain_for_test();
    controller.stop();

    REQUIRE(count.load() == 200);
}

TEST_CASE("RobotController event exception enters FaultStopped",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.start();

    controller.post_for_test([] { throw std::runtime_error("boom"); });
    controller.drain_for_test();

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(actions.emergency_stop_count == 1);
    controller.stop();
}

TEST_CASE("RobotController submit_command works through running queue",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.start();

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-queued"});

    controller.stop();

    REQUIRE(result.accepted);
    REQUIRE(result.reason == "accepted");
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController completes configured single-dock mission through two endpoints",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    controller.complete_self_check_for_test(true);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);
    REQUIRE(controller.snapshot().state == "ExecutingMission");

    position = robot::domain::PositionState::AtA;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    finish_endpoint_settle(controller);
    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE(actions.stop_count == 1);
}

TEST_CASE("RobotController waits in SettlingEndpoint before starting next segment",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    RobotController::ConfigPorts config;
    config.endpoint_settle_delay_ms = 3000;
    controller.set_config_ports(std::move(config));
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    REQUIRE(actions.start_segment_count == 1);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);

    CHECK(controller.snapshot().state == "SettlingEndpoint");
    CHECK(actions.hold_at_endpoint_count == 1);
    CHECK(actions.start_segment_count == 1);
    const auto stop_result = controller.submit_command(
        RobotCommand{RobotCommandKind::Stop, CommandSource::Rpc, "stop-settling"});
    CHECK_FALSE(stop_result.accepted);
    CHECK(stop_result.reason == "not_running");

    controller.handle_tick_for_test(std::chrono::steady_clock::time_point::min());
    CHECK(controller.snapshot().state == "SettlingEndpoint");
    CHECK(actions.start_segment_count == 1);

    controller.handle_tick_for_test(std::chrono::steady_clock::time_point::max());
    CHECK(controller.snapshot().state == "ExecutingMission");
    CHECK(actions.start_segment_count == 2);
}

TEST_CASE("RobotController ignores repeated settled endpoint during endpoint hold",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    REQUIRE(controller.snapshot().state == "SettlingEndpoint");
    REQUIRE(actions.hold_at_endpoint_count == 1);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);

    CHECK(controller.snapshot().state == "SettlingEndpoint");
    CHECK_FALSE(controller.snapshot().fault.has_value());
    CHECK(actions.hold_at_endpoint_count == 1);
    CHECK(actions.emergency_stop_count == 0);
    finish_endpoint_settle(controller);
    CHECK(controller.snapshot().state == "ExecutingMission");
    CHECK(actions.start_segment_count == 2);
}

TEST_CASE("RobotController ignores different settled endpoint during endpoint hold",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    REQUIRE(controller.snapshot().state == "SettlingEndpoint");

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);

    CHECK(controller.snapshot().state == "SettlingEndpoint");
    CHECK_FALSE(controller.snapshot().fault.has_value());
    CHECK(actions.hold_at_endpoint_count == 1);
    CHECK(actions.emergency_stop_count == 0);
    finish_endpoint_settle(controller);
    CHECK(controller.snapshot().state == "ExecutingMission");
    CHECK(actions.start_segment_count == 2);
}

TEST_CASE("RobotController locks before disabling walk when final endpoint settles",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    RobotController::ConfigPorts config;
    config.endpoint_settle_delay_ms = 3000;
    controller.set_config_ports(std::move(config));
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    controller.handle_tick_for_test(std::chrono::steady_clock::time_point::max());
    actions.action_order.clear();

    position = robot::domain::PositionState::AtA;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    CHECK(controller.snapshot().state == "SettlingEndpoint");
    controller.handle_tick_for_test(std::chrono::steady_clock::time_point::max());

    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE(actions.action_order.size() == 3);
    CHECK(actions.action_order[0] == "hold");
    CHECK(actions.action_order[1] == "lock");
    CHECK(actions.action_order[2] == "stop");
}

TEST_CASE("RobotController faults when endpoint hold fails", "[app][robot_controller]") {
    RecordingRobotActions actions;
    actions.hold_at_endpoint_result = false;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);

    CHECK(controller.snapshot().state == "FaultStopped");
    CHECK(controller.snapshot().fault == robot::domain::FaultCode::kSegmentStartFailed);
    CHECK(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController restarts current segment on source endpoint repeat",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    REQUIRE(actions.start_segment_count == 1);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);

    REQUIRE(controller.snapshot().state == "ExecutingMission");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
    REQUIRE(actions.start_segment_count == 2);
}

TEST_CASE("RobotController does not fault after repeated source endpoint triggers",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    for (int i = 0; i < 20; ++i) {
        controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    }
    REQUIRE(controller.snapshot().state == "ExecutingMission");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
    REQUIRE(actions.emergency_stop_count == 0);
}

TEST_CASE("RobotController ignores settled endpoint while idle", "[app][robot_controller]") {
    RobotController controller;

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);

    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController ignores settled endpoint during recovery",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    controller.apply_error_decision(make_recovery_decision());

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);

    REQUIRE(controller.snapshot().state == "Recovering");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController FaultStopped error decision resets back to Idle",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.apply_error_decision(
        make_fault_stopped_decision(robot::app::ErrorCode::AttitudeLimitBoth));

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kAttitudeLimitBoth);

    const auto reset = controller.submit_command(
        RobotCommand{RobotCommandKind::FaultReset, CommandSource::Rpc, "reset-1"});
    REQUIRE(reset.accepted);
    REQUIRE(controller.snapshot().state == "Idle");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
}

TEST_CASE("RobotController maps ErrorManager faults to specific domain fault codes",
          "[app][robot_controller]") {
    struct Case {
        robot::app::ErrorCode code;
        robot::app::ComponentKind component;
        uint32_t expected_fault;
    };

    const Case cases[] = {
        {robot::app::ErrorCode::DriverCommError,
         robot::app::ComponentKind::WalkMotorGroup,
         robot::domain::FaultCode::kCanCommunicationLost},
        {robot::app::ErrorCode::DriverCommError,
         robot::app::ComponentKind::Gps,
         robot::domain::FaultCode::kGpsCommunicationLost},
        {robot::app::ErrorCode::DriverCommError,
         robot::app::ComponentKind::Bms,
         robot::domain::FaultCode::kBmsCommunicationLost},
        {robot::app::ErrorCode::DriverCommError,
         robot::app::ComponentKind::BrushMotor,
         robot::domain::FaultCode::kBrushMotorCommunicationLost},
        {robot::app::ErrorCode::DriverCommError,
         robot::app::ComponentKind::Imu,
         robot::domain::FaultCode::kImuCommunicationLost},
        {robot::app::ErrorCode::WalkMotorStall,
         robot::app::ComponentKind::WalkMotorGroup,
         robot::domain::FaultCode::kWalkMotorStall},
        {robot::app::ErrorCode::BrushMotorFault,
         robot::app::ComponentKind::BrushMotor,
         robot::domain::FaultCode::kBrushMotorFault},
        {robot::app::ErrorCode::GpsStuck,
         robot::app::ComponentKind::GpsStuckService,
         robot::domain::FaultCode::kGpsStuck},
        {robot::app::ErrorCode::AttitudeLimitBoth,
         robot::app::ComponentKind::AttitudeLimitSwitch,
         robot::domain::FaultCode::kAttitudeLimitBoth},
    };

    for (const auto& item : cases) {
        RobotController controller;
        controller.apply_error_decision(make_fault_stopped_decision(item.code, item.component));

        INFO("error code index maps to expected domain fault");
        REQUIRE(controller.snapshot().state == "FaultStopped");
        CHECK(controller.snapshot().fault == item.expected_fault);
    }
}

TEST_CASE("RobotController recoverable fault enters Recovering from ExecutingMission",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.apply_error_decision(make_recovery_decision());

    REQUIRE(controller.snapshot().state == "Recovering");
}

TEST_CASE("RobotController ignores recovery decision during self check",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");

    controller.apply_error_decision(
        make_recovery_decision(robot::app::RecoveryPlanId::RecoverAttitudeCenter));

    CHECK(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController self check failure enters FaultStopped", "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    controller.complete_self_check_for_test(false);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kSelfCheckFailed);
    REQUIRE(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController keeps mission during recovery and resumes current segment",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::CleanTowardOppositeEndpoint, CommandSource::Local, "cmd"})
                .accepted);
    controller.complete_self_check_for_test(true);
    REQUIRE(actions.start_segment_count == 1);

    controller.apply_error_decision(
        make_recovery_decision(robot::app::RecoveryPlanId::RecoverAttitudeCenter));
    CHECK(controller.snapshot().state == "Recovering");

    controller.start();
    controller.post_recovery_finished(true);
    controller.drain_for_test();
    controller.stop();
    CHECK(controller.snapshot().state == "ExecutingMission");
    CHECK(actions.start_segment_count >= 2);
}

TEST_CASE("RobotController starts motion after successful self check",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(actions.start_segment_count == 1);
}

TEST_CASE("RobotController advances configured segment when target is already active before motion",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    robot::domain::Endpoint started_target{robot::domain::Endpoint::B};
    auto ports = actions.ports();
    ports.start_segment = [&](const robot::domain::MissionSegment& segment) {
        ++actions.start_segment_count;
        started_target = segment.target;
        return true;
    };

    RobotController controller(ports);
    controller.set_position_state_query([] { return robot::domain::PositionState::AtB; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    finish_endpoint_settle(controller);

    const auto snap = controller.snapshot();
    REQUIRE(snap.state == "ExecutingMission");
    REQUIRE(snap.current_segment_target.has_value());
    CHECK(*snap.current_segment_target == robot::domain::Endpoint::A);
    CHECK(started_target == robot::domain::Endpoint::A);
    CHECK(actions.lock_close_count == 1);
    CHECK(actions.start_segment_count == 1);
    CHECK(actions.lock_open_count == 0);
}

TEST_CASE("RobotController completes directional dock mission when target is already active before motion",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::CleanTowardPrimaryDock, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "Idle");
    CHECK(actions.lock_close_count == 1);
    CHECK(actions.start_segment_count == 0);
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 1);
}

TEST_CASE("RobotController snapshot exposes current segment for attitude recovery key",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    controller.complete_self_check_for_test(true);

    const auto snap = controller.snapshot();
    REQUIRE(snap.current_segment_target.has_value());
    REQUIRE(snap.current_segment_mode.has_value());
    CHECK(*snap.current_segment_target == robot::domain::Endpoint::B);
    CHECK(*snap.current_segment_mode == robot::domain::SegmentMode::Cleaning);
}

TEST_CASE("RobotController closes lock motor before first mission segment",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "ExecutingMission");
    CHECK(actions.lock_close_count == 1);
    CHECK(actions.start_segment_count == 1);
}

TEST_CASE("RobotController enters FaultStopped when lock close fails before mission",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    actions.lock_close_result = false;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    CHECK(controller.snapshot().fault == robot::domain::FaultCode::kLockMotorCloseFailed);
    CHECK(actions.start_segment_count == 0);
    CHECK(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController opens lock motor when mission finishes at dock",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);
    position = robot::domain::PositionState::AtA;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "Idle");
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 1);
}

TEST_CASE("RobotController locks using settled endpoint after dock switch releases",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "Idle");
    CHECK_FALSE(controller.snapshot().fault.has_value());
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 1);
    CHECK(actions.emergency_stop_count == 0);
}

TEST_CASE("RobotController does not open lock motor when single-dock mission finishes away from dock",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::CleanTowardOppositeEndpoint, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "Idle");
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 0);
}

TEST_CASE("RobotController opens lock motor at either dock in dual-dock mode",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_config_ports(RobotController::ConfigPorts{
        [] {
            robot::domain::RuntimeConfig cfg;
            cfg.primary_dock = robot::domain::Endpoint::A;
            return cfg;
        },
        [] { return std::optional<robot::domain::RuntimeConfig>{}; },
        [](const robot::domain::RuntimeConfig&) { return 1ull; },
        [] { return true; },
        [] {
            robot::domain::LaneConfig lane;
            lane.dock_mode = robot::domain::DockMode::DualDock;
            lane.primary_dock = robot::domain::Endpoint::A;
            return lane;
        }});
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::CleanTowardOppositeEndpoint, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "Idle");
    CHECK(actions.stop_count == 1);
    CHECK(actions.lock_open_count == 1);
}

TEST_CASE("RobotController enters FaultStopped when lock open fails after mission",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    actions.lock_open_result = false;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);
    position = robot::domain::PositionState::AtA;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);
    finish_endpoint_settle(controller);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    CHECK(controller.snapshot().fault == robot::domain::FaultCode::kLockMotorOpenFailed);
    CHECK(actions.stop_count == 0);
    CHECK(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController converts motion start failure into FaultStopped",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    auto ports = actions.ports();
    ports.start_segment = [](const robot::domain::MissionSegment&) { return false; };
    RobotController controller(ports);
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });

    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kSegmentStartFailed);
    REQUIRE(actions.emergency_stop_count == 1);
}

TEST_CASE("RobotController applies watchdog decision from ErrorManager",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    robot::app::ErrorManager manager;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    const auto decision = manager.submit_watchdog_timeout("walk_ctrl", 1000);
    controller.apply_error_decision(decision);

    REQUIRE(controller.snapshot().state == "Recovering");
    REQUIRE_FALSE(controller.snapshot().fault.has_value());
    REQUIRE(actions.stop_count == 1);
    REQUIRE(actions.start_recovery_count == 1);
}

TEST_CASE("RobotController runs error decision actions outside state lock",
          "[app][robot_controller]") {
    RobotController* controller_ptr = nullptr;
    bool snapshot_called_from_action = false;
    RecordingRobotActions actions;
    auto ports = actions.ports();
    ports.stop_motion = [&] {
        const auto snapshot = controller_ptr->snapshot();
        snapshot_called_from_action = snapshot.state == "Recovering";
        ++actions.stop_count;
    };

    RobotController controller(ports);
    controller_ptr = &controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);

    auto future = std::async(std::launch::async, [&] {
        controller.apply_error_decision(make_recovery_decision());
    });

    REQUIRE(future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready);
    future.get();
    CHECK(snapshot_called_from_action);
    CHECK(actions.stop_count == 1);
    CHECK(actions.start_recovery_count == 1);
}

TEST_CASE("RobotController runs lock close action outside state lock",
          "[app][robot_controller]") {
    RobotController* controller_ptr = nullptr;
    std::promise<void> close_entered;
    std::promise<void> release_promise;
    std::shared_future<void> close_release = release_promise.get_future().share();

    RecordingRobotActions actions;
    auto ports = actions.ports();
    ports.close_lock_motor = [&] {
        ++actions.lock_close_count;
        close_entered.set_value();
        close_release.wait();
        return true;
    };

    RobotController controller(ports);
    controller_ptr = &controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);

    auto complete_future = std::async(std::launch::async, [&] {
        controller.complete_self_check_for_test(true);
    });
    REQUIRE(close_entered.get_future().wait_for(std::chrono::milliseconds(200)) ==
            std::future_status::ready);

    auto snapshot_future = std::async(std::launch::async, [&] {
        return controller_ptr->snapshot();
    });
    CHECK(snapshot_future.wait_for(std::chrono::milliseconds(200)) ==
          std::future_status::ready);

    release_promise.set_value();
    complete_future.get();
    if (snapshot_future.valid()) {
        (void)snapshot_future.get();
    }
}

TEST_CASE("RobotController runs final stop and lock open actions outside state lock",
          "[app][robot_controller]") {
    RobotController* controller_ptr = nullptr;
    std::promise<void> stop_entered;
    std::promise<void> stop_release;
    RecordingRobotActions actions;
    auto ports = actions.ports();
    ports.stop_motion = [&] {
        ++actions.stop_count;
        stop_entered.set_value();
        stop_release.get_future().wait();
    };

    RobotController controller(ports);
    controller_ptr = &controller;
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    finish_endpoint_settle(controller);
    position = robot::domain::PositionState::AtA;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::A);

    auto finish_future = std::async(std::launch::async, [&] {
        finish_endpoint_settle(controller);
    });
    REQUIRE(stop_entered.get_future().wait_for(std::chrono::milliseconds(200)) ==
            std::future_status::ready);

    auto snapshot_future = std::async(std::launch::async, [&] {
        return controller_ptr->snapshot();
    });
    CHECK(snapshot_future.wait_for(std::chrono::milliseconds(200)) ==
          std::future_status::ready);

    stop_release.set_value();
    finish_future.get();
    if (snapshot_future.valid()) {
        (void)snapshot_future.get();
    }
}

TEST_CASE("RobotController recovery failure callback enters FaultStopped",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"})
                .accepted);
    controller.complete_self_check_for_test(true);
    controller.apply_error_decision(make_recovery_decision());
    REQUIRE(controller.snapshot().state == "Recovering");
    controller.start();

    controller.post_recovery_finished(false);
    controller.drain_for_test();
    controller.stop();

    REQUIRE(controller.snapshot().state == "FaultStopped");
    REQUIRE(controller.snapshot().fault == robot::domain::FaultCode::kRecoveryFailed);
}

TEST_CASE("RobotController single-dock scheduler assumes primary dock when position is on segment",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Scheduler, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController scheduler remains rejected while FaultStopped",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });
    controller.apply_error_decision(
        make_fault_stopped_decision(robot::app::ErrorCode::AttitudeLimitBoth));

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Scheduler, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    CHECK(result.reason == "busy");
    CHECK(controller.snapshot().state == "FaultStopped");
    CHECK(actions.start_segment_count == 0);
}

TEST_CASE("RobotController scheduler remains rejected while SettlingEndpoint",
          "[app][robot_controller]") {
    RecordingRobotActions actions;
    RobotController controller(actions.ports());
    auto position = robot::domain::PositionState::AtA;
    controller.set_position_state_query([&position] { return position; });
    REQUIRE(controller
                .submit_command(RobotCommand{
                    RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "rpc-start"})
                .accepted);
    controller.complete_self_check_for_test(true);
    position = robot::domain::PositionState::AtB;
    controller.handle_limit_settled_for_test(robot::domain::Endpoint::B);
    REQUIRE(controller.snapshot().state == "SettlingEndpoint");

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Scheduler, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    CHECK(result.reason == "busy");
    CHECK(controller.snapshot().state == "SettlingEndpoint");
}

TEST_CASE("RobotController dual-dock scheduler still requires a detected endpoint",
          "[app][robot_controller]") {
    RobotController controller;
    RobotController::ConfigPorts config;
    config.lane_config = [] {
        return robot::domain::LaneConfig{
            robot::domain::DockMode::DualDock, robot::domain::Endpoint::A};
    };
    controller.set_config_ports(std::move(config));
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Scheduler, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    CHECK(result.reason == "configured_mission_requires_start_endpoint");
}

TEST_CASE("RobotController RPC skips start battery and position checks",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::OnSegment; });
    controller.set_battery_soc_query([] { return 10.0f; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Rpc, "cmd-1"});

    REQUIRE(result.accepted);
    REQUIRE(controller.snapshot().state == "SelfChecking");
}

TEST_CASE("RobotController scheduler rejects low battery before start",
          "[app][robot_controller]") {
    RobotController controller;
    controller.set_position_state_query([] { return robot::domain::PositionState::AtA; });
    controller.set_battery_soc_query([] { return 10.0f; });

    const auto result = controller.submit_command(
        RobotCommand{RobotCommandKind::StartConfiguredMission, CommandSource::Scheduler, "cmd-1"});

    REQUIRE_FALSE(result.accepted);
    REQUIRE(result.reason == "battery_below_start_threshold");
}
