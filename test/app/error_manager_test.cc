#include <catch2/catch.hpp>

#include <optional>

#include "pv_cleaning_robot/app/error_manager.h"

using namespace robot::app;
using robot::domain::DiagnosticsSnapshot;

namespace {

ErrorFact fact(ErrorCode code, ComponentKind kind, uint64_t ts = 1000) {
    return ErrorFact{code, ComponentId{kind, 0}, "test", ts};
}

}  // namespace

TEST_CASE("ErrorManager maps component driver errors to device recovery",
          "[app][error_manager]") {
    ErrorManager manager;

    auto walk = manager.submit_error(
        fact(ErrorCode::DriverCommError, ComponentKind::WalkMotorGroup));
    CHECK(walk.action == ErrorAction::StartRecovery);
    CHECK(walk.plan == RecoveryPlanId::RecoverWalkMotorGroup);

    auto brush = manager.submit_error(
        fact(ErrorCode::DriverCommError, ComponentKind::BrushMotor));
    CHECK(brush.action == ErrorAction::StartRecovery);
    CHECK(brush.plan == RecoveryPlanId::RecoverBrushMotor);
}

TEST_CASE("ErrorManager maps watchdog thread names to recovery decisions",
          "[app][error_manager]") {
    ErrorManager manager;

    auto walk = manager.submit_watchdog_timeout("walk_ctrl", 1000);
    CHECK(walk.plan == RecoveryPlanId::RecoverWalkMotorGroup);
    CHECK(walk.requires_robot_recovering);

    auto brush = manager.submit_watchdog_timeout("brush", 2000);
    CHECK(brush.plan == RecoveryPlanId::RecoverBrushMotor);
}

TEST_CASE("ErrorManager maps attitude limit conflict to FaultStopped",
          "[app][error_manager]") {
    ErrorManager manager;

    auto attitude = manager.submit_error(
        fact(ErrorCode::AttitudeLimitBoth, ComponentKind::AttitudeLimitSwitch));
    CHECK(attitude.action == ErrorAction::FaultStopped);
}

TEST_CASE("ErrorManager reports BMS comm error after update_count stalls for 3 seconds",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.bms_update.enabled = true;
    snap.bms_update.last_update_ms = 1000;

    manager.update(snap, 1000);
    manager.update(snap, 3999);
    CHECK(manager.drain_decisions().empty());
    manager.update(snap, 4000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverBms);
}

TEST_CASE("ErrorManager reports brush motor fault when brush fault code is nonzero",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.brush_fault_active = true;

    manager.update(snap, 1000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].root_error.code == ErrorCode::BrushMotorFault);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverBrushMotor);
}

TEST_CASE("ErrorManager reports brush comm error after 10 consecutive error increments",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.brush.enabled = true;

    for (uint32_t i = 1; i <= 9; ++i) {
        snap.brush.error_count = i;
        manager.update(snap, 1000 + i * 500);
        CHECK(manager.drain_decisions().empty());
    }

    snap.brush.error_count = 10;
    manager.update(snap, 6000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverBrushMotor);
}

TEST_CASE("ErrorManager reports GPS comm error after sentence_count stalls for 3 seconds",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.gps.enabled = true;
    snap.gps.last_update_ms = 1000;

    manager.update(snap, 1000);
    manager.update(snap, 3999);
    CHECK(manager.drain_decisions().empty());
    manager.update(snap, 4000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverGps);
}

TEST_CASE("ErrorManager does not report GPS comm error when 1Hz data keeps updating",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.gps.enabled = true;

    for (uint64_t now = 1000; now <= 5000; now += 50) {
        if ((now - 1000) % 1000 == 0) {
            snap.gps.last_update_ms = now;
        }
        manager.update(snap, now);
        CHECK(manager.drain_decisions().empty());
    }
}

TEST_CASE("ErrorManager reports IMU comm error after frame_count stalls for 3 seconds",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.imu.enabled = true;
    snap.imu.last_update_ms = 1000;

    manager.update(snap, 1000);
    manager.update(snap, 3999);
    CHECK(manager.drain_decisions().empty());
    manager.update(snap, 4000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverImu);
}

TEST_CASE("ErrorManager reports walk feedback comm error after feedback stalls for 3 seconds",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.walk_feedback_expected = true;
    snap.walk_feedback[0].enabled = true;
    snap.walk_feedback[0].last_update_ms = 1000;

    manager.update(snap, 1000);
    manager.update(snap, 3999);
    CHECK(manager.drain_decisions().empty());
    manager.update(snap, 4000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverWalkMotorGroup);
}

TEST_CASE("ErrorManager reports one stalled walk feedback even when other wheels update",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.walk_feedback_expected = true;
    for (auto& health : snap.walk_feedback) {
        health.enabled = true;
        health.last_update_ms = 1000;
    }

    manager.update(snap, 1000);
    CHECK(manager.drain_decisions().empty());

    for (uint64_t now = 1500; now < 4000; now += 500) {
        snap.walk_feedback[1].last_update_ms = now;
        snap.walk_feedback[2].last_update_ms = now;
        snap.walk_feedback[3].last_update_ms = now;
        manager.update(snap, now);
        CHECK(manager.drain_decisions().empty());
    }

    snap.walk_feedback[1].last_update_ms = 4000;
    snap.walk_feedback[2].last_update_ms = 4000;
    snap.walk_feedback[3].last_update_ms = 4000;
    manager.update(snap, 4000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].root_error.component.kind == ComponentKind::WalkMotorGroup);
    CHECK(decisions[0].root_error.component.index == 0);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverWalkMotorGroup);

    manager.update(snap, 4500);
    CHECK(manager.drain_decisions().empty());
}

TEST_CASE("ErrorManager reports walk stall after 5 seconds continuous stall",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.walk_stall_active = true;

    manager.update(snap, 1000);
    manager.update(snap, 5999);
    CHECK(manager.drain_decisions().empty());
    manager.update(snap, 6000);

    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverWalkStall);
}

TEST_CASE("ErrorManager faults after 3 walk stall recovery events in 1 minute",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.walk_stall_active = true;

    for (uint64_t ts : {1000ull, 20000ull, 59000ull}) {
        manager.update(snap, ts);
        manager.update(snap, ts + 5000);

        auto decisions = manager.drain_decisions();
        REQUIRE(decisions.size() == 1);
        if (ts != 59000ull) {
            CHECK(decisions[0].action == ErrorAction::StartRecovery);
            CHECK(decisions[0].plan == RecoveryPlanId::RecoverWalkStall);
        } else {
            CHECK(decisions[0].action == ErrorAction::FaultStopped);
        }

        // 每次堵转消失后才允许下一次上升沿重新计数。
        snap.walk_stall_active = false;
        manager.update(snap, ts + 6000);
        CHECK(manager.drain_decisions().empty());
        snap.walk_stall_active = true;
    }
}

TEST_CASE("ErrorManager faults after 3 GPS stuck events in 1 minute",
          "[app][error_manager]") {
    ErrorManager manager;

    for (uint64_t ts : {1000ull, 20000ull, 59000ull}) {
        auto decision = manager.submit_error(
            ErrorFact{ErrorCode::GpsStuck,
                      ComponentId{ComponentKind::GpsStuckService, 0},
                      "stuck",
                      ts});

        if (ts != 59000ull) {
            CHECK(decision.action == ErrorAction::StartRecovery);
        } else {
            CHECK(decision.action == ErrorAction::FaultStopped);
        }
        manager.drain_decisions();
    }
}

TEST_CASE("ErrorManager faults only after 3 consecutive recovery failures",
          "[app][error_manager]") {
    ErrorManager manager;
    auto decision = manager.submit_error(
        fact(ErrorCode::DriverCommError, ComponentKind::WalkMotorGroup));

    for (uint64_t attempt = 1; attempt <= 3; ++attempt) {
        auto result = manager.mark_recovery_finished(
            RecoveryResultFact{decision, false, "restart failed", 1000 + attempt});

        if (attempt < 3) {
            CHECK(result.action == ErrorAction::StartRecovery);
            CHECK(result.plan == RecoveryPlanId::RecoverWalkMotorGroup);
        } else {
            CHECK(result.action == ErrorAction::FaultStopped);
        }
    }
}

TEST_CASE("ErrorManager deduplicates repeated stream timeout for the same component",
          "[app][error_manager]") {
    ErrorManager manager;
    DiagnosticsSnapshot snap{};
    snap.gps.enabled = true;
    snap.gps.last_update_ms = 1000;

    manager.update(snap, 4000);
    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverGps);

    manager.update(snap, 4100);
    CHECK(manager.drain_decisions().empty());

    snap.gps.last_update_ms = 4200;
    manager.update(snap, 4200);
    CHECK(manager.drain_decisions().empty());

    manager.update(snap, 7200);
    decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverGps);
}

TEST_CASE("ErrorManager suppresses expected child errors while recovery is active",
          "[app][error_manager]") {
    ErrorManager manager;
    auto decision = manager.submit_error(
        fact(ErrorCode::DriverCommError, ComponentKind::Gps));
    manager.drain_decisions();
    manager.mark_recovery_started(decision);

    auto gps_stuck = manager.submit_error(
        fact(ErrorCode::GpsStuck, ComponentKind::GpsStuckService, 2000));
    CHECK(gps_stuck.action == ErrorAction::Ignore);

    DiagnosticsSnapshot snap{};
    snap.gps.enabled = true;
    snap.gps.last_update_ms = 1000;
    manager.update(snap, 5000);
    CHECK(manager.drain_decisions().empty());

    manager.mark_recovery_finished(RecoveryResultFact{decision, true, "", 6000});
    manager.update(snap, 7000);
    auto decisions = manager.drain_decisions();
    REQUIRE(decisions.size() == 1);
    CHECK(decisions[0].plan == RecoveryPlanId::RecoverGps);
}

TEST_CASE("ErrorManager does not fault on failed motion recovery result",
          "[app][error_manager]") {
    ErrorManager manager;
    auto decision = manager.submit_error(
        fact(ErrorCode::WalkMotorStall, ComponentKind::WalkMotorGroup));
    REQUIRE(decision.plan == RecoveryPlanId::RecoverWalkStall);
    manager.drain_decisions();

    for (uint64_t attempt = 1; attempt <= 3; ++attempt) {
        auto result = manager.mark_recovery_finished(
            RecoveryResultFact{decision, false, "motion timeout", 1000 + attempt});
        CHECK(result.action == ErrorAction::Ignore);
        CHECK(manager.drain_decisions().empty());
    }
}

TEST_CASE("ErrorHandlingService executes device recovery while idle",
          "[app][error_manager]") {
    ErrorManager manager;
    std::optional<ErrorFact> pending =
        fact(ErrorCode::DriverCommError, ComponentKind::Bms);
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"Idle"}; },
            [&]() {
                auto out = pending;
                pending.reset();
                return out;
            },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [](const ErrorDecision&) {},
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1000};
            },
            [](bool) {},
            [] { return 1000ull; },
        });

    service.update();
    CHECK(recoveries == 1);
}

TEST_CASE("ErrorHandlingService skips motion recovery outside executing mission",
          "[app][error_manager]") {
    ErrorManager manager;
    std::optional<ErrorFact> pending =
        fact(ErrorCode::AttitudeLimit, ComponentKind::AttitudeLimitSwitch);
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"Idle"}; },
            [&]() {
                auto out = pending;
                pending.reset();
                return out;
            },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [](const ErrorDecision&) {},
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1000};
            },
            [](bool) {},
            [] { return 1000ull; },
        });

    service.update();
    CHECK(recoveries == 0);
}

TEST_CASE("ErrorHandlingService skips all recovery during self check",
          "[app][error_manager]") {
    ErrorManager manager;
    manager.submit_error(fact(ErrorCode::DriverCommError, ComponentKind::Bms, 1000));
    int applied = 0;
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"SelfChecking"}; },
            [] { return std::optional<ErrorFact>{}; },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [&](const ErrorDecision&) { ++applied; },
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1100};
            },
            [](bool) {},
            [] { return 1100ull; },
        });

    service.update();

    CHECK(applied == 0);
    CHECK(recoveries == 0);
}

TEST_CASE("ErrorHandlingService skips all recovery while charging", "[app][error_manager]") {
    ErrorManager manager;
    manager.submit_error(fact(ErrorCode::DriverCommError, ComponentKind::Bms, 1000));
    int applied = 0;
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"Charging"}; },
            [] { return std::optional<ErrorFact>{}; },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [&](const ErrorDecision&) { ++applied; },
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1100};
            },
            [](bool) {},
            [] { return 1100ull; },
        });

    service.update();

    CHECK(applied == 0);
    CHECK(recoveries == 0);
}

TEST_CASE("ErrorHandlingService enables GPS stuck only while executing mission",
          "[app][error_manager]") {
    ErrorManager manager;
    std::string state = "Idle";
    std::vector<bool> gps_enabled;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [&] { return state; },
            [] { return std::optional<ErrorFact>{}; },
            [] { return DiagnosticsSnapshot{}; },
            [&](bool enabled) { gps_enabled.push_back(enabled); },
            [](const ErrorDecision&) {},
            [](const ErrorDecision& decision) {
                return RecoveryResultFact{decision, true, "", 1000};
            },
            [](bool) {},
            [] { return 1000ull; },
        });

    service.update();
    CHECK(gps_enabled.empty());

    state = "ExecutingMission";
    service.update();
    REQUIRE(gps_enabled.size() == 1);
    CHECK(gps_enabled.back());

    state = "Charging";
    service.update();
    REQUIRE(gps_enabled.size() == 2);
    CHECK_FALSE(gps_enabled.back());
}

TEST_CASE("ErrorHandlingService handles only the highest priority decision per update",
          "[app][error_manager]") {
    ErrorManager manager;
    manager.submit_error(fact(ErrorCode::DriverCommError, ComponentKind::Bms, 1000));
    manager.submit_error(fact(ErrorCode::AttitudeLimitBoth, ComponentKind::AttitudeLimitSwitch, 1001));

    std::vector<ErrorAction> applied_actions;
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"ExecutingMission"}; },
            [] { return std::optional<ErrorFact>{}; },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [&](const ErrorDecision& decision) { applied_actions.push_back(decision.action); },
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1100};
            },
            [](bool) {},
            [] { return 1100ull; },
        });

    service.update();

    REQUIRE(applied_actions.size() == 1);
    CHECK(applied_actions.front() == ErrorAction::FaultStopped);
    CHECK(recoveries == 0);
}

TEST_CASE("ErrorHandlingService ignores single attitude limit outside executing mission",
          "[app][error_manager]") {
    ErrorManager manager;
    std::optional<ErrorFact> pending =
        fact(ErrorCode::AttitudeLimit, ComponentKind::AttitudeLimitSwitch, 1000);
    int applied = 0;
    int recoveries = 0;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"Idle"}; },
            [&]() {
                auto out = pending;
                pending.reset();
                return out;
            },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [&](const ErrorDecision&) { ++applied; },
            [&](const ErrorDecision& decision) {
                ++recoveries;
                return RecoveryResultFact{decision, true, "", 1100};
            },
            [](bool) {},
            [] { return 1100ull; },
        });

    service.update();

    CHECK(applied == 0);
    CHECK(recoveries == 0);
}

TEST_CASE("ErrorHandlingService keeps attitude both hard fault outside executing mission",
          "[app][error_manager]") {
    ErrorManager manager;
    std::optional<ErrorFact> pending =
        fact(ErrorCode::AttitudeLimitBoth, ComponentKind::AttitudeLimitSwitch, 1000);
    std::vector<ErrorAction> applied_actions;

    ErrorHandlingService service(
        manager,
        ErrorHandlingService::Ports{
            [] { return std::string{"Idle"}; },
            [&]() {
                auto out = pending;
                pending.reset();
                return out;
            },
            [] { return DiagnosticsSnapshot{}; },
            [](bool) {},
            [&](const ErrorDecision& decision) { applied_actions.push_back(decision.action); },
            [](const ErrorDecision& decision) {
                return RecoveryResultFact{decision, true, "", 1100};
            },
            [](bool) {},
            [] { return 1100ull; },
        });

    service.update();

    REQUIRE(applied_actions.size() == 1);
    CHECK(applied_actions.front() == ErrorAction::FaultStopped);
}
