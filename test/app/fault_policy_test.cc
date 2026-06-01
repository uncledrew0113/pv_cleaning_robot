#include <catch2/catch.hpp>

#include "pv_cleaning_robot/app/fault_policy.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

using robot::app::FaultAction;
using robot::app::FaultFact;
using robot::app::FaultPolicy;
using robot::app::FaultSource;
namespace FaultCode = robot::domain::FaultCode;

TEST_CASE("FaultPolicy maps endpoint conflicts to emergency latch", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::SafetyMonitor, FaultCode::kConflictingLimitSides, "both_limits"});

    REQUIRE(decision.action == FaultAction::EmergencyStopAndLatch);
    REQUIRE(decision.latch);
}

TEST_CASE("FaultPolicy maps transient attitude to recovery", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::FaultDetector, FaultCode::kTransientAttitudeError, "tilt"});

    REQUIRE(decision.action == FaultAction::StartRecovery);
    REQUIRE_FALSE(decision.latch);
}

TEST_CASE("FaultPolicy maps low battery start gate to reject start", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision = policy.decide(
        FaultFact{FaultSource::SelfCheck, FaultCode::kStartRejectedLowBattery, "soc_low"});

    REQUIRE(decision.action == FaultAction::RejectStart);
    REQUIRE_FALSE(decision.latch);
}

TEST_CASE("FaultPolicy defaults 0x1xxx faults to emergency latch", "[app][fault_policy]") {
    FaultPolicy policy;
    const auto decision =
        policy.decide(FaultFact{FaultSource::Watchdog, 0x1F55u, "unknown_p0"});

    REQUIRE(decision.action == FaultAction::EmergencyStopAndLatch);
    REQUIRE(decision.latch);
}
