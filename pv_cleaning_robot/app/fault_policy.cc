#include "pv_cleaning_robot/app/fault_policy.h"

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {
namespace {

bool is_p0_code(uint32_t code) noexcept {
    return (code & 0xF000u) == 0x1000u;
}

}  // namespace

FaultDecision FaultPolicy::decide(const FaultFact& fact) const noexcept {
    namespace FaultCode = robot::domain::FaultCode;

    switch (fact.code) {
    case FaultCode::kStartRejectedLowBattery:
    case FaultCode::kStartRejectedInvalidPosition:
    case FaultCode::kStartRejectedBusy:
        return {FaultAction::RejectStart, false};
    case FaultCode::kTransientAttitudeError:
        return {FaultAction::StartRecovery, false};
    case FaultCode::kWheelSpinFree:
    case FaultCode::kCanCommunicationLost:
    case FaultCode::kSegmentStartFailed:
    case FaultCode::kP1DuringReturnEscalatedToP0:
    case FaultCode::kTaskContextInconsistent:
    case FaultCode::kUnexpectedLimitSide:
    case FaultCode::kConflictingLimitSides:
    case FaultCode::kLimitUnstableAfterEmergencyStop:
        return {FaultAction::EmergencyStopAndLatch, true};
    default:
        if (is_p0_code(fact.code)) {
            return {FaultAction::EmergencyStopAndLatch, true};
        }
        return {FaultAction::WarnOnly, false};
    }
}

}  // namespace robot::app
