#include "pv_cleaning_robot/app/fault_detector.h"

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {

std::vector<FaultFact> FaultDetector::detect(const Input& input) const {
    namespace FaultCode = robot::domain::FaultCode;
    std::vector<FaultFact> facts;

    if (input.left_limit_active && input.right_limit_active) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kConflictingLimitSides,
                         "conflicting_limit_sides"});
    }
    if (input.executing_mission && input.robot_stuck_detected) {
        facts.push_back(
            {FaultSource::FaultDetector, FaultCode::kRobotStuck, "robot_stuck"});
    }
    if (input.executing_mission && input.imu_fresh && input.attitude_out_of_range) {
        facts.push_back({FaultSource::FaultDetector,
                         FaultCode::kTransientAttitudeError,
                         "transient_attitude_error"});
    }
    if (!input.imu_fresh) {
        facts.push_back({FaultSource::FaultDetector, FaultCode::kCanCommunicationLost, "imu_stale"});
    }
    if (input.bms_critical_alarm) {
        facts.push_back(
            {FaultSource::FaultDetector, FaultCode::kCanCommunicationLost, "bms_critical_alarm"});
    }
    if (input.brush_critical_fault) {
        facts.push_back(
            {FaultSource::FaultDetector, FaultCode::kCanCommunicationLost, "brush_critical_fault"});
    }
    if (input.motor_driver_fault) {
        facts.push_back(
            {FaultSource::FaultDetector, FaultCode::kCanCommunicationLost, "motor_driver_fault"});
    }

    return facts;
}

}  // namespace robot::app
