#include "pv_cleaning_robot/app/recovery_executor.h"

#include <utility>

namespace robot::app {

RecoveryExecutor::RecoveryExecutor(Ports ports) : ports_(std::move(ports)) {}

RecoveryResult RecoveryExecutor::fail_if_missing(bool present, const char* name) const {
    if (present) {
        return RecoveryResult{true, ""};
    }
    return RecoveryResult{false, std::string("missing recovery port: ") + name};
}

RecoveryResult RecoveryExecutor::fail_step(const char* name) const {
    return RecoveryResult{false, std::string("recovery step failed: ") + name};
}

RecoveryResult RecoveryExecutor::run_steps(std::initializer_list<Step> steps) {
    for (const auto& step : steps) {
        auto missing = fail_if_missing(step.fn && static_cast<bool>(*step.fn), step.name);
        if (!missing.ok) return missing;
    }

    for (const auto& step : steps) {
        if (!(*step.fn)()) return fail_step(step.name);
    }
    return RecoveryResult{true, ""};
}

RecoveryResult RecoveryExecutor::run_with_gps_stuck_paused(std::initializer_list<Step> steps) {
    // 所有可能急停或恢复运动/设备的流程，先暂停 GpsStuck，避免恢复动作被误判为卡滞。
    auto missing =
        fail_if_missing(static_cast<bool>(ports_.pause_gps_stuck), "pause_gps_stuck");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.resume_gps_stuck), "resume_gps_stuck");
    if (!missing.ok) return missing;

    ports_.pause_gps_stuck();
    auto result = run_steps(steps);
    ports_.resume_gps_stuck();
    return result;
}

RecoveryResult RecoveryExecutor::recover_reverse_motion() {
    auto missing =
        fail_if_missing(static_cast<bool>(ports_.pause_gps_stuck), "pause_gps_stuck");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.resume_gps_stuck), "resume_gps_stuck");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.stop_walk), "stop_walk");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.reverse_walk_motion),
                              "reverse_walk_motion");
    if (!missing.ok) return missing;

    ports_.pause_gps_stuck();
    (void)ports_.stop_walk();
    (void)ports_.reverse_walk_motion();
    ports_.resume_gps_stuck();
    return RecoveryResult{true, ""};
}

RecoveryResult RecoveryExecutor::recover_attitude_center() {
    auto missing =
        fail_if_missing(static_cast<bool>(ports_.pause_gps_stuck), "pause_gps_stuck");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.resume_gps_stuck), "resume_gps_stuck");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.stop_walk), "stop_walk");
    if (!missing.ok) return missing;
    missing = fail_if_missing(static_cast<bool>(ports_.lower_attitude_center),
                              "lower_attitude_center");
    if (!missing.ok) return missing;

    ports_.pause_gps_stuck();
    (void)ports_.stop_walk();
    (void)ports_.lower_attitude_center();
    ports_.resume_gps_stuck();
    return RecoveryResult{true, ""};
}

RecoveryResult RecoveryExecutor::execute(const RecoveryRequest& request) {
    switch (request.plan) {
    case RecoveryPlanId::RecoverWalkMotorGroup:
        return run_with_gps_stuck_paused({{"stop_walk", &ports_.stop_walk},
                                          {"stop_walk_executor", &ports_.stop_walk_executor},
                                          {"restart_walk_driver", &ports_.restart_walk_driver},
                                          {"start_walk_executor",
                                           &ports_.start_walk_executor}});
    case RecoveryPlanId::RecoverBrushMotor:
        return run_with_gps_stuck_paused({{"stop_walk", &ports_.stop_walk},
                                          {"stop_brush", &ports_.stop_brush},
                                          {"stop_brush_executor", &ports_.stop_brush_executor},
                                          {"restart_brush_driver", &ports_.restart_brush_driver},
                                          {"start_brush_executor",
                                           &ports_.start_brush_executor}});
    case RecoveryPlanId::RecoverBms:
        return run_with_gps_stuck_paused({{"stop_walk", &ports_.stop_walk},
                                          {"stop_bms_executor", &ports_.stop_bms_executor},
                                          {"restart_bms_driver", &ports_.restart_bms_driver},
                                          {"start_bms_executor",
                                           &ports_.start_bms_executor}});
    case RecoveryPlanId::RecoverGps:
        return run_with_gps_stuck_paused({{"stop_walk", &ports_.stop_walk},
                                          {"stop_gps_executor", &ports_.stop_gps_executor},
                                          {"restart_gps_driver", &ports_.restart_gps_driver},
                                          {"start_gps_executor",
                                           &ports_.start_gps_executor}});
    case RecoveryPlanId::RecoverImu:
        return run_with_gps_stuck_paused({{"stop_walk", &ports_.stop_walk},
                                          {"stop_imu_executor", &ports_.stop_imu_executor},
                                          {"restart_imu_driver", &ports_.restart_imu_driver},
                                          {"start_imu_executor",
                                           &ports_.start_imu_executor}});
    case RecoveryPlanId::RecoverWalkStall:
    case RecoveryPlanId::RecoverGpsStuckReverse:
        return recover_reverse_motion();
    case RecoveryPlanId::RecoverAttitudeCenter:
        return recover_attitude_center();
    case RecoveryPlanId::None:
        return RecoveryResult{false, "no recovery plan"};
    }
    return RecoveryResult{false, "unknown recovery plan"};
}

}  // namespace robot::app
