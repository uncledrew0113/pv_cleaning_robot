/**
 * @file recovery_executor.cc
 * @brief 应用层恢复流程执行器实现。
 *
 * 本文件只按恢复计划调用外部端口，不直接判断硬件是否已经真正恢复。后续诊断快照会继续
 * 驱动 ErrorManager 做错误闭环。
 */
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

RecoveryResult RecoveryExecutor::recover_attitude_center_then_reverse() {
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
    missing = fail_if_missing(static_cast<bool>(ports_.reverse_walk_motion),
                              "reverse_walk_motion");
    if (!missing.ok) return missing;

    ports_.pause_gps_stuck();
    (void)ports_.stop_walk();
    (void)ports_.lower_attitude_center();
    (void)ports_.reverse_walk_motion();
    (void)ports_.stop_walk();
    ports_.resume_gps_stuck();
    return RecoveryResult{true, ""};
}

RecoveryResult RecoveryExecutor::execute(const RecoveryRequest& request) {
    switch (request.plan) {
    case RecoveryPlanId::RecoverAttitudeCenter:
        return recover_attitude_center();
    case RecoveryPlanId::RecoverAttitudeCenterThenReverse:
        return recover_attitude_center_then_reverse();
    case RecoveryPlanId::None:
        return RecoveryResult{false, "no recovery plan"};
    }
    return RecoveryResult{false, "unknown recovery plan"};
}

}  // namespace robot::app
