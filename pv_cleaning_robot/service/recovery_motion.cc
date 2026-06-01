#include "pv_cleaning_robot/service/recovery_motion.h"

#include <cmath>

namespace robot::service {

void RecoveryMotion::start() {
    phase_ = Phase::Stop;
    attempts_ = 0;
}

RecoveryMotion::Result RecoveryMotion::step() {
    switch (phase_) {
    case Phase::Idle:
        return Result::Failed;
    case Phase::Stop:
        phase_ = Phase::Stabilizing;
        return Result::Running;
    case Phase::Stabilizing:
        phase_ = Phase::Sampling;
        return Result::Running;
    case Phase::Sampling:
        if (std::abs(pose_error_deg_) <= 0.5f) {
            phase_ = Phase::Done;
            return Result::Done;
        }
        phase_ = Phase::MicroMoving;
        return Result::Running;
    case Phase::MicroMoving:
        ++attempts_;
        phase_ = Phase::Verifying;
        return Result::Running;
    case Phase::Verifying:
        if (std::abs(pose_error_deg_) <= 0.5f) {
            phase_ = Phase::Done;
            return Result::Done;
        }
        if (attempts_ >= max_attempts_) {
            phase_ = Phase::Failed;
            return Result::Failed;
        }
        phase_ = Phase::Sampling;
        return Result::Running;
    case Phase::Done:
        return Result::Done;
    case Phase::Failed:
        return Result::Failed;
    }
    return Result::Failed;
}

RecoveryMotion::Phase RecoveryMotion::phase() const noexcept {
    return phase_;
}

void RecoveryMotion::set_pose_error_deg(float value) noexcept {
    pose_error_deg_ = value;
}

void RecoveryMotion::set_max_attempts(uint32_t value) noexcept {
    max_attempts_ = value == 0 ? 1 : value;
}

}  // namespace robot::service
