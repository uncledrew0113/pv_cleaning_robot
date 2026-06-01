#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/recovery_motion.h"

using robot::service::RecoveryMotion;

TEST_CASE("RecoveryMotion progresses stop sample micro-move verify done", "[service][recovery]") {
    RecoveryMotion recovery;
    recovery.start();

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Stabilizing);

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Sampling);

    recovery.set_pose_error_deg(3.0f);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::MicroMoving);

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Verifying);

    recovery.set_pose_error_deg(0.3f);
    CHECK(recovery.step() == RecoveryMotion::Result::Done);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Done);
}

TEST_CASE("RecoveryMotion fails after retry limit", "[service][recovery]") {
    RecoveryMotion recovery;
    recovery.set_max_attempts(2);
    recovery.start();
    recovery.set_pose_error_deg(8.0f);

    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Running);
    CHECK(recovery.step() == RecoveryMotion::Result::Failed);
    CHECK(recovery.phase() == RecoveryMotion::Phase::Failed);
}
