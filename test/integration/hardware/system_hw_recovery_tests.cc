#include "system_hw_common.h"

#include "pv_cleaning_robot/app/recovery_executor.h"

namespace {

robot::app::RecoveryRequest recovery_request(robot::app::RecoveryPlanId plan,
                                             robot::app::ComponentKind kind) {
    return robot::app::RecoveryRequest{plan, robot::app::ComponentId{kind, 0}};
}

bool wait_walk_feedback(SystemHwFixture& f, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        f.walk_group->update();
        const auto diag = f.walk_group->get_group_diagnostics();
        bool ready = true;
        for (const auto& wheel : diag.wheel) {
            ready = ready && wheel.feedback_frame_count > 0;
        }
        if (ready) {
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

bool wait_gps_sentence(SystemHwFixture& f, uint32_t previous, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (f.gps->get_diagnostics().sentence_count > previous) {
            return true;
        }
        std::this_thread::sleep_for(200ms);
    }
    return false;
}

bool wait_imu_frame(SystemHwFixture& f, uint32_t previous, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (f.imu->get_diagnostics().frame_count > previous) {
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

bool wait_bms_update(SystemHwFixture& f, uint32_t previous, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        f.bms->update();
        if (f.bms->get_diagnostics().update_count > previous) {
            return true;
        }
        std::this_thread::sleep_for(200ms);
    }
    return false;
}

robot::app::RecoveryExecutor::Ports make_non_moving_recovery_ports(SystemHwFixture& f) {
    robot::app::RecoveryExecutor::Ports ports;
    ports.pause_gps_stuck = [&f] { f.gps_stuck->set_monitoring_enabled(false); };
    ports.resume_gps_stuck = [&f] { f.gps_stuck->set_monitoring_enabled(true); };
    ports.stop_walk = [&f] {
        f.motion->emergency_stop();
        return true;
    };

    ports.stop_walk_executor = [] { return true; };
    ports.restart_walk_driver = [&f] {
        f.walk_group->close();
        if (f.walk_group->open() != robot::device::DeviceError::OK) {
            return false;
        }
        return f.walk_group->set_feedback_mode_all(10u) == robot::device::DeviceError::OK;
    };
    ports.start_walk_executor = [] { return true; };

    ports.stop_bms_executor = [] { return true; };
    ports.restart_bms_driver = [&f] {
        f.bms->close();
        return f.bms->open() == robot::device::DeviceError::OK;
    };
    ports.start_bms_executor = [] { return true; };

    ports.stop_gps_executor = [] { return true; };
    ports.restart_gps_driver = [&f] {
        f.gps->close();
        return f.gps->open();
    };
    ports.start_gps_executor = [] { return true; };

    ports.stop_imu_executor = [] { return true; };
    ports.restart_imu_driver = [&f] {
        f.imu->close();
        if (!f.imu->open()) {
            return false;
        }
        return f.imu->set_output_rate(100) == robot::device::DeviceError::OK;
    };
    ports.start_imu_executor = [] { return true; };

    return ports;
}

}  // namespace

TEST_CASE("RecoveryExecutor 真实硬件驱动恢复后数据流仍可用",
          "[hw_system][recovery_driver_smoke]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    REQUIRE(wait_walk_feedback(f, 3s));
    REQUIRE(wait_bms_update(f, f.bms->get_diagnostics().update_count, 3s));
    REQUIRE(wait_gps_sentence(f, f.gps->get_diagnostics().sentence_count, 5s));
    REQUIRE(wait_imu_frame(f, f.imu->get_diagnostics().frame_count, 2s));

    robot::app::RecoveryExecutor executor(make_non_moving_recovery_ports(f));

    auto result = executor.execute(recovery_request(
        robot::app::RecoveryPlanId::RecoverWalkMotorGroup,
        robot::app::ComponentKind::WalkMotorGroup));
    REQUIRE(result.ok);
    CHECK(wait_walk_feedback(f, 3s));

    const auto gps_before = f.gps->get_diagnostics().sentence_count;
    result = executor.execute(
        recovery_request(robot::app::RecoveryPlanId::RecoverGps, robot::app::ComponentKind::Gps));
    REQUIRE(result.ok);
    CHECK(wait_gps_sentence(f, gps_before, 5s));

    const auto bms_before = f.bms->get_diagnostics().update_count;
    result = executor.execute(
        recovery_request(robot::app::RecoveryPlanId::RecoverBms, robot::app::ComponentKind::Bms));
    REQUIRE(result.ok);
    CHECK(wait_bms_update(f, bms_before, 5s));

    const auto imu_before = f.imu->get_diagnostics().frame_count;
    result = executor.execute(
        recovery_request(robot::app::RecoveryPlanId::RecoverImu, robot::app::ComponentKind::Imu));
    REQUIRE(result.ok);
    CHECK(wait_imu_frame(f, imu_before, 3s));
}
