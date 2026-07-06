/**
 * @file system_hw_motion_tests.cc
 * @brief 真实硬件运动链路系统测试。
 *
 * 本文件验证行走电机、滚刷和运动服务在任务段执行中的实机行为。测试会驱动机器人运动，
 * 运行前必须确认测试区域安全。
 */
#include "system_hw_common.h"

TEST_CASE("系统组合根初始化后处于 Idle", "[hw_system][full_init]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    CHECK(f.controller->snapshot().state == "Idle");

    std::this_thread::sleep_for(500ms);
    const auto imu = f.imu->get_latest();
    f.gps_stuck->update();
    const auto gps_stuck = f.gps_stuck->get_status();
    spdlog::info("[hw_system][full_init] imu_valid={} yaw={:.2f} pitch={:.2f} roll={:.2f}",
                 imu.valid,
                 imu.yaw_deg,
                 imu.pitch_deg,
                 imu.roll_deg);
    spdlog::info("[hw_system][full_init] gps_stuck_state={} stuck={} reason={}",
                 static_cast<int>(gps_stuck.state),
                 gps_stuck.robot_stuck_detected,
                 gps_stuck.reason);
}

TEST_CASE("HealthService DIAGNOSTICS 落盘真实传感器数据", "[hw_system][health_real_data]") {
    const std::filesystem::path path = kp.health_jsonl_path;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    remove_rotated_health_logs(path);

    SystemHwFixture f;
    REQUIRE(f.init(false, false, path.string()));
    REQUIRE(f.health != nullptr);
    for (int i = 0; i < 10; ++i) {
        f.bms->update();
        f.walk_group->update();
        std::this_thread::sleep_for(100ms);
    }
    for (int i = 0; i < 5; ++i) {
        f.health->update();
        std::this_thread::sleep_for(50ms);
    }

    require_diagnostics_health_log(path);
}

TEST_CASE("SafetyMonitor 空闲状态不误触发限位事件", "[hw_system][safety_idle]") {
    hw::DeviceFixture f;
    REQUIRE(f.left_sw->open(0, 2, 0, false));
    REQUIRE(f.right_sw->open(0, 2, 0, false));

    robot::middleware::EventBus bus;
    std::atomic<int> settled_count{0};
    bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const auto&) { ++settled_count; });

    robot::middleware::SafetyMonitor safety(
        [&]() { f.walk_group->emergency_override(0.0f); }, f.left_sw, f.right_sw, bus);
    REQUIRE(safety.start());
    std::this_thread::sleep_for(2s);
    safety.stop();

    CHECK(settled_count.load() == 0);
}

TEST_CASE("运动 1s 后 emergency_override 急停", "[hw_system][motion_then_stop]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);
    std::this_thread::sleep_for(300ms);
    REQUIRE(f.walk_group->set_speed_uniform(kp.test_speed_rpm) == robot::device::DeviceError::OK);

    for (int i = 0; i < 4; ++i) {
        f.walk_group->update();
        std::this_thread::sleep_for(250ms);
    }

    REQUIRE(f.walk_group->emergency_override(0.0f) == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->is_override_active());
    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    f.walk_group->update();
    std::this_thread::sleep_for(200ms);
    CHECK(f.walk_group->get_group_diagnostics().ctrl_frame_count == frames_before);

    std::this_thread::sleep_for(500ms);
    const auto diag = f.walk_group->get_group_diagnostics();
    CHECK(std::abs(diag.wheel[0].speed_rpm) < 5.0f);
    CHECK(std::abs(diag.wheel[1].speed_rpm) < 5.0f);

    f.walk_group->clear_override();
    f.walk_group->update();
    CHECK_FALSE(f.walk_group->is_override_active());
}
TEST_CASE("WatchdogMgr 超时后触发回调", "[hw_system][watchdog_timeout]") {
    robot::app::WatchdogMgr watchdog;
    std::atomic<bool> timed_out{false};
    watchdog.set_timeout_callback([&](const std::string&) { timed_out.store(true); });
    REQUIRE(watchdog.start());
    const int ticket = watchdog.register_thread("hw_system_watchdog", 100);
    REQUIRE(ticket >= 0);
    std::this_thread::sleep_for(300ms);
    watchdog.stop();
    CHECK(timed_out.load());
}

TEST_CASE("WatchdogMgr 正常心跳不触发超时", "[hw_system][watchdog_heartbeat]") {
    robot::app::WatchdogMgr watchdog;
    std::atomic<bool> timed_out{false};
    watchdog.set_timeout_callback([&](const std::string&) { timed_out.store(true); });
    REQUIRE(watchdog.start());
    const int ticket = watchdog.register_thread("hw_system_watchdog", 500);
    REQUIRE(ticket >= 0);

    for (int i = 0; i < 5; ++i) {
        watchdog.heartbeat(ticket);
        std::this_thread::sleep_for(200ms);
    }

    watchdog.stop();
    CHECK_FALSE(timed_out.load());
}

TEST_CASE("P0 故障链路急停并由云端复位回 Idle", "[hw_system][p0_fault_chain]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    f.position_state = robot::domain::PositionState::OnSegment;
    REQUIRE(f.start_directional_to_opposite().accepted);
    REQUIRE(f.controller->snapshot().state == "ExecutingMission");

    robot::app::ErrorDecision decision;
    decision.action = robot::app::ErrorAction::FaultStopped;
    decision.latch_fault = true;
    decision.root_error.code = robot::app::ErrorCode::AttitudeLimitBoth;
    f.controller->apply_error_decision(decision);

    CHECK(f.controller->snapshot().state == "FaultStopped");
    CHECK(f.controller->snapshot().fault ==
          robot::domain::FaultCode::kAttitudeLimitBoth);
    const auto reset = f.controller->submit_command(
        robot::domain::RobotCommand{robot::domain::RobotCommandKind::FaultReset,
                                    robot::domain::CommandSource::Rpc,
                                    "hw-reset"});
    REQUIRE(reset.accepted);
    CHECK(f.controller->snapshot().state == "Idle");
    CHECK_FALSE(f.controller->snapshot().fault.has_value());
}

TEST_CASE("配置完整任务通过真实限位闭环完成", "[hw_system][n1_clean_cycle]") {
    SystemHwFixture f;
    f.repeat_count = 1;
    REQUIRE(f.init());
    REQUIRE(f.start_safety_bridge());
    REQUIRE(f.start_configured_assuming_primary_dock().accepted);
    CHECK(f.controller->snapshot().state == "ExecutingMission");
    CHECK(f.wait_until_state("Idle", std::chrono::seconds(kp.limit_timeout_sec * 2)));
}

TEST_CASE("回停机位并通过辅助停机接近稳定对齐", "[hw_system][dock_align]") {
    SystemHwFixture f;
    run_dock_align_test(f);
}

TEST_CASE("N 趟完整任务链 + 全程持续采集健康数据", "[hw_system][combined]") {
    SystemHwFixture f;
    run_configured_system_chain(f, "hw_system][combined", kp.combined_passes, false, false, false);
}

TEST_CASE("完整任务链 + 姿态极限触发后回中恢复", "[hw_system][combined_attitude_recover]") {
    SystemHwFixture f;
    run_configured_system_chain_with_attitude_recovery(
        f, "hw_system][combined_attitude_recover", kp.combined_passes);
}

TEST_CASE("完整任务链 + NVM 真实参数", "[hw_system][combined_nvm_real]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][combined_nvm_real", kp.combined_passes, false, false, false);
}

TEST_CASE("N 趟完整任务链 + 真实滚刷 + 全程持续采集健康数据", "[hw_system][combined_brush_real]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][combined_brush_real", kp.combined_passes, true, false, false);
}

TEST_CASE("视觉 PID 完整任务链 + 真实滚刷", "[hw_system][pid_combined]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][pid_combined", kp.combined_passes, true, false, true);
}

TEST_CASE("仅 IMU/GPS/HealthService 持续采集并本地落盘", "[hw_system][imu_gps_health_only]") {
    const std::filesystem::path path = std::filesystem::path(kp.health_jsonl_path)
                                           .replace_filename("hw_imu_gps_health_only.jsonl");
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::filesystem::remove(path);

    hw::ImuGpsHealthFixture f;
    REQUIRE(f.init(path.string()));
    for (int i = 0; i < 20; ++i) {
        f.health->update();
        std::this_thread::sleep_for(100ms);
    }

    require_health_log_written(path);
}
