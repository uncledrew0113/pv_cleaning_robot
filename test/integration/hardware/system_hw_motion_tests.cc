#include "system_hw_common.h"

TEST_CASE("系统组合根初始化后处于 Idle", "[hw_system][full_init]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    CHECK(f.controller->snapshot().state == "Idle");

    std::this_thread::sleep_for(500ms);
    const auto imu = f.imu->get_latest();
    const auto odom = f.nav->get_fused_odometry();
    spdlog::info("[hw_system][full_init] imu_valid={} yaw={:.2f} pitch={:.2f} roll={:.2f}",
                 imu.valid,
                 imu.yaw_deg,
                 imu.pitch_deg,
                 imu.roll_deg);
    spdlog::info("[hw_system][full_init] odom valid={} top={:.3f} bottom={:.3f} fused={:.3f}",
                 odom.valid,
                 odom.top_distance_m,
                 odom.bottom_distance_m,
                 odom.fused_distance_m);
}

TEST_CASE("融合里程计在静止和低速运动时输出有效", "[hw_system][nav_fused_odometry]") {
    SystemHwFixture f;
    REQUIRE(f.init());

    std::this_thread::sleep_for(1500ms);
    f.nav->update();
    const auto idle = f.nav->get_fused_odometry();
    spdlog::info(
        "[hw_system][nav_fused_odometry] idle valid={} top={:.3f} bottom={:.3f} fused={:.3f} "
        "diff={:.3f}",
        idle.valid,
        idle.top_distance_m,
        idle.bottom_distance_m,
        idle.fused_distance_m,
        idle.distance_diff_m);

    CHECK(idle.valid);
    CHECK(std::isfinite(idle.top_distance_m));
    CHECK(std::isfinite(idle.bottom_distance_m));
    CHECK(std::isfinite(idle.fused_distance_m));
    CHECK(std::isfinite(idle.distance_diff_m));

    const auto target = robot::domain::opposite_endpoint(kp.primary_dock);
    REQUIRE(f.motion->start_segment(
        robot::domain::MissionSegment{target, robot::domain::SegmentMode::Cleaning}));
    const auto moving_deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < moving_deadline) {
        f.motion->update();
        f.nav->update();
        std::this_thread::sleep_for(50ms);
    }
    f.motion->emergency_stop();
    std::this_thread::sleep_for(500ms);

    const auto moving = f.nav->get_fused_odometry();
    spdlog::info(
        "[hw_system][nav_fused_odometry] moving valid={} top={:.3f} bottom={:.3f} fused={:.3f} "
        "diff={:.3f}",
        moving.valid,
        moving.top_distance_m,
        moving.bottom_distance_m,
        moving.fused_distance_m,
        moving.distance_diff_m);

    CHECK(moving.valid);
    CHECK(std::isfinite(moving.top_distance_m));
    CHECK(std::isfinite(moving.bottom_distance_m));
    CHECK(std::isfinite(moving.fused_distance_m));
    CHECK(std::isfinite(moving.distance_diff_m));
    CHECK(std::abs(moving.fused_distance_m - idle.fused_distance_m) > 0.02);
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
    std::atomic<int> unstable_count{0};
    bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const auto&) { ++settled_count; });
    bus.subscribe<robot::middleware::SafetyMonitor::LimitUnstableEvent>(
        [&](const auto&) { ++unstable_count; });

    robot::middleware::SafetyMonitor safety(
        [&]() { f.walk_group->emergency_override(0.0f); }, f.left_sw, f.right_sw, bus);
    REQUIRE(safety.start());
    std::this_thread::sleep_for(2s);
    safety.stop();

    CHECK(settled_count.load() == 0);
    CHECK(unstable_count.load() == 0);
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

    f.controller->post_fault(robot::app::FaultFact{robot::app::FaultSource::Watchdog,
                                                   robot::domain::FaultCode::kCanCommunicationLost,
                                                   "hw_p0"});
    f.controller->drain_for_test();

    CHECK(f.controller->snapshot().state == "FaultStopped");
    CHECK(f.controller->snapshot().fault == robot::domain::FaultCode::kCanCommunicationLost);
    const auto reset = f.controller->submit_command(
        robot::domain::RobotCommand{robot::domain::RobotCommandKind::FaultReset,
                                    robot::domain::CommandSource::Rpc,
                                    "hw-reset"});
    REQUIRE(reset.accepted);
    CHECK(f.controller->snapshot().state == "Idle");
    CHECK_FALSE(f.fault->has_active_fault());
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

TEST_CASE("N 趟完整任务链 + 全程持续采集健康数据", "[hw_system][combined]") {
    SystemHwFixture f;
    run_configured_system_chain(f, "hw_system][combined", kp.combined_passes, false, false, false);
}

TEST_CASE("完整任务链 + 融合里程计日志", "[hw_system][combined_nvm_real]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][combined_nvm_real", kp.combined_passes, false, true, false);
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
