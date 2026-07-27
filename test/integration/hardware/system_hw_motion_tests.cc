/**
 * @file system_hw_motion_tests.cc
 * @brief 真实硬件运动链路系统测试。
 *
 * 本文件验证行走电机、滚刷和运动服务在任务段执行中的实机行为。测试会驱动机器人运动，
 * 运行前必须确认测试区域安全。
 */
#include "system_hw_common.h"

TEST_CASE("SafetyMonitor 空闲状态不误触发限位事件", "[safety][safety.idle]") {
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

TEST_CASE("N 趟完整任务链 + 全程持续采集健康数据",
          "[flow][flow.combined][manual][long]") {
    SystemHwFixture f;
    run_configured_system_chain(f, "hw_system][combined", kp.combined_passes, false, false, false);
}
