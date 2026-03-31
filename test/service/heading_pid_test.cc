/**
 * HeadingPidController 单元测试
 * [service][heading_pid]
 *
 * 测试策略：
 *   - 纯软件，无硬件依赖
 *   - 覆盖：使能/禁用、自动初始化、PID 计算、限幅、reset()、方向约定
 */
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/heading_pid_controller.h"

using robot::service::HeadingPidController;

// ── 辅助：kp-only 参数（ki=kd=0，便于精确验证）─────────────────────────
static HeadingPidController::Params kp_only(float kp, float max_out = 100.0f) {
    HeadingPidController::Params p;
    p.kp             = kp;
    p.ki             = 0.0f;
    p.kd             = 0.0f;
    p.max_output     = max_out;
    p.integral_limit = 50.0f;
    return p;
}

// ── 基础行为 ──────────────────────────────────────────────────────────────

TEST_CASE("HeadingPidController: 未使能时 compute() 始终返回 0", "[service][heading_pid]") {
    HeadingPidController pid;
    REQUIRE(pid.compute(0.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.compute(30.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.is_enabled() == false);
}

TEST_CASE("HeadingPidController: 使能后首次 compute() 自动锁定当前航向为目标", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(1.0f));
    pid.enable(true);
    // 首次调用时 yaw=45°，应自动锁定 target=45°，误差=0，correction=0
    float correction = pid.compute(45.0f, 0.02f);
    REQUIRE(correction == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 比例项方向正确", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(2.0f));
    pid.enable(true);
    pid.set_target(10.0f);   // 目标 10°

    // yaw=0°，误差=+10°，correction 应为正（偏右，左轮加速）
    float c = pid.compute(0.0f, 0.0f);  // dt=0 → 微分项为0
    REQUIRE(c > 0.0f);
    REQUIRE(c == Approx(2.0f * 10.0f).margin(0.01f));  // kp*err = 2*10 = 20
}

TEST_CASE("HeadingPidController: 输出限幅", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(10.0f, 30.0f);  // max_output=30
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(90.0f);

    // err=90°，kp*err=900，应被限幅到 30
    float c = pid.compute(0.0f, 0.0f);
    REQUIRE(c == Approx(30.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 跨 ±180° 边界的角度规范化", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(1.0f));
    pid.enable(true);
    pid.set_target(170.0f);

    // yaw=-170°，误差应为 170-(-170)=340°，规范化后为 -20°（最短路径）
    float c = pid.compute(-170.0f, 0.0f);
    REQUIRE(c == Approx(-20.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: set_target 重置积分", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 积累一些积分
    pid.compute(5.0f, 1.0f);  // err=-5, integral=-5
    pid.compute(5.0f, 1.0f);  // integral=-10

    // set_target 应清零积分
    pid.set_target(0.0f);
    float c = pid.compute(0.0f, 0.0f);  // 误差=0，积分=0，correction应为0
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: enable(false) 重置积分并返回 0", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 1.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.max_output = 100.0f; p.integral_limit = 50.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(20.0f);
    pid.compute(0.0f, 0.02f);  // 产生修正量

    pid.enable(false);
    REQUIRE(pid.is_enabled() == false);
    REQUIRE(pid.compute(0.0f, 0.02f) == Approx(0.0f));

    // 重新使能后应自动重新初始化（因为 enable(false) 清了 initialized_）
    pid.enable(true);
    float c = pid.compute(5.0f, 0.0f);  // 首次调用，自动锁定 target=5°，err=0
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: reset() 不改变使能状态", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(1.0f));
    pid.enable(true);
    pid.set_target(10.0f);
    pid.compute(0.0f, 0.02f);

    pid.reset();
    REQUIRE(pid.is_enabled() == true);

    // 重置后首次调用应重新自动初始化
    float c = pid.compute(20.0f, 0.0f);  // 锁定 target=20°，err=0
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 积分限幅", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f;
    p.integral_limit = 10.0f;  // 积分最大 ±10
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 每次 dt=1s，err=5°，积分应被限幅到 10
    for (int i = 0; i < 5; ++i)
        pid.compute(5.0f, 1.0f);  // err=-5 per step → integral accumulating negative

    // 再积累一次，确认限幅生效
    float c = pid.compute(5.0f, 0.0f);  // 仅积分项 = ki * integral，但 dt=0 不再增加
    // 积分应被限幅到 -10（5次 * (-5) * 1s = -25，限幅到 -10）
    // correction = ki * integral = 1 * (-10) = -10
    REQUIRE(c == Approx(-10.0f).margin(0.5f));
}

TEST_CASE("HeadingPidController: set_params 热更新不复位积分", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(1.0f);
    p.ki = 1.0f;
    p.integral_limit = 1000.0f;
    p.max_output = 1000.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 积累积分
    pid.compute(5.0f, 1.0f);  // integral=-5

    // 热更新 kp，积分不应清零
    HeadingPidController::Params p2 = p;
    p2.kp = 2.0f;
    pid.set_params(p2);

    // 再一次 compute（dt=0，不再增积分），correction = kp*err + ki*integral
    // err = 0-5 = -5, correction = 2*(-5) + 1*(-5) = -15
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(-15.0f).margin(0.5f));
}
