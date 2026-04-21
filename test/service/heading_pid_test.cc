/**
 * HeadingPidController 单元测试（速率 PID 版本）
 * [service][heading_pid]
 *
 * 测试策略：
 *   - 纯软件，无硬件依赖
 *   - 覆盖：使能/禁用、死区、方向约定（CCW/CW）、限幅、积分、reset() 等
 *   - 验证正返程对称性（相同 omega_z 输入得到相同 correction）
 */
#include <catch2/catch.hpp>
#include <cmath>

#include "pv_cleaning_robot/service/heading_pid_controller.h"

using robot::service::HeadingPidController;

// ── 辅助：kp-only 参数（ki=kd=0，deadband=0，便于精确验证）─────────────────────────
static HeadingPidController::Params kp_only(float kp, float max_out = 100.0f) {
    HeadingPidController::Params p;
    p.kp = kp;
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.max_output = max_out;
    p.integral_limit = 50.0f;
    p.deadband_rate_dps = 0.0f;
    return p;
}

// ── 基础行为 ──────────────────────────────────────────────────────────────────

TEST_CASE("HeadingPidController: 未使能时 compute() 始终返回 0", "[service][heading_pid]") {
    HeadingPidController pid;
    REQUIRE(pid.compute(0.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.compute(10.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.compute(-10.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.is_enabled() == false);
}

TEST_CASE("HeadingPidController: 死区内返回 0", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_rate_dps = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    // |omega_z| < deadband → 0
    REQUIRE(pid.compute(1.5f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.compute(-1.5f, 0.02f) == Approx(0.0f));
    // |omega_z| == deadband（≤ 判断）→ 0
    REQUIRE(pid.compute(2.0f, 0.02f) == Approx(0.0f));
    REQUIRE(pid.compute(-2.0f, 0.02f) == Approx(0.0f));
}

TEST_CASE("HeadingPidController: CCW旋转（omega_z>0）→ correction<0（CW纠偏）",
          "[service][heading_pid]") {
    HeadingPidController pid(kp_only(2.0f));
    pid.enable(true);

    // omega_z = +10 deg/s (CCW) → error = -10 → correction = 2 * (-10) = -20
    float c = pid.compute(10.0f, 0.0f);
    REQUIRE(c < 0.0f);
    REQUIRE(c == Approx(-20.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: CW旋转（omega_z<0）→ correction>0（CCW纠偏）",
          "[service][heading_pid]") {
    HeadingPidController pid(kp_only(2.0f));
    pid.enable(true);

    // omega_z = -10 deg/s (CW) → error = +10 → correction = 2 * 10 = +20
    float c = pid.compute(-10.0f, 0.0f);
    REQUIRE(c > 0.0f);
    REQUIRE(c == Approx(20.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 输出限幅", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(10.0f, 30.0f));
    pid.enable(true);

    // omega_z = 100 → err = -100 → kp*err = -1000 → clamped to -30
    REQUIRE(pid.compute(100.0f, 0.0f) == Approx(-30.0f).margin(0.01f));
    // omega_z = -100 → err = +100 → kp*err = +1000 → clamped to +30
    REQUIRE(pid.compute(-100.0f, 0.0f) == Approx(30.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 正返程对称（相同 omega_z 给出相同 correction）",
          "[service][heading_pid]") {
    // 正向和返程机器人面对相反方向，但 omega_z 的物理意义不变。
    // 同一 omega_z 输入在两次独立 PID 实例上应产生相同的 correction。
    HeadingPidController pid1(kp_only(2.0f));
    HeadingPidController pid2(kp_only(2.0f));
    pid1.enable(true);
    pid2.enable(true);

    float c1 = pid1.compute(5.0f, 0.02f);
    float c2 = pid2.compute(5.0f, 0.02f);
    REQUIRE(c1 == Approx(c2).margin(0.001f));
    REQUIRE(c1 < 0.0f);  // omega_z > 0 → correction < 0
}

// ── 积分行为 ──────────────────────────────────────────────────────────────────

TEST_CASE("HeadingPidController: 积分累积", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_rate_dps = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    // omega_z = 5 → err = -5，dt=1s: integral += -5 per step
    pid.compute(5.0f, 1.0f);  // integral = -5
    pid.compute(5.0f, 1.0f);  // integral = -10
    // dt=0, not increasing; output = ki * integral = 1 * (-10) = -10
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(-10.0f).margin(0.1f));
}

TEST_CASE("HeadingPidController: 积分限幅", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 10.0f;
    p.deadband_rate_dps = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    // 连续多次积分，应被限幅到 -10（integral_limit）
    for (int i = 0; i < 20; ++i)
        pid.compute(5.0f, 1.0f);  // err = -5 per step

    float c = pid.compute(5.0f, 0.0f);  // dt=0，不再增加
    REQUIRE(c == Approx(-10.0f).margin(0.1f));
}

TEST_CASE("HeadingPidController: 误差过零时清零积分", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_rate_dps = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    // omega_z = +5 → err = -5，积分两次
    pid.compute(5.0f, 1.0f);  // integral = -5
    pid.compute(5.0f, 1.0f);  // integral = -10

    // omega_z 反向 → err 符号翻转 → 触发积分清零
    float c = pid.compute(-5.0f, 0.0f);  // err=+5, sign change → integral = 0
    // correction = kp*err + ki*0 = 0 + 0 = 0（kp=0）
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 死区内积分不累积", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_rate_dps = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    // 在死区内积分不应累积
    for (int i = 0; i < 10; ++i)
        pid.compute(1.0f, 1.0f);  // |omega_z|=1 < deadband=2

    // 退出死区（omega_z=5），dt=0 不再增加积分，correction=0（积分为0）
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

// ── 使能/禁用/复位 ────────────────────────────────────────────────────────────

TEST_CASE("HeadingPidController: enable(false) 返回 0 并禁用", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(1.0f));
    pid.enable(true);
    pid.compute(10.0f, 0.02f);

    pid.enable(false);
    REQUIRE(pid.is_enabled() == false);
    REQUIRE(pid.compute(10.0f, 0.02f) == Approx(0.0f));
}

TEST_CASE("HeadingPidController: enable(false) 后重新使能正常工作", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(2.0f));
    pid.enable(true);
    pid.compute(10.0f, 0.02f);
    pid.enable(false);

    pid.enable(true);
    // omega_z = +5 → err = -5 → correction = 2 * (-5) = -10
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(-10.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: reset() 不改变使能状态", "[service][heading_pid]") {
    HeadingPidController pid(kp_only(1.0f));
    pid.enable(true);
    pid.compute(5.0f, 0.02f);

    pid.reset();
    REQUIRE(pid.is_enabled() == true);
}

TEST_CASE("HeadingPidController: reset() 清零积分", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_rate_dps = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    pid.compute(5.0f, 1.0f);  // integral = -5

    pid.reset();

    // 积分清零后，dt=0 时 correction = ki*0 + kp*(-5) = 0（kp=0）
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: set_params 热更新不复位积分", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_rate_dps = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);

    pid.compute(5.0f, 1.0f);  // integral = -5

    // 热更新 kp=2，积分不清零
    HeadingPidController::Params p2 = p;
    p2.kp = 2.0f;
    pid.set_params(p2);

    // dt=0: correction = kp*err + ki*integral = 2*(-5) + 1*(-5) = -15
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(-15.0f).margin(0.5f));
}
