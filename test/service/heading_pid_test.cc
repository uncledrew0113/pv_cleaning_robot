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

    // yaw=0°，误差=+10°，correction 应为正（偏左 yaw < target，上轨加速/下轨减速）
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

// ── 新增：积分 Anti-Windup 行为验证 ──────────────────────────────────────────

TEST_CASE("HeadingPidController: 死区内不累积积分（Fix 2）", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    p.deadband_deg = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 在死区内调用10次，误差=1° < 2°（死区内）
    for (int i = 0; i < 10; ++i)
        pid.compute(1.0f, 1.0f);  // err=-1°, 在死区内，积分不应累积

    // 退出死区：err=-3°，如果积分为0，correction = ki*0 + kp*(-3) = 0
    // 如果积分错误累积了 10 × (-1) × 1 = -10，则 correction = 1*(-10) = -10（会失败）
    p.kp = 0.0f;  // 关掉 P 项，只看积分贡献
    pid.set_params(p);
    float c = pid.compute(3.0f, 0.0f);  // err=-3°（超出死区），dt=0，积分不再增加
    // 积分应为 0（死区内未累积），correction = 0
    REQUIRE(c == Approx(0.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 误差过零时清零积分（Fix 1）", "[service][heading_pid]") {
    HeadingPidController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.max_output = 1000.0f; p.integral_limit = 1000.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 先积累正向积分（err > 0）
    pid.compute(-5.0f, 1.0f);  // err = 0-(-5) = +5，积分 = +5
    pid.compute(-5.0f, 1.0f);  // err = +5，积分 = +10

    // 误差从正（+5）翻转为负（-5）：过零触发积分清零
    float c = pid.compute(5.0f, 0.0f);  // err = 0-5 = -5（与前一帧+5异号）
    // 积分应已清零，correction = ki*0 + kp*(-5) = 0（kp=0）
    REQUIRE(c == Approx(0.0f).margin(0.01f));

    // 再验证：过零后积分从0重新累积（1次 err=-5, dt=1s → integral=-5）
    float c2 = pid.compute(5.0f, 1.0f);  // err=-5, no sign change, integral=-5
    REQUIRE(c2 == Approx(-5.0f).margin(0.1f));
}


TEST_CASE("HeadingPidController: deadband=0 时行为不变", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_deg = 0.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // |err|=5°，deadband=0 → 正常计算，correction = kp*err = 2*(-5) = -10
    float c = pid.compute(5.0f, 0.0f);  // err = 0 - 5 = -5
    REQUIRE(c != Approx(0.0f).margin(0.01f));
    REQUIRE(c == Approx(-10.0f).margin(0.01f));
}

TEST_CASE("HeadingPidController: 误差在死区内返回 0", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_deg = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // |err|=1.5° < 2.0° → correction = 0
    float c = pid.compute(1.5f, 0.0f);
    REQUIRE(c == Approx(0.0f).margin(0.001f));
}

TEST_CASE("HeadingPidController: 误差恰好等于死区边界返回 0", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_deg = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // |err|=2.0° == deadband（≤ 判断）→ correction = 0
    float c = pid.compute(2.0f, 0.0f);
    REQUIRE(c == Approx(0.0f).margin(0.001f));
}

TEST_CASE("HeadingPidController: 超出死区范围正常计算", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_deg = 2.0f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // |err|=5° > 2.0° → correction = kp * err = 2 * (-5) = -10
    float c = pid.compute(5.0f, 0.0f);
    REQUIRE(c == Approx(-10.0f).margin(0.01f));
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

// ─── 自适应目标跟踪测试 ──────────────────────────────────────────────────────

TEST_CASE("HeadingPidController: alpha=1.0（禁用跟踪）目标保持不变", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f);
    p.deadband_deg = 0.0f;
    p.target_tracking_alpha = 1.0f;  // 禁用跟踪
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // 调用 100 次 compute，yaw=10°；目标不动，err 始终为 norm(0-10)=-10°
    float last_c = 0.0f;
    for (int i = 0; i < 100; ++i) {
        last_c = pid.compute(10.0f, 0.02f);
    }
    // kp=2, err≈-10 → correction≈-20（被 max_output=100 限制在 -20）
    REQUIRE(last_c == Approx(-20.0f).margin(1.0f));
}

TEST_CASE("HeadingPidController: alpha=0.99 目标缓慢跟踪 yaw（误差指数衰减）", "[service][heading_pid]") {
    HeadingPidController::Params p = kp_only(2.0f, 1000.0f);  // 大 max_output 避免限幅
    p.ki = 0.0f;  // 纯 P，便于验证
    p.kd = 0.0f;
    p.deadband_deg = 0.0f;
    p.target_tracking_alpha = 0.99f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(0.0f);

    // yaw 突变至 10°，alpha=0.99，每步 target += 0.01*(10-target)
    // 经 n 步后 err = 10° × 0.99^n（指数衰减）

    // 第 1 步：target += 0.01*(10-0) = 0.1°，err = 0.1-10 = -9.9°
    float c1 = pid.compute(10.0f, 0.0f);
    REQUIRE(c1 == Approx(2.0f * (-9.9f)).margin(0.05f));  // kp * err = 2 * (-9.9)

    // 经 100 步：err ≈ 10° × 0.99^100 ≈ 3.66°，误差应明显衰减
    for (int i = 0; i < 99; ++i) {
        pid.compute(10.0f, 0.0f);
    }
    float c100 = pid.compute(10.0f, 0.0f);
    // err ≈ -10° × 0.99^100 ≈ -3.66° → correction ≈ 2 * (-3.66) = -7.32
    REQUIRE(c100 < -5.0f);   // 已明显衰减
    REQUIRE(c100 > -10.0f);  // 但尚未归零（未到 deadband）
}

TEST_CASE("HeadingPidController: alpha=0.99 对返程漂移误差有界（防脱轨）", "[service][heading_pid]") {
    // 模拟返程场景：target=141°（由 set_target 设），yaw 以 0.3°/s 从 143° 缓慢降至 134°
    // 共 90 帧 × 0.1s/帧（慢速测试帧率）= 9s，yaw = 143 - 0.1°/帧
    HeadingPidController::Params p = kp_only(1.5f, 10.0f);
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.deadband_deg = 0.5f;
    p.target_tracking_alpha = 0.99f;
    HeadingPidController pid(p);
    pid.enable(true);
    pid.set_target(141.0f);  // 返程初始目标 = 正向终点 yaw

    float max_correction = 0.0f;
    float yaw = 143.0f;
    for (int i = 0; i < 90; ++i) {
        yaw -= 0.1f;  // 每帧下降 0.1°（模拟轨道几何复位力）
        float c = pid.compute(yaw, 0.1f);  // dt = 0.1s（测试帧率）
        if (std::abs(c) > max_correction)
            max_correction = std::abs(c);
    }

    // 自适应跟踪后，最大纠偏力远小于旧代码的 10 RPM 上限
    // 旧代码（alpha=1.0）：err 最大达 7°，correction=10 RPM（被 max_output 截断）
    // 新代码（alpha=0.99）：稳态误差 ≈ 0.6° → correction ≈ 0.9 RPM → 远低于上限
    REQUIRE(max_correction < 5.0f);  // 验证纠偏力有界（不触发脱轨量级的差速）
}
