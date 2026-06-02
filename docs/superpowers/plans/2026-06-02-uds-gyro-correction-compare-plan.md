# UDS Gyro Correction Compare Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add configurable UDS+gyro correction comparison hardware tests while preserving the existing production default correction behavior.

**Architecture:** Add reusable but default-disabled correction features to `HeadingCorrector`: optional UDS+gyro Kalman fusion, optional slow-on-error base RPM, and optional wheel correction mode. Refactor the large hardware system test into fixture, metrics, motion tests, and attitude/correction tests so the new comparison cases can run the same mission-chain loop as `[hw_system][pid_combined]`.

**Tech Stack:** C++17, Catch2, RapidJSON, spdlog, existing CMake `unit_tests` and `hw_tests` targets.

---

## File Structure

- Create `include/pv_cleaning_robot/service/uds_gyro_yaw_fusion.h`
  - Owns the 1D Kalman filter for UDS angle + IMU `gyro_z`.
- Create `pv_cleaning_robot/service/uds_gyro_yaw_fusion.cc`
  - Implements prediction, UDS update, gyro-only timeout, and reset.
- Create `test/service/uds_gyro_yaw_fusion_test.cc`
  - Pure unit tests for sign convention, update behavior, and timeout.
- Modify `include/pv_cleaning_robot/service/heading_corrector.h`
  - Add config fields for fusion and correction strategy, plus optional IMU gyro input.
- Modify `pv_cleaning_robot/service/heading_corrector.cc`
  - Use optional fusion output as PID angle source.
  - Apply selected wheel correction strategy.
  - Preserve current defaults.
- Modify `pv_cleaning_robot/service/motion_service.cc`
  - Pass IMU `gyro_z` and IMU validity into `HeadingCorrector::Input`.
- Modify `test/service/heading_pid_test.cc`
  - Add strategy command-generation tests.
- Modify `test/integration/hardware/hw_config.h`
  - Load `correction_compare` typed config from JSON.
- Modify `test/integration/hardware/hw_test_config.json`
  - Add the new tuning section.
- Create `test/integration/hardware/system_hw_fixture.h`
  - Declares `SystemHwFixture`, common helper functions, and `make_motion_config`.
- Create `test/integration/hardware/system_hw_fixture.cc`
  - Moves fixture implementation and common helpers out of `system_hw_test.cc`.
- Create `test/integration/hardware/system_hw_metrics.h`
  - Declares JSONL metric helpers.
- Create `test/integration/hardware/system_hw_metrics.cc`
  - Moves metrics JSON builders and file helpers out of `system_hw_test.cc`.
- Create `test/integration/hardware/system_hw_motion_tests.cc`
  - Moves existing mission-chain system tests.
- Create `test/integration/hardware/system_hw_attitude_tests.cc`
  - Moves existing UDS/attitude tests and adds correction comparison test tags.
- Modify `test/integration/hardware/system_hw_test.cc`
  - Reduce it to either a small compatibility include-free file or remove it from CMake after moving tests.
- Modify `test/CMakeLists.txt`
  - Add new service sources and new test source files.

---

### Task 1: Add UDS Gyro Fusion Class

**Files:**
- Create: `include/pv_cleaning_robot/service/uds_gyro_yaw_fusion.h`
- Create: `pv_cleaning_robot/service/uds_gyro_yaw_fusion.cc`
- Create: `test/service/uds_gyro_yaw_fusion_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Write the failing unit tests**

Create `test/service/uds_gyro_yaw_fusion_test.cc` with tests for initialization, gyro prediction, UDS correction, and gyro-only timeout:

```cpp
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

using robot::service::UdsGyroYawFusion;

TEST_CASE("UdsGyroYawFusion initializes from first valid UDS sample", "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    fusion.set_params(params);

    const auto out = fusion.update({.dt_s = 0.02f,
                                    .uds_yaw_deg = -2.0f,
                                    .uds_valid = true,
                                    .uds_confidence = 1.0f,
                                    .uds_age_ms = 20,
                                    .gyro_z_rad_s = 0.0f,
                                    .imu_valid = true});

    REQUIRE(out.valid);
    CHECK(out.fused_yaw_deg == Approx(-2.0f).margin(0.05f));
    CHECK(out.gyro_z_dps == Approx(0.0f));
}

TEST_CASE("UdsGyroYawFusion preserves gyro sign convention", "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.max_gyro_only_ms = 500;
    fusion.set_params(params);

    fusion.update({.dt_s = 0.02f,
                   .uds_yaw_deg = 0.0f,
                   .uds_valid = true,
                   .uds_confidence = 1.0f,
                   .uds_age_ms = 0,
                   .gyro_z_rad_s = 0.0f,
                   .imu_valid = true});

    const auto out = fusion.update({.dt_s = 0.1f,
                                    .uds_yaw_deg = 0.0f,
                                    .uds_valid = false,
                                    .uds_confidence = 0.0f,
                                    .uds_age_ms = 100,
                                    .gyro_z_rad_s = -0.1745329f,
                                    .imu_valid = true});

    REQUIRE(out.valid);
    CHECK(out.gyro_z_dps == Approx(-10.0f).margin(0.05f));
    CHECK(out.fused_yaw_deg < 0.0f);
}

TEST_CASE("UdsGyroYawFusion UDS update pulls prediction toward measurement", "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.measurement_noise_uds = 0.5f;
    fusion.set_params(params);

    fusion.update({.dt_s = 0.02f,
                   .uds_yaw_deg = 0.0f,
                   .uds_valid = true,
                   .uds_confidence = 1.0f,
                   .uds_age_ms = 0,
                   .gyro_z_rad_s = 0.0f,
                   .imu_valid = true});

    const auto out = fusion.update({.dt_s = 0.02f,
                                    .uds_yaw_deg = 4.0f,
                                    .uds_valid = true,
                                    .uds_confidence = 1.0f,
                                    .uds_age_ms = 0,
                                    .gyro_z_rad_s = 0.0f,
                                    .imu_valid = true});

    REQUIRE(out.valid);
    CHECK(out.fused_yaw_deg > 0.0f);
    CHECK(out.fused_yaw_deg < 4.1f);
    CHECK(out.innovation_deg > 0.0f);
}

TEST_CASE("UdsGyroYawFusion invalidates after gyro-only timeout", "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.max_gyro_only_ms = 100;
    fusion.set_params(params);

    fusion.update({.dt_s = 0.02f,
                   .uds_yaw_deg = 1.0f,
                   .uds_valid = true,
                   .uds_confidence = 1.0f,
                   .uds_age_ms = 0,
                   .gyro_z_rad_s = 0.0f,
                   .imu_valid = true});

    const auto out = fusion.update({.dt_s = 0.2f,
                                    .uds_yaw_deg = 0.0f,
                                    .uds_valid = false,
                                    .uds_confidence = 0.0f,
                                    .uds_age_ms = 200,
                                    .gyro_z_rad_s = 0.0f,
                                    .imu_valid = true});

    CHECK_FALSE(out.valid);
}
```

- [ ] **Step 2: Add the test to CMake and verify it fails**

Modify `test/CMakeLists.txt`:

```cmake
# COMMON_SRCS service layer
  ${PROJ}/service/uds_gyro_yaw_fusion.cc

# unit_tests service layer
  service/uds_gyro_yaw_fusion_test.cc
```

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `uds_gyro_yaw_fusion.h` does not exist.

- [ ] **Step 3: Create the public fusion header**

Create `include/pv_cleaning_robot/service/uds_gyro_yaw_fusion.h`:

```cpp
#pragma once

namespace robot::service {

class UdsGyroYawFusion {
   public:
    struct Params {
        float process_noise_angle{0.05f};
        float process_noise_bias{0.001f};
        float measurement_noise_uds{0.5f};
        float initial_angle_variance{1.0f};
        float initial_bias_variance{1.0f};
        int max_gyro_only_ms{300};
    };

    struct Input {
        float dt_s{0.0f};
        float uds_yaw_deg{0.0f};
        bool uds_valid{false};
        float uds_confidence{0.0f};
        int64_t uds_age_ms{-1};
        float gyro_z_rad_s{0.0f};
        bool imu_valid{false};
    };

    struct Output {
        bool valid{false};
        float fused_yaw_deg{0.0f};
        float gyro_z_dps{0.0f};
        float gyro_bias_dps{0.0f};
        float innovation_deg{0.0f};
        float kalman_gain_angle{0.0f};
        float kalman_gain_bias{0.0f};
    };

    void set_params(const Params& params);
    void reset();
    Output update(const Input& input);

   private:
    Params params_{};
    bool initialized_{false};
    int gyro_only_elapsed_ms_{0};
    float angle_deg_{0.0f};
    float gyro_bias_dps_{0.0f};
    float p00_{1.0f};
    float p01_{0.0f};
    float p10_{0.0f};
    float p11_{1.0f};
};

}  // namespace robot::service
```

- [ ] **Step 4: Implement the minimal fusion logic**

Create `pv_cleaning_robot/service/uds_gyro_yaw_fusion.cc`:

```cpp
#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

#include <algorithm>
#include <cmath>

namespace robot::service {

namespace {

constexpr float kRadToDeg = 57.29577951308232f;

float non_negative(float v) {
    return std::max(0.0f, v);
}

}  // namespace

void UdsGyroYawFusion::set_params(const Params& params) {
    params_ = params;
    params_.process_noise_angle = non_negative(params_.process_noise_angle);
    params_.process_noise_bias = non_negative(params_.process_noise_bias);
    params_.measurement_noise_uds = std::max(1e-4f, params_.measurement_noise_uds);
    params_.initial_angle_variance = std::max(1e-4f, params_.initial_angle_variance);
    params_.initial_bias_variance = std::max(1e-4f, params_.initial_bias_variance);
    params_.max_gyro_only_ms = std::max(0, params_.max_gyro_only_ms);
}

void UdsGyroYawFusion::reset() {
    initialized_ = false;
    gyro_only_elapsed_ms_ = 0;
    angle_deg_ = 0.0f;
    gyro_bias_dps_ = 0.0f;
    p00_ = params_.initial_angle_variance;
    p01_ = 0.0f;
    p10_ = 0.0f;
    p11_ = params_.initial_bias_variance;
}

UdsGyroYawFusion::Output UdsGyroYawFusion::update(const Input& input) {
    Output out{};
    const float dt = std::max(0.0f, input.dt_s);
    const float gyro_z_dps = input.imu_valid ? input.gyro_z_rad_s * kRadToDeg : 0.0f;
    out.gyro_z_dps = gyro_z_dps;

    if (!initialized_) {
        if (!input.uds_valid) {
            return out;
        }
        initialized_ = true;
        gyro_only_elapsed_ms_ = 0;
        angle_deg_ = input.uds_yaw_deg;
        gyro_bias_dps_ = 0.0f;
        p00_ = params_.initial_angle_variance;
        p01_ = 0.0f;
        p10_ = 0.0f;
        p11_ = params_.initial_bias_variance;
    } else if (dt > 0.0f) {
        angle_deg_ += (gyro_z_dps - gyro_bias_dps_) * dt;

        const float old_p00 = p00_;
        const float old_p01 = p01_;
        const float old_p10 = p10_;
        const float old_p11 = p11_;

        p00_ = old_p00 - dt * (old_p10 + old_p01) + dt * dt * old_p11 +
               params_.process_noise_angle;
        p01_ = old_p01 - dt * old_p11;
        p10_ = old_p10 - dt * old_p11;
        p11_ = old_p11 + params_.process_noise_bias;
    }

    if (input.uds_valid) {
        gyro_only_elapsed_ms_ = 0;
        const float innovation = input.uds_yaw_deg - angle_deg_;
        const float s = std::max(1e-4f, p00_ + params_.measurement_noise_uds);
        const float k0 = p00_ / s;
        const float k1 = p10_ / s;

        angle_deg_ += k0 * innovation;
        gyro_bias_dps_ += k1 * innovation;

        const float old_p00 = p00_;
        const float old_p01 = p01_;
        p00_ -= k0 * old_p00;
        p01_ -= k0 * old_p01;
        p10_ -= k1 * old_p00;
        p11_ -= k1 * old_p01;

        out.innovation_deg = innovation;
        out.kalman_gain_angle = k0;
        out.kalman_gain_bias = k1;
    } else {
        gyro_only_elapsed_ms_ += static_cast<int>(std::lround(dt * 1000.0f));
    }

    out.valid = gyro_only_elapsed_ms_ <= params_.max_gyro_only_ms;
    out.fused_yaw_deg = angle_deg_;
    out.gyro_bias_dps = gyro_bias_dps_;
    return out;
}

}  // namespace robot::service
```

- [ ] **Step 5: Run fusion unit tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: `unit_tests` builds successfully.

- [ ] **Step 6: Commit**

```bash
git add include/pv_cleaning_robot/service/uds_gyro_yaw_fusion.h \
        pv_cleaning_robot/service/uds_gyro_yaw_fusion.cc \
        test/service/uds_gyro_yaw_fusion_test.cc \
        test/CMakeLists.txt
git commit -m "Add UDS gyro yaw fusion filter"
```

---

### Task 2: Add Correction Strategy Parameters and Wheel Command Tests

**Files:**
- Modify: `include/pv_cleaning_robot/service/heading_corrector.h`
- Modify: `pv_cleaning_robot/service/heading_corrector.cc`
- Modify: `test/service/heading_pid_test.cc`

- [ ] **Step 1: Add failing strategy tests**

Append tests to `test/service/heading_pid_test.cc`. These should use existing `MockUdsServer` helpers in that file and assert command generation for both travel directions:

```cpp
TEST_CASE("HeadingCorrector: lower-only strategy keeps upper wheels at base", "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.wheel_strategy = HeadingCorrector::WheelStrategy::LOWER_ONLY;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(-2.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::A, 20.0f);
    auto out = ctrl.compute(input);

    REQUIRE(out.correction_rpm == Approx(2.0f));
    CHECK(out.speed_command.lt_rpm == Approx(20.0f));
    CHECK(out.speed_command.rt_rpm == Approx(20.0f));
    CHECK(out.speed_command.lb_rpm == Approx(-18.0f));
    CHECK(out.speed_command.rb_rpm == Approx(-18.0f));
}

TEST_CASE("HeadingCorrector: top-decel-only prevents upper wheel acceleration toward A",
          "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.wheel_strategy = HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(-2.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::A, 20.0f);
    auto out = ctrl.compute(input);

    REQUIRE(out.correction_rpm == Approx(2.0f));
    CHECK(out.speed_command.lt_rpm == Approx(20.0f));
    CHECK(out.speed_command.rt_rpm == Approx(20.0f));
    CHECK(out.speed_command.lb_rpm == Approx(-18.0f));
    CHECK(out.speed_command.rb_rpm == Approx(-18.0f));
}

TEST_CASE("HeadingCorrector: top-decel-only allows upper wheel deceleration toward A",
          "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.wheel_strategy = HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(2.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::A, 20.0f);
    auto out = ctrl.compute(input);

    REQUIRE(out.correction_rpm == Approx(-2.0f));
    CHECK(out.speed_command.lt_rpm == Approx(18.0f));
    CHECK(out.speed_command.rt_rpm == Approx(18.0f));
    CHECK(out.speed_command.lb_rpm == Approx(-22.0f));
    CHECK(out.speed_command.rb_rpm == Approx(-22.0f));
}

TEST_CASE("HeadingCorrector: top-decel-only prevents upper wheel acceleration toward B",
          "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.wheel_strategy = HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(2.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::B, 20.0f);
    auto out = ctrl.compute(input);

    REQUIRE(out.correction_rpm == Approx(-2.0f));
    CHECK(out.speed_command.lt_rpm == Approx(-20.0f));
    CHECK(out.speed_command.rt_rpm == Approx(-20.0f));
    CHECK(out.speed_command.lb_rpm == Approx(18.0f));
    CHECK(out.speed_command.rb_rpm == Approx(18.0f));
}
```

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails because `HeadingCorrector::WheelStrategy` does not exist.

- [ ] **Step 3: Add strategy enum and params**

Modify `include/pv_cleaning_robot/service/heading_corrector.h`:

```cpp
#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

class HeadingCorrector {
   public:
    enum class AngleSource : uint8_t {
        RAW_UDS = 0,
        FUSED_UDS_GYRO,
    };

    enum class WheelStrategy : uint8_t {
        ALL_WHEELS = 0,
        LOWER_ONLY,
        TOP_DECEL_ONLY,
    };

    struct Params {
        // existing fields stay unchanged
        AngleSource angle_source{AngleSource::RAW_UDS};
        WheelStrategy wheel_strategy{WheelStrategy::ALL_WHEELS};
        bool slow_on_error{false};
        float slow_base_rpm{15.0f};
        float yaw_slow_threshold_deg{1.0f};
        UdsGyroYawFusion::Params fusion{};
    };

    struct Input {
        // existing fields stay unchanged
        bool imu_valid{false};
        float gyro_z_rad_s{0.0f};
    };

    struct DebugState {
        // existing fields stay unchanged
        float fused_yaw_deg{0.0f};
        bool fused_yaw_valid{false};
        float gyro_z_dps{0.0f};
        float fusion_innovation_deg{0.0f};
        float fusion_kalman_gain_angle{0.0f};
    };
```

Add private member:

```cpp
UdsGyroYawFusion yaw_fusion_{};
UdsGyroYawFusion::Output last_fusion_output_{};
```

- [ ] **Step 4: Implement strategy command generation**

Modify `pv_cleaning_robot/service/heading_corrector.cc`:

```cpp
HeadingCorrector::SpeedCommand HeadingCorrector::apply_correction(const SpeedCommand& base,
                                                                  float correction_rpm,
                                                                  WheelStrategy strategy) {
    SpeedCommand corrected = base;
    auto adjust_wheel = [](float base_rpm, float correction) {
        if (base_rpm > 0.0f) {
            return clamp(base_rpm + correction, 0.0f, kWheelRpmLimit);
        }
        if (base_rpm < 0.0f) {
            return clamp(base_rpm + correction, -kWheelRpmLimit, 0.0f);
        }
        return 0.0f;
    };
    auto decel_only = [](float base_rpm, float candidate) {
        if (base_rpm > 0.0f) {
            return clamp(candidate, 0.0f, base_rpm);
        }
        if (base_rpm < 0.0f) {
            return clamp(candidate, base_rpm, 0.0f);
        }
        return 0.0f;
    };

    switch (strategy) {
    case WheelStrategy::ALL_WHEELS:
        corrected.lt_rpm = adjust_wheel(base.lt_rpm, correction_rpm);
        corrected.rt_rpm = adjust_wheel(base.rt_rpm, correction_rpm);
        corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
        corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
        break;
    case WheelStrategy::LOWER_ONLY:
        corrected.lt_rpm = base.lt_rpm;
        corrected.rt_rpm = base.rt_rpm;
        corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
        corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
        break;
    case WheelStrategy::TOP_DECEL_ONLY:
        corrected.lt_rpm = decel_only(base.lt_rpm, adjust_wheel(base.lt_rpm, correction_rpm));
        corrected.rt_rpm = decel_only(base.rt_rpm, adjust_wheel(base.rt_rpm, correction_rpm));
        corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
        corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
        break;
    }
    return corrected;
}
```

Update the header declaration to include `WheelStrategy strategy`.

- [ ] **Step 5: Wire strategy into `compute()`**

Replace:

```cpp
output.speed_command = apply_correction(input.base_command, correction);
```

with:

```cpp
output.speed_command = apply_correction(input.base_command, correction, params_.wheel_strategy);
```

- [ ] **Step 6: Run strategy tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: `unit_tests` builds successfully.

- [ ] **Step 7: Commit**

```bash
git add include/pv_cleaning_robot/service/heading_corrector.h \
        pv_cleaning_robot/service/heading_corrector.cc \
        test/service/heading_pid_test.cc
git commit -m "Add heading correction wheel strategies"
```

---

### Task 3: Wire Optional Fusion and Slow Base Speed into HeadingCorrector

**Files:**
- Modify: `include/pv_cleaning_robot/service/heading_corrector.h`
- Modify: `pv_cleaning_robot/service/heading_corrector.cc`
- Modify: `pv_cleaning_robot/service/motion_service.cc`
- Modify: `test/service/heading_pid_test.cc`

- [ ] **Step 1: Add failing tests for fused angle source and slow base speed**

Append to `test/service/heading_pid_test.cc`:

```cpp
TEST_CASE("HeadingCorrector: slow-on-error lowers base command before correction",
          "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.slow_on_error = true;
    p.slow_base_rpm = 15.0f;
    p.yaw_slow_threshold_deg = 1.0f;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(2.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::A, 20.0f);
    auto out = ctrl.compute(input);

    REQUIRE(out.correction_rpm == Approx(-2.0f));
    CHECK(out.speed_command.lt_rpm == Approx(13.0f));
    CHECK(out.speed_command.rt_rpm == Approx(13.0f));
    CHECK(out.speed_command.lb_rpm == Approx(-17.0f));
    CHECK(out.speed_command.rb_rpm == Approx(-17.0f));
}

TEST_CASE("HeadingCorrector: fused source uses gyro prediction during short UDS gap",
          "[service][heading_pid]") {
    MockUdsServer server;
    HeadingCorrector::Params p = test_params(server.path);
    p.kp = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.angle_source = HeadingCorrector::AngleSource::FUSED_UDS_GYRO;
    p.fusion.max_gyro_only_ms = 300;

    HeadingCorrector ctrl(p);
    ctrl.enable(true);
    server.send_yaw(0.0f);
    wait_for_sample(ctrl);

    auto input = make_input_with_speed(Endpoint::A, 20.0f);
    input.imu_valid = true;
    input.gyro_z_rad_s = -0.1745329f;
    input.dt_s = 0.1f;

    auto out = ctrl.compute(input);
    const auto state = ctrl.debug_state();

    REQUIRE(state.fused_yaw_valid);
    CHECK(state.fused_yaw_deg < 0.0f);
    CHECK(out.correction_rpm > 0.0f);
}
```

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build fails until slow base and fused source are implemented.

- [ ] **Step 3: Implement slow base command helper**

Add helper in `heading_corrector.cc`:

```cpp
HeadingCorrector::SpeedCommand apply_base_abs_rpm(const HeadingCorrector::SpeedCommand& base,
                                                  float abs_rpm) {
    HeadingCorrector::SpeedCommand out{};
    auto with_abs = [abs_rpm](float current) {
        if (current > 0.0f) {
            return abs_rpm;
        }
        if (current < 0.0f) {
            return -abs_rpm;
        }
        return 0.0f;
    };
    out.lt_rpm = with_abs(base.lt_rpm);
    out.rt_rpm = with_abs(base.rt_rpm);
    out.lb_rpm = with_abs(base.lb_rpm);
    out.rb_rpm = with_abs(base.rb_rpm);
    return out;
}
```

In `compute()`, before applying correction:

```cpp
SpeedCommand base_for_correction = input.base_command;
if (params_.slow_on_error && std::abs(error) >= params_.yaw_slow_threshold_deg) {
    base_for_correction =
        apply_base_abs_rpm(input.base_command, std::max(0.0f, params_.slow_base_rpm));
}
output.speed_command =
    apply_correction(base_for_correction, correction, params_.wheel_strategy);
```

- [ ] **Step 4: Implement fused source selection**

In `set_params()`:

```cpp
yaw_fusion_.set_params(params_.fusion);
```

In `reset_control_state_locked()`:

```cpp
yaw_fusion_.reset();
last_fusion_output_ = {};
```

In `compute()`, after `sample_ready`:

```cpp
float raw_yaw_for_control = latest_result_.yaw_deg;
if (params_.angle_source == AngleSource::FUSED_UDS_GYRO) {
    last_fusion_output_ = yaw_fusion_.update({.dt_s = input.dt_s,
                                              .uds_yaw_deg = latest_result_.yaw_deg,
                                              .uds_valid = sample_ready,
                                              .uds_confidence = latest_result_.confidence,
                                              .uds_age_ms = result_age_ms,
                                              .gyro_z_rad_s = input.gyro_z_rad_s,
                                              .imu_valid = input.imu_valid});
    if (!last_fusion_output_.valid) {
        reset_control_state_locked();
        mode_ = Mode::STALE;
        return output;
    }
    raw_yaw_for_control = last_fusion_output_.fused_yaw_deg;
}
float error = normalize_yaw_to_control_error(
    input.primary_dock, input.travel_direction, raw_yaw_for_control);
```

Keep raw mode behavior unchanged.

- [ ] **Step 5: Pass IMU gyro from MotionService**

Modify `pv_cleaning_robot/service/motion_service.cc` inside the PID block:

```cpp
const auto imu = imu_ ? imu_->get_latest() : robot::device::ImuDevice::ImuData{};
input.imu_valid = imu.valid;
input.gyro_z_rad_s = imu.valid ? imu.gyro[2] : 0.0f;
```

- [ ] **Step 6: Extend debug state**

In `debug_state()`, populate the new fields:

```cpp
last_fusion_output_.fused_yaw_deg,
last_fusion_output_.valid,
last_fusion_output_.gyro_z_dps,
last_fusion_output_.innovation_deg,
last_fusion_output_.kalman_gain_angle
```

Keep existing field order compatible by appending new fields at the end of `DebugState`.

- [ ] **Step 7: Run tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: `unit_tests` builds successfully.

- [ ] **Step 8: Commit**

```bash
git add include/pv_cleaning_robot/service/heading_corrector.h \
        pv_cleaning_robot/service/heading_corrector.cc \
        pv_cleaning_robot/service/motion_service.cc \
        test/service/heading_pid_test.cc
git commit -m "Wire fused yaw and slow correction options"
```

---

### Task 4: Add Hardware Config for Correction Compare

**Files:**
- Modify: `test/integration/hardware/hw_config.h`
- Modify: `test/integration/hardware/hw_test_config.json`

- [ ] **Step 1: Add failing compile use sites**

In `test/integration/hardware/hw_config.h`, add a `CorrectionCompareConfig` struct inside `HwParams`:

```cpp
struct CorrectionCompareConfig {
    float slow_base_rpm{15.0f};
    float yaw_slow_threshold_deg{1.0f};
    float max_output{10.0f};
    float min_effective_output{1.0f};
    float kp{5.0f};
    float ki{0.0f};
    float kd{0.0f};
    float integral_limit{1.0f};
    struct FusionConfig {
        float process_noise_angle{0.05f};
        float process_noise_bias{0.001f};
        float measurement_noise_uds{0.5f};
        float initial_angle_variance{1.0f};
        float initial_bias_variance{1.0f};
        int max_gyro_only_ms{300};
    } fusion;
} correction_compare;
```

- [ ] **Step 2: Load JSON values**

In `load_hw_test_config()`, add:

```cpp
p.correction_compare.slow_base_rpm =
    cfg.get<float>("correction_compare.slow_base_rpm", p.correction_compare.slow_base_rpm);
p.correction_compare.yaw_slow_threshold_deg =
    cfg.get<float>("correction_compare.yaw_slow_threshold_deg",
                   p.correction_compare.yaw_slow_threshold_deg);
p.correction_compare.max_output =
    cfg.get<float>("correction_compare.max_output", p.correction_compare.max_output);
p.correction_compare.min_effective_output =
    cfg.get<float>("correction_compare.min_effective_output",
                   p.correction_compare.min_effective_output);
p.correction_compare.kp = cfg.get<float>("correction_compare.kp", p.correction_compare.kp);
p.correction_compare.ki = cfg.get<float>("correction_compare.ki", p.correction_compare.ki);
p.correction_compare.kd = cfg.get<float>("correction_compare.kd", p.correction_compare.kd);
p.correction_compare.integral_limit =
    cfg.get<float>("correction_compare.integral_limit",
                   p.correction_compare.integral_limit);
p.correction_compare.fusion.process_noise_angle =
    cfg.get<float>("correction_compare.fusion.process_noise_angle",
                   p.correction_compare.fusion.process_noise_angle);
p.correction_compare.fusion.process_noise_bias =
    cfg.get<float>("correction_compare.fusion.process_noise_bias",
                   p.correction_compare.fusion.process_noise_bias);
p.correction_compare.fusion.measurement_noise_uds =
    cfg.get<float>("correction_compare.fusion.measurement_noise_uds",
                   p.correction_compare.fusion.measurement_noise_uds);
p.correction_compare.fusion.initial_angle_variance =
    cfg.get<float>("correction_compare.fusion.initial_angle_variance",
                   p.correction_compare.fusion.initial_angle_variance);
p.correction_compare.fusion.initial_bias_variance =
    cfg.get<float>("correction_compare.fusion.initial_bias_variance",
                   p.correction_compare.fusion.initial_bias_variance);
p.correction_compare.fusion.max_gyro_only_ms =
    cfg.get<int>("correction_compare.fusion.max_gyro_only_ms",
                 p.correction_compare.fusion.max_gyro_only_ms);
```

- [ ] **Step 3: Add JSON config**

Add to `test/integration/hardware/hw_test_config.json` at the top level:

```json
  "correction_compare": {
    "slow_base_rpm": 15.0,
    "yaw_slow_threshold_deg": 1.0,
    "max_output": 10.0,
    "min_effective_output": 1.0,
    "kp": 5.0,
    "ki": 0.0,
    "kd": 0.0,
    "integral_limit": 1.0,
    "fusion": {
      "process_noise_angle": 0.05,
      "process_noise_bias": 0.001,
      "measurement_noise_uds": 0.5,
      "initial_angle_variance": 1.0,
      "initial_bias_variance": 1.0,
      "max_gyro_only_ms": 300
    }
  }
```

- [ ] **Step 4: Build hw tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: `hw_tests` builds successfully.

- [ ] **Step 5: Commit**

```bash
git add test/integration/hardware/hw_config.h \
        test/integration/hardware/hw_test_config.json
git commit -m "Add hardware correction comparison config"
```

---

### Task 5: Split Hardware System Fixture and Metrics

**Files:**
- Create: `test/integration/hardware/system_hw_fixture.h`
- Create: `test/integration/hardware/system_hw_fixture.cc`
- Create: `test/integration/hardware/system_hw_metrics.h`
- Create: `test/integration/hardware/system_hw_metrics.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Create fixture header**

Create `system_hw_fixture.h` with declarations moved from the anonymous namespace in `system_hw_test.cc`:

```cpp
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "hw_config.h"
#include "hw_exit_guard.h"
#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

namespace hw_system {

extern const hw::HwParams kp;

const char* endpoint_to_config(robot::domain::Endpoint endpoint);
std::string fault_to_string(const std::optional<uint32_t>& fault);
robot::service::MotionService::Config make_motion_config(bool pid_enabled);
robot::service::MotionService::Config make_correction_compare_motion_config(
    robot::service::HeadingCorrector::AngleSource angle_source,
    robot::service::HeadingCorrector::WheelStrategy wheel_strategy,
    bool slow_on_error);

class SystemHwFixture : public hw::IGracefulShutdown {
   public:
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("hw_system")};

    robot::middleware::EventBus bus;
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<MockSerialPort> mock_brush_serial;
    std::shared_ptr<robot::driver::LibSerialPort> real_brush_serial;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::driver::LibSerialPort> bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::device::GpsDevice> gps;
    std::shared_ptr<robot::service::HealthService> health;
    std::unique_ptr<robot::app::WatchdogMgr> watchdog;
    std::shared_ptr<robot::service::NavService> nav;
    std::unique_ptr<robot::service::ConfigService> config;
    std::shared_ptr<robot::service::FaultService> fault;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::app::RobotController> controller;
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio;
    std::shared_ptr<robot::device::LimitSwitch> left_sw;
    std::shared_ptr<robot::device::LimitSwitch> right_sw;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;

    robot::domain::PositionState position_state{robot::domain::PositionState::OnSegment};
    bool real_brush{false};
    bool initialized{false};
    uint32_t repeat_count{1};
    std::optional<robot::domain::Endpoint> active_segment_target;

    bool init(bool use_real_brush = false,
              bool pid_enabled = false,
              const std::string& health_jsonl_path = {},
              std::optional<robot::service::MotionService::Config> motion_config_override = {});
    bool start_safety_bridge();
    robot::domain::CommandResult start_configured_assuming_primary_dock();
    void graceful_shutdown() override;
    ~SystemHwFixture() override;

   private:
    void write_config_files();
};

}  // namespace hw_system
```

Include `integration/thingsboard_test_support.h` and `mock/mock_serial_port.h` in the header if the compiler reports incomplete types for `TempSplitConfigPaths` or `MockSerialPort`.

- [ ] **Step 2: Move fixture implementation**

Create `system_hw_fixture.cc` and move these current `system_hw_test.cc` sections into namespace `hw_system`:

- includes required by fixture
- `const hw::HwParams kp`
- `endpoint_to_config`
- `fault_code_name`
- `fault_to_string`
- `position_at`
- `make_motion_config`
- `SystemHwFixture` class method bodies

Change `SystemHwFixture::init()` so the motion service config is:

```cpp
const auto motion_cfg =
    motion_config_override ? *motion_config_override : make_motion_config(pid_enabled);
motion = std::make_shared<robot::service::MotionService>(
    walk_group, brush, imu, bus, motion_cfg);
```

Add `make_correction_compare_motion_config()`:

```cpp
robot::service::MotionService::Config make_correction_compare_motion_config(
    robot::service::HeadingCorrector::AngleSource angle_source,
    robot::service::HeadingCorrector::WheelStrategy wheel_strategy,
    bool slow_on_error) {
    auto cfg = make_motion_config(true);
    cfg.pid.kp = kp.correction_compare.kp;
    cfg.pid.ki = kp.correction_compare.ki;
    cfg.pid.kd = kp.correction_compare.kd;
    cfg.pid.integral_limit = kp.correction_compare.integral_limit;
    cfg.pid.max_output = kp.correction_compare.max_output;
    cfg.pid.min_effective_output = kp.correction_compare.min_effective_output;
    cfg.pid.angle_source = angle_source;
    cfg.pid.wheel_strategy = wheel_strategy;
    cfg.pid.slow_on_error = slow_on_error;
    cfg.pid.slow_base_rpm = kp.correction_compare.slow_base_rpm;
    cfg.pid.yaw_slow_threshold_deg = kp.correction_compare.yaw_slow_threshold_deg;
    cfg.pid.fusion.process_noise_angle =
        kp.correction_compare.fusion.process_noise_angle;
    cfg.pid.fusion.process_noise_bias =
        kp.correction_compare.fusion.process_noise_bias;
    cfg.pid.fusion.measurement_noise_uds =
        kp.correction_compare.fusion.measurement_noise_uds;
    cfg.pid.fusion.initial_angle_variance =
        kp.correction_compare.fusion.initial_angle_variance;
    cfg.pid.fusion.initial_bias_variance =
        kp.correction_compare.fusion.initial_bias_variance;
    cfg.pid.fusion.max_gyro_only_ms =
        kp.correction_compare.fusion.max_gyro_only_ms;
    return cfg;
}
```

- [ ] **Step 3: Create metrics header and implementation**

Move these existing helpers into `system_hw_metrics.h/.cc` under namespace `hw_system`:

- `parse_json_line`
- `pid_mode_name`
- `open_jsonl_for_metrics`
- `build_pid_sample_json`
- `build_segment_summary_json`
- `build_final_summary_json`
- `collect_rotated_health_logs`
- `remove_rotated_health_logs`
- `require_diagnostics_health_log`

Add a new declaration for strategy samples:

```cpp
std::string build_correction_compare_sample_json(
    int64_t ts_ms,
    const std::string& strategy,
    int segment,
    const std::string& state,
    float imu_yaw_deg,
    const robot::device::WalkMotorGroup::GroupDiagnostics& walk,
    const robot::service::NavService::FusedOdometry& odom,
    const robot::service::HeadingCorrector::DebugState& pid);
```

Implement it by writing the existing PID fields plus the appended fusion debug fields.

- [ ] **Step 4: Update CMake**

In `test/CMakeLists.txt`, replace `integration/hardware/system_hw_test.cc` with:

```cmake
  integration/hardware/system_hw_fixture.cc
  integration/hardware/system_hw_metrics.cc
  integration/hardware/system_hw_motion_tests.cc
  integration/hardware/system_hw_attitude_tests.cc
```

Keep `system_hw_test.cc` out of `hw_tests` after tests have moved.

- [ ] **Step 5: Build to reveal missing declarations**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: initial compile errors for helpers not yet moved or namespace-qualified. Fix by qualifying moved symbols with `hw_system::` or adding declarations to the new headers. Do not change test behavior.

- [ ] **Step 6: Commit fixture and metrics split**

```bash
git add test/integration/hardware/system_hw_fixture.h \
        test/integration/hardware/system_hw_fixture.cc \
        test/integration/hardware/system_hw_metrics.h \
        test/integration/hardware/system_hw_metrics.cc \
        test/integration/hardware/system_hw_test.cc \
        test/CMakeLists.txt
git commit -m "Split hardware system fixture and metrics"
```

---

### Task 6: Move Existing Hardware Tests into Split Files

**Files:**
- Create: `test/integration/hardware/system_hw_motion_tests.cc`
- Create: `test/integration/hardware/system_hw_attitude_tests.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`
- Modify: `test/CMakeLists.txt`

- [ ] **Step 1: Move mission-chain tests**

Create `system_hw_motion_tests.cc` with:

```cpp
#include <catch2/catch.hpp>
#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

#include "system_hw_fixture.h"
#include "system_hw_metrics.h"

using namespace std::chrono_literals;

namespace {
using hw_system::SystemHwFixture;
using hw_system::kp;

void run_configured_system_chain(SystemHwFixture& f,
                                 const char* tag,
                                 uint32_t repeat_count,
                                 bool expect_real_brush,
                                 bool log_fused_odometry,
                                 bool log_heading_pid_debug);
}
```

Move from `system_hw_test.cc`:

- `run_configured_system_chain`
- `full_init`
- `nav_fused_odometry`
- `health_real_data`
- `safety_idle`
- `motion_then_stop`
- `watchdog_timeout`
- `watchdog_heartbeat`
- `p0_fault_chain`
- `n1_clean_cycle`
- `combined`
- `combined_nvm_real`
- `combined_brush_real`
- `pid_combined`
- `imu_gps_health_only`

Keep test tag strings unchanged.

- [ ] **Step 2: Move attitude tests**

Create `system_hw_attitude_tests.cc` with:

```cpp
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include <spdlog/spdlog.h>

#include "system_hw_fixture.h"
#include "system_hw_metrics.h"
#include "pv_cleaning_robot/device/attitude_limit_switch.h"

using namespace std::chrono_literals;
```

Move from `system_hw_test.cc`:

- `UdsYawSample`
- `read_uds_yaw_sample`
- `wait_uds_yaw_sample`
- `compute_lower_uds_zero_cmd`
- attitude limit helper structs and functions
- `run_lower_uds_zero_test`
- `run_lower_attitude_center_test`
- `lower_uds_zero` logic test
- `lower_attitude_center` logic test
- `lower_uds_zero` hardware test
- `lower_attitude_center` hardware test
- `lower_pitch_peak`
- `lower_pitch_score`

Keep test tag strings unchanged.

- [ ] **Step 3: Remove old system file from CMake or reduce it to empty**

If all tests moved, remove `integration/hardware/system_hw_test.cc` from `hw_tests`.
Leave the file deleted or with only a short comment if the repository prefers stable paths.

- [ ] **Step 4: Build hw tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: `hw_tests` builds successfully.

- [ ] **Step 5: Commit test split**

```bash
git add test/integration/hardware/system_hw_motion_tests.cc \
        test/integration/hardware/system_hw_attitude_tests.cc \
        test/integration/hardware/system_hw_test.cc \
        test/CMakeLists.txt
git commit -m "Split hardware system tests by behavior"
```

---

### Task 7: Add Correction Comparison Hardware Tests

**Files:**
- Modify: `test/integration/hardware/system_hw_attitude_tests.cc`
- Modify: `test/integration/hardware/system_hw_metrics.h`
- Modify: `test/integration/hardware/system_hw_metrics.cc`

- [ ] **Step 1: Define strategy descriptor**

Add to `system_hw_attitude_tests.cc`:

```cpp
struct CorrectionCompareCase {
    const char* tag;
    robot::service::HeadingCorrector::AngleSource angle_source;
    robot::service::HeadingCorrector::WheelStrategy wheel_strategy;
    bool slow_on_error;
};
```

- [ ] **Step 2: Add runner that mirrors pid_combined chain**

Add a runner that calls the same mission-chain helper shape as `[hw_system][pid_combined]`.
If `run_configured_system_chain` lives in `system_hw_motion_tests.cc`, move its declaration
and implementation to `system_hw_fixture.h/.cc` or a new `system_hw_chain.h/.cc` so both
motion and attitude tests can call it.

Runner:

```cpp
void run_correction_compare_case(const CorrectionCompareCase& c) {
    SystemHwFixture f;
    const auto motion_cfg = hw_system::make_correction_compare_motion_config(
        c.angle_source, c.wheel_strategy, c.slow_on_error);
    run_configured_system_chain(
        f,
        c.tag,
        kp.combined_passes,
        true,
        true,
        true,
        motion_cfg);
}
```

If `run_configured_system_chain` currently accepts no override, change its signature to:

```cpp
void run_configured_system_chain(
    SystemHwFixture& f,
    const char* tag,
    uint32_t repeat_count,
    bool expect_real_brush,
    bool log_fused_odometry,
    bool log_heading_pid_debug,
    std::optional<robot::service::MotionService::Config> motion_config_override = {});
```

Inside it, call:

```cpp
REQUIRE(f.init(expect_real_brush,
               log_heading_pid_debug,
               health_path.string(),
               motion_config_override));
```

- [ ] **Step 3: Add eight hardware test tags**

Add tests:

```cpp
TEST_CASE("原始 UDS 降速，上下轮共同纠偏", "[hw_system][corr_raw_slow_all]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_all",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 true});
}

TEST_CASE("原始 UDS 降速，只纠偏下轮", "[hw_system][corr_raw_slow_lower_only]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 true});
}

TEST_CASE("原始 UDS 降速，上轮只允许减速", "[hw_system][corr_raw_slow_top_decel_only]") {
    run_correction_compare_case({"hw_system][corr_raw_slow_top_decel_only",
                                 robot::service::HeadingCorrector::AngleSource::RAW_UDS,
                                 robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY,
                                 true});
}

TEST_CASE("融合角不降速，上下轮共同纠偏", "[hw_system][corr_fused_fast_all]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_all",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 false});
}

TEST_CASE("融合角不降速，只纠偏下轮", "[hw_system][corr_fused_fast_lower_only]") {
    run_correction_compare_case({"hw_system][corr_fused_fast_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 false});
}

TEST_CASE("融合角降速，上下轮共同纠偏", "[hw_system][corr_fused_slow_all]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_all",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS,
                                 true});
}

TEST_CASE("融合角降速，只纠偏下轮", "[hw_system][corr_fused_slow_lower_only]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_lower_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY,
                                 true});
}

TEST_CASE("融合角降速，上轮只允许减速", "[hw_system][corr_fused_slow_top_decel_only]") {
    run_correction_compare_case({"hw_system][corr_fused_slow_top_decel_only",
                                 robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO,
                                 robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY,
                                 true});
}
```

- [ ] **Step 4: Log comparison config at test start**

At the start of `run_correction_compare_case`, add:

```cpp
spdlog::warn(
    "[{}] correction_compare slow_base={:.1f} threshold={:.2f} kp={:.2f} ki={:.2f} "
    "kd={:.2f} max_output={:.1f} fusion(q_angle={:.4f} q_bias={:.4f} r_uds={:.4f} "
    "gyro_only_ms={})",
    c.tag,
    kp.correction_compare.slow_base_rpm,
    kp.correction_compare.yaw_slow_threshold_deg,
    kp.correction_compare.kp,
    kp.correction_compare.ki,
    kp.correction_compare.kd,
    kp.correction_compare.max_output,
    kp.correction_compare.fusion.process_noise_angle,
    kp.correction_compare.fusion.process_noise_bias,
    kp.correction_compare.fusion.measurement_noise_uds,
    kp.correction_compare.fusion.max_gyro_only_ms);
```

- [ ] **Step 5: Build hw tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: `hw_tests` builds successfully.

- [ ] **Step 6: Commit**

```bash
git add test/integration/hardware/system_hw_attitude_tests.cc \
        test/integration/hardware/system_hw_metrics.h \
        test/integration/hardware/system_hw_metrics.cc
git commit -m "Add hardware correction comparison tests"
```

---

### Task 8: Final Verification

**Files:**
- No new files unless previous tasks reveal required fixes.

- [ ] **Step 1: Check worktree**

Run:

```bash
git status --short
```

Expected: no unexpected unrelated changes. If there are uncommitted changes from the tasks, inspect them with `git diff` before continuing.

- [ ] **Step 2: Build unit tests**

Run:

```bash
cmake --build --preset rk3576-build --target unit_tests
```

Expected: build succeeds.

- [ ] **Step 3: Build hardware tests**

Run:

```bash
cmake --build --preset rk3576-build --target hw_tests
```

Expected: build succeeds.

- [ ] **Step 4: Document target-board smoke commands**

Tell the operator to run on the target board:

```bash
./hw_tests "[hw_system][pid_combined]"
./hw_tests "[hw_system][corr_raw_slow_top_decel_only]"
./hw_tests "[hw_system][corr_fused_slow_top_decel_only]"
```

Expected: all three complete the configured mission and write metrics. Compare yaw, fusion, wheel RPM targets/actuals, and segment durations.

- [ ] **Step 5: Commit any final build fixes**

If final verification required code changes:

```bash
git add <changed-files>
git commit -m "Fix correction comparison build issues"
```

If no changes were required, do not create an empty commit.

---

## Self-Review

- Spec coverage:
  - Hardware test split is covered by Tasks 5 and 6.
  - UDS+gyro fusion is covered by Task 1.
  - Correction strategies are covered by Tasks 2, 3, and 7.
  - Config-file tuning is covered by Task 4.
  - Same mission execution flow as `[hw_system][pid_combined]` is covered by Task 7.
  - Odometry is not used for control; Task 7 only permits observation logging.
- Placeholder scan:
  - The plan contains no deferred-work markers or unspecified test cases.
- Type consistency:
  - `AngleSource`, `WheelStrategy`, and `UdsGyroYawFusion` names are used consistently across tasks.
  - New `DebugState` fields are appended so existing code using old fields remains source-compatible after initializer updates are handled.
