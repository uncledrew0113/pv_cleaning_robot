#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

using robot::service::UdsGyroYawFusion;

namespace {

UdsGyroYawFusion::Input make_input(float dt_s,
                                   float uds_yaw_deg,
                                   bool uds_valid,
                                   float gyro_z_rad_s,
                                   bool imu_valid = true) {
    UdsGyroYawFusion::Input input;
    input.dt_s = dt_s;
    input.uds_yaw_deg = uds_yaw_deg;
    input.uds_valid = uds_valid;
    input.uds_confidence = uds_valid ? 1.0f : 0.0f;
    input.uds_age_ms = uds_valid ? 0 : 100;
    input.gyro_z_rad_s = gyro_z_rad_s;
    input.imu_valid = imu_valid;
    return input;
}

}  // namespace

TEST_CASE("UdsGyroYawFusion initializes from first valid UDS sample",
          "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    fusion.set_params(params);

    const auto out = fusion.update(make_input(0.02f, -2.0f, true, 0.0f));

    REQUIRE(out.valid);
    CHECK(out.fused_yaw_deg == Approx(-2.0f).margin(0.05f));
    CHECK(out.gyro_z_dps == Approx(0.0f));
}

TEST_CASE("UdsGyroYawFusion preserves gyro sign convention", "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.max_gyro_only_ms = 500;
    fusion.set_params(params);

    fusion.update(make_input(0.02f, 0.0f, true, 0.0f));
    const auto out = fusion.update(make_input(0.1f, 0.0f, false, -0.1745329f));

    REQUIRE(out.valid);
    CHECK(out.gyro_z_dps == Approx(-10.0f).margin(0.05f));
    CHECK(out.fused_yaw_deg < 0.0f);
}

TEST_CASE("UdsGyroYawFusion UDS update pulls prediction toward measurement",
          "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.measurement_noise_uds = 0.5f;
    fusion.set_params(params);

    fusion.update(make_input(0.02f, 0.0f, true, 0.0f));
    const auto out = fusion.update(make_input(0.02f, 4.0f, true, 0.0f));

    REQUIRE(out.valid);
    CHECK(out.fused_yaw_deg > 0.0f);
    CHECK(out.fused_yaw_deg < 4.1f);
    CHECK(out.innovation_deg > 0.0f);
}

TEST_CASE("UdsGyroYawFusion invalidates after gyro-only timeout",
          "[service][uds_gyro_fusion]") {
    UdsGyroYawFusion fusion;
    UdsGyroYawFusion::Params params;
    params.max_gyro_only_ms = 100;
    fusion.set_params(params);

    fusion.update(make_input(0.02f, 1.0f, true, 0.0f));
    const auto out = fusion.update(make_input(0.2f, 0.0f, false, 0.0f));

    CHECK_FALSE(out.valid);
}
