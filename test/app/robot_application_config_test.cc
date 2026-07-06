#include <catch2/catch.hpp>

#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/app/robot_application.h"
#include "pv_cleaning_robot/service/config_service.h"

TEST_CASE("RobotApplication motion config matches fused fast all correction profile",
          "[app][robot_application]") {
    auto paths = tb_test_support::make_temp_split_config_paths("robot_application_motion_config");
    tb_test_support::write_split_config(paths,
                                        R"({
  "robot": {
    "clean_speed_rpm": 20.0,
    "return_speed_rpm": 20.0,
    "brush_rpm": 1000,
    "heading_pid_en": true,
    "pid": {
      "uds_path": "/tmp/pv_edge_tracker.sock",
      "reconnect_interval_ms": 500,
      "result_timeout_ms": 500,
      "min_confidence": 0.6,
      "deadband_yaw_deg": 1.0,
      "kp": 5.0,
      "ki": 0.0,
      "kd": 0.0,
      "integral_limit": 1.0,
      "max_output": 10.0,
      "min_effective_output": 1.0,
      "yaw_alpha": 0.35,
      "output_sign": 1.0,
      "angle_source": "fused_uds_gyro",
      "wheel_strategy": "all_wheels",
      "slow_on_error": false,
      "slow_base_rpm": 15.0,
      "yaw_slow_threshold_deg": 1.0,
      "fusion": {
        "process_noise_angle": 0.05,
        "process_noise_bias": 0.001,
        "measurement_noise_uds": 0.5,
        "initial_angle_variance": 1.0,
        "initial_bias_variance": 1.0,
        "max_gyro_only_ms": 300
      }
    }
  }
})",
                                        R"({
  "installation": {
    "brush_direction_sign": -1
  }
})");

    robot::service::ConfigService cfg(paths.runtime_path.string(), paths.fixed_path.string());
    REQUIRE(cfg.load());

    const auto motion_cfg = robot::app::make_motion_config_from_config(cfg);

    CHECK(motion_cfg.clean_speed_rpm == Approx(20.0f));
    CHECK(motion_cfg.return_speed_rpm == Approx(20.0f));
    CHECK(motion_cfg.brush_rpm == 1000);
    CHECK(motion_cfg.brush_direction_sign == -1);
    CHECK(motion_cfg.heading_pid_en);
    CHECK(motion_cfg.control_dt_s == Approx(0.05f));
    CHECK(motion_cfg.pid.kp == Approx(5.0f));
    CHECK(motion_cfg.pid.max_output == Approx(10.0f));
    CHECK(motion_cfg.pid.angle_source ==
          robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO);
    CHECK(motion_cfg.pid.wheel_strategy ==
          robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS);
    CHECK_FALSE(motion_cfg.pid.slow_on_error);
    CHECK(motion_cfg.pid.slow_base_rpm == Approx(15.0f));
    CHECK(motion_cfg.pid.yaw_slow_threshold_deg == Approx(1.0f));
    CHECK(motion_cfg.pid.fusion.process_noise_angle == Approx(0.05f));
    CHECK(motion_cfg.pid.fusion.process_noise_bias == Approx(0.001f));
    CHECK(motion_cfg.pid.fusion.measurement_noise_uds == Approx(0.5f));
    CHECK(motion_cfg.pid.fusion.initial_angle_variance == Approx(1.0f));
    CHECK(motion_cfg.pid.fusion.initial_bias_variance == Approx(1.0f));
    CHECK(motion_cfg.pid.fusion.max_gyro_only_ms == 300);

    tb_test_support::cleanup_split_config_paths(paths);
}
