/**
 * @file robot_application.cc
 * @brief PV 清扫机器人应用生命周期编排实现。
 *
 * 本文件作为组合根加载运行配置和固定配置，构造 HAL、驱动、设备、中间件、服务和应用层对象，
 * 再按安全依赖顺序启动线程。硬件初始化失败时启动过程直接失败，退出时按固定顺序停止运动、
 * 监控、通信和缓存服务。
 */
#include <cmath>
#include <sys/mman.h>
#include <unistd.h>

#include "pv_cleaning_robot/app/robot_application.h"
#include "pv_cleaning_robot/middleware/logger.h"
#include "pv_cleaning_robot/service/config_service.h"

// HAL / Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"

// Device
#include "pv_cleaning_robot/device/attitude_limit_switch.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/lock_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Middleware
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/lorawan_transport.h"
#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

// Service
#include "pv_cleaning_robot/service/attitude_limit_service.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/diagnostics_collector.h"
#include "pv_cleaning_robot/service/gps_stuck_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

// App
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <string_view>
#include <thread>

#include "pv_cleaning_robot/app/error_manager.h"
#include "pv_cleaning_robot/app/recovery_executor.h"
#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

namespace {

// 主程序行走控制周期。MotionService::Config::control_dt_s 必须与该值一致，
// 否则视觉纠偏 PID / 融合预测会使用错误时间基准。
constexpr int kWalkCtrlPeriodMs = 50;
constexpr float kWalkCtrlPeriodS = static_cast<float>(kWalkCtrlPeriodMs) / 1000.0f;
constexpr int kWalkCtrlPriority = 80;
constexpr int kWalkCtrlCpuMask = 1 << 5;
constexpr int kWalkCtrlWatchdogMs = 500;
constexpr int kGpsStuckPeriodMs = 500;
constexpr int kGpsStuckPriority = 65;
constexpr int kGpsStuckCpuMask = 1 << 6;
constexpr int kGpsStuckWatchdogMs = 2000;
constexpr int kGpioMonitorPriority = 95;
constexpr int kGpioMonitorCpuMask = 1 << 4;
constexpr int kBackgroundCpuMask = 0x0F;
constexpr int kBmsWatchdogMs = 5000;
constexpr int kBrushWatchdogMs = 2000;
constexpr int kErrorManagerDefaultPeriodMs = 100;

uint64_t steady_now_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

std::string attitude_limit_key_from_snapshot(
    const robot::app::RobotControllerSnapshot& snapshot) {
    if (!snapshot.current_segment_target || !snapshot.current_segment_mode) {
        return "attitude_limit_key:unknown";
    }
    const char* mode =
        *snapshot.current_segment_mode == robot::domain::SegmentMode::Cleaning ? "Cleaning"
                                                                               : "Unknown";
    return std::string{"attitude_limit_key:target="} +
           robot::domain::endpoint_config_string(*snapshot.current_segment_target) +
           ";mode=" + mode;
}

bool should_log_runtime_diagnostics(std::string_view state) {
    return state == "ExecutingMission" || state == "Recovering" ||
           state == "SettlingEndpoint";
}

const char* optional_endpoint_name(const std::optional<robot::domain::Endpoint>& endpoint) {
    if (!endpoint.has_value()) {
        return "none";
    }
    return robot::domain::endpoint_config_string(*endpoint);
}

void log_runtime_diagnostics(
    const robot::app::RobotControllerSnapshot& controller_snapshot,
    const robot::service::DiagnosticsCollector::Snapshot& diagnostics_snapshot) {
    if (!should_log_runtime_diagnostics(controller_snapshot.state)) {
        return;
    }

    const auto& walk = diagnostics_snapshot.walk_diagnostics;
    const auto& brush = diagnostics_snapshot.brush_diagnostics;
    const auto& bms = diagnostics_snapshot.bms_diagnostics;
    spdlog::info(
        "[runtime_diag] state={} target={} progress={}/{} "
        "walk_target=[{:.1f},{:.1f},{:.1f},{:.1f}] "
        "walk_rpm=[{:.1f},{:.1f},{:.1f},{:.1f}] "
        "walk_a=[{:.2f},{:.2f},{:.2f},{:.2f}] "
        "brush_rpm={} brush_a={:.2f} brush_fault={} "
        "soc={:.1f} bms_a={:.2f} charging={} low_battery={}",
        controller_snapshot.state,
        optional_endpoint_name(controller_snapshot.current_segment_target),
        controller_snapshot.completed_cycles,
        controller_snapshot.repeat_count,
        walk.wheel[0].target_value,
        walk.wheel[1].target_value,
        walk.wheel[2].target_value,
        walk.wheel[3].target_value,
        walk.wheel[0].speed_rpm,
        walk.wheel[1].speed_rpm,
        walk.wheel[2].speed_rpm,
        walk.wheel[3].speed_rpm,
        walk.wheel[0].torque_a,
        walk.wheel[1].torque_a,
        walk.wheel[2].torque_a,
        walk.wheel[3].torque_a,
        brush.actual_rpm,
        brush.current_a,
        brush.fault_code,
        bms.soc_pct,
        bms.current_a,
        bms.charging,
        bms.low_battery);
}

void publish_startup_position_status(const std::shared_ptr<spdlog::logger>& log,
                                     robot::domain::PositionState startup_position,
                                     bool configured_mission_allowed) {
    if (startup_position == robot::domain::PositionState::AtA ||
        startup_position == robot::domain::PositionState::AtB) {
        if (configured_mission_allowed) {
            return;
        }
        log->warn("[Main] 启动位置异常：当前不在主停机端，禁止自动启动清扫");
        return;
    }
    if (startup_position == robot::domain::PositionState::Inconsistent) {
        log->warn("[Main] 启动位置异常：A/B 两端同时触发，禁止启动清扫");
        return;
    }
    log->warn("[Main] 启动位置异常：当前不在任一端点，禁止配置任务自动启动");
}

bool stream_ready_for_self_check(const robot::domain::StreamHealth& health,
                                 uint64_t now_ms,
                                 uint64_t max_age_ms) {
    return health.enabled && health.last_update_ms != 0 && now_ms >= health.last_update_ms &&
           now_ms - health.last_update_ms <= max_age_ms;
}

bool self_check_ready(const robot::service::DiagnosticsCollector::Snapshot& snapshot,
                      uint64_t now_ms) {
    // 自检只做确定性健康门槛：关键通信流必须已有真实计数，且最近 3s 内更新。
    // 更细的业务质量（例如 GPS 是否定位、BMS 电量门槛）仍由各自既有启动校验负责。
    constexpr uint64_t kSelfCheckStreamMaxAgeMs = 3000;
    if (snapshot.ts_ms == 0) {
        return false;
    }
    if (snapshot.bms_diagnostics.update_count == 0 ||
        snapshot.gps_diagnostics.sentence_count == 0 || snapshot.imu_diagnostics.frame_count == 0) {
        return false;
    }
    if (!stream_ready_for_self_check(snapshot.error.bms_update, now_ms, kSelfCheckStreamMaxAgeMs) ||
        !stream_ready_for_self_check(snapshot.error.gps, now_ms, kSelfCheckStreamMaxAgeMs) ||
        !stream_ready_for_self_check(snapshot.error.imu, now_ms, kSelfCheckStreamMaxAgeMs)) {
        return false;
    }
    if (!snapshot.error.walk_feedback_expected) {
        return false;
    }
    for (auto i = 0U; i < snapshot.error.walk_feedback.size(); ++i) {
        if (snapshot.walk_diagnostics.wheel[i].feedback_frame_count == 0 ||
            !stream_ready_for_self_check(
                snapshot.error.walk_feedback[i], now_ms, kSelfCheckStreamMaxAgeMs)) {
            return false;
        }
    }
    return !snapshot.error.walk_stall_active && !snapshot.error.brush_fault_active &&
           !snapshot.error.gps_stuck;
}

constexpr uint64_t kSelfCheckTimeoutMs = 30000;

struct GpioOpenStatus {
    bool left_limit{false};
    bool right_limit{false};
    bool left_attitude{false};
    bool right_attitude{false};
};

std::unique_ptr<robot::service::ConfigService> load_config_or_fallback() {
    auto cfg_ptr = std::make_unique<robot::service::ConfigService>(
        "/opt/robot/config/config.runtime.json", "/opt/robot/config/config.fixed.json");
    if (cfg_ptr->load()) {
        return cfg_ptr;
    }

    cfg_ptr = std::make_unique<robot::service::ConfigService>("config/config.runtime.json",
                                                              "config/config.fixed.json");
    if (!cfg_ptr->load()) {
        fprintf(stderr, "[FATAL] 无法加载 config.runtime.json/config.fixed.json\n");
        return nullptr;
    }
    return cfg_ptr;
}

std::shared_ptr<spdlog::logger> initialize_logging(robot::service::ConfigService& cfg) {
    robot::middleware::Logger::Config log_cfg;
    log_cfg.log_dir = cfg.get<std::string>("logging.log_dir", "logs");
    log_cfg.level = cfg.get<std::string>("logging.level", "info");
    log_cfg.console_output = cfg.get<bool>("logging.console", true);
    robot::middleware::Logger::init(log_cfg);
    auto log = robot::middleware::Logger::get();
    log->info("[Main] 配置加载完成，日志已初始化");
    return log;
}

void lock_memory_pages(const std::shared_ptr<spdlog::logger>& log) {
    // MCL_CURRENT: 锁定当前已映射页；MCL_FUTURE: 锁定后续 mmap/堆增长页。
    // 需要 CAP_IPC_LOCK 或运行为 root；失败仅警告，不阻止启动。
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        log->warn(
            "[Main] mlockall(MCL_CURRENT|MCL_FUTURE) 失败: {} "
            "（内存分页中断可能增加 RT 延迟抖动，建议以 root 运行或设置 RLIMIT_MEMLOCK）",
            strerror(errno));
        return;
    }
    log->info("[Main] 内存已全部锁定，RT 延迟抖动最小化");
}

robot::device::LockMotor::Config make_lock_motor_config(robot::service::ConfigService& cfg) {
    robot::device::LockMotor::Config lock_motor_cfg;
    lock_motor_cfg.pulse_ms =
        static_cast<uint32_t>(std::max(0, cfg.get_fixed<int>("gpio.lock_motor.pulse_ms", 200)));
    lock_motor_cfg.settle_ms =
        static_cast<uint32_t>(std::max(0, cfg.get_fixed<int>("gpio.lock_motor.settle_ms", 8000)));
    return lock_motor_cfg;
}

robot::middleware::MqttTransport::Config make_mqtt_config(robot::service::ConfigService& cfg) {
    robot::middleware::MqttTransport::Config mqtt_cfg;
    mqtt_cfg.broker_uri = cfg.get<std::string>("network.mqtt.broker_uri", "tcp://localhost:1883");
    mqtt_cfg.client_id = cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001");
    mqtt_cfg.tls_enabled = cfg.get<bool>("network.mqtt.tls_enabled", false);
    mqtt_cfg.username = cfg.get<std::string>("network.mqtt.username", "");
    mqtt_cfg.password = cfg.get<std::string>("network.mqtt.password", "");
    mqtt_cfg.ca_cert_path = cfg.get<std::string>("network.mqtt.ca_cert_path", "");
    mqtt_cfg.client_cert_path = cfg.get<std::string>("network.mqtt.client_cert_path", "");
    mqtt_cfg.client_key_path = cfg.get<std::string>("network.mqtt.client_key_path", "");
    mqtt_cfg.insecure_skip_server_name_check =
        cfg.get<bool>("network.mqtt.insecure_skip_server_name_check", false);
    mqtt_cfg.keep_alive_sec = cfg.get<int>("network.mqtt.keep_alive_s", 60);
    mqtt_cfg.qos = cfg.get<int>("network.mqtt.qos", 1);
    return mqtt_cfg;
}

robot::middleware::NetworkManager::Mode network_mode_from_config(
    robot::service::ConfigService& cfg) {
    const std::string transport_mode = cfg.get<std::string>("network.transport_mode", "mqtt_only");
    if (transport_mode == "lorawan_only") {
        return robot::middleware::NetworkManager::Mode::LORAWAN_ONLY;
    }
    if (transport_mode == "dual_parallel") {
        return robot::middleware::NetworkManager::Mode::DUAL_PARALLEL;
    }
    return robot::middleware::NetworkManager::Mode::MQTT_ONLY;
}

robot::service::HeadingCorrector::AngleSource parse_angle_source(
    const std::string& value,
    robot::service::HeadingCorrector::AngleSource fallback) {
    if (value == "fused_uds_gyro") {
        return robot::service::HeadingCorrector::AngleSource::FUSED_UDS_GYRO;
    }
    if (value == "raw_uds") {
        return robot::service::HeadingCorrector::AngleSource::RAW_UDS;
    }
    return fallback;
}

robot::service::HeadingCorrector::WheelStrategy parse_wheel_strategy(
    const std::string& value,
    robot::service::HeadingCorrector::WheelStrategy fallback) {
    if (value == "all_wheels") {
        return robot::service::HeadingCorrector::WheelStrategy::ALL_WHEELS;
    }
    if (value == "lower_only") {
        return robot::service::HeadingCorrector::WheelStrategy::LOWER_ONLY;
    }
    if (value == "top_decel_only") {
        return robot::service::HeadingCorrector::WheelStrategy::TOP_DECEL_ONLY;
    }
    return fallback;
}

robot::service::MotionService::Config read_motion_config_fields(
    robot::service::ConfigService& cfg) {
    robot::service::MotionService::Config motion_cfg;
    motion_cfg.clean_speed_rpm = cfg.get<float>("robot.clean_speed_rpm", 300.0f);
    motion_cfg.return_speed_rpm = cfg.get<float>("robot.return_speed_rpm", 300.0f);
    motion_cfg.brush_rpm = cfg.get<int>("robot.brush_rpm", 1000);
    motion_cfg.brush_direction_sign =
        cfg.get_fixed<int>("installation.brush_direction_sign", motion_cfg.brush_direction_sign);
    motion_cfg.heading_pid_en = cfg.get<bool>("robot.heading_pid_en", false);
    motion_cfg.control_dt_s = kWalkCtrlPeriodS;
    // 视觉纠偏参数从 robot.pid.* 读取；未配置时使用控制器头文件默认值。
    motion_cfg.pid.uds_path = cfg.get<std::string>("robot.pid.uds_path", motion_cfg.pid.uds_path);
    motion_cfg.pid.reconnect_interval_ms =
        cfg.get<int>("robot.pid.reconnect_interval_ms", motion_cfg.pid.reconnect_interval_ms);
    motion_cfg.pid.result_timeout_ms =
        cfg.get<int>("robot.pid.result_timeout_ms", motion_cfg.pid.result_timeout_ms);
    motion_cfg.pid.min_confidence =
        cfg.get<float>("robot.pid.min_confidence", motion_cfg.pid.min_confidence);
    motion_cfg.pid.deadband_yaw_deg = cfg.get<float>(
        "robot.pid.deadband_yaw_deg",
        cfg.get<float>("robot.pid.deadband_slope",
                       cfg.get<float>("robot.pid.deadband_norm", motion_cfg.pid.deadband_yaw_deg)));
    motion_cfg.pid.kp = cfg.get<float>("robot.pid.kp", motion_cfg.pid.kp);
    motion_cfg.pid.ki = cfg.get<float>("robot.pid.ki", motion_cfg.pid.ki);
    motion_cfg.pid.kd = cfg.get<float>("robot.pid.kd", motion_cfg.pid.kd);
    motion_cfg.pid.integral_limit =
        cfg.get<float>("robot.pid.integral_limit", motion_cfg.pid.integral_limit);
    motion_cfg.pid.max_output = cfg.get<float>("robot.pid.max_output", motion_cfg.pid.max_output);
    motion_cfg.pid.min_effective_output =
        cfg.get<float>("robot.pid.min_effective_output", motion_cfg.pid.min_effective_output);
    motion_cfg.pid.yaw_alpha = cfg.get<float>(
        "robot.pid.yaw_alpha",
        cfg.get<float>("robot.pid.slope_alpha",
                       cfg.get<float>("robot.pid.offset_alpha", motion_cfg.pid.yaw_alpha)));
    motion_cfg.pid.output_sign =
        cfg.get<float>("robot.pid.output_sign", motion_cfg.pid.output_sign);
    // 下面这些字段用于把硬件测试验证过的 corr_fused_fast_all 纠偏策略带到主程序：
    // 融合角、上下轮共同纠偏、错误时不降速，以及 UDS/IMU 融合滤波参数。
    motion_cfg.pid.angle_source = parse_angle_source(
        cfg.get<std::string>("robot.pid.angle_source", "raw_uds"),
        motion_cfg.pid.angle_source);
    motion_cfg.pid.wheel_strategy = parse_wheel_strategy(
        cfg.get<std::string>("robot.pid.wheel_strategy", "all_wheels"),
        motion_cfg.pid.wheel_strategy);
    motion_cfg.pid.slow_on_error =
        cfg.get<bool>("robot.pid.slow_on_error", motion_cfg.pid.slow_on_error);
    motion_cfg.pid.slow_base_rpm =
        cfg.get<float>("robot.pid.slow_base_rpm", motion_cfg.pid.slow_base_rpm);
    motion_cfg.pid.yaw_slow_threshold_deg =
        cfg.get<float>("robot.pid.yaw_slow_threshold_deg",
                       motion_cfg.pid.yaw_slow_threshold_deg);
    motion_cfg.pid.fusion.process_noise_angle =
        cfg.get<float>("robot.pid.fusion.process_noise_angle",
                       motion_cfg.pid.fusion.process_noise_angle);
    motion_cfg.pid.fusion.process_noise_bias =
        cfg.get<float>("robot.pid.fusion.process_noise_bias",
                       motion_cfg.pid.fusion.process_noise_bias);
    motion_cfg.pid.fusion.measurement_noise_uds =
        cfg.get<float>("robot.pid.fusion.measurement_noise_uds",
                       motion_cfg.pid.fusion.measurement_noise_uds);
    motion_cfg.pid.fusion.initial_angle_variance =
        cfg.get<float>("robot.pid.fusion.initial_angle_variance",
                       motion_cfg.pid.fusion.initial_angle_variance);
    motion_cfg.pid.fusion.initial_bias_variance =
        cfg.get<float>("robot.pid.fusion.initial_bias_variance",
                       motion_cfg.pid.fusion.initial_bias_variance);
    motion_cfg.pid.fusion.max_gyro_only_ms =
        cfg.get<int>("robot.pid.fusion.max_gyro_only_ms",
                     motion_cfg.pid.fusion.max_gyro_only_ms);
    return motion_cfg;
}

robot::service::GpsStuckConfig read_gps_stuck_config(robot::service::ConfigService& cfg) {
    robot::service::GpsStuckConfig gps_cfg;
    gps_cfg.min_fix_quality =
        cfg.get_fixed<uint8_t>("gps_stuck.min_fix_quality", gps_cfg.min_fix_quality);
    gps_cfg.min_satellites_used =
        cfg.get_fixed<uint8_t>("gps_stuck.min_satellites_used", gps_cfg.min_satellites_used);
    gps_cfg.max_hdop = cfg.get_fixed<float>("gps_stuck.max_hdop", gps_cfg.max_hdop);
    gps_cfg.max_pdop = cfg.get_fixed<float>("gps_stuck.max_pdop", gps_cfg.max_pdop);
    gps_cfg.moving_speed_mps =
        cfg.get_fixed<float>("gps_stuck.moving_speed_mps", gps_cfg.moving_speed_mps);
    gps_cfg.moving_speed_confirm_samples = cfg.get_fixed<int>(
        "gps_stuck.moving_speed_confirm_samples",
        gps_cfg.moving_speed_confirm_samples);
    gps_cfg.stuck_timeout = std::chrono::milliseconds(
        cfg.get_fixed<int>("gps_stuck.stuck_timeout_ms",
                           static_cast<int>(gps_cfg.stuck_timeout.count())));
    gps_cfg.sample_stale_timeout = std::chrono::milliseconds(
        cfg.get_fixed<int>("gps_stuck.sample_stale_timeout_ms",
                           static_cast<int>(gps_cfg.sample_stale_timeout.count())));
    return gps_cfg;
}

robot::service::AttitudeLimitService::CenterConfig read_attitude_center_config(
    robot::service::ConfigService& cfg) {
    robot::service::AttitudeLimitService::CenterConfig center_cfg;
    center_cfg.lower_rpm =
        cfg.get_fixed<float>("recovery.attitude_center.lower_rpm", center_cfg.lower_rpm);
    center_cfg.stable_samples_required = cfg.get_fixed<int>(
        "recovery.attitude_center.stable_samples_required",
        center_cfg.stable_samples_required);
    center_cfg.overall_timeout = std::chrono::milliseconds(
        cfg.get_fixed<int>("recovery.attitude_center.timeout_ms",
                           static_cast<int>(center_cfg.overall_timeout.count())));
    center_cfg.tick = std::chrono::milliseconds(
        cfg.get_fixed<int>("recovery.attitude_center.tick_ms",
                           static_cast<int>(center_cfg.tick.count())));
    return center_cfg;
}

robot::app::ErrorManager::Config read_error_manager_config(robot::service::ConfigService& cfg) {
    robot::app::ErrorManager::Config error_cfg;
    error_cfg.consecutive_error_limit = cfg.get_fixed<uint32_t>(
        "error_manager.consecutive_error_limit",
        error_cfg.consecutive_error_limit);
    error_cfg.stream_timeout_ms =
        cfg.get_fixed<uint64_t>("error_manager.stream_timeout_ms", error_cfg.stream_timeout_ms);
    error_cfg.walk_stall_duration_ms = cfg.get_fixed<uint64_t>(
        "error_manager.walk_stall_duration_ms",
        error_cfg.walk_stall_duration_ms);
    error_cfg.attitude_repeat_gap_ms = cfg.get_fixed<uint64_t>(
        "error_manager.attitude_repeat_gap_ms",
        error_cfg.attitude_repeat_gap_ms);
    error_cfg.attitude_reverse_attempt_count = cfg.get_fixed<uint32_t>(
        "error_manager.attitude_reverse_attempt_count",
        error_cfg.attitude_reverse_attempt_count);
    error_cfg.attitude_fault_count =
        cfg.get_fixed<uint32_t>("error_manager.attitude_fault_count",
                                error_cfg.attitude_fault_count);
    return error_cfg;
}

robot::middleware::SafetyMonitor::Config read_safety_monitor_config(
    robot::service::ConfigService& cfg) {
    robot::middleware::SafetyMonitor::Config safety_cfg;
    safety_cfg.limit_settle_stable_ms = cfg.get_fixed<uint64_t>(
        "safety.limit_settle_stable_ms",
        safety_cfg.limit_settle_stable_ms);
    safety_cfg.limit_release_stable_ms = cfg.get_fixed<uint64_t>(
        "safety.limit_release_stable_ms",
        safety_cfg.limit_release_stable_ms);
    return safety_cfg;
}

bool initialize_devices(
    const std::shared_ptr<spdlog::logger>& log,
    const std::shared_ptr<robot::device::WalkMotorGroup>& walk_group,
    const std::shared_ptr<robot::device::ImuDevice>& imu,
    const std::shared_ptr<robot::device::GpsDevice>& gps,
    const std::shared_ptr<robot::device::LimitSwitch>& left_switch,
    const std::shared_ptr<robot::device::LimitSwitch>& right_switch,
    const std::shared_ptr<robot::device::AttitudeLimitSwitch>& left_attitude_switch,
    const std::shared_ptr<robot::device::AttitudeLimitSwitch>& right_attitude_switch,
    const std::shared_ptr<robot::device::LockMotor>& lock_motor,
    const std::shared_ptr<robot::device::BrushMotor>& brush_motor,
    const std::shared_ptr<robot::device::BMS>& bms,
    robot::service::ConfigService& cfg,
    GpioOpenStatus& gpio_status) {
    // 行走电机组是运动控制核心（open 失败则整体无法运行）。
    if (walk_group->open() != robot::device::DeviceError::OK) {
        log->error("[Main] walk_group CAN 初始化失败，退出");
        return false;
    }
    const auto walk_feedback_period_ms =
        cfg.get_fixed<uint8_t>("can.walk_motor.feedback_period_ms", 10u);
    // 主动配置电机反馈方式：默认 10ms 主动上报（100Hz），与运动控制采样对齐。
    // 不依赖上次写入 EEPROM 的值，确保每次上电行为确定性。
    if (walk_group->set_feedback_mode_all(walk_feedback_period_ms) !=
        robot::device::DeviceError::OK) {
        log->warn("[Main] 电机反馈模式配置失败，将使用硬件保存值");
    } else {
        log->info("[Main] 电机反馈配置：{}ms 主动上报",
                  static_cast<int>(walk_feedback_period_ms));
    }

    if (!imu->open()) {
        log->warn("[Main] IMU 初始化失败");
    } else {
        const auto imu_output_rate_hz = cfg.get_fixed<int>("serial.imu.output_rate_hz", 100);
        // 主动配置 IMU 输出频率，默认 100Hz（RRATE=0x09），与 imu_read 线程和姿态纠偏周期对齐。
        // 不依赖硬件 EEPROM 保存值，确保上电后频率确定。
        if (imu->set_output_rate(imu_output_rate_hz) != robot::device::DeviceError::OK) {
            log->warn("[Main] IMU 频率配置失败，将使用硬件保存值");
        } else {
            log->info("[Main] IMU 输出频率配置：{}Hz", imu_output_rate_hz);
        }
    }
    if (!gps->open()) {
        log->warn("[Main] GPS 初始化失败");
    }

    // gpio.use_irq: 若 GPIO 控制器不支持硬件 IRQ（如 RK3576 gpiochip5），配置为 false 使用
    // 驱动层固定周期软件轮询。pca953x 这类 I2C GPIO 扩展芯片不能承受多路 1ms 高频并发读。
    const bool gpio_use_irq = cfg.get<bool>("gpio.use_irq", false);
    const int gpio_input_debounce_ms = cfg.get_fixed<int>("gpio.input_debounce_ms", 2);
    gpio_status.left_limit = left_switch->open(
        kGpioMonitorPriority, gpio_input_debounce_ms, kGpioMonitorCpuMask, gpio_use_irq);
    gpio_status.right_limit = right_switch->open(
        kGpioMonitorPriority, gpio_input_debounce_ms, kGpioMonitorCpuMask, gpio_use_irq);
    if (!gpio_status.left_limit) {
        log->warn("[Main] 左限位开关初始化失败");
    }
    if (!gpio_status.right_limit) {
        log->warn("[Main] 右限位开关初始化失败");
    }

    // 姿态限位用于急停和回中测量，RK3576 对应 GPIO 线按驱动层固定周期软件轮询处理；
    // 回中流程本身也通过 read_current_level() 轮询判断“刚释放/刚触发”。
    gpio_status.left_attitude =
        left_attitude_switch->open(
            kGpioMonitorPriority, gpio_input_debounce_ms, kGpioMonitorCpuMask, false);
    gpio_status.right_attitude =
        right_attitude_switch->open(
            kGpioMonitorPriority, gpio_input_debounce_ms, kGpioMonitorCpuMask, false);
    if (!gpio_status.left_attitude) {
        log->warn("[Main] 左姿态限位开关初始化失败");
    }
    if (!gpio_status.right_attitude) {
        log->warn("[Main] 右姿态限位开关初始化失败");
    }

    // 锁止电机只有 DO 控制，没有反馈信号。初始化失败不直接退出主程序；
    // 后续任务启动/完成时执行开关动作失败，会由 RobotController 锁存 FaultStopped。
    if (!lock_motor->initialize()) {
        log->error("[Main] 锁止电机 GPIO 初始化失败，后续开/关动作会进入 FaultStopped");
    }

    if (!brush_motor->open()) {
        log->warn("[Main] 滚刷电机 RS485 初始化失败");
    }
    if (bms->open() != robot::device::DeviceError::OK) {
        log->warn("[Main] BMS RS485 初始化失败");
    }
    return true;
}

void run_application_loop(
    const std::atomic<bool>& running,
    robot::service::SchedulerService& scheduler,
    const std::shared_ptr<robot::app::RobotController>& controller,
    const std::shared_ptr<robot::service::DiagnosticsCollector>& diagnostics_collector,
    robot::middleware::ThreadExecutor& cloud_exec,
    int active_report_period,
    int idle_report_period,
    robot::middleware::ThreadExecutor* local_diag_exec,
    int active_local_log_period,
    int idle_local_log_period) {
    // 主循环只推进低频业务编排：调度窗口、自检完成条件、云端上报周期和控制器 tick。
    // RT 运动控制、诊断采集、错误处理和云端发送均由各自 ThreadExecutor 执行。
    std::optional<uint64_t> self_check_started_ms;
    while (running.load()) {
        scheduler.tick();

        const std::string current_state = controller->snapshot().state;
        const auto now_ms = steady_now_ms();
        if (current_state == "SelfChecking") {
            if (!self_check_started_ms) {
                self_check_started_ms = now_ms;
            }
            if (self_check_ready(diagnostics_collector->snapshot(), now_ms)) {
                self_check_started_ms.reset();
                controller->complete_self_check(true);
            } else if (now_ms - *self_check_started_ms >= kSelfCheckTimeoutMs) {
                // 自检只等待关键诊断流变为健康，不在该状态执行自动恢复；
                // 30s 后仍不满足启动门槛，锁存自检失败并等待人工复位。
                self_check_started_ms.reset();
                controller->complete_self_check(false);
            }
        } else {
            self_check_started_ms.reset();
        }

        const bool active_motion_state =
            current_state == "ExecutingMission" || current_state == "Recovering";
        const int desired_report_period =
            active_motion_state ? active_report_period : idle_report_period;
        if (cloud_exec.period_ms() != desired_report_period) {
            cloud_exec.set_period_ms(desired_report_period);
        }
        if (local_diag_exec) {
            const int desired_local_log_period =
                active_motion_state ? active_local_log_period : idle_local_log_period;
            if (local_diag_exec->period_ms() != desired_local_log_period) {
                local_diag_exec->set_period_ms(desired_local_log_period);
            }
        }
        controller->post_tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool stop_executor_with_timeout(const std::shared_ptr<spdlog::logger>& log,
                                const char* name,
                                robot::middleware::ThreadExecutor& executor,
                                std::chrono::milliseconds timeout) {
    if (executor.stop_with_timeout(timeout)) {
        return true;
    }
    // 退出阶段必须显式暴露未收敛线程。这里不 detach 线程，避免硬件句柄仍被后台访问；
    // 如果日志出现该错误，说明对应模块还缺少协作取消或底层 I/O 仍可能长期阻塞。
    log->error("[Main] 线程 {} 在 {}ms 内未退出", name, timeout.count());
    return false;
}

bool shutdown_runtime(
    const std::shared_ptr<spdlog::logger>& log,
    robot::middleware::ThreadExecutor& walk_exec,
    robot::middleware::ThreadExecutor& gps_stuck_exec,
    robot::middleware::ThreadExecutor& bms_exec,
    robot::middleware::ThreadExecutor& brush_exec,
    robot::middleware::ThreadExecutor& error_exec,
    robot::middleware::ThreadExecutor& diagnostics_exec,
    robot::middleware::ThreadExecutor* local_diag_exec,
    robot::middleware::ThreadExecutor& cloud_exec,
    const std::shared_ptr<robot::service::AttitudeLimitService>& attitude_limit,
    robot::middleware::SafetyMonitor& safety_monitor,
    robot::app::WatchdogMgr& watchdog,
    const std::shared_ptr<robot::app::RobotController>& controller,
    const std::shared_ptr<robot::service::MotionService>& motion,
    const std::shared_ptr<robot::device::BrushMotor>& brush_motor,
    const std::shared_ptr<robot::device::WalkMotorGroup>& walk_group,
    const std::shared_ptr<robot::device::ImuDevice>& imu,
    const std::shared_ptr<robot::device::GpsDevice>& gps,
    const std::shared_ptr<robot::device::BMS>& bms,
    const std::shared_ptr<robot::device::LimitSwitch>& left_switch,
    const std::shared_ptr<robot::device::LimitSwitch>& right_switch,
    const std::shared_ptr<robot::device::AttitudeLimitSwitch>& left_attitude_switch,
    const std::shared_ptr<robot::device::AttitudeLimitSwitch>& right_attitude_switch,
    const std::shared_ptr<robot::middleware::NetworkManager>& net_mgr,
    const std::shared_ptr<robot::middleware::DataCache>& data_cache) {
    // 关闭顺序保持原 main 逻辑：先停调度线程，再停安全/看门狗/控制器，最后停运动和释放设备。
    // 对可能阻塞在串口事务的设备，先发协作停止请求，再等待 ThreadExecutor 收敛。
    log->info("[Main] 收到退出信号，正在关闭...");
    bms->request_stop();
    brush_motor->request_stop();
    gps->request_stop();
    imu->request_stop();
    net_mgr->request_stop();
    bool stopped_cleanly = true;
    stopped_cleanly &= stop_executor_with_timeout(log, "walk_ctrl", walk_exec, std::chrono::milliseconds(1000));
    stopped_cleanly &= stop_executor_with_timeout(log, "gps_stuck", gps_stuck_exec, std::chrono::milliseconds(1000));
    stopped_cleanly &= stop_executor_with_timeout(log, "bms", bms_exec, std::chrono::milliseconds(2000));
    stopped_cleanly &= stop_executor_with_timeout(log, "brush", brush_exec, std::chrono::milliseconds(2000));
    stopped_cleanly &= stop_executor_with_timeout(log, "error_mgr", error_exec, std::chrono::milliseconds(1000));
    stopped_cleanly &= stop_executor_with_timeout(log, "diagnostics", diagnostics_exec, std::chrono::milliseconds(1000));
    if (local_diag_exec) {
        stopped_cleanly &= stop_executor_with_timeout(
            log, "local_diag", *local_diag_exec, std::chrono::milliseconds(2000));
    }
    stopped_cleanly &= stop_executor_with_timeout(log, "cloud", cloud_exec, std::chrono::milliseconds(2000));
    attitude_limit->stop_monitoring();
    safety_monitor.stop();
    watchdog.stop();
    controller->stop();
    motion->emergency_stop();
    brush_motor->close();
    walk_group->close();
    imu->close();
    gps->close();
    bms->close();
    left_switch->close();
    right_switch->close();
    left_attitude_switch->close();
    right_attitude_switch->close();
    net_mgr->disconnect();
    data_cache->close();
    if (!stopped_cleanly) {
        log->error("[Main] 部分运行线程未在超时时间内退出，关闭流程不完整");
        return false;
    }
    log->info("[Main] 正常退出");
    return true;
}

void connect_cloud_and_publish_startup(
    const std::atomic<bool>& running,
    const std::shared_ptr<spdlog::logger>& log,
    robot::service::ConfigService& cfg,
    const std::shared_ptr<robot::service::ThingsBoardControlPlane>& tb_control,
    const std::shared_ptr<robot::middleware::NetworkManager>& net_mgr,
    robot::domain::PositionState startup_state,
    bool dual_dock_mode) {
    tb_control->subscribe_shared_attributes();
    // 在 connect() 前完成 shared attributes / RPC 注册，避免首个云端下行消息丢失。
    tb_control->register_rpc_handlers();

    if (!running.load(std::memory_order_acquire)) {
        net_mgr->request_stop();
        return;
    }

    net_mgr->connect(&running);

    if (!running.load(std::memory_order_acquire)) {
        net_mgr->request_stop();
        return;
    }

    if (net_mgr->is_connected()) {
        tb_control->request_shared_attributes_snapshot();
        if (cfg.last_load_used_backup()) {
            log->warn("[Main] 主配置加载失败，已从 backup 配置回退启动");
        }
        tb_control->publish_startup_attributes();
        log->info("[Main] 设备静态属性已发布至云端");
    }

    publish_startup_position_status(
        log,
        startup_state,
        robot::domain::can_start_configured_mission(
            robot::domain::LaneConfig{dual_dock_mode ? robot::domain::DockMode::DualDock
                                                     : robot::domain::DockMode::SingleDock,
                                      cfg.active_runtime_config().primary_dock},
            startup_state));
}

bool start_runtime_threads(const std::shared_ptr<spdlog::logger>& log,
                           robot::middleware::ThreadExecutor& walk_exec,
                           robot::middleware::ThreadExecutor& gps_stuck_exec,
                           robot::middleware::ThreadExecutor& bms_exec,
                           robot::middleware::ThreadExecutor& brush_exec,
                           robot::middleware::ThreadExecutor& diagnostics_exec,
                           robot::middleware::ThreadExecutor& error_exec,
                           robot::middleware::ThreadExecutor* local_diag_exec,
                           robot::middleware::ThreadExecutor& cloud_exec) {
    // 启动顺序保持原业务流程：先运动/设备采集，再诊断、错误处理和云端上报。
    if (!walk_exec.start()) {
        log->error("[Main] walk_ctrl 启动失败");
        return false;
    }
    if (!gps_stuck_exec.start()) {
        log->error("[Main] gps_stuck 启动失败");
        return false;
    }
    if (!bms_exec.start()) {
        log->error("[Main] bms 启动失败");
        return false;
    }
    if (!brush_exec.start()) {
        log->error("[Main] brush 启动失败");
        return false;
    }
    if (!diagnostics_exec.start()) {
        log->error("[Main] diagnostics 启动失败");
        return false;
    }
    if (!error_exec.start()) {
        log->error("[Main] error_mgr 启动失败");
        return false;
    }
    if (local_diag_exec && !local_diag_exec->start()) {
        log->warn("[Main] local_diag 启动失败，本地高频诊断日志不可用");
    }
    if (!cloud_exec.start()) {
        log->error("[Main] cloud 启动失败，云端上报/RPC 不可用，机器人本体继续运行");
    }
    log->info("[Main] 核心运行线程启动完成，进入主循环...");
    return true;
}

robot::app::RecoveryExecutor::Ports make_recovery_ports(
    const std::shared_ptr<robot::service::GpsStuckService>& gps_stuck,
    const std::shared_ptr<robot::service::MotionService>& motion,
    const std::shared_ptr<robot::service::AttitudeLimitService>& attitude_limit,
    robot::service::AttitudeLimitService::CenterConfig attitude_center_config,
    std::chrono::milliseconds reverse_duration,
    std::chrono::milliseconds reverse_tick) {
    robot::app::RecoveryExecutor::Ports recovery_ports;
    recovery_ports.pause_gps_stuck = [gps_stuck] { gps_stuck->set_monitoring_enabled(false); };
    recovery_ports.resume_gps_stuck = [gps_stuck] {
        // 恢复执行器只负责恢复前后成对开关；最终是否启用由 ErrorHandlingService 按状态统一校正。
        gps_stuck->set_monitoring_enabled(true);
    };
    recovery_ports.stop_walk = [motion] {
        motion->emergency_stop();
        return true;
    };
    recovery_ports.reverse_walk_motion = [motion, reverse_duration, reverse_tick] {
        return motion->reverse_for_recovery(reverse_duration, reverse_tick, [] { return false; });
    };
    recovery_ports.lower_attitude_center = [attitude_limit, attitude_center_config] {
        const auto result = attitude_limit->lower_attitude_center(attitude_center_config);
        switch (result.outcome) {
        case robot::service::AttitudeLimitService::CenterOutcome::Completed:
            return robot::app::RecoveryStepResult{
                robot::app::RecoveryStepOutcome::Completed};
        case robot::service::AttitudeLimitService::CenterOutcome::TimedOut:
            return robot::app::RecoveryStepResult{robot::app::RecoveryStepOutcome::TimedOut};
        case robot::service::AttitudeLimitService::CenterOutcome::InterruptedBySafetyOverride:
            return robot::app::RecoveryStepResult{
                robot::app::RecoveryStepOutcome::InterruptedBySafetyOverride};
        }
        return robot::app::RecoveryStepResult{robot::app::RecoveryStepOutcome::TimedOut};
    };
    return recovery_ports;
}

std::shared_ptr<robot::app::ErrorHandlingService> make_error_handling_service(
    const std::shared_ptr<robot::app::ErrorManager>& error_manager,
    const std::shared_ptr<robot::app::RobotController>& controller,
    const std::shared_ptr<robot::service::AttitudeLimitService>& attitude_limit,
    const std::shared_ptr<robot::service::DiagnosticsCollector>& diagnostics_collector,
    const std::shared_ptr<robot::service::GpsStuckService>& gps_stuck,
    robot::app::RecoveryExecutor& recovery_executor) {
    return std::make_shared<robot::app::ErrorHandlingService>(
        *error_manager,
        robot::app::ErrorHandlingService::Ports{
            [controller] { return controller->snapshot().state; },
            [attitude_limit, controller]() -> std::optional<robot::app::ErrorFact> {
                if (const auto event = attitude_limit->consume_pending_event()) {
                    const auto code =
                        event->type ==
                                robot::service::AttitudeLimitService::EventType::AttitudeLimitBoth
                            ? robot::app::ErrorCode::AttitudeLimitBoth
                            : robot::app::ErrorCode::AttitudeLimit;
                    const auto detail =
                        code == robot::app::ErrorCode::AttitudeLimit
                            ? attitude_limit_key_from_snapshot(controller->snapshot())
                            : "attitude_limit_switch";
                    return robot::app::ErrorFact{
                        code,
                        robot::app::ComponentId{robot::app::ComponentKind::AttitudeLimitSwitch,
                                                static_cast<int>(event->side)},
                        detail,
                        steady_now_ms()};
                }
                return std::nullopt;
            },
            [diagnostics_collector] { return diagnostics_collector->error_snapshot(); },
            [gps_stuck](bool enabled) { gps_stuck->set_monitoring_enabled(enabled); },
            [controller](const robot::app::ErrorDecision& decision) {
                controller->apply_error_decision(decision);
            },
            [&recovery_executor](const robot::app::ErrorDecision& decision) {
                const auto result = recovery_executor.execute(
                    robot::app::RecoveryRequest{decision.plan, decision.component});
                return robot::app::RecoveryResultFact{
                    decision, result.ok, result.reason, steady_now_ms()};
            },
            [controller](bool ok) { controller->post_recovery_finished(ok); },
            [] { return steady_now_ms(); },
        });
}

}  // namespace

namespace robot::app {

service::MotionService::Config make_motion_config_from_config(service::ConfigService& cfg) {
    return read_motion_config_fields(cfg);
}

int RobotApplication::run(const std::atomic<bool>& running) {
    // 配置服务必须最先初始化，后续硬件路径、线程周期和安全阈值均来自配置。
    auto cfg_ptr = load_config_or_fallback();
    if (!cfg_ptr) {
        return 1;
    }
    auto& cfg = *cfg_ptr;

    // 日志初始化依赖配置路径；后续启动失败都通过该 logger 上报。
    auto log = initialize_logging(cfg);

    // 锁定内存页以降低实时线程运行期缺页中断带来的延迟抖动。
    lock_memory_pages(log);

    // EventBus 负责模块间同步事件发布，不直接承载硬件控制动作。
    robot::middleware::EventBus event_bus;

    // CAN 驱动用于行走电机组控制和反馈。
    auto can_bus = std::make_shared<robot::driver::LinuxCanSocket>(
        cfg.get<std::string>("can.interface", "can0"));

    // WalkMotorGroup：4轮统一控制，含通信超时与锁存式 emergency override；
    // 视觉纠偏由 MotionService 持有的 HeadingCorrector 负责
    // comm_timeout 从配置读取，建议设置为行走控制周期的数倍，避免单帧抖动误判断联。
    auto walk_group = std::make_shared<robot::device::WalkMotorGroup>(
        can_bus,
        cfg.get<uint8_t>("can.walk_motor.motor_id", 1u),
        cfg.get<uint16_t>("can.walk_motor.comm_timeout_ms", 200u),
        cfg.get<bool>("can.walk_motor.termination_init_enabled", true),
        cfg.get<uint8_t>("can.walk_motor.termination_init_retry_count", 3u),
        cfg.get<uint8_t>("can.walk_motor.termination_motor_id", 2u));

    // 串口驱动用于滚刷、BMS、IMU、GPS 和 LoRaWAN 等外设。
    auto brush_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.brush.port", "/dev/ttyS3"),
        robot::hal::UartConfig{cfg.get<int>("serial.brush.baudrate", 115200)});
    // BMS 使用嘉佰达通用协议 V4（UART，9600 bps，8-N-1）
    auto bms_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.bms.port", "/dev/ttyS8"),
        robot::hal::UartConfig{cfg.get<int>("serial.bms.baudrate", 9600)});

    auto brush_motor = std::make_shared<robot::device::BrushMotor>(
        brush_serial, cfg.get<uint8_t>("serial.brush.axis", 0u));

    auto bms = std::make_shared<robot::device::BMS>(bms_serial,
                                                    cfg.get<float>("robot.charge_stop_soc", 95.0f),
                                                    cfg.get<float>("robot.min_battery_soc", 30.0f),
                                                    cfg.get_fixed<float>(
                                                        "bms.charging_current_threshold_a",
                                                        0.05f));

    // IMU 和 GPS 在这里选择具体数据源，后续统一通过设备接口读取。
    auto imu_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.imu.port", "/dev/ttyS1"),
        robot::hal::UartConfig{cfg.get<int>("serial.imu.baudrate", 921600)});
    auto imu = std::make_shared<robot::device::ImuDevice>(imu_serial);

    std::shared_ptr<robot::device::GpsDevice> gps;
    const auto gps_source = cfg.get<std::string>("gps.source", "serial");
    if (gps_source == "gpsd") {
        robot::device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = cfg.get<std::string>("gps.gpsd.host", "127.0.0.1");
        gpsd_cfg.port = cfg.get<int>("gps.gpsd.port", 2947);
        gpsd_cfg.watch =
            cfg.get<std::string>("gps.gpsd.watch", "?WATCH={\"enable\":true,\"json\":true};");
        gps = robot::device::GpsDevice::create_gpsd(gpsd_cfg);
    } else {
        if (gps_source != "serial") {
            log->warn("[Main] 未知 GPS source={}，回退到 serial", gps_source);
        }

        auto gps_serial = std::make_shared<robot::driver::LibSerialPort>(
            cfg.get<std::string>("gps.serial.port", "/dev/ttyS2"),
            robot::hal::UartConfig{cfg.get<int>("gps.serial.baudrate", 9600)});
        gps = std::make_shared<robot::device::GpsDevice>(gps_serial);
    }

    // GPIO 输入用于主限位和姿态限位；GPIO 输出用于锁止电机。
    auto left_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.left_limit.chip", "gpiochip5"),
        cfg.get<int>("gpio.left_limit.line", 0));
    auto right_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.right_limit.chip", "gpiochip5"),
        cfg.get<int>("gpio.right_limit.line", 1));
    auto left_attitude_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.left_attitude_limit.chip", "gpiochip5"),
        cfg.get<int>("gpio.left_attitude_limit.line", 2));
    auto right_attitude_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.right_attitude_limit.chip", "gpiochip5"),
        cfg.get<int>("gpio.right_attitude_limit.line", 3));
    auto lock_open_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get_fixed<std::string>("gpio.lock_motor.open.chip", "gpiochip5"),
        cfg.get_fixed<int>("gpio.lock_motor.open.line", 8),
        "lock_motor_open");
    auto lock_close_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get_fixed<std::string>("gpio.lock_motor.close.chip", "gpiochip5"),
        cfg.get_fixed<int>("gpio.lock_motor.close.line", 9),
        "lock_motor_close");

    auto left_switch =
        std::make_shared<robot::device::LimitSwitch>(left_gpio, robot::device::LimitSide::LEFT);
    auto right_switch =
        std::make_shared<robot::device::LimitSwitch>(right_gpio, robot::device::LimitSide::RIGHT);
    auto left_attitude_switch = std::make_shared<robot::device::AttitudeLimitSwitch>(
        left_attitude_gpio, robot::device::AttitudeLimitSide::LEFT_LOWER);
    auto right_attitude_switch = std::make_shared<robot::device::AttitudeLimitSwitch>(
        right_attitude_gpio, robot::device::AttitudeLimitSide::RIGHT_LOWER);
    auto lock_motor = std::make_shared<robot::device::LockMotor>(
        lock_open_gpio, lock_close_gpio, make_lock_motor_config(cfg));

    // 打开设备并记录 GPIO 可用性；关键硬件初始化失败时直接终止启动。
    GpioOpenStatus gpio_status;
    if (!initialize_devices(log,
                            walk_group,
                            imu,
                            gps,
                            left_switch,
                            right_switch,
                            left_attitude_switch,
                            right_attitude_switch,
                            lock_motor,
                            brush_motor,
                            bms,
                            cfg,
                            gpio_status)) {
        return 1;
    }

    // 安全监控器只负责主限位首响急停和到位事件发布，任务推进由状态机处理。
    robot::middleware::SafetyMonitor safety_monitor(
        [walk_group]() { walk_group->emergency_override(0.0f); },
        left_switch,
        right_switch,
        event_bus,
        read_safety_monitor_config(cfg));
    // 这里只构造安全监控器，不立即启动 GPIO polling 线程。上电位置自检仍需要同步读取
    // 主限位电平；若此时已经有多个 RT GPIO 线程在同一颗 pca953x 上轮询，主线程可能在
    // 内核 GPIO 锁上不可中断等待。监控线程会在上电自检读取完成后统一启动。

    // 网络传输按配置选择 MQTT、LoRaWAN 或双通道并行。
    auto mqtt = std::make_shared<robot::middleware::MqttTransport>(make_mqtt_config(cfg));
    const auto net_mode = network_mode_from_config(cfg);

    std::shared_ptr<robot::middleware::INetworkTransport> lorawan_transport;
    if (net_mode == robot::middleware::NetworkManager::Mode::LORAWAN_ONLY ||
        net_mode == robot::middleware::NetworkManager::Mode::DUAL_PARALLEL) {
        auto lorawan_serial = std::make_shared<robot::driver::LibSerialPort>(
            cfg.get<std::string>("network.lorawan.port", "/dev/ttyS5"),
            robot::hal::UartConfig{cfg.get<int>("network.lorawan.baudrate", 9600)});
        robot::middleware::LoRaWANTransport::Config lora_cfg;
        lora_cfg.dev_eui = cfg.get<std::string>("network.lorawan.dev_eui", "");
        lora_cfg.app_key = cfg.get<std::string>("network.lorawan.app_key", "");
        lorawan_transport =
            std::make_shared<robot::middleware::LoRaWANTransport>(lorawan_serial, lora_cfg);
    }

    auto net_mgr =
        std::make_shared<robot::middleware::NetworkManager>(mqtt, lorawan_transport, net_mode);

    // 数据缓存用于网络不可用时暂存待上报遥测。
    auto data_cache = std::make_shared<robot::middleware::DataCache>(
        cfg.get<std::string>("storage.cache_path", "/var/robot/telemetry_cache.jsonl"));
    data_cache->open();

    // 服务层封装运动控制、卡滞检测、姿态限位、云端控制和诊断采集。
    auto motion = std::make_shared<robot::service::MotionService>(
        walk_group, brush_motor, imu, event_bus, make_motion_config_from_config(cfg));
    auto gps_stuck =
        std::make_shared<robot::service::GpsStuckService>(gps, read_gps_stuck_config(cfg));
    // GpsStuck 只在执行清扫任务时启用；启动期/自检/空闲/充电都不应判定“卡住”。
    gps_stuck->set_monitoring_enabled(false);
    auto attitude_limit = std::make_shared<robot::service::AttitudeLimitService>(
        left_attitude_switch,
        right_attitude_switch,
        robot::service::AttitudeLimitService::MotionPorts{
            [motion] { motion->emergency_stop(); },
            [motion] { return motion->begin_attitude_center_motion(); },
            [motion](float rpm) { return motion->command_lower_wheels_for_attitude_center(rpm); },
            [motion] { return motion->stop_attitude_center_motion(); },
        });
    // 姿态限位监控同样延后启动，避免上电自检读取主限位时与姿态限位轮询线程并发访问
    // 同一颗 gpiochip5/pca953x。
    robot::service::SchedulerService scheduler;
    auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
    auto command_tracker = std::make_shared<robot::service::CommandTracker>();
    auto diagnostics_collector = std::make_shared<robot::service::DiagnosticsCollector>(
        walk_group, brush_motor, bms, imu, gps, gps_stuck);

    auto error_manager =
        std::make_shared<robot::app::ErrorManager>(read_error_manager_config(cfg));

    // 应用层状态机只依赖端口回调，不直接持有底层硬件对象。
    auto controller =
        std::make_shared<robot::app::RobotController>(robot::app::RobotController::ActionPorts{
            [motion](const robot::domain::MissionSegment& segment) {
                return motion->start_segment(segment);
            },
            [motion] { motion->stop_cleaning(); },
            [motion] { motion->emergency_stop(); },
            [] {},
            // 故障锁存由 RobotController 自身维护，reset 时不需要额外清理服务缓存。
            [] {},
            [lock_motor] { return lock_motor->open_lock(); },
            [lock_motor] { return lock_motor->close_lock(); },
            [motion] { return motion->hold_at_endpoint(); },
        });
    auto command_port = robot::service::RobotCommandPort{
        [controller](const robot::domain::RobotCommand& command) {
            const auto result = controller->submit_command(command);
            return robot::service::RobotCommandResult{result.accepted, result.reason};
        },
        [controller]() {
            const auto snap = controller->snapshot();
            robot::domain::RobotRuntimeSnapshot runtime;
            runtime.state = snap.state;
            runtime.fault = snap.fault.value_or(0u);
            runtime.repeat_count = snap.repeat_count;
            runtime.completed_cycles = static_cast<uint32_t>(snap.completed_cycles);
            runtime.cfg_ver = snap.cfg_ver;
            return runtime;
        }};
    auto tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
        cfg, &scheduler, cloud, command_tracker, command_port);
    cfg.apply_active_runtime_schedules(scheduler);
    motion->set_primary_dock_query([&cfg]() { return cfg.active_runtime_config().primary_dock; });
    motion->set_runtime_config_query([&cfg]() { return cfg.active_runtime_config(); });
    const auto current_battery_soc = [&bms]() { return bms->get_data().soc_pct; };
    const auto current_limit_levels =
        [&left_switch, &right_switch, gpio_status]() -> std::pair<bool, bool> {
        const bool left_sensor_active =
            gpio_status.left_limit && !left_switch->read_current_level();
        const bool right_sensor_active =
            gpio_status.right_limit && !right_switch->read_current_level();
        return {left_sensor_active, right_sensor_active};
    };
    const auto current_start_position_state = [current_limit_levels]() {
        const auto [left_sensor_active, right_sensor_active] = current_limit_levels();
        return robot::domain::estimate_position(
            robot::domain::LimitState{left_sensor_active, right_sensor_active});
    };
    controller->set_position_state_query(current_start_position_state);
    controller->set_battery_soc_query(current_battery_soc);
    controller->set_config_ports(robot::app::RobotController::ConfigPorts{
        [&cfg] { return cfg.active_runtime_config(); },
        [&cfg] { return cfg.pending_runtime_config(); },
        [&cfg](const robot::domain::RuntimeConfig& runtime_cfg) {
            return cfg.runtime_config_version(runtime_cfg);
        },
        [&cfg] { return cfg.promote_pending_runtime_to_active(); },
        [&cfg] {
            const auto dock_mode =
                cfg.get<std::string>("robot.dock_mode", "single_dock") == "dual_dock"
                    ? robot::domain::DockMode::DualDock
                    : robot::domain::DockMode::SingleDock;
            return robot::domain::LaneConfig{dock_mode, cfg.active_runtime_config().primary_dock};
        },
        cfg.get_fixed<uint64_t>("mission.endpoint_settle_delay_ms", 3000),
    });
    controller->start();
    const auto configured_primary_dock_text =
        robot::domain::endpoint_config_string(cfg.active_runtime_config().primary_dock);
    const auto [startup_left_limit_active, startup_right_limit_active] = current_limit_levels();
    const auto startup_state = robot::domain::estimate_position(
        robot::domain::LimitState{startup_left_limit_active, startup_right_limit_active});
    if (!robot::domain::is_at_target(startup_state, cfg.active_runtime_config().primary_dock)) {
        log->warn(
            "[Main] [自检警告] 当前未检测到主停机端限位触发，"
            "设备可能不在配置主停机端（primary_dock={}），请手动归位再启动",
            configured_primary_dock_text);
    }
    const bool dual_dock_mode =
        cfg.get<std::string>("robot.dock_mode", "single_dock") == "dual_dock";
    const auto opposite_dock =
        robot::domain::opposite_endpoint(cfg.active_runtime_config().primary_dock);
    if (robot::domain::is_at_target(startup_state, opposite_dock) && !dual_dock_mode) {
        log->warn(
            "[Main] [自检警告] 对侧限位当前处于触发状态，"
            "可能有遮挡或传感器异常，请检查环境");
    }
    log->info(
        "[Main] 上电自检：primary_dock={} at_A={} at_B={} (left_sensor={} "
        "right_sensor={})",
        configured_primary_dock_text,
        startup_state == robot::domain::PositionState::AtA,
        startup_state == robot::domain::PositionState::AtB,
        startup_left_limit_active,
        startup_right_limit_active);

    // 上电位置读取完成后再启动所有 GPIO 轮询监控线程，避免启动阶段主线程与多个 RT
    // GPIO 线程同时抢占 pca953x 的内核访问锁。
    safety_monitor.set_limit_settled_callback([controller](robot::domain::Endpoint endpoint) {
        controller->post_limit_settled(endpoint);
    });
    if (!safety_monitor.start()) {
        log->error("[Main] 安全监控启动失败，退出");
        motion->emergency_stop();
        controller->stop();
        return 1;
    }
    if (gpio_status.left_attitude && gpio_status.right_attitude) {
        attitude_limit->start_monitoring();
    }

    connect_cloud_and_publish_startup(
        running, log, cfg, tb_control, net_mgr, startup_state, dual_dock_mode);

    // 调度服务只读取 active runtime 的时间窗口，命中后把启动请求交给状态机。
    scheduler.set_on_window_hit([controller] { controller->post_schedule_window_hit(); });

    robot::app::WatchdogMgr watchdog(cfg.get<std::string>("system.hw_watchdog", "/dev/watchdog"));
    watchdog.set_timeout_callback([error_manager](const std::string& thread_name) {
        error_manager->submit_watchdog_timeout(thread_name, steady_now_ms());
    });
    watchdog.start();

    // 上报服务从 DiagnosticsCollector 获取唯一诊断事实，按运行状态动态调整周期。
    std::string diag_mode = cfg.get<std::string>("diagnostics.mode", "production");
    const bool cloud_upload = cfg.get<bool>("diagnostics.cloud_upload", true);
    const bool local_log = cfg.get<bool>("diagnostics.local_log", true);
    // DiagnosticsCollector 是诊断数据唯一输出源：
    //   production  -> Mode::HEALTH      （精简状态字段）
    //   development -> Mode::DIAGNOSTICS （完整诊断字段）
    // local_log_path 非空时额外把每帧 JSON payload 以 JSONL 追加到本地文件，
    // 离线调试阶段可直接 cat/grep 查看，完全独立于 MQTT/LoRaWAN。
    std::string local_tel_path =
        local_log ? cfg.get<std::string>("diagnostics.local_log_path", "") : "";
    const size_t local_log_max_bytes = static_cast<size_t>(
        std::max(1, cfg.get<int>("diagnostics.local_log_max_bytes", 10 * 1024 * 1024)));
    const size_t local_log_max_files =
        static_cast<size_t>(std::max(1, cfg.get<int>("diagnostics.local_log_max_files", 3)));
    const auto health_mode =
        diag_mode == "development" ? robot::service::HealthService::Mode::DIAGNOSTICS
                                   : robot::service::HealthService::Mode::HEALTH;
    std::shared_ptr<robot::service::CloudService> health_cloud = cloud_upload ? cloud : nullptr;
    auto cloud_reporter = std::make_shared<robot::service::HealthService>(
        diagnostics_collector,
        health_cloud,
        health_mode,
        "",
        local_log_max_bytes,
        local_log_max_files);
    std::shared_ptr<robot::service::HealthService> local_reporter;
    if (!local_tel_path.empty()) {
        local_reporter = std::make_shared<robot::service::HealthService>(
            diagnostics_collector,
            nullptr,
            health_mode,
            local_tel_path,
            local_log_max_bytes,
            local_log_max_files);
    }

    // RK3576 CPU 拓扑：
    //   CPU 0-3: Cortex-A55 (LITTLE 核)  → 非 RT 后台任务
    //   CPU 4-7: Cortex-A76 (BIG 核)     → RT 任务专用
    //
    // 线程绑定策略（与 safety_monitor/gpio 线程配合，不竞争同一核心）：
    //   CPU 4: gpio_left/right(95) + safety_monitor(94)  → GPIO 边沿优先，轮询兜底
    //   CPU 5: group_recv(82) + walk_ctrl(80)            → 先接收后控制，降低读旧数据概率
    //   CPU 6: gps_stuck(65) + imu_read(68)              → GPS 监测路径，专用核
    //   CPU 7: watchdog(50) + main(SCHED_OTHER)          → 管理任务
    //   CPU 0-3: bms + cloud + gps（LITTLE 核，低功耗后台）
    //
    // 行走控制线程：SCHED_FIFO 80, 50ms (20Hz)，绑定 CPU 5。
    // MotionService::control_dt_s 与此周期绑定，保证 PID / 融合预测的时间基准一致。
    robot::middleware::ThreadExecutor walk_exec(
        {"walk_ctrl", kWalkCtrlPeriodMs, SCHED_FIFO, kWalkCtrlPriority, kWalkCtrlCpuMask});
    walk_exec.add_runnable(motion);
    // 心跳在 walk_ctrl 线程自身内汇报（超时 = 该线程死锁，而非主线程死锁）
    int walk_wd = watchdog.register_thread("walk_ctrl", kWalkCtrlWatchdogMs);
    walk_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, walk_wd]() { watchdog.heartbeat(walk_wd); }));

    // GPS 卡住检测线程：SCHED_FIFO 65, 500ms，绑定 CPU 6，匹配 1Hz GPS 更新。
    robot::middleware::ThreadExecutor gps_stuck_exec(
        {"gps_stuck", kGpsStuckPeriodMs, SCHED_FIFO, kGpsStuckPriority, kGpsStuckCpuMask});
    gps_stuck_exec.add_runnable(gps_stuck);
    int gps_stuck_wd = watchdog.register_thread("gps_stuck", kGpsStuckWatchdogMs);
    gps_stuck_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, gps_stuck_wd]() { watchdog.heartbeat(gps_stuck_wd); }));

    const int bms_poll_period_ms = cfg.get_fixed<int>("bms.poll_interval_ms", 500);
    const int brush_poll_period_ms = cfg.get_fixed<int>("brush.poll_interval_ms", 500);

    // BMS 采集线程：默认 500ms，绑定 LITTLE 核 CPU 0-3（低功耗后台）。
    // BMS 休眠或无响应时单次 update 可能包含多轮唤醒重试；watchdog 只判定线程卡死，
    // 通信异常由 DiagnosticsCollector/ErrorManager 的 update_count 停滞规则判定。
    robot::middleware::ThreadExecutor bms_exec(
        {"bms", bms_poll_period_ms, SCHED_OTHER, 0, kBackgroundCpuMask});
    bms_exec.add_runnable(
        std::make_shared<robot::middleware::RunnableAdapter>([&bms]() { bms->update(); }));
    int bms_wd = watchdog.register_thread("bms", kBmsWatchdogMs);
    bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, bms_wd]() { watchdog.heartbeat(bms_wd); }));

    // BrushMotor 状态不需要实时性，默认 500ms 周期足够。独立线程便于恢复时只停止滚刷通信。
    robot::middleware::ThreadExecutor brush_exec(
        {"brush", brush_poll_period_ms, SCHED_OTHER, 0, kBackgroundCpuMask});
    brush_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&brush_motor]() { brush_motor->update(); }));
    int brush_wd = watchdog.register_thread("brush", kBrushWatchdogMs);
    brush_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, brush_wd]() { watchdog.heartbeat(brush_wd); }));

    robot::app::RecoveryExecutor recovery_executor(
        make_recovery_ports(
            gps_stuck,
            motion,
            attitude_limit,
            read_attitude_center_config(cfg),
            std::chrono::milliseconds(
                cfg.get_fixed<int>("recovery.reverse.duration_ms", 2000)),
            std::chrono::milliseconds(cfg.get_fixed<int>("recovery.reverse.tick_ms", 20))));
    auto error_handling = make_error_handling_service(error_manager,
                                                      controller,
                                                      attitude_limit,
                                                      diagnostics_collector,
                                                      gps_stuck,
                                                      recovery_executor);

    // 诊断采集与云端上报线程：绑定 LITTLE 核 CPU 0-3（低功耗后台）
    const int diagnostics_collect_period =
        std::max(50, cfg.get<int>("diagnostics.collect_interval_ms", 50));
    const int error_period =
        std::max(50, cfg.get<int>("error_manager.period_ms", kErrorManagerDefaultPeriodMs));
    const int active_report_period =
        std::max(1, cfg.get<int>("diagnostics.publish_interval_active_ms", 1000));
    const int idle_report_period =
        std::max(1, cfg.get<int>("diagnostics.publish_interval_idle_ms", 300000));
    const int active_local_log_period =
        std::max(50, cfg.get<int>("diagnostics.local_log_interval_active_ms", 50));
    const int idle_local_log_period =
        std::max(1, cfg.get<int>("diagnostics.local_log_interval_idle_ms", 300000));
    robot::middleware::ThreadExecutor diagnostics_exec(
        {"diagnostics", diagnostics_collect_period, SCHED_OTHER, 0, kBackgroundCpuMask});
    diagnostics_exec.add_runnable(diagnostics_collector);
    robot::middleware::ThreadExecutor error_exec(
        {"error_mgr", error_period, SCHED_OTHER, 0, kBackgroundCpuMask});
    error_exec.add_runnable(error_handling);
    robot::middleware::ThreadExecutor local_diag_exec(
        {"local_diag", active_local_log_period, SCHED_OTHER, 0, kBackgroundCpuMask});
    robot::middleware::ThreadExecutor* local_diag_exec_ptr = nullptr;
    if (local_reporter) {
        local_diag_exec.add_runnable(local_reporter);
        local_diag_exec_ptr = &local_diag_exec;
    }
    robot::middleware::ThreadExecutor cloud_exec(
        {"cloud", active_report_period, SCHED_OTHER, 0, kBackgroundCpuMask});
    cloud_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [controller, diagnostics_collector]() {
            log_runtime_diagnostics(controller->snapshot(), diagnostics_collector->snapshot());
        }));
    if (cloud_upload) {
        cloud_exec.add_runnable(cloud_reporter);
        cloud_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [tb_control]() { tb_control->publish_business_telemetry(); }));
        cloud_exec.add_runnable(cloud);
    }
    if (!start_runtime_threads(log,
                               walk_exec,
                               gps_stuck_exec,
                               bms_exec,
                               brush_exec,
                               diagnostics_exec,
                               error_exec,
                               local_diag_exec_ptr,
                               cloud_exec)) {
        log->error("[Main] 核心运行线程启动失败，退出");
        static_cast<void>(shutdown_runtime(log,
                                           walk_exec,
                                           gps_stuck_exec,
                                           bms_exec,
                                           brush_exec,
                                           error_exec,
                                           diagnostics_exec,
                                           local_diag_exec_ptr,
                                           cloud_exec,
                                           attitude_limit,
                                           safety_monitor,
                                           watchdog,
                                           controller,
                                           motion,
                                           brush_motor,
                                           walk_group,
                                           imu,
                                           gps,
                                           bms,
                                           left_switch,
                                           right_switch,
                                           left_attitude_switch,
                                           right_attitude_switch,
                                           net_mgr,
                                           data_cache));
        return 1;
    }

    // 主循环处理调度 tick、诊断上报周期切换和退出信号。
    run_application_loop(running,
                         scheduler,
                         controller,
                         diagnostics_collector,
                         cloud_exec,
                         active_report_period,
                         idle_report_period,
                         local_diag_exec_ptr,
                         active_local_log_period,
                         idle_local_log_period);

    // 退出时按依赖反向停止线程、监控器、状态机、运动和通信资源。
    const bool shutdown_ok = shutdown_runtime(log,
                                             walk_exec,
                                             gps_stuck_exec,
                                             bms_exec,
                                             brush_exec,
                                             error_exec,
                                             diagnostics_exec,
                                             local_diag_exec_ptr,
                                             cloud_exec,
                                             attitude_limit,
                                             safety_monitor,
                                             watchdog,
                                             controller,
                                             motion,
                                             brush_motor,
                                             walk_group,
                                             imu,
                                             gps,
                                             bms,
                                             left_switch,
                                             right_switch,
                                             left_attitude_switch,
                                             right_attitude_switch,
                                             net_mgr,
                                             data_cache);
    return shutdown_ok ? 0 : 1;
}

}  // namespace robot::app
