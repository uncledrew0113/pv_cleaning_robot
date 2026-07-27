/**
 * @file system_hw_main_flow_tests.cc
 * @brief 真实硬件主任务流程系统测试。
 *
 * 本文件验证启动、自检、任务段切换、主限位到位、任务结束和锁止动作等完整链路。
 */
#include <sched.h>

#include "pv_cleaning_robot/app/error_manager.h"
#include "pv_cleaning_robot/app/recovery_executor.h"
#include "pv_cleaning_robot/app/robot_application.h"
#include "pv_cleaning_robot/device/lock_motor.h"
#include "pv_cleaning_robot/middleware/logger.h"
#include "pv_cleaning_robot/service/attitude_limit_service.h"
#include "pv_cleaning_robot/service/diagnostics_collector.h"
#include "system_hw_common.h"

namespace {

constexpr uint64_t kSelfCheckStreamMaxAgeMs = 3000;

uint64_t hw_steady_now_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

bool stream_ready_for_hw_self_check(const robot::domain::StreamHealth& health, uint64_t now_ms) {
    return health.enabled && health.last_update_ms != 0 && now_ms >= health.last_update_ms &&
           now_ms - health.last_update_ms <= kSelfCheckStreamMaxAgeMs;
}

bool hw_self_check_ready(const robot::service::DiagnosticsCollector::Snapshot& snapshot,
                         uint64_t now_ms) {
    if (snapshot.ts_ms == 0) {
        return false;
    }
    if (snapshot.bms_diagnostics.update_count == 0 ||
        snapshot.gps_diagnostics.sentence_count == 0 || snapshot.imu_diagnostics.frame_count == 0) {
        return false;
    }
    if (!stream_ready_for_hw_self_check(snapshot.error.bms_update, now_ms) ||
        !stream_ready_for_hw_self_check(snapshot.error.gps, now_ms) ||
        !stream_ready_for_hw_self_check(snapshot.error.imu, now_ms)) {
        return false;
    }
    if (!snapshot.error.walk_feedback_expected) {
        return false;
    }

    for (std::size_t i = 0; i < snapshot.walk_diagnostics.wheel.size(); ++i) {
        if (snapshot.walk_diagnostics.wheel[i].feedback_frame_count == 0 ||
            !stream_ready_for_hw_self_check(snapshot.error.walk_feedback[i], now_ms)) {
            return false;
        }
    }

    return !snapshot.error.walk_stall_active && !snapshot.error.brush_fault_active &&
           !snapshot.error.gps_stuck;
}

robot::device::LockMotor::Config make_hw_lock_motor_config() {
    robot::device::LockMotor::Config cfg;
    cfg.pulse_ms = kp.lock_motor_pulse_ms;
    cfg.settle_ms = kp.lock_motor_settle_ms;
    return cfg;
}

class MainLikeHardwareFixture : public hw::IGracefulShutdown {
   public:
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("hw_main_like")};

    robot::middleware::EventBus bus;
    std::unique_ptr<robot::service::ConfigService> config;
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<robot::driver::LibSerialPort> brush_serial;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<robot::driver::LibSerialPort> bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::device::GpsDevice> gps;
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> left_attitude_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_attitude_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> lock_open_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> lock_close_gpio;
    std::shared_ptr<robot::device::LimitSwitch> left_sw;
    std::shared_ptr<robot::device::LimitSwitch> right_sw;
    std::shared_ptr<robot::device::AttitudeLimitSwitch> left_attitude_sw;
    std::shared_ptr<robot::device::AttitudeLimitSwitch> right_attitude_sw;
    std::shared_ptr<robot::device::LockMotor> lock_motor;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::service::GpsStuckService> gps_stuck;
    std::shared_ptr<robot::service::AttitudeLimitService> attitude_limit;
    std::shared_ptr<robot::service::DiagnosticsCollector> diagnostics;
    std::shared_ptr<robot::service::HealthService> health;
    std::shared_ptr<robot::app::ErrorManager> error_manager;
    std::unique_ptr<robot::app::RecoveryExecutor> recovery_executor;
    std::shared_ptr<robot::app::ErrorHandlingService> error_handling;
    std::shared_ptr<robot::app::RobotController> controller;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::unique_ptr<robot::app::WatchdogMgr> watchdog;
    std::unique_ptr<robot::middleware::ThreadExecutor> walk_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> gps_stuck_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> bms_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> brush_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> diagnostics_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> error_exec;
    std::unique_ptr<robot::middleware::ThreadExecutor> health_exec;

    bool initialized{false};
    int lock_open_count{0};
    int lock_close_count{0};
    std::atomic<int> attitude_center_attempts{0};
    std::atomic<int> attitude_center_completed{0};
    // 无错误处理版本用于验证“任务主链路”本身，不启动 ErrorManager/RecoveryExecutor。
    bool enable_error_handling{true};

    explicit MainLikeHardwareFixture(bool enable_error_handling_in = true)
        : enable_error_handling(enable_error_handling_in) {}

    bool init() {
        write_config_files();
        std::filesystem::create_directories(log_dir());
        robot::middleware::Logger::init(robot::middleware::Logger::Config{
            log_dir().string(), "hw_main_like", 10u * 1024u * 1024u, 3, true, "info"});

        config = std::make_unique<robot::service::ConfigService>(paths.runtime_path.string(),
                                                                 paths.fixed_path.string());
        if (!config->load()) {
            return false;
        }

        if (!construct_and_open_devices()) {
            return false;
        }
        construct_services();
        construct_controller();
        construct_threads();
        if (!start_threads()) {
            return false;
        }

        initialized = true;
        hw::HwExitGuard::instance().install();
        hw::HwExitGuard::instance().set_active(this);
        return true;
    }

    robot::app::CommandResult submit_rpc(robot::domain::RobotCommandKind kind,
                                         const char* command_id) {
        return controller->submit_command(
            robot::domain::RobotCommand{kind, robot::domain::CommandSource::Rpc, command_id});
    }

    bool complete_self_check_from_diagnostics(std::chrono::seconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!hw::HwExitGuard::instance().exit_requested() &&
               std::chrono::steady_clock::now() < deadline) {
            const auto snap = diagnostics->snapshot();
            if (hw_self_check_ready(snap, hw_steady_now_ms())) {
                controller->complete_self_check(true);
                controller->drain_for_test();
                if (controller->snapshot().state == "ExecutingMission") {
                    return true;
                }
                log_failure_context("self_check_transition");
                return false;
            }
            if (controller->snapshot().state == "FaultStopped") {
                log_failure_context("self_check_fault");
                return false;
            }
            std::this_thread::sleep_for(100ms);
        }
        controller->complete_self_check(false);
        controller->drain_for_test();
        log_failure_context("self_check_timeout");
        return false;
    }

    bool wait_until_idle(std::chrono::seconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!hw::HwExitGuard::instance().exit_requested() &&
               std::chrono::steady_clock::now() < deadline) {
            const auto state = controller->snapshot().state;
            if (state == "Idle") {
                return true;
            }
            if (state == "FaultStopped") {
                log_failure_context("mission_fault");
                return false;
            }
            std::this_thread::sleep_for(100ms);
        }
        if (controller->snapshot().state == "Idle") {
            return true;
        }
        log_failure_context("mission_timeout");
        return false;
    }

    bool wait_for_attitude_recovery(std::chrono::seconds timeout) {
        spdlog::warn(
            "[hw_system][main_like_attitude_recover] 请在任务运行中触发任意单侧下姿态限位；"
            "测试要求生产恢复链路完成一次回中");
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        bool saw_recovering = false;
        while (!hw::HwExitGuard::instance().exit_requested() &&
               std::chrono::steady_clock::now() < deadline) {
            const auto state = controller->snapshot().state;
            saw_recovering = saw_recovering || state == "Recovering";
            if (attitude_center_completed.load() > 0) {
                if (!saw_recovering) {
                    log_failure_context("attitude_recovery_state_not_observed");
                }
                return saw_recovering;
            }
            if (state == "FaultStopped" || state == "Idle") {
                log_failure_context("attitude_recovery_not_completed");
                return false;
            }
            std::this_thread::sleep_for(20ms);
        }
        log_failure_context("attitude_recovery_timeout");
        return false;
    }

    void stop_and_require_diagnostics_log() {
        health_exec->stop();
        require_diagnostics_health_log(telemetry_path());
    }

    void shutdown() override {
        if (health_exec)
            health_exec->stop();
        if (error_exec)
            error_exec->stop();
        if (diagnostics_exec)
            diagnostics_exec->stop();
        if (brush_exec)
            brush_exec->stop();
        if (bms_exec)
            bms_exec->stop();
        if (gps_stuck_exec)
            gps_stuck_exec->stop();
        if (walk_exec)
            walk_exec->stop();
        if (attitude_limit)
            attitude_limit->stop_monitoring();
        if (safety)
            safety->stop();
        if (watchdog)
            watchdog->stop();
        if (controller)
            controller->stop();
        if (motion)
            motion->emergency_stop();
        if (brush)
            brush->close();
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        if (imu)
            imu->close();
        if (gps)
            gps->close();
        if (bms)
            bms->close();
        if (lock_motor)
            lock_motor->shutdown();
        if (left_sw)
            left_sw->close();
        if (right_sw)
            right_sw->close();
        if (left_attitude_sw)
            left_attitude_sw->close();
        if (right_attitude_sw)
            right_attitude_sw->close();
        if (initialized) {
            hw::HwExitGuard::instance().clear_active(this);
            initialized = false;
        }
        tb_test_support::cleanup_split_config_paths(paths);
    }

    ~MainLikeHardwareFixture() override {
        shutdown();
    }

   private:
    void log_failure_context(const char* phase) const {
        const auto controller_snapshot = controller->snapshot();
        const auto diagnostic_snapshot = diagnostics->snapshot();
        const auto& wheel = diagnostic_snapshot.walk_diagnostics.wheel;
        spdlog::error(
            "[hw_system][main_like] phase={} state={} exit_requested={} diag_ts={} "
            "walk_expected={} walk_frames=[{},{},{},{}] bms_updates={} gps_sentences={} "
            "imu_frames={} stall={} brush_fault={} gps_stuck={}",
            phase,
            controller_snapshot.state,
            hw::HwExitGuard::instance().exit_requested(),
            diagnostic_snapshot.ts_ms,
            diagnostic_snapshot.error.walk_feedback_expected,
            wheel[0].feedback_frame_count,
            wheel[1].feedback_frame_count,
            wheel[2].feedback_frame_count,
            wheel[3].feedback_frame_count,
            diagnostic_snapshot.bms_diagnostics.update_count,
            diagnostic_snapshot.gps_diagnostics.sentence_count,
            diagnostic_snapshot.imu_diagnostics.frame_count,
            diagnostic_snapshot.error.walk_stall_active,
            diagnostic_snapshot.error.brush_fault_active,
            diagnostic_snapshot.error.gps_stuck);
    }

    std::filesystem::path log_dir() const {
        return std::filesystem::path(paths.runtime_path.string() + ".logs");
    }

    std::filesystem::path telemetry_path() const {
        return std::filesystem::path(paths.runtime_path.string() + ".telemetry.jsonl");
    }

    void write_config_files() const {
        std::ostringstream runtime;
        runtime << "{\n"
                << "  \"scheduler\": { \"windows\": [] },\n"
                << "  \"robot\": {\n"
                << "    \"dock_mode\": \"single_dock\",\n"
                << "    \"repeat_count\": 1,\n"
                << "    \"primary_dock\": \"" << endpoint_to_config(kp.primary_dock) << "\",\n"
                << "    \"clean_speed_rpm\": " << std::abs(kp.test_speed_rpm) << ",\n"
                << "    \"return_speed_rpm\": " << std::abs(kp.test_return_rpm) << ",\n"
                << "    \"brush_rpm\": "
                << static_cast<int>(std::lround(std::abs(kp.brush_test_rpm))) << ",\n"
                << "    \"heading_pid_en\": true,\n"
                << "    \"pid\": {\n"
                << "      \"uds_path\": \"" << kp.pid.uds_path << "\",\n"
                << "      \"reconnect_interval_ms\": " << kp.pid.reconnect_interval_ms << ",\n"
                << "      \"result_timeout_ms\": " << kp.pid.result_timeout_ms << ",\n"
                << "      \"min_confidence\": " << kp.pid.min_confidence << ",\n"
                << "      \"deadband_yaw_deg\": " << kp.pid.deadband_yaw_deg << ",\n"
                << "      \"kp\": " << kp.pid.kp << ",\n"
                << "      \"ki\": " << kp.pid.ki << ",\n"
                << "      \"kd\": " << kp.pid.kd << ",\n"
                << "      \"integral_limit\": " << kp.pid.integral_limit << ",\n"
                << "      \"max_output\": " << kp.pid.max_output << ",\n"
                << "      \"min_effective_output\": " << kp.pid.min_effective_output << ",\n"
                << "      \"yaw_alpha\": " << kp.pid.yaw_alpha << ",\n"
                << "      \"output_sign\": " << kp.pid.output_sign << ",\n"
                << "      \"angle_source\": \"fused_uds_gyro\",\n"
                << "      \"wheel_strategy\": \"all_wheels\",\n"
                << "      \"slow_on_error\": false,\n"
                << "      \"slow_base_rpm\": " << kp.correction_compare.slow_base_rpm << ",\n"
                << "      \"yaw_slow_threshold_deg\": "
                << kp.correction_compare.yaw_slow_threshold_deg << ",\n"
                << "      \"fusion\": {\n"
                << "        \"process_noise_angle\": "
                << kp.correction_compare.fusion.process_noise_angle << ",\n"
                << "        \"process_noise_bias\": "
                << kp.correction_compare.fusion.process_noise_bias << ",\n"
                << "        \"measurement_noise_uds\": "
                << kp.correction_compare.fusion.measurement_noise_uds << ",\n"
                << "        \"initial_angle_variance\": "
                << kp.correction_compare.fusion.initial_angle_variance << ",\n"
                << "        \"initial_bias_variance\": "
                << kp.correction_compare.fusion.initial_bias_variance << ",\n"
                << "        \"max_gyro_only_ms\": " << kp.correction_compare.fusion.max_gyro_only_ms
                << "\n"
                << "      }\n"
                << "    },\n"
                << "    \"min_battery_soc\": 30.0,\n"
                << "    \"charge_stop_soc\": 95.0\n"
                << "  }\n"
                << "}\n";

        std::ostringstream fixed;
        fixed << "{\n"
              << "  \"logging\": { \"log_dir\": \"" << log_dir().string()
              << "\", \"level\": \"info\", \"console\": true },\n"
              << "  \"can\": { \"interface\": \"" << kp.can_iface << "\", \"walk_motor\": {\n"
              << "    \"motor_id\": " << static_cast<int>(kp.motor_id_base) << ",\n"
              << "    \"comm_timeout_ms\": " << kp.comm_timeout_ms << ",\n"
              << "    \"termination_init_enabled\": "
              << (kp.termination_init_enabled ? "true" : "false") << ",\n"
              << "    \"termination_init_retry_count\": "
              << static_cast<int>(kp.termination_init_retry_count) << ",\n"
              << "    \"termination_motor_id\": " << static_cast<int>(kp.termination_motor_id)
              << " } },\n"
              << "  \"serial\": {\n"
              << "    \"imu\": { \"port\": \"" << kp.imu_port << "\", \"baudrate\": " << kp.imu_baud
              << " },\n"
              << "    \"brush\": { \"port\": \"" << kp.brush_port
              << "\", \"baudrate\": " << kp.brush_baud
              << ", \"axis\": " << static_cast<int>(kp.brush_axis) << " },\n"
              << "    \"bms\": { \"port\": \"" << kp.bms_port << "\", \"baudrate\": " << kp.bms_baud
              << " }\n"
              << "  },\n"
              << "  \"gps\": { \"source\": \"gpsd\", \"gpsd\": { \"host\": \"" << kp.gpsd_host
              << "\", \"port\": " << kp.gpsd_port << ", \"watch\": \"" << kp.gpsd_watch
              << "\" } },\n"
              << "  \"gpio\": {\n"
              << "    \"left_limit\": { \"chip\": \"" << kp.gpio_chip
              << "\", \"line\": " << kp.left_limit_line << " },\n"
              << "    \"right_limit\": { \"chip\": \"" << kp.gpio_chip
              << "\", \"line\": " << kp.right_limit_line << " },\n"
              << "    \"left_attitude_limit\": { \"chip\": \"" << kp.gpio_chip
              << "\", \"line\": " << kp.left_attitude_limit_line << " },\n"
              << "    \"right_attitude_limit\": { \"chip\": \"" << kp.gpio_chip
              << "\", \"line\": " << kp.right_attitude_limit_line << " },\n"
              << "    \"lock_motor\": {\n"
              << "      \"open\": { \"chip\": \"" << kp.lock_motor_open_chip
              << "\", \"line\": " << kp.lock_motor_open_line << " },\n"
              << "      \"close\": { \"chip\": \"" << kp.lock_motor_close_chip
              << "\", \"line\": " << kp.lock_motor_close_line << " },\n"
              << "      \"pulse_ms\": " << kp.lock_motor_pulse_ms << ",\n"
              << "      \"settle_ms\": " << kp.lock_motor_settle_ms << "\n"
              << "    },\n"
              << "    \"use_irq\": false\n"
              << "  },\n"
              << "  \"diagnostics\": { \"mode\": \"development\", \"cloud_upload\": false,\n"
              << "    \"local_log\": true, \"local_log_path\": \"" << telemetry_path().string()
              << "\",\n"
              << "    \"local_log_max_bytes\": 10485760, \"local_log_max_files\": 3,\n"
              << "    \"collect_interval_ms\": 50, \"publish_interval_active_ms\": 1000,\n"
              << "    \"publish_interval_idle_ms\": 300000,\n"
              << "    \"local_log_interval_active_ms\": 50,\n"
              << "    \"local_log_interval_idle_ms\": 300000 },\n"
              << "  \"storage\": { \"cache_path\": \"" << paths.cache_path.string() << "\" },\n"
              << "  \"system\": { \"hw_watchdog\": \"\" }\n"
              << "}\n";

        tb_test_support::write_split_config(paths, runtime.str(), fixed.str());
    }

    bool construct_and_open_devices() {
        can_bus = std::make_shared<robot::driver::LinuxCanSocket>(
            config->get<std::string>("can.interface", kp.can_iface));
        walk_group = std::make_shared<robot::device::WalkMotorGroup>(
            can_bus,
            config->get<uint8_t>("can.walk_motor.motor_id", kp.motor_id_base),
            config->get<uint16_t>("can.walk_motor.comm_timeout_ms", kp.comm_timeout_ms),
            config->get<bool>("can.walk_motor.termination_init_enabled", true),
            config->get<uint8_t>("can.walk_motor.termination_init_retry_count",
                                 kp.termination_init_retry_count),
            config->get<uint8_t>("can.walk_motor.termination_motor_id", kp.termination_motor_id));
        if (walk_group->open() != robot::device::DeviceError::OK) {
            return false;
        }
        walk_group->set_feedback_mode_all(10u);

        brush_serial = std::make_shared<robot::driver::LibSerialPort>(
            config->get<std::string>("serial.brush.port", kp.brush_port),
            robot::hal::UartConfig{config->get<int>("serial.brush.baudrate", kp.brush_baud)});
        brush = std::make_shared<robot::device::BrushMotor>(
            brush_serial, config->get<uint8_t>("serial.brush.axis", kp.brush_axis));
        if (!brush->open()) {
            return false;
        }

        bms_serial = std::make_shared<robot::driver::LibSerialPort>(
            config->get<std::string>("serial.bms.port", kp.bms_port),
            robot::hal::UartConfig{config->get<int>("serial.bms.baudrate", kp.bms_baud)});
        bms = std::make_shared<robot::device::BMS>(bms_serial, 95.0f, 15.0f);
        bms->open();

        imu_serial = std::make_shared<robot::driver::LibSerialPort>(
            config->get<std::string>("serial.imu.port", kp.imu_port),
            robot::hal::UartConfig{config->get<int>("serial.imu.baudrate", kp.imu_baud)});
        imu = std::make_shared<robot::device::ImuDevice>(imu_serial);
        if (!imu->open()) {
            return false;
        }
        imu->set_output_rate(100);

        robot::device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = config->get<std::string>("gps.gpsd.host", kp.gpsd_host);
        gpsd_cfg.port = config->get<int>("gps.gpsd.port", kp.gpsd_port);
        gpsd_cfg.watch = config->get<std::string>("gps.gpsd.watch", kp.gpsd_watch);
        gps = robot::device::GpsDevice::create_gpsd(gpsd_cfg);
        gps->open();

        left_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get<std::string>("gpio.left_limit.chip", kp.gpio_chip),
            config->get<int>("gpio.left_limit.line", static_cast<int>(kp.left_limit_line)));
        right_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get<std::string>("gpio.right_limit.chip", kp.gpio_chip),
            config->get<int>("gpio.right_limit.line", static_cast<int>(kp.right_limit_line)));
        left_attitude_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get<std::string>("gpio.left_attitude_limit.chip", kp.gpio_chip),
            config->get<int>("gpio.left_attitude_limit.line",
                             static_cast<int>(kp.left_attitude_limit_line)));
        right_attitude_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get<std::string>("gpio.right_attitude_limit.chip", kp.gpio_chip),
            config->get<int>("gpio.right_attitude_limit.line",
                             static_cast<int>(kp.right_attitude_limit_line)));
        lock_open_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get_fixed<std::string>("gpio.lock_motor.open.chip", kp.lock_motor_open_chip),
            config->get_fixed<int>("gpio.lock_motor.open.line",
                                   static_cast<int>(kp.lock_motor_open_line)),
            "hw_main_lock_open");
        lock_close_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            config->get_fixed<std::string>("gpio.lock_motor.close.chip", kp.lock_motor_close_chip),
            config->get_fixed<int>("gpio.lock_motor.close.line",
                                   static_cast<int>(kp.lock_motor_close_line)),
            "hw_main_lock_close");

        left_sw =
            std::make_shared<robot::device::LimitSwitch>(left_gpio, robot::device::LimitSide::LEFT);
        right_sw = std::make_shared<robot::device::LimitSwitch>(right_gpio,
                                                                robot::device::LimitSide::RIGHT);
        left_attitude_sw = std::make_shared<robot::device::AttitudeLimitSwitch>(
            left_attitude_gpio, robot::device::AttitudeLimitSide::LEFT_LOWER);
        right_attitude_sw = std::make_shared<robot::device::AttitudeLimitSwitch>(
            right_attitude_gpio, robot::device::AttitudeLimitSide::RIGHT_LOWER);
        if (!left_sw->open(95, 2, 1 << 4, false) || !right_sw->open(95, 2, 1 << 4, false)) {
            return false;
        }
        left_attitude_sw->open(95, 2, 1 << 4, false);
        right_attitude_sw->open(95, 2, 1 << 4, false);

        lock_motor = std::make_shared<robot::device::LockMotor>(
            lock_open_gpio, lock_close_gpio, make_hw_lock_motor_config());
        return lock_motor->initialize();
    }

    void construct_services() {
        motion = std::make_shared<robot::service::MotionService>(
            walk_group, brush, imu, bus, robot::app::make_motion_config_from_config(*config));
        motion->set_runtime_config_query([this] { return config->active_runtime_config(); });
        motion->set_primary_dock_query(
            [this] { return config->active_runtime_config().primary_dock; });
        gps_stuck = std::make_shared<robot::service::GpsStuckService>(gps);
        gps_stuck->set_monitoring_enabled(false);
        attitude_limit = std::make_shared<robot::service::AttitudeLimitService>(
            left_attitude_sw,
            right_attitude_sw,
            robot::service::AttitudeLimitService::MotionPorts{
                [this] { motion->emergency_stop(); },
                [this] { return motion->begin_attitude_center_motion(); },
                [this](float rpm) { return motion->command_lower_wheels_for_attitude_center(rpm); },
                [this] { return motion->stop_attitude_center_motion(); },
            });
        diagnostics = std::make_shared<robot::service::DiagnosticsCollector>(
            walk_group, brush, bms, imu, gps, gps_stuck);
        health = std::make_shared<robot::service::HealthService>(
            diagnostics,
            nullptr,
            robot::service::HealthService::Mode::DIAGNOSTICS,
            telemetry_path().string(),
            kp.health_log_max_bytes,
            kp.health_log_max_files);
        if (enable_error_handling) {
            error_manager = std::make_shared<robot::app::ErrorManager>();
        }
    }

    void construct_controller() {
        controller =
            std::make_shared<robot::app::RobotController>(robot::app::RobotController::ActionPorts{
                [this](const robot::domain::MissionSegment& segment) {
                    return motion->start_segment(segment);
                },
                [this] { motion->stop_cleaning(); },
                [this] { motion->emergency_stop(); },
                [] {},
                [] {},
                [this] {
                    ++lock_open_count;
                    return lock_motor->open_lock();
                },
                [this] {
                    ++lock_close_count;
                    return lock_motor->close_lock();
                },
            });
        controller->set_position_state_query([this] {
            const bool left_active = !left_sw->read_current_level();
            const bool right_active = !right_sw->read_current_level();
            return robot::domain::estimate_position(
                robot::domain::LimitState{left_active, right_active});
        });
        controller->set_battery_soc_query([this] { return bms->get_data().soc_pct; });
        controller->set_config_ports(robot::app::RobotController::ConfigPorts{
            [this] { return config->active_runtime_config(); },
            [this] { return config->pending_runtime_config(); },
            [this](const robot::domain::RuntimeConfig& runtime) {
                return config->runtime_config_version(runtime);
            },
            [this] { return config->promote_pending_runtime_to_active(); },
            [this] {
                return robot::domain::LaneConfig{
                    config->get<std::string>("robot.dock_mode", "single_dock") == "dual_dock"
                        ? robot::domain::DockMode::DualDock
                        : robot::domain::DockMode::SingleDock,
                    config->active_runtime_config().primary_dock};
            },
        });
        controller->start();

        safety = std::make_unique<robot::middleware::SafetyMonitor>(
            [this] { walk_group->emergency_override(0.0f); }, left_sw, right_sw, bus);
        safety->set_limit_settled_callback(
            [this](robot::domain::Endpoint endpoint) { controller->post_limit_settled(endpoint); });
        safety->start();
        attitude_limit->start_monitoring();
    }

    void construct_threads() {
        watchdog = std::make_unique<robot::app::WatchdogMgr>("");
        watchdog->set_timeout_callback([this](const std::string& thread_name) {
            if (!error_manager)
                return;
            error_manager->submit_watchdog_timeout(thread_name, hw_steady_now_ms());
        });
        watchdog->start();

        walk_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{"walk_ctrl", 50, SCHED_FIFO, 80, 1 << 5});
        walk_exec->add_runnable(motion);
        const int walk_wd = watchdog->register_thread("walk_ctrl", 500);
        walk_exec->add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [this, walk_wd] { watchdog->heartbeat(walk_wd); }));

        gps_stuck_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{"gps_stuck", 500, SCHED_FIFO, 65, 1 << 6});
        gps_stuck_exec->add_runnable(gps_stuck);
        const int gps_stuck_wd = watchdog->register_thread("gps_stuck", 2000);
        gps_stuck_exec->add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [this, gps_stuck_wd] { watchdog->heartbeat(gps_stuck_wd); }));

        bms_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{"bms", 500, SCHED_OTHER, 0, 0x0F});
        bms_exec->add_runnable(
            std::make_shared<robot::middleware::RunnableAdapter>([this] { bms->update(); }));
        const int bms_wd = watchdog->register_thread("bms", 5000);
        bms_exec->add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [this, bms_wd] { watchdog->heartbeat(bms_wd); }));

        brush_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{"brush", 500, SCHED_OTHER, 0, 0x0F});
        brush_exec->add_runnable(
            std::make_shared<robot::middleware::RunnableAdapter>([this] { brush->update(); }));
        const int brush_wd = watchdog->register_thread("brush", 2000);
        brush_exec->add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [this, brush_wd] { watchdog->heartbeat(brush_wd); }));

        const int diagnostics_collect_period =
            std::max(50, config->get<int>("diagnostics.collect_interval_ms", 50));
        diagnostics_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{
                "diagnostics", diagnostics_collect_period, SCHED_OTHER, 0, 0x0F});
        diagnostics_exec->add_runnable(diagnostics);

        if (enable_error_handling) {
            recovery_executor =
                std::make_unique<robot::app::RecoveryExecutor>(make_recovery_ports());
            error_handling = std::make_shared<robot::app::ErrorHandlingService>(
                *error_manager,
                robot::app::ErrorHandlingService::Ports{
                    [this] { return controller->snapshot().state; },
                    [this]() -> std::optional<robot::app::ErrorFact> {
                        if (const auto event = attitude_limit->consume_pending_event()) {
                            const auto code = event->type == robot::service::AttitudeLimitService::
                                                                 EventType::AttitudeLimitBoth
                                                  ? robot::app::ErrorCode::AttitudeLimitBoth
                                                  : robot::app::ErrorCode::AttitudeLimit;
                            const auto snap = controller->snapshot();
                            const std::string detail =
                                code == robot::app::ErrorCode::AttitudeLimit &&
                                        snap.current_segment_target && snap.current_segment_mode
                                    ? std::string{"attitude_limit_key:target="} +
                                          robot::domain::endpoint_config_string(
                                              *snap.current_segment_target) +
                                          ";mode=Cleaning"
                                    : "attitude_limit_switch";
                            return robot::app::ErrorFact{
                                code,
                                robot::app::ComponentId{
                                    robot::app::ComponentKind::AttitudeLimitSwitch,
                                    static_cast<int>(event->side)},
                                detail,
                                hw_steady_now_ms()};
                        }
                        return std::nullopt;
                    },
                    [this] { return diagnostics->error_snapshot(); },
                    [this](bool enabled) { gps_stuck->set_monitoring_enabled(enabled); },
                    [this](const robot::app::ErrorDecision& decision) {
                        controller->apply_error_decision(decision);
                    },
                    [this](const robot::app::ErrorDecision& decision) {
                        const auto result = recovery_executor->execute(
                            robot::app::RecoveryRequest{decision.plan, decision.component});
                        return robot::app::RecoveryResultFact{
                            decision, result.ok, result.reason, hw_steady_now_ms()};
                    },
                    [this](bool ok) { controller->post_recovery_finished(ok); },
                    [] { return hw_steady_now_ms(); },
                });
            error_exec = std::make_unique<robot::middleware::ThreadExecutor>(
                robot::middleware::ThreadExecutor::Config{"error_mgr", 100, SCHED_OTHER, 0, 0x0F});
            error_exec->add_runnable(error_handling);
        }

        const int active_local_log_period =
            std::max(50, config->get<int>("diagnostics.local_log_interval_active_ms", 50));
        health_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{
                "health", active_local_log_period, SCHED_OTHER, 0, 0x0F});
        health_exec->add_runnable(health);
    }

    robot::app::RecoveryExecutor::Ports make_recovery_ports() {
        robot::app::RecoveryExecutor::Ports ports;
        ports.pause_gps_stuck = [this] { gps_stuck->set_monitoring_enabled(false); };
        ports.resume_gps_stuck = [this] { gps_stuck->set_monitoring_enabled(true); };
        ports.stop_walk = [this] {
            motion->emergency_stop();
            return true;
        };
        ports.reverse_walk_motion = [this] {
            return motion->reverse_for_recovery(2s, 20ms, [] { return false; });
        };
        ports.lower_attitude_center = [this] {
            ++attitude_center_attempts;
            const auto result = attitude_limit->lower_attitude_center();
            switch (result.outcome) {
                case robot::service::AttitudeLimitService::CenterOutcome::Completed:
                    ++attitude_center_completed;
                    spdlog::info(
                        "[hw_system][main_like_attitude_recover] production center completed");
                    return robot::app::RecoveryStepResult{
                        robot::app::RecoveryStepOutcome::Completed};
                case robot::service::AttitudeLimitService::CenterOutcome::TimedOut:
                    spdlog::error(
                        "[hw_system][main_like_attitude_recover] production center timed out");
                    return robot::app::RecoveryStepResult{
                        robot::app::RecoveryStepOutcome::TimedOut};
                case robot::service::AttitudeLimitService::CenterOutcome::
                    InterruptedBySafetyOverride:
                    spdlog::error(
                        "[hw_system][main_like_attitude_recover] production center interrupted by "
                        "safety override");
                    return robot::app::RecoveryStepResult{
                        robot::app::RecoveryStepOutcome::InterruptedBySafetyOverride};
            }
            return robot::app::RecoveryStepResult{robot::app::RecoveryStepOutcome::TimedOut};
        };
        return ports;
    }

    bool start_threads() {
        if (!(walk_exec->start() && gps_stuck_exec->start() && bms_exec->start() &&
              brush_exec->start() && diagnostics_exec->start())) {
            return false;
        }
        if (error_exec && !error_exec->start()) {
            return false;
        }
        return health_exec->start();
    }
};

void run_main_like_rpc_flow(robot::domain::RobotCommandKind command,
                            const char* command_id,
                            std::chrono::seconds mission_timeout,
                            int expected_lock_open_count,
                            bool enable_error_handling = true,
                            bool expect_attitude_recovery = false) {
    MainLikeHardwareFixture f(enable_error_handling);
    REQUIRE(f.init());

    const auto start = f.submit_rpc(command, command_id);
    REQUIRE(start.accepted);
    REQUIRE(f.controller->snapshot().state == "SelfChecking");

    REQUIRE(f.complete_self_check_from_diagnostics(30s));
    REQUIRE(f.controller->snapshot().state == "ExecutingMission");
    CHECK(f.lock_close_count == 1);
    CHECK(f.lock_open_count == 0);

    if (expect_attitude_recovery) {
        REQUIRE(enable_error_handling);
        REQUIRE(f.wait_for_attitude_recovery(60s));
        CHECK(f.attitude_center_attempts.load() == 1);
        CHECK(f.attitude_center_completed.load() == 1);
    }

    REQUIRE(f.wait_until_idle(mission_timeout));
    CHECK(f.controller->snapshot().state == "Idle");
    CHECK(f.lock_close_count == 1);
    CHECK(f.lock_open_count == expected_lock_open_count);
    f.stop_and_require_diagnostics_log();
}

}  // namespace

TEST_CASE("主程序等价链路执行 RPC ConfiguredMission 后安全退出",
          "[flow][flow.config][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::StartConfiguredMission,
                           "hw-main-like-configured",
                           std::chrono::seconds(kp.limit_timeout_sec * 2 + 90),
                           1);
}

TEST_CASE("主程序等价链路执行 RPC ConfiguredMission 且不启用错误处理后安全退出",
          "[flow][flow.config-no-error][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::StartConfiguredMission,
                           "hw-main-like-configured-no-error",
                           std::chrono::seconds(kp.limit_timeout_sec * 2 + 90),
                           1,
                           false);
}

TEST_CASE("完整任务链 + 姿态极限触发后回中恢复",
          "[flow][flow.attitude-recovery][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::StartConfiguredMission,
                           "hw-main-like-attitude-recover",
                           std::chrono::seconds(kp.limit_timeout_sec * 2 + 90),
                           1,
                           true,
                           true);
}

TEST_CASE("主程序等价链路执行 RPC CleanTowardOppositeEndpoint 后安全退出",
          "[flow][flow.opposite][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                           "hw-main-like-opposite",
                           std::chrono::seconds(kp.limit_timeout_sec + 90),
                           0);
}

TEST_CASE("主程序等价链路执行 RPC CleanTowardOppositeEndpoint 且不启用错误处理后安全退出",
          "[flow][flow.opposite-no-error][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
                           "hw-main-like-opposite-no-error",
                           std::chrono::seconds(kp.limit_timeout_sec + 90),
                           0,
                           false);
}

TEST_CASE("主程序等价链路执行 RPC CleanTowardPrimaryDock 后安全退出",
          "[flow][flow.primary][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::CleanTowardPrimaryDock,
                           "hw-main-like-primary",
                           std::chrono::seconds(kp.limit_timeout_sec + 90),
                           1);
}

TEST_CASE("主程序等价链路执行 RPC CleanTowardPrimaryDock 且不启用错误处理后安全退出",
          "[flow][flow.primary-no-error][manual][long]") {
    run_main_like_rpc_flow(robot::domain::RobotCommandKind::CleanTowardPrimaryDock,
                           "hw-main-like-primary-no-error",
                           std::chrono::seconds(kp.limit_timeout_sec + 90),
                           1,
                           false);
}
