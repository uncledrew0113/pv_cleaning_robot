/**
 * @file system_hw_common.h
 * @brief 真实硬件系统级集成测试公共夹具。
 *
 * 本文件搭建与主程序一致的核心硬件、服务和状态机组合，用于验证真实机器人上的运动、限位、
 * 诊断和恢复链路。测试会实际驱动电机和 GPIO，运行前必须确认设备处于安全测试环境。
 */
#include <algorithm>
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hw_config.h"
#include "hw_exit_guard.h"
#include "integration/thingsboard_test_support.h"
#include "mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/gps_stuck_service.h"
#include "pv_cleaning_robot/service/uds_gyro_yaw_fusion.h"

using namespace std::chrono_literals;

namespace {

const hw::HwParams kp = hw::load_hw_test_config();

rapidjson::Document parse_json_line(const std::string& line) {
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

const char* endpoint_to_config(robot::domain::Endpoint endpoint) {
    return endpoint == robot::domain::Endpoint::A ? "A" : "B";
}

int endpoint_to_code(robot::domain::Endpoint endpoint) {
    return endpoint == robot::domain::Endpoint::A ? 1 : 2;
}

const char* fault_code_name(uint32_t code) {
    using namespace robot::domain::FaultCode;
    switch (code) {
        case kCanCommunicationLost:
            return "CanCommunicationLost";
        case kGpsCommunicationLost:
            return "GpsCommunicationLost";
        case kSegmentStartFailed:
            return "SegmentStartFailed";
        case kRecoveryFailed:
            return "RecoveryFailed";
        case kTaskContextInconsistent:
            return "TaskContextInconsistent";
        case kLockMotorCloseFailed:
            return "LockMotorCloseFailed";
        case kLockMotorOpenFailed:
            return "LockMotorOpenFailed";
        case kUnexpectedLimitSide:
            return "UnexpectedLimitSide";
        case kConflictingLimitSides:
            return "ConflictingLimitSides";
        case kBmsCommunicationLost:
            return "BmsCommunicationLost";
        case kBrushMotorCommunicationLost:
            return "BrushMotorCommunicationLost";
        case kImuCommunicationLost:
            return "ImuCommunicationLost";
        case kWalkMotorStall:
            return "WalkMotorStall";
        case kBrushMotorFault:
            return "BrushMotorFault";
        case kAttitudeLimitBoth:
            return "AttitudeLimitBoth";
        case kRepeatedAttitudeLimit:
            return "RepeatedAttitudeLimit";
        case kGpsStuck:
            return "GpsStuck";
        case kSelfCheckFailed:
            return "SelfCheckFailed";
        default:
            return "Unknown";
    }
}

std::string fault_to_string(const std::optional<uint32_t>& fault) {
    if (!fault.has_value()) {
        return "none";
    }
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << *fault << "(" << fault_code_name(*fault) << ")";
    return oss.str();
}

const char* pid_mode_name(robot::service::HeadingCorrector::Mode mode) {
    using Mode = robot::service::HeadingCorrector::Mode;
    switch (mode) {
        case Mode::UNINITIALIZED:
            return "UNINITIALIZED";
        case Mode::DISCONNECTED:
            return "DISCONNECTED";
        case Mode::STALE:
            return "STALE";
        case Mode::TRACK:
            return "TRACK";
    }
    return "UNKNOWN";
}

robot::domain::PositionState position_at(robot::domain::Endpoint endpoint) {
    return endpoint == robot::domain::Endpoint::A ? robot::domain::PositionState::AtA
                                                  : robot::domain::PositionState::AtB;
}

robot::service::HeadingCorrector::Params make_heading_probe_params() {
    robot::service::HeadingCorrector::Params params;
    params.uds_path = kp.pid.uds_path;
    params.reconnect_interval_ms = kp.pid.reconnect_interval_ms;
    params.result_timeout_ms = kp.pid.result_timeout_ms;
    params.min_confidence = kp.pid.min_confidence;
    params.deadband_yaw_deg = kp.pid.deadband_yaw_deg;
    params.kp = kp.pid.kp;
    params.ki = kp.pid.ki;
    params.kd = kp.pid.kd;
    params.integral_limit = kp.pid.integral_limit;
    params.max_output = kp.pid.max_output;
    params.min_effective_output = kp.pid.min_effective_output;
    params.yaw_alpha = kp.pid.yaw_alpha;
    params.output_sign = kp.pid.output_sign;
    return params;
}

std::vector<std::filesystem::path> collect_rotated_health_logs(
    const std::filesystem::path& base_path) {
    std::vector<std::filesystem::path> paths;
    const auto dir =
        base_path.parent_path().empty() ? std::filesystem::path(".") : base_path.parent_path();
    const auto filename = base_path.filename().string();
    if (!std::filesystem::exists(dir)) {
        return paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto candidate = entry.path().filename().string();
        if (candidate == filename || candidate.rfind(filename + ".", 0) == 0) {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

void remove_rotated_health_logs(const std::filesystem::path& base_path) {
    for (const auto& path : collect_rotated_health_logs(base_path)) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
}

bool open_jsonl_for_metrics(const std::filesystem::path& path, std::ofstream& out) {
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            spdlog::error("[hw_system][pid_combined] create metrics dir failed: {}", ec.message());
            return false;
        }
    }
    out.open(path, std::ios::trunc);
    out.setf(std::ios::unitbuf);
    return out.is_open();
}

std::string build_pid_sample_json(int64_t ts_ms,
                                  int segment_index,
                                  const std::string& state,
                                  float yaw_deg,
                                  const robot::device::WalkMotorGroup::GroupDiagnostics& walk,
                                  const robot::service::HeadingCorrector::DebugState& pid) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("ts_ms");
    writer.Int64(ts_ms);
    writer.Key("seg");
    writer.Int(segment_index);
    writer.Key("state");
    writer.String(state.c_str(), static_cast<rapidjson::SizeType>(state.size()));
    writer.Key("yaw");
    writer.Double(yaw_deg);
    writer.Key("lt_rpm");
    writer.Double(walk.wheel[0].speed_rpm);
    writer.Key("rt_rpm");
    writer.Double(walk.wheel[1].speed_rpm);
    writer.Key("lb_rpm");
    writer.Double(walk.wheel[2].speed_rpm);
    writer.Key("rb_rpm");
    writer.Double(walk.wheel[3].speed_rpm);
    writer.Key("pid_mode");
    writer.String(pid_mode_name(pid.mode));
    writer.Key("pid_connected");
    writer.Bool(pid.connected);
    writer.Key("pid_latest_valid");
    writer.Bool(pid.latest_valid);
    writer.Key("pid_latest_yaw");
    writer.Double(pid.latest_yaw_deg);
    writer.Key("pid_confidence");
    writer.Double(pid.latest_confidence);
    writer.Key("pid_filtered_yaw");
    writer.Double(pid.filtered_yaw_deg);
    writer.Key("pid_correction");
    writer.Double(pid.last_correction);
    writer.Key("pid_age_ms");
    writer.Int64(pid.result_age_ms);
    writer.Key("pid_fused_yaw");
    writer.Double(pid.fused_yaw_deg);
    writer.Key("pid_fused_valid");
    writer.Bool(pid.fused_yaw_valid);
    writer.Key("pid_gyro_z_dps");
    writer.Double(pid.gyro_z_dps);
    writer.Key("pid_fusion_innovation");
    writer.Double(pid.fusion_innovation_deg);
    writer.Key("pid_fusion_k_gain");
    writer.Double(pid.fusion_kalman_gain_angle);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_segment_summary_json(int segment_index,
                                       uint32_t settled_count,
                                       const std::string& state,
                                       double duration_s) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("type");
    writer.String("segment_summary");
    writer.Key("seg");
    writer.Int(segment_index);
    writer.Key("settled_count");
    writer.Uint(settled_count);
    writer.Key("state");
    writer.String(state.c_str(), static_cast<rapidjson::SizeType>(state.size()));
    writer.Key("duration_s");
    writer.Double(duration_s);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_final_summary_json(int segments,
                                     int health_records,
                                     float max_yaw_drift_deg,
                                     float max_output) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("type");
    writer.String("final_summary");
    writer.Key("total_segs");
    writer.Int(segments);
    writer.Key("total_records");
    writer.Int(health_records);
    writer.Key("max_drift_all_deg");
    writer.Double(max_yaw_drift_deg);
    writer.Key("max_output");
    writer.Double(max_output);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

robot::service::MotionService::Config make_motion_config(bool pid_enabled) {
    robot::service::MotionService::Config cfg;
    cfg.clean_speed_rpm = std::abs(kp.test_speed_rpm);
    cfg.return_speed_rpm = std::abs(kp.test_return_rpm);
    cfg.brush_rpm = static_cast<int>(std::lround(std::abs(kp.brush_test_rpm)));
    cfg.heading_pid_en = pid_enabled;
    cfg.control_dt_s = static_cast<float>(kp.loop_period_ms) / 1000.0f;
    cfg.pid.uds_path = kp.pid.uds_path;
    cfg.pid.reconnect_interval_ms = kp.pid.reconnect_interval_ms;
    cfg.pid.result_timeout_ms = kp.pid.result_timeout_ms;
    cfg.pid.min_confidence = kp.pid.min_confidence;
    cfg.pid.deadband_yaw_deg = kp.pid.deadband_yaw_deg;
    cfg.pid.kp = kp.pid.kp;
    cfg.pid.ki = kp.pid.ki;
    cfg.pid.kd = kp.pid.kd;
    cfg.pid.integral_limit = kp.pid.integral_limit;
    cfg.pid.max_output = kp.pid.max_output;
    cfg.pid.min_effective_output = kp.pid.min_effective_output;
    cfg.pid.yaw_alpha = kp.pid.yaw_alpha;
    cfg.pid.output_sign = kp.pid.output_sign;
    return cfg;
}

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
    std::shared_ptr<robot::service::GpsStuckService> gps_stuck;
    std::unique_ptr<robot::service::ConfigService> config;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::app::RobotController> controller;
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio;
    std::shared_ptr<robot::device::LimitSwitch> left_sw;
    std::shared_ptr<robot::device::LimitSwitch> right_sw;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::unique_ptr<robot::middleware::ThreadExecutor> bms_exec;

    robot::domain::PositionState position_state{robot::domain::PositionState::OnSegment};
    bool real_brush{false};
    bool initialized{false};
    uint32_t repeat_count{1};
    std::optional<robot::domain::Endpoint> active_segment_target;
    std::atomic<int> active_segment_target_code{0};

    bool init(bool use_real_brush = false,
              bool pid_enabled = false,
              const std::string& health_jsonl_path = {},
              std::optional<robot::service::MotionService::Config> motion_config_override = {}) {
        real_brush = use_real_brush;
        write_config_files();

        config = std::make_unique<robot::service::ConfigService>(paths.runtime_path.string(),
                                                                 paths.fixed_path.string());
        if (!config->load()) {
            return false;
        }

        can_bus = std::make_shared<robot::driver::LinuxCanSocket>(kp.can_iface);
        walk_group =
            std::make_shared<robot::device::WalkMotorGroup>(can_bus,
                                                            kp.motor_id_base,
                                                            kp.comm_timeout_ms,
                                                            kp.termination_init_enabled,
                                                            kp.termination_init_retry_count,
                                                            kp.termination_motor_id);
        if (walk_group->open() != robot::device::DeviceError::OK) {
            return false;
        }
        walk_group->set_feedback_mode_all(10u);

        if (real_brush) {
            robot::hal::UartConfig brush_uart{};
            brush_uart.baudrate = kp.brush_baud;
            brush_uart.data_bits = 8;
            brush_uart.parity = 'N';
            brush_uart.stop_bits = 1;
            brush_uart.write_timeout_ms = 500;
            real_brush_serial =
                std::make_shared<robot::driver::LibSerialPort>(kp.brush_port, brush_uart);
            brush = std::make_shared<robot::device::BrushMotor>(real_brush_serial, kp.brush_axis);
        } else {
            mock_brush_serial = std::make_shared<MockSerialPort>();
            mock_brush_serial->open_result = true;
            brush = std::make_shared<robot::device::BrushMotor>(mock_brush_serial, kp.brush_axis);
        }
        if (!brush->open()) {
            return false;
        }

        imu_serial = std::make_shared<robot::driver::LibSerialPort>(
            kp.imu_port, robot::hal::UartConfig{kp.imu_baud});
        imu = std::make_shared<robot::device::ImuDevice>(imu_serial);
        if (!imu->open()) {
            return false;
        }

        bms_serial = std::make_shared<robot::driver::LibSerialPort>(
            kp.bms_port, robot::hal::UartConfig{kp.bms_baud});
        bms = std::make_shared<robot::device::BMS>(bms_serial, 95.0f, 15.0f);
        bms->open();

        robot::device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = kp.gpsd_host;
        gpsd_cfg.port = kp.gpsd_port;
        gpsd_cfg.watch = kp.gpsd_watch;
        gps = robot::device::GpsDevice::create_gpsd(gpsd_cfg);
        gps->open();
        gps_stuck = std::make_shared<robot::service::GpsStuckService>(gps);
        const auto motion_cfg =
            motion_config_override ? *motion_config_override : make_motion_config(pid_enabled);
        motion =
            std::make_shared<robot::service::MotionService>(walk_group, brush, imu, bus, motion_cfg);
        motion->set_runtime_config_query([this]() { return config->active_runtime_config(); });
        motion->set_primary_dock_query(
            [this]() { return config->active_runtime_config().primary_dock; });

        robot::app::RobotController::ActionPorts actions;
        actions.start_segment = [this](const robot::domain::MissionSegment& segment) {
            active_segment_target = segment.target;
            active_segment_target_code.store(endpoint_to_code(segment.target),
                                             std::memory_order_release);
            spdlog::info("[hw_system] start_segment target={} mode=Cleaning",
                         endpoint_to_config(segment.target));
            return motion->start_segment(segment);
        };
        actions.stop_motion = [this]() { motion->stop_cleaning(); };
        actions.emergency_stop = [this]() { motion->emergency_stop(); };
        // RobotController 自身锁存/清除故障，测试夹具不再维护额外故障缓存。
        actions.clear_fault = []() {};
        controller = std::make_shared<robot::app::RobotController>(actions);
        controller->set_config_ports(robot::app::RobotController::ConfigPorts{
            [this]() { return config->active_runtime_config(); },
            [this]() { return config->pending_runtime_config(); },
            [this](const robot::domain::RuntimeConfig& cfg) {
                return config->runtime_config_version(cfg);
            },
            [this]() { return config->promote_pending_runtime_to_active(); },
            [this]() {
                const auto dock_mode =
                    config->get<std::string>("robot.dock_mode", "single_dock") == "dual_dock"
                        ? robot::domain::DockMode::DualDock
                        : robot::domain::DockMode::SingleDock;
                return robot::domain::LaneConfig{dock_mode,
                                                 config->active_runtime_config().primary_dock};
            },
        });
        controller->set_battery_soc_query([] { return 80.0f; });
        controller->set_position_state_query([this]() { return position_state; });
        controller->start();
        if (!health_jsonl_path.empty()) {
            health = std::make_shared<robot::service::HealthService>(
                walk_group,
                brush,
                bms,
                imu,
                gps,
                nullptr,
                robot::service::HealthService::Mode::DIAGNOSTICS,
                health_jsonl_path,
                kp.health_log_max_bytes,
                kp.health_log_max_files);
        }
        watchdog = std::make_unique<robot::app::WatchdogMgr>("");

        initialized = true;
        hw::HwExitGuard::instance().install();
        hw::HwExitGuard::instance().set_active(this);
        return true;
    }

    bool start_safety_bridge() {
        left_gpio = std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip, kp.left_limit_line);
        right_gpio =
            std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip, kp.right_limit_line);
        left_sw =
            std::make_shared<robot::device::LimitSwitch>(left_gpio, robot::device::LimitSide::LEFT);
        right_sw = std::make_shared<robot::device::LimitSwitch>(right_gpio,
                                                                robot::device::LimitSide::RIGHT);
        // RK3576 gpiochip5 does not provide edge-event IRQs for these lines on
        // the test rig, so force polling mode for the real limit close loop.
        if (!left_sw->open(0, 2, 0, false) || !right_sw->open(0, 2, 0, false)) {
            return false;
        }

        safety = std::make_unique<robot::middleware::SafetyMonitor>(
            [this]() {
                if (walk_group) {
                    walk_group->emergency_override(0.0f);
                }
            },
            left_sw,
            right_sw,
            bus);
        safety->set_limit_settled_callback(
            [this](robot::domain::Endpoint endpoint) { controller->post_limit_settled(endpoint); });
        return safety->start();
    }

    bool start_bms_polling() {
        if (!bms) {
            return false;
        }
        if (bms_exec && bms_exec->is_running()) {
            return true;
        }
        bms_exec = std::make_unique<robot::middleware::ThreadExecutor>(
            robot::middleware::ThreadExecutor::Config{"bms", 500, 0, 0, 0x0F});
        bms_exec->add_runnable(
            std::make_shared<robot::middleware::RunnableAdapter>([this]() {
                if (bms) {
                    bms->update();
                }
            }));
        return bms_exec->start();
    }

    void stop_bms_polling() {
        if (bms_exec) {
            bms_exec->stop();
            bms_exec.reset();
        }
    }

    robot::app::CommandResult start_directional_to_opposite() {
        auto result = controller->submit_command(robot::domain::RobotCommand{
            robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint,
            robot::domain::CommandSource::Rpc,
            "hw-clean-opposite"});
        if (!result.accepted) {
            return result;
        }
        controller->complete_self_check(true);
        controller->drain_for_test();
        return {true, "accepted"};
    }

    robot::app::CommandResult start_directional_to_primary_dock_from_segment() {
        position_state = robot::domain::PositionState::OnSegment;
        auto result = controller->submit_command(robot::domain::RobotCommand{
            robot::domain::RobotCommandKind::CleanTowardPrimaryDock,
            robot::domain::CommandSource::Rpc,
            "hw-clean-primary-dock"});
        if (!result.accepted) {
            return result;
        }
        controller->complete_self_check(true);
        controller->drain_for_test();
        return {true, "accepted"};
    }

    robot::app::CommandResult start_configured_assuming_primary_dock() {
        // 系统级测试按配置假定机器人从 primary_dock 语义出发，避免测试夹具启动时读取
        // 真实主限位导致用例间相互影响。
        position_state = position_at(kp.primary_dock);
        auto result = controller->submit_command(
            robot::domain::RobotCommand{robot::domain::RobotCommandKind::StartConfiguredMission,
                                        robot::domain::CommandSource::Rpc,
                                        "hw-configured"});
        if (!result.accepted) {
            return result;
        }
        controller->complete_self_check(true);
        controller->drain_for_test();
        return {true, "accepted"};
    }

    bool wait_until_state(const std::string& state, std::chrono::seconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!hw::HwExitGuard::instance().exit_requested() &&
               std::chrono::steady_clock::now() < deadline) {
            if (controller->snapshot().state == state) {
                return true;
            }
            if (motion) {
                motion->update();
            }
            if (gps_stuck) {
                gps_stuck->update();
            }
            if (health) {
                health->update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
        }
        return controller->snapshot().state == state;
    }

    void shutdown() override {
        stop_bms_polling();
        if (safety) {
            safety->stop();
        }
        if (controller) {
            controller->stop();
        }
        if (motion) {
            motion->emergency_stop();
        } else if (walk_group) {
            walk_group->emergency_override(0.0f);
            walk_group->disable_all();
        }
        if (brush) {
            brush->stop();
        }
        if (left_sw) {
            left_sw->close();
        }
        if (right_sw) {
            right_sw->close();
        }
        if (gps) {
            gps->close();
        }
        if (imu) {
            imu->close();
        }
        if (bms) {
            bms->close();
        }
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        if (real_brush_serial) {
            real_brush_serial->close();
        }
        if (initialized) {
            hw::HwExitGuard::instance().clear_active(this);
            initialized = false;
        }
        tb_test_support::cleanup_split_config_paths(paths);
    }

    ~SystemHwFixture() override {
        shutdown();
    }

   private:
    void write_config_files() const {
        std::ostringstream out;
        out << "{\n"
            << "  \"robot\": {\n"
            << "    \"dock_mode\": \"single_dock\",\n"
            << "    \"repeat_count\": " << std::max<uint32_t>(1u, repeat_count) << ",\n"
            << "    \"clean_speed_rpm\": " << std::abs(kp.test_speed_rpm) << ",\n"
            << "    \"return_speed_rpm\": " << std::abs(kp.test_return_rpm) << ",\n"
            << "    \"brush_rpm\": " << static_cast<int>(std::lround(std::abs(kp.brush_test_rpm)))
            << ",\n"
            << "    \"primary_dock\": \"" << endpoint_to_config(kp.primary_dock) << "\",\n"
            << "    \"min_battery_soc\": 30.0,\n"
            << "    \"charge_stop_soc\": 95.0\n"
            << "  },\n"
            << "  \"scheduler\": { \"windows\": [] }\n"
            << "}\n";
        const std::string runtime_json = out.str();
        tb_test_support::write_split_config(paths, runtime_json, R"({})");
    }
};

struct WalkImuStopGuard : hw::IGracefulShutdown {
    explicit WalkImuStopGuard(robot::device::WalkMotorGroup& group) : group_(group) {
        hw::HwExitGuard::instance().install();
        hw::HwExitGuard::instance().set_active(this);
    }

    ~WalkImuStopGuard() override {
        shutdown();
        hw::HwExitGuard::instance().clear_active(this);
    }

    void shutdown() override {
        group_.emergency_override(0.0f);
        group_.disable_all();
    }

    robot::device::WalkMotorGroup& group_;
};

struct CombinedAttitudeRecoveryContext {
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio{
        std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip,
                                                     kp.left_attitude_limit_line)};
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio{
        std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip,
                                                     kp.right_attitude_limit_line)};
    robot::device::AttitudeLimitSwitch left_sw{left_gpio,
                                               robot::device::AttitudeLimitSide::LEFT_LOWER};
    robot::device::AttitudeLimitSwitch right_sw{right_gpio,
                                                robot::device::AttitudeLimitSide::RIGHT_LOWER};
    int recovery_count{0};

    bool open() {
        return left_sw.open(0, 2, 0, false) && right_sw.open(0, 2, 0, false);
    }
};

enum class LowerAttitudeCenterOutcome {
    Completed,
    InterruptedByEndpointA,
    InterruptedByEndpointB,
};

using EndpointInterruptQuery = std::function<std::optional<robot::domain::Endpoint>()>;

struct PitchSample {
    float pitch_min{0.0f};
    float pitch_max{0.0f};
    float roll_abs_max{0.0f};
    int samples{0};
};

PitchSample sample_lower_wheel_pitch() {
    hw::DeviceFixture f;
    REQUIRE(f.walk_group->open() == robot::device::DeviceError::OK);
    REQUIRE(f.imu->open());
    WalkImuStopGuard guard(*f.walk_group);

    REQUIRE(f.walk_group->set_feedback_mode_all(10u) == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);

    // 只驱动下轨道两轮，观察 IMU 姿态响应。上轨道轮保持 0，避免整车高速移动。
    const float lower_rpm = std::abs(kp.sweep_rpm);
    REQUIRE(f.walk_group->set_speeds(0.0f, 0.0f, lower_rpm, lower_rpm) ==
            robot::device::DeviceError::OK);

    PitchSample sample;
    sample.pitch_min = 999.0f;
    sample.pitch_max = -999.0f;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kp.sweep_duration_ms);
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline) {
        f.walk_group->update();
        const auto imu_data = f.imu->get_latest();
        sample.pitch_min = std::min(sample.pitch_min, imu_data.pitch_deg);
        sample.pitch_max = std::max(sample.pitch_max, imu_data.pitch_deg);
        sample.roll_abs_max = std::max(sample.roll_abs_max, std::abs(imu_data.roll_deg));
        ++sample.samples;
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    f.walk_group->set_speed_uniform(0.0f);
    f.walk_group->update();
    return sample;
}

struct UdsYawSample {
    bool valid{false};
    float raw_yaw_deg{0.0f};
    float filtered_yaw_deg{0.0f};
    float confidence{0.0f};
    int64_t age_ms{-1};
};

UdsYawSample read_uds_yaw_sample(robot::service::HeadingCorrector& probe) {
    robot::service::HeadingCorrector::Input input;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.base_command = {};
    input.travel_direction = robot::domain::TravelDirection::BToA;
    input.primary_dock = kp.primary_dock;
    probe.compute(input);

    const auto state = probe.debug_state();
    const bool valid = state.mode == robot::service::HeadingCorrector::Mode::TRACK &&
                       state.connected && state.latest_valid &&
                       state.latest_confidence >= kp.pid.min_confidence &&
                       state.result_age_ms >= 0 && state.result_age_ms <= kp.pid.result_timeout_ms;
    return {valid,
            state.latest_yaw_deg,
            state.filtered_yaw_deg,
            state.latest_confidence,
            state.result_age_ms};
}

UdsYawSample wait_uds_yaw_sample(robot::service::HeadingCorrector& probe,
                                 robot::device::WalkMotorGroup& group,
                                 std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    UdsYawSample sample;
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline) {
        group.update();
        sample = read_uds_yaw_sample(probe);
        if (sample.valid) {
            return sample;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }
    return sample;
}

robot::service::UdsGyroYawFusion::Params make_probe_fusion_params() {
    robot::service::UdsGyroYawFusion::Params params;
    params.process_noise_angle = kp.correction_compare.fusion.process_noise_angle;
    params.process_noise_bias = kp.correction_compare.fusion.process_noise_bias;
    params.measurement_noise_uds = kp.correction_compare.fusion.measurement_noise_uds;
    params.initial_angle_variance = kp.correction_compare.fusion.initial_angle_variance;
    params.initial_bias_variance = kp.correction_compare.fusion.initial_bias_variance;
    params.max_gyro_only_ms = kp.correction_compare.fusion.max_gyro_only_ms;
    return params;
}

void run_uds_gyro_fusion_probe_test() {
    hw::DeviceFixture f;
    REQUIRE(f.imu->open());

    robot::service::HeadingCorrector uds_probe(make_heading_probe_params());
    uds_probe.enable(true);

    robot::service::UdsGyroYawFusion fusion;
    fusion.set_params(make_probe_fusion_params());

    spdlog::warn(
        "[hw_system][uds_gyro_fusion_probe] 只读连续采样，不驱动电机；duration={}ms "
        "period={}ms fusion(q_angle={:.4f} q_bias={:.4f} r_uds={:.4f} gyro_only_ms={})",
        kp.sweep_duration_ms,
        kp.loop_period_ms,
        kp.correction_compare.fusion.process_noise_angle,
        kp.correction_compare.fusion.process_noise_bias,
        kp.correction_compare.fusion.measurement_noise_uds,
        kp.correction_compare.fusion.max_gyro_only_ms);

    int samples = 0;
    int uds_valid_count = 0;
    int imu_valid_count = 0;
    int fused_valid_count = 0;
    auto last = std::chrono::steady_clock::now();
    const auto deadline = last + std::chrono::milliseconds(kp.sweep_duration_ms);

    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        float dt_s = std::chrono::duration<float>(now - last).count();
        if (dt_s <= 0.0f) {
            dt_s = static_cast<float>(kp.loop_period_ms) / 1000.0f;
        }
        last = now;

        const auto uds = read_uds_yaw_sample(uds_probe);
        const auto imu = f.imu->get_latest();

        robot::service::UdsGyroYawFusion::Input input;
        input.dt_s = dt_s;
        input.uds_yaw_deg = uds.raw_yaw_deg;
        input.uds_valid = uds.valid;
        input.uds_confidence = uds.confidence;
        input.uds_age_ms = uds.age_ms;
        input.gyro_z_rad_s = imu.valid ? imu.gyro[2] : 0.0f;
        input.imu_valid = imu.valid;
        const auto fused = fusion.update(input);

        ++samples;
        if (uds.valid) {
            ++uds_valid_count;
        }
        if (imu.valid) {
            ++imu_valid_count;
        }
        if (fused.valid) {
            ++fused_valid_count;
        }

        spdlog::info(
            "[hw_system][uds_gyro_fusion_probe] sample={} dt={:.3f}s "
            "uds(valid={} raw={:.3f} filtered={:.3f} conf={:.2f} age={}ms) "
            "imu(valid={} yaw={:.2f} pitch={:.2f} roll={:.2f} gyro_z={:.3f}dps) "
            "fused(valid={} yaw={:.3f} bias={:.3f} innovation={:.3f} k={:.3f})",
            samples,
            dt_s,
            uds.valid,
            uds.raw_yaw_deg,
            uds.filtered_yaw_deg,
            uds.confidence,
            uds.age_ms,
            imu.valid,
            imu.yaw_deg,
            imu.pitch_deg,
            imu.roll_deg,
            fused.gyro_z_dps,
            fused.valid,
            fused.fused_yaw_deg,
            fused.gyro_bias_dps,
            fused.innovation_deg,
            fused.kalman_gain_angle);

        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    spdlog::warn(
        "[hw_system][uds_gyro_fusion_probe] summary samples={} uds_valid={} imu_valid={} "
        "fused_valid={}",
        samples,
        uds_valid_count,
        imu_valid_count,
        fused_valid_count);

    CHECK(samples > 0);
    CHECK(imu_valid_count > 0);
    CHECK(uds_valid_count > 0);
    CHECK(fused_valid_count > 0);
}

float compute_lower_uds_zero_cmd(float raw_yaw_deg,
                                 float deadband_deg,
                                 float min_rpm,
                                 float max_rpm,
                                 float kp_rpm_per_deg) {
    const float abs_yaw = std::abs(raw_yaw_deg);
    if (abs_yaw <= deadband_deg) {
        return 0.0f;
    }

    const float error_deg = abs_yaw - deadband_deg;
    const float rpm = std::clamp(error_deg * kp_rpm_per_deg, min_rpm, max_rpm);
    // 物理标定：向 A 时下轮 LB/RB 为负转；向 B 时下轮 LB/RB 为正转。
    return raw_yaw_deg > 0.0f ? -rpm : rpm;
}

struct LowerAttitudeCenterPlan {
    robot::device::AttitudeLimitSide release_side{robot::device::AttitudeLimitSide::LEFT_LOWER};
    robot::device::AttitudeLimitSide opposite_side{robot::device::AttitudeLimitSide::RIGHT_LOWER};
    float initial_lower_rpm{0.0f};
    float return_lower_rpm{0.0f};
};

enum class LowerAttitudeState {
    NONE,
    LEFT,
    RIGHT,
    BOTH,
};

const char* attitude_side_name(robot::device::AttitudeLimitSide side) {
    return side == robot::device::AttitudeLimitSide::LEFT_LOWER ? "LEFT_LOWER" : "RIGHT_LOWER";
}

LowerAttitudeCenterPlan make_lower_attitude_center_plan(
    robot::device::AttitudeLimitSide active_side,
    float lower_rpm) {
    const float rpm = std::abs(lower_rpm);
    if (active_side == robot::device::AttitudeLimitSide::LEFT_LOWER) {
        return {robot::device::AttitudeLimitSide::LEFT_LOWER,
                robot::device::AttitudeLimitSide::RIGHT_LOWER,
                rpm,
                -rpm};
    }
    return {robot::device::AttitudeLimitSide::RIGHT_LOWER,
            robot::device::AttitudeLimitSide::LEFT_LOWER,
            -rpm,
            rpm};
}

LowerAttitudeState classify_lower_attitude_state(
    const robot::device::AttitudeLimitSwitch::Status& left,
    const robot::device::AttitudeLimitSwitch::Status& right) {
    if (left.active_low_asserted && right.active_low_asserted) {
        return LowerAttitudeState::BOTH;
    }
    if (left.active_low_asserted) {
        return LowerAttitudeState::LEFT;
    }
    if (right.active_low_asserted) {
        return LowerAttitudeState::RIGHT;
    }
    return LowerAttitudeState::NONE;
}

std::optional<robot::device::AttitudeLimitSide> active_side_from_state(LowerAttitudeState state) {
    if (state == LowerAttitudeState::LEFT) {
        return robot::device::AttitudeLimitSide::LEFT_LOWER;
    }
    if (state == LowerAttitudeState::RIGHT) {
        return robot::device::AttitudeLimitSide::RIGHT_LOWER;
    }
    return std::nullopt;
}

robot::device::AttitudeLimitSwitch::Status read_attitude_status(
    robot::device::AttitudeLimitSide side,
    robot::device::AttitudeLimitSwitch& left_sw,
    robot::device::AttitudeLimitSwitch& right_sw) {
    return side == robot::device::AttitudeLimitSide::LEFT_LOWER ? left_sw.read_status()
                                                                : right_sw.read_status();
}

void log_attitude_status(const char* tag,
                         const robot::device::AttitudeLimitSwitch::Status& left,
                         const robot::device::AttitudeLimitSwitch::Status& right) {
    spdlog::info("{} left(level={} active={} triggered={}) right(level={} active={} triggered={})",
                 tag,
                 left.level_high ? "high" : "low",
                 left.active_low_asserted,
                 left.triggered,
                 right.level_high ? "high" : "low",
                 right.active_low_asserted,
                 right.triggered);
}

void prepare_lower_attitude_motion(robot::device::WalkMotorGroup& group) {
    if (group.is_override_active()) {
        group.clear_override();
        group.update();
    }
    REQUIRE(group.enable_all() == robot::device::DeviceError::OK);
    REQUIRE(group.set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);
}

void command_lower_wheels(robot::device::WalkMotorGroup& group, float lower_rpm) {
    REQUIRE(group.set_speeds(0.0f, 0.0f, lower_rpm, lower_rpm) == robot::device::DeviceError::OK);
    group.update();
}

bool open_aux_dock_pin(robot::driver::LibGpiodPin& pin) {
    robot::hal::GpioConfig cfg;
    cfg.direction = robot::hal::GpioDirection::INPUT;
    cfg.bias = robot::hal::GpioBias::PULL_UP;
    cfg.debounce_ms = 2;
    cfg.use_irq = false;
    return pin.open(cfg);
}

bool is_dock_limit_active(const SystemHwFixture& f, robot::domain::Endpoint dock) {
    const auto& sw = dock == robot::domain::Endpoint::A ? f.left_sw : f.right_sw;
    return sw && !sw->read_current_level();
}

bool is_aux_dock_active(robot::driver::LibGpiodPin& aux_pin) {
    return !aux_pin.read_value();
}

bool wait_dock_pair_stable(const SystemHwFixture& f,
                           robot::driver::LibGpiodPin& aux_pin,
                           robot::domain::Endpoint dock,
                           int stable_samples_required,
                           std::chrono::milliseconds timeout,
                           const char* tag) {
    int stable_samples = 0;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline &&
           stable_samples < stable_samples_required) {
        const bool dock_active = is_dock_limit_active(f, dock);
        const bool aux_active = is_aux_dock_active(aux_pin);
        if (dock_active && aux_active) {
            ++stable_samples;
        } else {
            stable_samples = 0;
        }
        spdlog::info("[{}][dock_align][stable_check] dock={} dock_active={} aux_active={} "
                     "stable={}/{}",
                     tag,
                     endpoint_to_config(dock),
                     dock_active,
                     aux_active,
                     stable_samples,
                     stable_samples_required);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }
    return stable_samples >= stable_samples_required;
}

void command_dock_adjust_pulse(robot::device::WalkMotorGroup& group,
                               robot::domain::Endpoint dock,
                               int pattern,
                               float rpm) {
    const int dir = dock == robot::domain::Endpoint::A ? 1 : -1;
    const float top = std::abs(rpm) * static_cast<float>(dir);
    const float lower = -top;
    robot::device::WalkMotorGroup::SpeedCmd cmd{};
    switch (pattern % 5) {
        case 0:
            cmd = {top, top, lower, lower};
            break;
        case 1:
            cmd = {top, top, 0.0f, 0.0f};
            break;
        case 2:
            cmd = {0.0f, 0.0f, lower, lower};
            break;
        case 3:
            cmd = {-top, -top, 0.0f, 0.0f};
            break;
        default:
            cmd = {0.0f, 0.0f, -lower, -lower};
            break;
    }
    REQUIRE(group.set_speeds(cmd) == robot::device::DeviceError::OK);
    group.update();
}

void stop_walk_group(robot::device::WalkMotorGroup& group) {
    REQUIRE(group.set_speeds(0.0f, 0.0f, 0.0f, 0.0f) == robot::device::DeviceError::OK);
    group.update();
}

void run_dock_align_test(SystemHwFixture& f) {
    const auto dock = kp.primary_dock;
    auto aux_gpio = std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip,
                                                                  kp.aux_dock_limit_line);

    REQUIRE(f.init(false, true));
    REQUIRE(f.start_safety_bridge());
    REQUIRE(open_aux_dock_pin(*aux_gpio));
    REQUIRE(f.walk_group->set_feedback_mode_all(10u) == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);

    constexpr int kStableSamplesRequired = 4;
    constexpr auto kStableCheckTimeout = 800ms;
    constexpr auto kAdjustMaxRun = 30000ms;
    constexpr auto kAdjustPulse = 250ms;
    const float adjust_rpm = std::clamp(std::abs(kp.limit_test_rpm) * 0.3f, 1.0f, 4.0f);

    spdlog::warn("[hw_system][dock_align] single RPC to dock with PID enabled; target_dock={} "
                 "aux={}/{} adjust_rpm={:.1f}",
                 endpoint_to_config(dock),
                 kp.gpio_chip,
                 kp.aux_dock_limit_line,
                 adjust_rpm);

    if (!wait_dock_pair_stable(f,
                               *aux_gpio,
                               dock,
                               kStableSamplesRequired,
                               kStableCheckTimeout,
                               "hw_system")) {
        if (!is_dock_limit_active(f, dock)) {
            REQUIRE(f.start_directional_to_primary_dock_from_segment().accepted);
            REQUIRE(f.wait_until_state("Idle", std::chrono::seconds(kp.limit_timeout_sec)));
        } else {
            f.motion->stop_cleaning();
        }
    }

    const auto adjust_deadline = std::chrono::steady_clock::now() + kAdjustMaxRun;
    int pattern = 0;
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < adjust_deadline) {
        if (wait_dock_pair_stable(f,
                                  *aux_gpio,
                                  dock,
                                  kStableSamplesRequired,
                                  kStableCheckTimeout,
                                  "hw_system")) {
            stop_walk_group(*f.walk_group);
            return;
        }

        f.walk_group->clear_override();
        f.walk_group->update();
        command_dock_adjust_pulse(*f.walk_group, dock, pattern, adjust_rpm);
        std::this_thread::sleep_for(kAdjustPulse);
        stop_walk_group(*f.walk_group);

        const bool dock_active = is_dock_limit_active(f, dock);
        const bool aux_active = is_aux_dock_active(*aux_gpio);
        spdlog::warn("[hw_system][dock_align][adjust] pattern={} dock_active={} aux_active={}",
                     pattern % 5,
                     dock_active,
                     aux_active);
        ++pattern;
    }

    stop_walk_group(*f.walk_group);
    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    INFO("dock=" << endpoint_to_config(dock));
    INFO("dock_active=" << is_dock_limit_active(f, dock));
    INFO("aux_active=" << is_aux_dock_active(*aux_gpio));
    REQUIRE(wait_dock_pair_stable(
        f, *aux_gpio, dock, kStableSamplesRequired, kStableCheckTimeout, "hw_system"));
}

LowerAttitudeCenterOutcome run_lower_attitude_center_recovery(
    robot::device::WalkMotorGroup& group,
    robot::device::AttitudeLimitSwitch& left_sw,
    robot::device::AttitudeLimitSwitch& right_sw,
    const char* tag,
    EndpointInterruptQuery endpoint_interrupt_query = {}) {
    REQUIRE(group.set_feedback_mode_all(10u) == robot::device::DeviceError::OK);
    prepare_lower_attitude_motion(group);

    constexpr float lower_rpm = 5.0f;
    constexpr int kStableSamplesRequired = 2;
    constexpr auto kOverallTimeout = 30000ms;
    constexpr auto kTriggerPause = 500ms;
    const auto overall_deadline = std::chrono::steady_clock::now() + kOverallTimeout;

    spdlog::warn(
        "[{}][lower_attitude_center] 任意单侧触发即可回中；无触发时先向 A 搜索左下边界；"
        "双侧同时触发视为异常。LT/RT=0，仅驱动 LB/RB 低速回中",
        tag);

    auto wait_until_overall_timeout = [&] {
        command_lower_wheels(group, 0.0f);
        while (!hw::HwExitGuard::instance().exit_requested() &&
               std::chrono::steady_clock::now() < overall_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
        }
    };
    auto finish_timeout = [&](const char* reason) {
        spdlog::warn("[{}][lower_attitude_center] timeout-style finish: {}", tag, reason);
        wait_until_overall_timeout();
        return LowerAttitudeCenterOutcome::Completed;
    };

    auto check_endpoint_interrupt = [&]() -> std::optional<LowerAttitudeCenterOutcome> {
        if (!endpoint_interrupt_query) {
            return std::nullopt;
        }
        const auto endpoint = endpoint_interrupt_query();
        if (!endpoint.has_value()) {
            return std::nullopt;
        }
        command_lower_wheels(group, 0.0f);
        spdlog::warn("[{}][lower_attitude_center] interrupted by endpoint={}",
                     tag,
                     endpoint_to_config(*endpoint));
        wait_until_overall_timeout();
        return *endpoint == robot::domain::Endpoint::A
                   ? LowerAttitudeCenterOutcome::InterruptedByEndpointA
                   : LowerAttitudeCenterOutcome::InterruptedByEndpointB;
    };

    auto initial_side = std::optional<robot::device::AttitudeLimitSide>{};
    bool search_started = false;
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < overall_deadline && !initial_side.has_value()) {
        if (const auto outcome = check_endpoint_interrupt()) {
            return *outcome;
        }
        const auto left = left_sw.read_status();
        const auto right = right_sw.read_status();
        const auto state = classify_lower_attitude_state(left, right);
        REQUIRE(state != LowerAttitudeState::BOTH);
        initial_side = active_side_from_state(state);

        if (!initial_side.has_value()) {
            if (!search_started) {
                search_started = true;
                log_attitude_status("[lower_attitude_center][search_start]", left, right);
                spdlog::info(
                    "[{}][lower_attitude_center][search_start] lower_rpm={:.2f} dir=A "
                    "timeout={}ms",
                    tag,
                    lower_rpm,
                    std::chrono::duration_cast<std::chrono::milliseconds>(kOverallTimeout).count());
            }
            command_lower_wheels(group, -lower_rpm);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    command_lower_wheels(group, 0.0f);
    if (!initial_side.has_value()) {
        REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
        return finish_timeout("initial side not found");
    }
    spdlog::info("[{}][lower_attitude_center][initial_pause] pause={}ms before reverse",
                 tag,
                 std::chrono::duration_cast<std::chrono::milliseconds>(kTriggerPause).count());
    if (std::chrono::steady_clock::now() < overall_deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            overall_deadline - std::chrono::steady_clock::now());
        std::this_thread::sleep_for(std::min(kTriggerPause, remaining));
    }
    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    log_attitude_status("[lower_attitude_center][initial_found]",
                        left_sw.read_status(),
                        right_sw.read_status());

    const auto plan = make_lower_attitude_center_plan(*initial_side, lower_rpm);
    spdlog::info(
        "[{}][lower_attitude_center] initial_side={} initial_lower_rpm={:.2f} "
        "return_lower_rpm={:.2f}",
        tag,
        attitude_side_name(*initial_side),
        plan.initial_lower_rpm,
        plan.return_lower_rpm);

    int stable_release_samples = 0;
    auto release_at = std::chrono::steady_clock::time_point{};
    spdlog::info(
        "[{}][lower_attitude_center][release_start] side={} lower_rpm={:.2f} timeout={}ms",
        tag,
        attitude_side_name(plan.release_side),
        plan.initial_lower_rpm,
        std::chrono::duration_cast<std::chrono::milliseconds>(kOverallTimeout).count());
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < overall_deadline &&
           stable_release_samples < kStableSamplesRequired) {
        if (const auto outcome = check_endpoint_interrupt()) {
            return *outcome;
        }
        command_lower_wheels(group, plan.initial_lower_rpm);
        const auto status = read_attitude_status(plan.release_side, left_sw, right_sw);
        if (!status.active_low_asserted) {
            ++stable_release_samples;
            if (release_at == std::chrono::steady_clock::time_point{}) {
                release_at = std::chrono::steady_clock::now();
            }
        } else {
            stable_release_samples = 0;
            release_at = {};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    if (release_at == std::chrono::steady_clock::time_point{}) {
        return finish_timeout("release side not released");
    }
    {
        const auto diag = group.get_group_diagnostics();
        spdlog::info(
            "[{}][lower_attitude_center][released] side={} stable={}/{} "
            "cmd=[{:.1f},{:.1f},{:.1f},{:.1f}] rpm=[{:.1f},{:.1f},{:.1f},{:.1f}]",
            tag,
            attitude_side_name(plan.release_side),
            stable_release_samples,
            kStableSamplesRequired,
            diag.wheel[0].target_value,
            diag.wheel[1].target_value,
            diag.wheel[2].target_value,
            diag.wheel[3].target_value,
            diag.wheel[0].speed_rpm,
            diag.wheel[1].speed_rpm,
            diag.wheel[2].speed_rpm,
            diag.wheel[3].speed_rpm);
    }

    auto opposite_at = std::chrono::steady_clock::time_point{};
    spdlog::info(
        "[{}][lower_attitude_center][opposite_start] side={} lower_rpm={:.2f} "
        "timeout={}ms",
        tag,
        attitude_side_name(plan.opposite_side),
        plan.initial_lower_rpm,
        std::chrono::duration_cast<std::chrono::milliseconds>(kOverallTimeout).count());
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < overall_deadline &&
           opposite_at == std::chrono::steady_clock::time_point{}) {
        if (const auto outcome = check_endpoint_interrupt()) {
            return *outcome;
        }
        command_lower_wheels(group, plan.initial_lower_rpm);
        const auto status = read_attitude_status(plan.opposite_side, left_sw, right_sw);
        if (status.active_low_asserted) {
            opposite_at = std::chrono::steady_clock::now();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    if (opposite_at == std::chrono::steady_clock::time_point{}) {
        return finish_timeout("opposite side not found");
    }
    log_attitude_status("[lower_attitude_center][opposite_triggered]",
                        left_sw.read_status(),
                        right_sw.read_status());

    const auto span = opposite_at - release_at;
    const auto return_duration = std::chrono::duration_cast<std::chrono::milliseconds>(span / 2);
    spdlog::info(
        "[{}][lower_attitude_center][return_start] release_to_opposite={}ms "
        "return_half={}ms return_lower_rpm={:.2f}",
        tag,
        std::chrono::duration_cast<std::chrono::milliseconds>(span).count(),
        return_duration.count(),
        plan.return_lower_rpm);

    const auto return_deadline = std::chrono::steady_clock::now() + return_duration;
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < return_deadline &&
           std::chrono::steady_clock::now() < overall_deadline) {
        if (const auto outcome = check_endpoint_interrupt()) {
            return *outcome;
        }
        command_lower_wheels(group, plan.return_lower_rpm);
        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }
    if (std::chrono::steady_clock::now() >= overall_deadline) {
        return finish_timeout("return motion not completed");
    }

    command_lower_wheels(group, 0.0f);
    log_attitude_status(
        "[lower_attitude_center][done]", left_sw.read_status(), right_sw.read_status());
    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    return LowerAttitudeCenterOutcome::Completed;
}

void run_lower_attitude_center_test() {
    hw::DeviceFixture f;
    REQUIRE(f.walk_group->open() == robot::device::DeviceError::OK);
    WalkImuStopGuard guard(*f.walk_group);

    auto left_gpio =
        std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip, kp.left_attitude_limit_line);
    auto right_gpio =
        std::make_shared<robot::driver::LibGpiodPin>(kp.gpio_chip, kp.right_attitude_limit_line);
    robot::device::AttitudeLimitSwitch left_sw(left_gpio,
                                               robot::device::AttitudeLimitSide::LEFT_LOWER);
    robot::device::AttitudeLimitSwitch right_sw(right_gpio,
                                                robot::device::AttitudeLimitSide::RIGHT_LOWER);

    REQUIRE(left_sw.open(0, 2, 0, false));
    REQUIRE(right_sw.open(0, 2, 0, false));
    run_lower_attitude_center_recovery(
        *f.walk_group, left_sw, right_sw, "hw_system][lower_attitude_center");
}

void run_lower_uds_zero_test() {
    hw::DeviceFixture f;
    REQUIRE(f.walk_group->open() == robot::device::DeviceError::OK);
    WalkImuStopGuard guard(*f.walk_group);

    REQUIRE(f.walk_group->set_feedback_mode_all(10u) == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);

    robot::service::HeadingCorrector uds_probe(make_heading_probe_params());
    uds_probe.enable(true);

    const float deadband = std::max(0.05f, kp.pid.deadband_yaw_deg);
    const float max_lower_rpm = std::clamp(std::abs(kp.limit_test_rpm) * 0.3f, 1.0f, 5.0f);
    constexpr float kMinLowerRpm = 0.8f;
    constexpr float kLowerKpRpmPerDeg = 3.0f;
    constexpr int kStableSamplesRequired = 2;
    constexpr auto kMaxRun = 30000ms;

    auto sample = wait_uds_yaw_sample(uds_probe, *f.walk_group, 5000ms);
    REQUIRE(sample.valid);
    int stable_samples = std::abs(sample.raw_yaw_deg) <= deadband ? 1 : 0;

    spdlog::warn(
        "[hw_system][lower_uds_zero] 上轮 LT/RT=0，仅调下轮 LB/RB，目标 |UDS raw_yaw| <= {:.2f}°",
        deadband);
    spdlog::warn("[hw_system][lower_uds_zero] 控制规则: raw_yaw>0 下轮向 A，raw_yaw<0 下轮向 B");
    spdlog::info(
        "[hw_system][lower_uds_zero] initial raw_yaw={:.3f} filtered_yaw={:.3f} conf={:.2f} "
        "age_ms={} lower_rpm={:.1f}",
        sample.raw_yaw_deg,
        sample.filtered_yaw_deg,
        sample.confidence,
        sample.age_ms,
        max_lower_rpm);

    const auto deadline = std::chrono::steady_clock::now() + kMaxRun;
    auto last_log_at = std::chrono::steady_clock::time_point{};
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline && stable_samples < kStableSamplesRequired) {
        sample = read_uds_yaw_sample(uds_probe);
        if (!sample.valid) {
            REQUIRE(f.walk_group->set_speeds(0.0f, 0.0f, 0.0f, 0.0f) ==
                    robot::device::DeviceError::OK);
            f.walk_group->update();
            stable_samples = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
            continue;
        }

        const float cmd = compute_lower_uds_zero_cmd(
            sample.raw_yaw_deg, deadband, kMinLowerRpm, max_lower_rpm, kLowerKpRpmPerDeg);
        REQUIRE(f.walk_group->set_speeds(0.0f, 0.0f, cmd, cmd) == robot::device::DeviceError::OK);
        f.walk_group->update();

        const auto now = std::chrono::steady_clock::now();
        const float abs_yaw = std::abs(sample.raw_yaw_deg);
        if (abs_yaw <= deadband) {
            ++stable_samples;
        } else {
            stable_samples = 0;
        }

        if (last_log_at == std::chrono::steady_clock::time_point{} || now - last_log_at >= 500ms) {
            last_log_at = now;
            const auto diag = f.walk_group->get_group_diagnostics();
            spdlog::info(
                "[hw_system][lower_uds_zero] raw_yaw={:.3f} filtered_yaw={:.3f} abs_raw={:.3f} "
                "stable={}/{} lower_dir={} conf={:.2f} age_ms={} kp={:.2f} "
                "cmd=[{:.1f},{:.1f},{:.1f},{:.1f}] rpm=[{:.1f},{:.1f},{:.1f},{:.1f}]",
                sample.raw_yaw_deg,
                sample.filtered_yaw_deg,
                abs_yaw,
                stable_samples,
                kStableSamplesRequired,
                cmd < 0.0f ? "A" : (cmd > 0.0f ? "B" : "STOP"),
                sample.confidence,
                sample.age_ms,
                kLowerKpRpmPerDeg,
                diag.wheel[0].target_value,
                diag.wheel[1].target_value,
                diag.wheel[2].target_value,
                diag.wheel[3].target_value,
                diag.wheel[0].speed_rpm,
                diag.wheel[1].speed_rpm,
                diag.wheel[2].speed_rpm,
                diag.wheel[3].speed_rpm);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    f.walk_group->set_speeds(0.0f, 0.0f, 0.0f, 0.0f);
    f.walk_group->update();

    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    INFO("final_raw_yaw=" << sample.raw_yaw_deg);
    INFO("final_filtered_yaw=" << sample.filtered_yaw_deg);
    INFO("deadband=" << deadband);
    INFO("stable_samples=" << stable_samples);
    CHECK(sample.valid);
    CHECK(std::abs(sample.raw_yaw_deg) <= deadband);
}

void require_health_log_written(const std::filesystem::path& path) {
    REQUIRE(std::filesystem::exists(path));
    REQUIRE(std::filesystem::file_size(path) > 0u);
    std::ifstream in(path);
    std::string line;
    REQUIRE(static_cast<bool>(std::getline(in, line)));
    CHECK(line.find('{') != std::string::npos);
}

void require_diagnostics_health_log(const std::filesystem::path& path) {
    const auto paths = collect_rotated_health_logs(path);
    REQUIRE_FALSE(paths.empty());
    int line_count = 0;
    for (const auto& log_path : paths) {
        std::ifstream in(log_path);
        REQUIRE(in.is_open());

        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) {
                continue;
            }
            ++line_count;
            auto json = parse_json_line(line);
            REQUIRE(json.IsObject());
            REQUIRE(json.HasMember("ts"));
            REQUIRE(json["ts"].IsUint64());
            REQUIRE(json.HasMember("values"));
            REQUIRE(json["values"].IsObject());

            const auto& values = json["values"];
            REQUIRE(values.HasMember("lt_rpm"));
            REQUIRE(values.HasMember("rt_rpm"));
            REQUIRE(values.HasMember("lb_rpm"));
            REQUIRE(values.HasMember("rb_rpm"));
            REQUIRE(values.HasMember("br_rpm"));
            REQUIRE(values.HasMember("bat_soc"));
            REQUIRE(values.HasMember("imu_y"));
            REQUIRE(values.HasMember("gps_fix"));
        }
    }
    CHECK(line_count > 0);
}

void run_configured_system_chain(SystemHwFixture& f,
                                 const char* tag,
                                 uint32_t repeat_count,
                                 bool expect_real_brush,
                                 bool /*log_fused_odometry*/,
                                 bool log_heading_pid_debug,
                                 std::optional<robot::service::MotionService::Config>
                                     motion_config_override = {},
                                 CombinedAttitudeRecoveryContext* attitude_recovery = nullptr) {
    const std::filesystem::path health_path = kp.health_jsonl_path;
    if (health_path.has_parent_path()) {
        std::filesystem::create_directories(health_path.parent_path());
    }
    remove_rotated_health_logs(health_path);

    std::ofstream pid_metrics;
    if (log_heading_pid_debug) {
        REQUIRE(open_jsonl_for_metrics(kp.pid_jsonl_path, pid_metrics));
    }

    f.repeat_count = std::max<uint32_t>(1u, repeat_count);
    REQUIRE(f.init(
        expect_real_brush, log_heading_pid_debug, health_path.string(), motion_config_override));
    REQUIRE(f.start_bms_polling());
    REQUIRE(f.start_safety_bridge());
    if (attitude_recovery != nullptr) {
        REQUIRE(attitude_recovery->open());
        spdlog::warn(
            "[{}] 姿态极限恢复测试已启用：任务运行中任意下姿态接近触发后，将暂停当前段、"
            "执行 lower_attitude_center 回中，然后恢复当前段",
            tag);
    }
    REQUIRE(f.health != nullptr);
    REQUIRE(f.watchdog != nullptr);

    std::atomic<bool> attitude_recovery_running{false};
    std::atomic<int> attitude_recovery_endpoint_interrupt{0};
    std::atomic<int> settled_count{0};
    std::atomic<int> target_settled_count{0};
    std::atomic<int> source_repeat_settled_count{0};
    f.bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const robot::middleware::SafetyMonitor::LimitSettledEvent& evt) {
            ++settled_count;
            const int target_code = f.active_segment_target_code.load(std::memory_order_acquire);
            if (target_code == endpoint_to_code(evt.endpoint)) {
                ++target_settled_count;
            } else {
                ++source_repeat_settled_count;
            }
            if (attitude_recovery_running.load(std::memory_order_acquire)) {
                attitude_recovery_endpoint_interrupt.store(
                    evt.endpoint == robot::domain::Endpoint::A ? 1 : 2,
                    std::memory_order_release);
            }
            spdlog::info("[{}] limit settled endpoint={}", tag, endpoint_to_config(evt.endpoint));
        });
    std::atomic<bool> watchdog_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& name) {
        spdlog::error("[{}] watchdog timeout: {}", tag, name);
        watchdog_timeout.store(true);
    });
    REQUIRE(f.watchdog->start());
    const int watchdog_ticket = f.watchdog->register_thread(tag, kp.limit_timeout_sec * 2 * 1000);
    REQUIRE(watchdog_ticket >= 0);

    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    int health_records = 0;
    int max_abs_brush_rpm = 0;
    int brush_fault_samples = 0;
    float max_yaw_drift = 0.0f;
    bool saw_pid_initialized = false;
    bool saw_pid_speed_command = false;
    bool saw_pid_nonzero_correction = false;
    const float target_yaw = f.imu->get_latest().yaw_deg;

    spdlog::warn("[{}] 完整配置任务启动：repeat_count={}，期望端点触发数>={}",
                 tag,
                 f.repeat_count,
                 f.repeat_count * 2u);
    REQUIRE(f.start_configured_assuming_primary_dock().accepted);
    REQUIRE(f.controller->snapshot().state == "ExecutingMission");

    const auto test_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(kp.limit_timeout_sec * 2 * static_cast<int>(f.repeat_count));
    auto segment_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(kp.limit_timeout_sec);
    auto segment_started_at = std::chrono::steady_clock::now();
    auto last_log_at = std::chrono::steady_clock::time_point{};
    int current_segment = 1;
    int last_target_settled_count = 0;
    int last_source_repeat_settled_count = 0;

    auto norm_angle = [](float deg) {
        while (deg > 180.0f)
            deg -= 360.0f;
        while (deg < -180.0f)
            deg += 360.0f;
        return deg;
    };

    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < test_deadline &&
           f.controller->snapshot().state != "Idle") {
        const auto controller_snapshot = f.controller->snapshot();
        if (controller_snapshot.state == "FaultStopped") {
            INFO("controller entered FaultStopped");
            INFO("fault=" << fault_to_string(controller_snapshot.fault));
            INFO("settled_count=" << settled_count.load());
            REQUIRE(controller_snapshot.state != "FaultStopped");
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= segment_deadline) {
            INFO("限位等待超时，请检查导轨/传感器接线");
            INFO("current_segment=" << current_segment);
            INFO("settled_count=" << settled_count.load());
            INFO("fault=" << fault_to_string(controller_snapshot.fault));
            REQUIRE(now < segment_deadline);
        }

        if (attitude_recovery != nullptr) {
            const auto left_attitude = attitude_recovery->left_sw.read_status();
            const auto right_attitude = attitude_recovery->right_sw.read_status();
            const auto attitude_state =
                classify_lower_attitude_state(left_attitude, right_attitude);
            REQUIRE(attitude_state != LowerAttitudeState::BOTH);
            if (const auto active_side = active_side_from_state(attitude_state)) {
                ++attitude_recovery->recovery_count;
                const auto resume_target = f.active_segment_target;
                REQUIRE(resume_target.has_value());
                log_attitude_status("[combined_attitude_recover][trigger]",
                                    left_attitude,
                                    right_attitude);
                spdlog::warn(
                    "[{}] attitude recovery #{} triggered by {}; pause target={} and center lower "
                    "wheels",
                    tag,
                    attitude_recovery->recovery_count,
                    attitude_side_name(*active_side),
                    endpoint_to_config(*resume_target));

                f.motion->stop_cleaning();
                attitude_recovery_endpoint_interrupt.store(0, std::memory_order_release);
                attitude_recovery_running.store(true, std::memory_order_release);
                const auto outcome = run_lower_attitude_center_recovery(
                    *f.walk_group,
                    attitude_recovery->left_sw,
                    attitude_recovery->right_sw,
                    tag,
                    [&]() -> std::optional<robot::domain::Endpoint> {
                        const int value =
                            attitude_recovery_endpoint_interrupt.load(std::memory_order_acquire);
                        if (value == 1) {
                            return robot::domain::Endpoint::A;
                        }
                        if (value == 2) {
                            return robot::domain::Endpoint::B;
                        }
                        return std::nullopt;
                    });
                attitude_recovery_running.store(false, std::memory_order_release);
                if (outcome == LowerAttitudeCenterOutcome::Completed) {
                    REQUIRE(f.motion->start_segment(robot::domain::MissionSegment{
                        *resume_target, robot::domain::SegmentMode::Cleaning}));
                } else {
                    const auto interrupted_endpoint =
                        outcome == LowerAttitudeCenterOutcome::InterruptedByEndpointA
                            ? robot::domain::Endpoint::A
                            : robot::domain::Endpoint::B;
                    spdlog::warn(
                        "[{}] attitude recovery #{} interrupted by endpoint={}; controller will "
                        "resume by current mission target={}",
                        tag,
                        attitude_recovery->recovery_count,
                        endpoint_to_config(interrupted_endpoint),
                        endpoint_to_config(*resume_target));
                }
                segment_started_at = std::chrono::steady_clock::now();
                segment_deadline = segment_started_at + std::chrono::seconds(kp.limit_timeout_sec);
                last_log_at = {};
                spdlog::warn("[{}] attitude recovery #{} done; target_before_recovery={}",
                             tag,
                             attitude_recovery->recovery_count,
                             endpoint_to_config(*resume_target));
                std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
                continue;
            }
        }

        f.motion->update();
        f.gps_stuck->update();
        // BMS 串口离线时一次 update 可能阻塞数百毫秒，组合运动链路测试中不轮询 BMS，
        // HealthService 直接读取最近缓存，避免打断行走电机控制心跳。
        if (f.brush) {
            f.brush->update();
        }
        if (f.health) {
            f.health->update();
            ++health_records;
        }
        f.watchdog->heartbeat(watchdog_ticket);

        const auto walk_diag = f.walk_group->get_group_diagnostics();
        const auto imu = f.imu->get_latest();
        const auto brush_diag = f.brush->get_diagnostics();
        max_abs_brush_rpm = std::max(max_abs_brush_rpm, std::abs(brush_diag.actual_rpm));
        if (brush_diag.fault) {
            ++brush_fault_samples;
        }

        const float yaw_drift = std::abs(norm_angle(target_yaw - imu.yaw_deg));
        max_yaw_drift = std::max(max_yaw_drift, yaw_drift);

        const int current_target_settled_count = target_settled_count.load();
        if (current_target_settled_count != last_target_settled_count) {
            const double segment_duration_s =
                std::chrono::duration<double>(now - segment_started_at).count();
            if (pid_metrics.is_open()) {
                pid_metrics << build_segment_summary_json(current_segment,
                                                          current_target_settled_count,
                                                          f.controller->snapshot().state,
                                                          segment_duration_s)
                            << '\n';
            }
            spdlog::info("[{}] 段 {} 完成：target_settled_count={} total_settled_count={} "
                         "source_repeat_count={} duration={:.1f}s",
                         tag,
                         current_segment,
                         current_target_settled_count,
                         settled_count.load(),
                         source_repeat_settled_count.load(),
                         segment_duration_s);
            last_target_settled_count = current_target_settled_count;
            ++current_segment;
            segment_started_at = now;
            segment_deadline = now + std::chrono::seconds(kp.limit_timeout_sec);
        }
        const int current_source_repeat_count = source_repeat_settled_count.load();
        if (current_source_repeat_count != last_source_repeat_settled_count) {
            spdlog::warn("[{}] 源端限位重复触发：source_repeat_count={} total_settled_count={}，"
                         "当前段继续执行",
                         tag,
                         current_source_repeat_count,
                         settled_count.load());
            last_source_repeat_settled_count = current_source_repeat_count;
            segment_deadline = now + std::chrono::seconds(kp.limit_timeout_sec);
        }

        const bool first_log = last_log_at == std::chrono::steady_clock::time_point{};
        const bool should_log = log_heading_pid_debug || first_log || now - last_log_at >= 500ms;
        if (should_log) {
            last_log_at = now;
            const auto snapshot = f.controller->snapshot();
            const auto pid = f.motion->heading_pid_debug_state();
            if (pid.mode != robot::service::HeadingCorrector::Mode::UNINITIALIZED) {
                saw_pid_initialized = true;
            }
            if (pid.mode == robot::service::HeadingCorrector::Mode::TRACK) {
                saw_pid_speed_command = true;
            }
            if (std::abs(pid.last_correction) > 1e-3f) {
                saw_pid_nonzero_correction = true;
            }

            if (pid_metrics.is_open()) {
                const int64_t ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count();
                pid_metrics
                    << build_pid_sample_json(
                           ts_ms, current_segment, snapshot.state, imu.yaw_deg, walk_diag, pid)
                    << '\n';
            }

            spdlog::info(
                "[{}] state={} target={} fault={} limits={} "
                "LT={:.1f}/{:.1f} RT={:.1f}/{:.1f} LB={:.1f}/{:.1f} RB={:.1f}/{:.1f} "
                "brush={} fault={} yaw={:.2f} drift={:.2f} pid(mode={} connected={} valid={} "
                "latest_yaw={:.3f} filtered_yaw={:.3f} conf={:.2f} age_ms={} corr={:.3f})",
                tag,
                snapshot.state,
                f.active_segment_target ? endpoint_to_config(*f.active_segment_target) : "none",
                fault_to_string(snapshot.fault),
                settled_count.load(),
                walk_diag.wheel[0].speed_rpm,
                walk_diag.wheel[0].target_value,
                walk_diag.wheel[1].speed_rpm,
                walk_diag.wheel[1].target_value,
                walk_diag.wheel[2].speed_rpm,
                walk_diag.wheel[2].target_value,
                walk_diag.wheel[3].speed_rpm,
                walk_diag.wheel[3].target_value,
                brush_diag.actual_rpm,
                brush_diag.fault,
                imu.yaw_deg,
                yaw_drift,
                static_cast<int>(pid.mode),
                pid.connected,
                pid.latest_valid,
                pid.latest_yaw_deg,
                pid.filtered_yaw_deg,
                pid.latest_confidence,
                pid.result_age_ms,
                pid.last_correction);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
    }

    f.watchdog->stop();
    if (pid_metrics.is_open()) {
        pid_metrics << build_final_summary_json(
                           current_segment - 1, health_records, max_yaw_drift, kp.pid.max_output)
                    << '\n';
        pid_metrics.close();
    }

    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    const auto final_snapshot = f.controller->snapshot();
    if (final_snapshot.state == "FaultStopped") {
        INFO("fault=" << fault_to_string(final_snapshot.fault));
        INFO("settled_count=" << settled_count.load());
        INFO("target_settled_count=" << target_settled_count.load());
        INFO("source_repeat_settled_count=" << source_repeat_settled_count.load());
    }
    REQUIRE(final_snapshot.state == "Idle");
    CHECK(target_settled_count.load() >= static_cast<int>(f.repeat_count * 2u));
    CHECK_FALSE(watchdog_timeout.load());
    CHECK(health_records > 0);

    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    CHECK(frames_after > frames_before + 10u * f.repeat_count);

    if (expect_real_brush) {
        CHECK(max_abs_brush_rpm >= 50);
        CHECK(brush_fault_samples == 0);
    }
    if (log_heading_pid_debug) {
        REQUIRE(std::filesystem::exists(kp.pid_jsonl_path));
        CHECK(saw_pid_initialized);
        CHECK(saw_pid_speed_command);
        if (!saw_pid_nonzero_correction) {
            spdlog::warn("[{}] PID correction stayed zero; check UDS source or deadband", tag);
        }
        CHECK(max_yaw_drift < kp.pid_max_drift_deg);
        spdlog::info("[{}] PID metrics saved to {}", tag, kp.pid_jsonl_path);
    }

    require_diagnostics_health_log(health_path);
}

void run_configured_system_chain_with_attitude_recovery(SystemHwFixture& f,
                                                        const char* tag,
                                                        uint32_t repeat_count) {
    CombinedAttitudeRecoveryContext attitude_recovery;
    run_configured_system_chain(
        f, tag, repeat_count, false, false, false, std::nullopt, &attitude_recovery);
    CHECK(attitude_recovery.recovery_count > 0);
}

}  // namespace
