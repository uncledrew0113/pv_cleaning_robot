/**
 * @file system_hw_test.cc
 * @brief 真实硬件系统链路测试。
 *
 * 这些用例补回旧 system_hw_test.cc 的测试意图，但适配当前收敛后的
 * RobotController API：
 *   [hw_system][full_init]        - 组合根初始化后控制器处于 Idle
 *   [hw_system][health_real_data] - IMU/GPS/HealthService 真实数据落盘
 *   [hw_system][safety_idle]      - 空闲限位监控不误触发
 *   [hw_system][motion_then_stop] - 运动 1s 后 emergency_override 急停
 *   [hw_system][lower_pitch_peak] - 下轨道轮低速采样 IMU 姿态峰值
 *   [hw_system][lower_pitch_score]- 下轨道轮低速采样姿态评分
 *   [hw_system][lower_uds_zero]   - 上轮 0 速，仅调下轮使 UDS yaw 进入死区
 *   [hw_system][watchdog_timeout] - 看门狗超时回调
 *   [hw_system][watchdog_heartbeat] - 看门狗正常心跳不误报
 *   [hw_system][p0_fault_chain]   - P0 故障链路：急停 + FaultStopped + 云端复位
 *   [hw_system][n1_clean_cycle]   - 配置完整任务真实限位闭环
 *   [hw_system][combined]         - N 趟完整任务链 + 全程持续采集健康数据
 *   [hw_system][combined_nvm_real]- N 趟完整任务链 + 全程打印融合里程计日志
 *   [hw_system][combined_brush_real] - N 趟完整任务链 + 真实滚刷
 *   [hw_system][pid_combined]     - 视觉 PID + 真实滚刷完整任务链
 *   [hw_system][imu_gps_health_only] - 仅 IMU/GPS/HealthService 落盘
 *
 * 安全约束：
 *   - 速度来自 hw_test_config.json 的低速测试参数。
 *   - 每个用例析构时都会 stop/急停/disable/close。
 *   - 带真实滚刷的用例需要确认滚刷处于安全工位。
 */
#include <algorithm>
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

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

const char* fault_code_name(uint32_t code) {
    using namespace robot::domain::FaultCode;
    switch (code) {
        case kCanCommunicationLost:
            return "CanCommunicationLost";
        case kSegmentStartFailed:
            return "SegmentStartFailed";
        case kUnexpectedLimitSide:
            return "UnexpectedLimitSide";
        case kConflictingLimitSides:
            return "ConflictingLimitSides";
        case kLimitUnstableAfterEmergencyStop:
            return "LimitUnstableAfterEmergencyStop";
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
              const std::string& health_jsonl_path = {}) {
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
        nav = std::make_shared<robot::service::NavService>(walk_group, imu, gps);
        fault = std::make_shared<robot::service::FaultService>(bus);
        motion = std::make_shared<robot::service::MotionService>(
            walk_group, brush, imu, bus, make_motion_config(pid_enabled));
        motion->set_runtime_config_query([this]() { return config->active_runtime_config(); });
        motion->set_primary_dock_query(
            [this]() { return config->active_runtime_config().primary_dock; });

        robot::app::RobotController::ActionPorts actions;
        actions.start_segment = [this](const robot::domain::MissionSegment& segment) {
            active_segment_target = segment.target;
            spdlog::info("[hw_system] start_segment target={} mode=Cleaning",
                         endpoint_to_config(segment.target));
            return motion->start_segment(segment);
        };
        actions.stop_motion = [this]() { motion->stop_cleaning(); };
        actions.emergency_stop = [this]() { motion->emergency_stop(); };
        actions.clear_fault = [this]() { fault->clear_active_fault(); };
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
        safety->set_limit_unstable_callback([this](robot::domain::Endpoint endpoint) {
            controller->post_limit_unstable(endpoint);
        });
        return safety->start();
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

    robot::app::CommandResult start_configured_assuming_primary_dock() {
        // 保持旧 system_hw_test.cc 行为：启动配置任务时不读取当前真实限位，
        // 而是按测试配置假定机器人从 primary_dock 语义出发。
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
            if (nav) {
                nav->update();
            }
            if (health) {
                health->update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kp.loop_period_ms));
        }
        return controller->snapshot().state == state;
    }

    void shutdown() override {
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
            REQUIRE(json.HasMember("walk"));
            REQUIRE(json.HasMember("brush"));
            REQUIRE(json.HasMember("bms"));
            REQUIRE(json.HasMember("imu"));
            REQUIRE(json.HasMember("gps"));
        }
    }
    CHECK(line_count > 0);
}

void run_configured_system_chain(SystemHwFixture& f,
                                 const char* tag,
                                 uint32_t repeat_count,
                                 bool expect_real_brush,
                                 bool log_fused_odometry,
                                 bool log_heading_pid_debug) {
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
    REQUIRE(f.init(expect_real_brush, log_heading_pid_debug, health_path.string()));
    REQUIRE(f.start_safety_bridge());
    REQUIRE(f.health != nullptr);
    REQUIRE(f.watchdog != nullptr);

    std::atomic<int> settled_count{0};
    f.bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const robot::middleware::SafetyMonitor::LimitSettledEvent& evt) {
            ++settled_count;
            spdlog::info("[{}] limit settled endpoint={}", tag, endpoint_to_config(evt.endpoint));
        });
    std::atomic<int> unstable_count{0};
    f.bus.subscribe<robot::middleware::SafetyMonitor::LimitUnstableEvent>(
        [&](const robot::middleware::SafetyMonitor::LimitUnstableEvent& evt) {
            ++unstable_count;
            spdlog::warn("[{}] limit unstable endpoint={} (emergency stop already sent)",
                         tag,
                         endpoint_to_config(evt.endpoint));
        });

    std::atomic<bool> watchdog_timeout{false};
    f.watchdog->set_timeout_callback([&](const std::string& name) {
        spdlog::error("[{}] watchdog timeout: {}", tag, name);
        watchdog_timeout.store(true);
    });
    REQUIRE(f.watchdog->start());
    const int watchdog_ticket = f.watchdog->register_thread(tag, kp.limit_timeout_sec * 2 * 1000);
    REQUIRE(watchdog_ticket >= 0);

    const auto odom_before = f.nav->get_fused_odometry();
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
    int last_settled_count = 0;

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
            INFO("unstable_count=" << unstable_count.load());
            REQUIRE(controller_snapshot.state != "FaultStopped");
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= segment_deadline) {
            INFO("限位等待超时，请检查导轨/传感器接线");
            INFO("current_segment=" << current_segment);
            INFO("settled_count=" << settled_count.load());
            INFO("unstable_count=" << unstable_count.load());
            INFO("fault=" << fault_to_string(controller_snapshot.fault));
            REQUIRE(now < segment_deadline);
        }

        f.motion->update();
        f.nav->update();
        // BMS 串口离线时一次 update 可能阻塞数百毫秒，组合运动链路测试中不轮询 BMS，
        // HealthService 直接读取最近缓存，避免打断行走电机 50Hz 心跳。
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

        const int current_settled_count = settled_count.load();
        if (current_settled_count != last_settled_count) {
            const double segment_duration_s =
                std::chrono::duration<double>(now - segment_started_at).count();
            if (pid_metrics.is_open()) {
                pid_metrics << build_segment_summary_json(current_segment,
                                                          current_settled_count,
                                                          f.controller->snapshot().state,
                                                          segment_duration_s)
                            << '\n';
            }
            spdlog::info("[{}] 段 {} 完成：settled_count={} duration={:.1f}s",
                         tag,
                         current_segment,
                         current_settled_count,
                         segment_duration_s);
            last_settled_count = current_settled_count;
            ++current_segment;
            segment_started_at = now;
            segment_deadline = now + std::chrono::seconds(kp.limit_timeout_sec);
        }

        const bool first_log = last_log_at == std::chrono::steady_clock::time_point{};
        const bool should_log =
            log_fused_odometry || log_heading_pid_debug || first_log || now - last_log_at >= 500ms;
        if (should_log) {
            last_log_at = now;
            const auto snapshot = f.controller->snapshot();
            const auto odom = f.nav->get_fused_odometry();
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
                "[{}] state={} target={} fault={} limits={} unstable={} "
                "LT={:.1f}/{:.1f} RT={:.1f}/{:.1f} LB={:.1f}/{:.1f} RB={:.1f}/{:.1f} "
                "brush={} fault={} yaw={:.2f} drift={:.2f} odom(valid={} top={:.3f} "
                "bottom={:.3f} fused={:.3f} diff={:.3f}) pid(mode={} connected={} valid={} "
                "latest_yaw={:.3f} filtered_yaw={:.3f} conf={:.2f} age_ms={} corr={:.3f})",
                tag,
                snapshot.state,
                f.active_segment_target ? endpoint_to_config(*f.active_segment_target) : "none",
                fault_to_string(snapshot.fault),
                settled_count.load(),
                unstable_count.load(),
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
                odom.valid,
                odom.top_distance_m,
                odom.bottom_distance_m,
                odom.fused_distance_m,
                odom.distance_diff_m,
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
        INFO("unstable_count=" << unstable_count.load());
    }
    REQUIRE(final_snapshot.state == "Idle");
    CHECK(settled_count.load() >= static_cast<int>(f.repeat_count * 2u));
    CHECK(unstable_count.load() == 0);
    CHECK_FALSE(watchdog_timeout.load());
    CHECK(health_records > 0);

    const uint32_t frames_after = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    CHECK(frames_after > frames_before + 10u * f.repeat_count);

    if (log_fused_odometry) {
        const auto odom_after = f.nav->get_fused_odometry();
        CHECK(odom_after.valid);
        CHECK(std::isfinite(odom_after.top_distance_m));
        CHECK(std::isfinite(odom_after.bottom_distance_m));
        CHECK(std::isfinite(odom_after.fused_distance_m));
        CHECK(std::isfinite(odom_after.distance_diff_m));
        CHECK(std::abs(odom_after.fused_distance_m - odom_before.fused_distance_m) > 0.02);
    }

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

}  // namespace

TEST_CASE("系统组合根初始化后处于 Idle", "[hw_system][full_init]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    CHECK(f.controller->snapshot().state == "Idle");

    std::this_thread::sleep_for(500ms);
    const auto imu = f.imu->get_latest();
    const auto odom = f.nav->get_fused_odometry();
    spdlog::info("[hw_system][full_init] imu_valid={} yaw={:.2f} pitch={:.2f} roll={:.2f}",
                 imu.valid,
                 imu.yaw_deg,
                 imu.pitch_deg,
                 imu.roll_deg);
    spdlog::info("[hw_system][full_init] odom valid={} top={:.3f} bottom={:.3f} fused={:.3f}",
                 odom.valid,
                 odom.top_distance_m,
                 odom.bottom_distance_m,
                 odom.fused_distance_m);
}

TEST_CASE("融合里程计在静止和低速运动时输出有效", "[hw_system][nav_fused_odometry]") {
    SystemHwFixture f;
    REQUIRE(f.init());

    std::this_thread::sleep_for(1500ms);
    f.nav->update();
    const auto idle = f.nav->get_fused_odometry();
    spdlog::info(
        "[hw_system][nav_fused_odometry] idle valid={} top={:.3f} bottom={:.3f} fused={:.3f} "
        "diff={:.3f}",
        idle.valid,
        idle.top_distance_m,
        idle.bottom_distance_m,
        idle.fused_distance_m,
        idle.distance_diff_m);

    CHECK(idle.valid);
    CHECK(std::isfinite(idle.top_distance_m));
    CHECK(std::isfinite(idle.bottom_distance_m));
    CHECK(std::isfinite(idle.fused_distance_m));
    CHECK(std::isfinite(idle.distance_diff_m));

    const auto target = robot::domain::opposite_endpoint(kp.primary_dock);
    REQUIRE(f.motion->start_segment(
        robot::domain::MissionSegment{target, robot::domain::SegmentMode::Cleaning}));
    const auto moving_deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < moving_deadline) {
        f.motion->update();
        f.nav->update();
        std::this_thread::sleep_for(50ms);
    }
    f.motion->emergency_stop();
    std::this_thread::sleep_for(500ms);

    const auto moving = f.nav->get_fused_odometry();
    spdlog::info(
        "[hw_system][nav_fused_odometry] moving valid={} top={:.3f} bottom={:.3f} fused={:.3f} "
        "diff={:.3f}",
        moving.valid,
        moving.top_distance_m,
        moving.bottom_distance_m,
        moving.fused_distance_m,
        moving.distance_diff_m);

    CHECK(moving.valid);
    CHECK(std::isfinite(moving.top_distance_m));
    CHECK(std::isfinite(moving.bottom_distance_m));
    CHECK(std::isfinite(moving.fused_distance_m));
    CHECK(std::isfinite(moving.distance_diff_m));
    CHECK(std::abs(moving.fused_distance_m - idle.fused_distance_m) > 0.02);
}

TEST_CASE("HealthService DIAGNOSTICS 落盘真实传感器数据", "[hw_system][health_real_data]") {
    const std::filesystem::path path = kp.health_jsonl_path;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    remove_rotated_health_logs(path);

    SystemHwFixture f;
    REQUIRE(f.init(false, false, path.string()));
    REQUIRE(f.health != nullptr);
    for (int i = 0; i < 10; ++i) {
        f.bms->update();
        f.walk_group->update();
        std::this_thread::sleep_for(100ms);
    }
    for (int i = 0; i < 5; ++i) {
        f.health->update();
        std::this_thread::sleep_for(50ms);
    }

    require_diagnostics_health_log(path);
}

TEST_CASE("SafetyMonitor 空闲状态不误触发限位事件", "[hw_system][safety_idle]") {
    hw::DeviceFixture f;
    REQUIRE(f.left_sw->open(0, 2, 0, false));
    REQUIRE(f.right_sw->open(0, 2, 0, false));

    robot::middleware::EventBus bus;
    std::atomic<int> settled_count{0};
    std::atomic<int> unstable_count{0};
    bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const auto&) { ++settled_count; });
    bus.subscribe<robot::middleware::SafetyMonitor::LimitUnstableEvent>(
        [&](const auto&) { ++unstable_count; });

    robot::middleware::SafetyMonitor safety(
        [&]() { f.walk_group->emergency_override(0.0f); }, f.left_sw, f.right_sw, bus);
    REQUIRE(safety.start());
    std::this_thread::sleep_for(2s);
    safety.stop();

    CHECK(settled_count.load() == 0);
    CHECK(unstable_count.load() == 0);
}

TEST_CASE("运动 1s 后 emergency_override 急停", "[hw_system][motion_then_stop]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    REQUIRE(f.walk_group->enable_all() == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->set_mode_all(robot::protocol::WalkMotorMode::SPEED) ==
            robot::device::DeviceError::OK);
    std::this_thread::sleep_for(300ms);
    REQUIRE(f.walk_group->set_speed_uniform(kp.test_speed_rpm) == robot::device::DeviceError::OK);

    for (int i = 0; i < 4; ++i) {
        f.walk_group->update();
        std::this_thread::sleep_for(250ms);
    }

    REQUIRE(f.walk_group->emergency_override(0.0f) == robot::device::DeviceError::OK);
    REQUIRE(f.walk_group->is_override_active());
    const uint32_t frames_before = f.walk_group->get_group_diagnostics().ctrl_frame_count;
    f.walk_group->update();
    std::this_thread::sleep_for(200ms);
    CHECK(f.walk_group->get_group_diagnostics().ctrl_frame_count == frames_before);

    std::this_thread::sleep_for(500ms);
    const auto diag = f.walk_group->get_group_diagnostics();
    CHECK(std::abs(diag.wheel[0].speed_rpm) < 5.0f);
    CHECK(std::abs(diag.wheel[1].speed_rpm) < 5.0f);

    f.walk_group->clear_override();
    f.walk_group->update();
    CHECK_FALSE(f.walk_group->is_override_active());
}

TEST_CASE("下轨道轮低速采样 IMU pitch 峰值", "[hw_system][lower_pitch_peak]") {
    const auto sample = sample_lower_wheel_pitch();
    INFO("samples=" << sample.samples);
    INFO("pitch_min=" << sample.pitch_min);
    INFO("pitch_max=" << sample.pitch_max);
    CHECK(sample.samples > 5);
    CHECK(sample.pitch_max >= sample.pitch_min);
}

TEST_CASE("下轨道轮低速采样姿态评分", "[hw_system][lower_pitch_score]") {
    const auto sample = sample_lower_wheel_pitch();
    const float pitch_span = sample.pitch_max - sample.pitch_min;
    const float score = pitch_span - 0.25f * sample.roll_abs_max;
    INFO("pitch_span=" << pitch_span);
    INFO("roll_abs_max=" << sample.roll_abs_max);
    INFO("score=" << score);
    CHECK(sample.samples > 5);
    CHECK(std::isfinite(score));
}

TEST_CASE("lower_uds_zero 调速随误差缩小并保持方向", "[hw_system][lower_uds_zero][logic]") {
    constexpr float kDeadband = 0.2f;
    constexpr float kMinRpm = 0.8f;
    constexpr float kMaxRpm = 5.0f;
    constexpr float kKp = 1.5f;

    CHECK(compute_lower_uds_zero_cmd(0.1f, kDeadband, kMinRpm, kMaxRpm, kKp) == 0.0f);
    CHECK(compute_lower_uds_zero_cmd(0.4f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(-0.8f));
    CHECK(compute_lower_uds_zero_cmd(-0.4f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(0.8f));
    CHECK(std::abs(compute_lower_uds_zero_cmd(3.0f, kDeadband, kMinRpm, kMaxRpm, kKp)) >
          std::abs(compute_lower_uds_zero_cmd(0.4f, kDeadband, kMinRpm, kMaxRpm, kKp)));
    CHECK(compute_lower_uds_zero_cmd(10.0f, kDeadband, kMinRpm, kMaxRpm, kKp) == Approx(-5.0f));
}

TEST_CASE("上轮 0 速仅调下轮使 UDS yaw 进入死区", "[hw_system][lower_uds_zero]") {
    run_lower_uds_zero_test();
}

TEST_CASE("WatchdogMgr 超时后触发回调", "[hw_system][watchdog_timeout]") {
    robot::app::WatchdogMgr watchdog;
    std::atomic<bool> timed_out{false};
    watchdog.set_timeout_callback([&](const std::string&) { timed_out.store(true); });
    REQUIRE(watchdog.start());
    const int ticket = watchdog.register_thread("hw_system_watchdog", 100);
    REQUIRE(ticket >= 0);
    std::this_thread::sleep_for(300ms);
    watchdog.stop();
    CHECK(timed_out.load());
}

TEST_CASE("WatchdogMgr 正常心跳不触发超时", "[hw_system][watchdog_heartbeat]") {
    robot::app::WatchdogMgr watchdog;
    std::atomic<bool> timed_out{false};
    watchdog.set_timeout_callback([&](const std::string&) { timed_out.store(true); });
    REQUIRE(watchdog.start());
    const int ticket = watchdog.register_thread("hw_system_watchdog", 500);
    REQUIRE(ticket >= 0);

    for (int i = 0; i < 5; ++i) {
        watchdog.heartbeat(ticket);
        std::this_thread::sleep_for(200ms);
    }

    watchdog.stop();
    CHECK_FALSE(timed_out.load());
}

TEST_CASE("P0 故障链路急停并由云端复位回 Idle", "[hw_system][p0_fault_chain]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    f.position_state = robot::domain::PositionState::OnSegment;
    REQUIRE(f.start_directional_to_opposite().accepted);
    REQUIRE(f.controller->snapshot().state == "ExecutingMission");

    f.controller->post_fault(robot::app::FaultFact{robot::app::FaultSource::Watchdog,
                                                   robot::domain::FaultCode::kCanCommunicationLost,
                                                   "hw_p0"});
    f.controller->drain_for_test();

    CHECK(f.controller->snapshot().state == "FaultStopped");
    CHECK(f.controller->snapshot().fault == robot::domain::FaultCode::kCanCommunicationLost);
    const auto reset = f.controller->submit_command(
        robot::domain::RobotCommand{robot::domain::RobotCommandKind::FaultReset,
                                    robot::domain::CommandSource::Rpc,
                                    "hw-reset"});
    REQUIRE(reset.accepted);
    CHECK(f.controller->snapshot().state == "Idle");
    CHECK_FALSE(f.fault->has_active_fault());
}

TEST_CASE("配置完整任务通过真实限位闭环完成", "[hw_system][n1_clean_cycle]") {
    SystemHwFixture f;
    f.repeat_count = 1;
    REQUIRE(f.init());
    REQUIRE(f.start_safety_bridge());
    REQUIRE(f.start_configured_assuming_primary_dock().accepted);
    CHECK(f.controller->snapshot().state == "ExecutingMission");
    CHECK(f.wait_until_state("Idle", std::chrono::seconds(kp.limit_timeout_sec * 2)));
}

TEST_CASE("N 趟完整任务链 + 全程持续采集健康数据", "[hw_system][combined]") {
    SystemHwFixture f;
    run_configured_system_chain(f, "hw_system][combined", kp.combined_passes, false, false, false);
}

TEST_CASE("完整任务链 + 融合里程计日志", "[hw_system][combined_nvm_real]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][combined_nvm_real", kp.combined_passes, false, true, false);
}

TEST_CASE("N 趟完整任务链 + 真实滚刷 + 全程持续采集健康数据", "[hw_system][combined_brush_real]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][combined_brush_real", kp.combined_passes, true, false, false);
}

TEST_CASE("视觉 PID 完整任务链 + 真实滚刷", "[hw_system][pid_combined]") {
    SystemHwFixture f;
    run_configured_system_chain(
        f, "hw_system][pid_combined", kp.combined_passes, true, false, true);
}

TEST_CASE("仅 IMU/GPS/HealthService 持续采集并本地落盘", "[hw_system][imu_gps_health_only]") {
    const std::filesystem::path path = std::filesystem::path(kp.health_jsonl_path)
                                           .replace_filename("hw_imu_gps_health_only.jsonl");
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::filesystem::remove(path);

    hw::ImuGpsHealthFixture f;
    REQUIRE(f.init(path.string()));
    for (int i = 0; i < 20; ++i) {
        f.health->update();
        std::this_thread::sleep_for(100ms);
    }

    require_health_log_written(path);
}
