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
#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rapidjson/document.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <thread>

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

robot::domain::PositionState position_at(robot::domain::Endpoint endpoint) {
    return endpoint == robot::domain::Endpoint::A ? robot::domain::PositionState::AtA
                                                  : robot::domain::PositionState::AtB;
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

    bool init(bool use_real_brush = false,
              bool pid_enabled = false,
              const std::string& health_jsonl_path = {}) {
        real_brush = use_real_brush;
        write_config_files();

        config = std::make_unique<robot::service::ConfigService>(
            paths.runtime_path.string(), paths.fixed_path.string());
        if (!config->load()) {
            return false;
        }

        can_bus = std::make_shared<robot::driver::LinuxCanSocket>(kp.can_iface);
        walk_group = std::make_shared<robot::device::WalkMotorGroup>(
            can_bus,
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

        imu_serial =
            std::make_shared<robot::driver::LibSerialPort>(kp.imu_port,
                                                           robot::hal::UartConfig{kp.imu_baud});
        imu = std::make_shared<robot::device::ImuDevice>(imu_serial);
        if (!imu->open()) {
            return false;
        }

        bms_serial =
            std::make_shared<robot::driver::LibSerialPort>(kp.bms_port,
                                                           robot::hal::UartConfig{kp.bms_baud});
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
        left_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            kp.gpio_chip, kp.left_limit_line);
        right_gpio = std::make_shared<robot::driver::LibGpiodPin>(
            kp.gpio_chip, kp.right_limit_line);
        left_sw =
            std::make_shared<robot::device::LimitSwitch>(left_gpio, robot::device::LimitSide::LEFT);
        right_sw = std::make_shared<robot::device::LimitSwitch>(
            right_gpio, robot::device::LimitSide::RIGHT);
        if (!left_sw->open() || !right_sw->open()) {
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
        safety->set_limit_settled_callback([this](robot::domain::Endpoint endpoint) {
            controller->post_limit_settled(endpoint);
        });
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
        auto result = controller->submit_command(robot::domain::RobotCommand{
            robot::domain::RobotCommandKind::StartConfiguredMission,
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
            << "    \"brush_rpm\": "
            << static_cast<int>(std::lround(std::abs(kp.brush_test_rpm))) << ",\n"
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
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kp.sweep_duration_ms);
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

void require_health_log_written(const std::filesystem::path& path) {
    REQUIRE(std::filesystem::exists(path));
    REQUIRE(std::filesystem::file_size(path) > 0u);
    std::ifstream in(path);
    std::string line;
    REQUIRE(static_cast<bool>(std::getline(in, line)));
    CHECK(line.find('{') != std::string::npos);
}

void require_diagnostics_health_log(const std::filesystem::path& path) {
    REQUIRE(std::filesystem::exists(path));
    std::ifstream in(path);
    REQUIRE(in.is_open());

    int line_count = 0;
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
    std::filesystem::remove(health_path);

    f.repeat_count = std::max<uint32_t>(1u, repeat_count);
    REQUIRE(f.init(expect_real_brush, log_heading_pid_debug, health_path.string()));
    REQUIRE(f.start_safety_bridge());
    REQUIRE(f.health != nullptr);
    REQUIRE(f.watchdog != nullptr);

    std::atomic<int> settled_count{0};
    f.bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&](const robot::middleware::SafetyMonitor::LimitSettledEvent& evt) {
            ++settled_count;
            spdlog::info("[{}] limit settled endpoint={}",
                         tag,
                         evt.endpoint == robot::domain::Endpoint::A ? "A" : "B");
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

    spdlog::warn("[{}] 完整配置任务启动：repeat_count={}，期望端点触发数>={}",
                 tag,
                 f.repeat_count,
                 f.repeat_count * 2u);
    REQUIRE(f.start_configured_assuming_primary_dock().accepted);
    REQUIRE(f.controller->snapshot().state == "ExecutingMission");

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(kp.limit_timeout_sec * 2 *
                                                                static_cast<int>(f.repeat_count));
    while (!hw::HwExitGuard::instance().exit_requested() &&
           std::chrono::steady_clock::now() < deadline &&
           f.controller->snapshot().state != "Idle") {
        f.motion->update();
        f.nav->update();
        if (f.bms) {
            f.bms->update();
        }
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

        if (log_fused_odometry || log_heading_pid_debug) {
            const auto odom = f.nav->get_fused_odometry();
            const auto pid = f.motion->heading_pid_debug_state();
            spdlog::info(
                "[{}] state={} limits={} LT={:.1f} RT={:.1f} LB={:.1f} RB={:.1f} "
                "brush={} fault={} yaw={:.2f} odom(valid={} top={:.3f} bottom={:.3f} "
                "fused={:.3f} diff={:.3f}) pid(mode={} connected={} valid={} corr={:.3f})",
                tag,
                f.controller->snapshot().state,
                settled_count.load(),
                walk_diag.wheel[0].speed_rpm,
                walk_diag.wheel[1].speed_rpm,
                walk_diag.wheel[2].speed_rpm,
                walk_diag.wheel[3].speed_rpm,
                brush_diag.actual_rpm,
                brush_diag.fault,
                imu.yaw_deg,
                odom.valid,
                odom.top_distance_m,
                odom.bottom_distance_m,
                odom.fused_distance_m,
                odom.distance_diff_m,
                static_cast<int>(pid.mode),
                pid.connected,
                pid.latest_valid,
                pid.last_correction);
        }

        std::this_thread::sleep_for(500ms);
    }

    f.watchdog->stop();

    REQUIRE_FALSE(hw::HwExitGuard::instance().exit_requested());
    REQUIRE(f.controller->snapshot().state == "Idle");
    CHECK(settled_count.load() >= static_cast<int>(f.repeat_count * 2u));
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

    require_diagnostics_health_log(health_path);
}

}  // namespace

TEST_CASE("系统组合根初始化后处于 Idle", "[hw_system][full_init]") {
    SystemHwFixture f;
    REQUIRE(f.init());
    CHECK(f.controller->snapshot().state == "Idle");
}

TEST_CASE("HealthService DIAGNOSTICS 落盘真实传感器数据", "[hw_system][health_real_data]") {
    const std::filesystem::path path = kp.health_jsonl_path;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::filesystem::remove(path);

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
    REQUIRE(f.left_sw->open());
    REQUIRE(f.right_sw->open());

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
    std::this_thread::sleep_for(1s);
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
    const auto reset = f.controller->submit_command(robot::domain::RobotCommand{
        robot::domain::RobotCommandKind::FaultReset,
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
    run_configured_system_chain(f,
                                "hw_system][combined",
                                kp.combined_passes,
                                false,
                                false,
                                false);
}

TEST_CASE("完整任务链 + 融合里程计日志", "[hw_system][combined_nvm_real]") {
    SystemHwFixture f;
    run_configured_system_chain(f,
                                "hw_system][combined_nvm_real",
                                kp.combined_passes,
                                false,
                                true,
                                false);
}

TEST_CASE("N 趟完整任务链 + 真实滚刷 + 全程持续采集健康数据",
          "[hw_system][combined_brush_real]") {
    SystemHwFixture f;
    run_configured_system_chain(f,
                                "hw_system][combined_brush_real",
                                kp.combined_passes,
                                true,
                                false,
                                false);
}

TEST_CASE("视觉 PID 完整任务链 + 真实滚刷", "[hw_system][pid_combined]") {
    SystemHwFixture f;
    run_configured_system_chain(f,
                                "hw_system][pid_combined",
                                kp.combined_passes,
                                true,
                                false,
                                true);
}

TEST_CASE("仅 IMU/GPS/HealthService 持续采集并本地落盘",
          "[hw_system][imu_gps_health_only]") {
    const std::filesystem::path path =
        std::filesystem::path(kp.health_jsonl_path).replace_filename("hw_imu_gps_health_only.jsonl");
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
