// test/integration/hardware/hw_config.h
#pragma once
/**
 * @file hw_config.h
 * @brief 硬件测试公共 Fixture
 *
 * DeviceFixture       — Driver + Device 层，用于限位和电机单元测试
 * FullSystemFixture   — 全层栈，用于集成测试（BrushMotor 用 MockSerialPort）
 *
 * 硬件接线（默认值与仓库 split config 对齐，可通过 hw_test_config.json 覆盖）：
 *   CAN      : can0（默认），行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1（默认），WIT Motion，9600 baud
 *   GPSD     : 127.0.0.1:2947（默认），由目标机 gpsd 服务提供 TCP JSON 数据
 *   BMS      : /dev/ttyS8（默认），嘉佰达通用协议 V4，9600 baud
 *   距离传感器: /dev/ttyS9（默认），RS485 Modbus RTU，9600 baud
 *   GPIO     : gpiochip5 line0=左限位，line1=右限位（默认）
 */
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

// Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"

// Device
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Middleware
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

// Service
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

// App
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/parking_side_runtime.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// Mock（滚刷电机未安装）
#include "integration/thingsboard_test_support.h"
#include "mock/mock_can_bus.h"
#include "mock/mock_serial_port.h"
#include "pv_cleaning_robot/service/config_service.h"

namespace hw {

// ── 运行时硬件测试参数（从 hw_test_config.json 加载，缺失则用内嵌默认值）──────
/// 所有参数均可通过 hw_test_config.json 覆盖，无需重新编译
struct HwParams {
    // hardware mapping
    std::string can_iface = "can0";
    uint8_t motor_id_base = 1u;
    uint16_t comm_timeout_ms = 500u;  ///< update 50ms × 10 倍余量
    bool termination_init_enabled = true;
    uint8_t termination_init_retry_count = 3u;
    uint8_t termination_motor_id = 2u;
    std::string imu_port = "/dev/ttyS1";
    int imu_baud = 9600;
    std::string brush_port = "/dev/ttyACM0";
    int brush_baud = 115200;
    uint8_t brush_axis = 0u;
    float brush_counts_per_rev = 8192.0f;
    bool brush_watchdog_enabled = false;
    float brush_watchdog_timeout_s = 0.5f;
    std::string gpsd_host = "127.0.0.1";
    int gpsd_port = 2947;
    std::string gpsd_watch = "?WATCH={\"enable\":true,\"json\":true};";
    std::string bms_port = "/dev/ttyS8";
    int bms_baud = 9600;
    std::string dist_port = "/dev/ttyS9";  ///< 距离传感器 RS485 串口
    int dist_baud = 9600;
    uint8_t dist_slave_id = 1u;
    uint8_t dist_channel_count = 2u;
    std::string gpio_chip = "gpiochip5";
    unsigned left_limit_line = 0u;
    unsigned right_limit_line = 1u;
    // timing
    int limit_timeout_sec = 60;   ///< 每段（单一限位）等待最大秒数
    int online_timeout_ms = 600;  ///< 等待电机上线最大毫秒数
    int sweep_duration_ms = 5000;
    int loop_period_ms = 50;
    int gpsd_message_timeout_sec = 5;  ///< 期望收到首条 gpsd JSON 报文的最大等待时间
    int gpsd_fix_timeout_sec = 30;     ///< 期望拿到有效 GPS fix 的最大等待时间
    // behavior
    float test_speed_rpm = 10.0f;  ///< 安全低速（测试专用）
    float test_return_rpm = 10.0f;
    float sweep_rpm = 20.0f;
    float limit_test_rpm = 10.0f;
    float brush_test_rpm = 3000.0f;  ///< ODrive 滚刷硬件测试目标转速
    float combined_passes = 50.0f;   ///< combined 测试趟数（1=一来回，2=两来回…）
    std::string health_jsonl_path = "/tmp/hw_system_test_health.jsonl";
    size_t health_log_max_bytes = 10u * 1024u * 1024u;  ///< HealthService 本地轮转单文件上限
    size_t health_log_max_files = 3u;  ///< HealthService 本地轮转保留文件数
    std::string pid_jsonl_path = "/tmp/hw_pid_test_metrics.jsonl";  ///< PID 指标 JSONL 路径
    float pid_max_drift_deg = 15.0f;  ///< pid_combined: 全程最大 yaw 漂移警告阈值（°）

    /// PID 参数，与 HeadingPidController::Params 字段一一对应
    struct PidParams {
        float kp{0.5f};
        float ki{0.05f};
        float kd{0.1f};
        float max_output{30.0f};
        float integral_limit{20.0f};
        float deadband_rate_dps{2.0f};
    } pid;
};

/// 按优先级查找配置：1. 环境变量 HW_TEST_CONFIG  2. CWD/hw_test_config.json  3. 内嵌默认值
inline HwParams load_hw_test_config() {
    HwParams p;
    std::string path;
    const char* env = std::getenv("HW_TEST_CONFIG");
    if (env && std::filesystem::exists(env))
        path = env;
    else if (std::filesystem::exists("hw_test_config.json"))
        path = "hw_test_config.json";

    if (path.empty()) {
        spdlog::warn("[hw_config] hw_test_config.json not found — using built-in defaults");
        return p;
    }
    try {
        robot::service::ConfigService cfg(path);
        if (!cfg.load()) {
            spdlog::warn("[hw_config] Failed to load {} — using built-in defaults", path);
            return p;
        }
        p.can_iface = cfg.get<std::string>("hardware.can_iface", p.can_iface);
        p.motor_id_base =
            static_cast<uint8_t>(cfg.get<int>("hardware.motor_id_base", (int)p.motor_id_base));
        p.comm_timeout_ms =
            static_cast<uint16_t>(cfg.get<int>("timing.comm_timeout_ms", (int)p.comm_timeout_ms));
        p.termination_init_enabled =
            cfg.get<bool>("hardware.termination_init_enabled", p.termination_init_enabled);
        p.termination_init_retry_count =
            static_cast<uint8_t>(cfg.get<int>("hardware.termination_init_retry_count",
                                              static_cast<int>(p.termination_init_retry_count)));
        p.termination_motor_id = static_cast<uint8_t>(cfg.get<int>(
            "hardware.termination_motor_id", static_cast<int>(p.termination_motor_id)));
        p.imu_port = cfg.get<std::string>("hardware.imu_port", p.imu_port);
        p.imu_baud = cfg.get<int>("hardware.imu_baud", p.imu_baud);
        p.brush_port = cfg.get<std::string>("hardware.brush_port", p.brush_port);
        p.brush_baud = cfg.get<int>("hardware.brush_baud", p.brush_baud);
        p.brush_axis = static_cast<uint8_t>(
            cfg.get<int>("hardware.brush_axis", static_cast<int>(p.brush_axis)));
        p.brush_counts_per_rev =
            cfg.get<float>("hardware.brush_counts_per_rev", p.brush_counts_per_rev);
        p.brush_watchdog_enabled =
            cfg.get<bool>("hardware.brush_watchdog_enabled", p.brush_watchdog_enabled);
        p.brush_watchdog_timeout_s =
            cfg.get<float>("hardware.brush_watchdog_timeout_s", p.brush_watchdog_timeout_s);
        p.gpsd_host = cfg.get<std::string>("hardware.gpsd_host", p.gpsd_host);
        p.gpsd_port = cfg.get<int>("hardware.gpsd_port", p.gpsd_port);
        p.gpsd_watch = cfg.get<std::string>("hardware.gpsd_watch", p.gpsd_watch);
        p.bms_port = cfg.get<std::string>("hardware.bms_port", p.bms_port);
        p.bms_baud = cfg.get<int>("hardware.bms_baud", p.bms_baud);
        p.dist_port = cfg.get<std::string>("hardware.dist_port", p.dist_port);
        p.dist_baud = cfg.get<int>("hardware.dist_baud", p.dist_baud);
        p.dist_slave_id =
            static_cast<uint8_t>(cfg.get<int>("hardware.dist_slave_id", (int)p.dist_slave_id));
        p.dist_channel_count = static_cast<uint8_t>(
            cfg.get<int>("hardware.dist_channel_count", (int)p.dist_channel_count));
        p.gpio_chip = cfg.get<std::string>("hardware.gpio_chip", p.gpio_chip);
        p.left_limit_line =
            static_cast<unsigned>(cfg.get<int>("hardware.left_limit_line", (int)p.left_limit_line));
        p.right_limit_line = static_cast<unsigned>(
            cfg.get<int>("hardware.right_limit_line", (int)p.right_limit_line));
        p.limit_timeout_sec = cfg.get<int>("timing.limit_timeout_sec", p.limit_timeout_sec);
        p.online_timeout_ms = cfg.get<int>("timing.online_timeout_ms", p.online_timeout_ms);
        p.sweep_duration_ms = cfg.get<int>("timing.sweep_duration_ms", p.sweep_duration_ms);
        p.loop_period_ms = cfg.get<int>("timing.loop_period_ms", p.loop_period_ms);
        p.gpsd_message_timeout_sec =
            cfg.get<int>("timing.gpsd_message_timeout_sec", p.gpsd_message_timeout_sec);
        p.gpsd_fix_timeout_sec =
            cfg.get<int>("timing.gpsd_fix_timeout_sec", p.gpsd_fix_timeout_sec);
        p.test_speed_rpm = cfg.get<float>("behavior.test_speed_rpm", p.test_speed_rpm);
        p.test_return_rpm = cfg.get<float>("behavior.test_return_rpm", p.test_return_rpm);
        p.sweep_rpm = cfg.get<float>("behavior.sweep_rpm", p.sweep_rpm);
        p.limit_test_rpm = cfg.get<float>("behavior.limit_test_rpm", p.limit_test_rpm);
        p.brush_test_rpm = cfg.get<float>("behavior.brush_test_rpm", p.brush_test_rpm);
        p.combined_passes = cfg.get<float>("behavior.combined_passes", p.combined_passes);
        p.health_jsonl_path =
            cfg.get<std::string>("behavior.health_jsonl_path", p.health_jsonl_path);
        p.health_log_max_bytes = static_cast<size_t>(cfg.get<int>(
            "behavior.health_log_max_bytes", static_cast<int>(p.health_log_max_bytes)));
        p.health_log_max_files = static_cast<size_t>(cfg.get<int>(
            "behavior.health_log_max_files", static_cast<int>(p.health_log_max_files)));
        p.pid_jsonl_path = cfg.get<std::string>("behavior.pid_jsonl_path", p.pid_jsonl_path);
        p.pid_max_drift_deg = cfg.get<float>("behavior.pid_max_drift_deg", p.pid_max_drift_deg);
        p.pid.kp = cfg.get<float>("pid.kp", p.pid.kp);
        p.pid.ki = cfg.get<float>("pid.ki", p.pid.ki);
        p.pid.kd = cfg.get<float>("pid.kd", p.pid.kd);
        p.pid.max_output = cfg.get<float>("pid.max_output", p.pid.max_output);
        p.pid.integral_limit = cfg.get<float>("pid.integral_limit", p.pid.integral_limit);
        p.pid.deadband_rate_dps = cfg.get<float>("pid.deadband_rate_dps", p.pid.deadband_rate_dps);
        spdlog::debug("[hw_config] Loaded config: {}", path);
    } catch (const std::exception& e) {
        spdlog::warn("[hw_config] Exception loading config {}: {} — using built-in defaults",
                     path,
                     e.what());
    }
    return p;
}

inline robot::middleware::MqttTransport::Config build_tb_mqtt_config(
    robot::service::ConfigService& cfg,
    const std::filesystem::path& repo_root) {
    return tb_test_support::build_mqtt_config(cfg, repo_root, "_hw_tb_itest");
}

// ── DeviceFixture：Driver + Device 层（限位 / 电机单元测试使用）────────────
struct DeviceFixture {
    HwParams p;  ///< 运行时硬件参数，必须为第一成员（构造时初始化列表先于 ctor 体）
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::driver::LibSerialPort> bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio;
    std::shared_ptr<robot::device::LimitSwitch> left_sw;
    std::shared_ptr<robot::device::LimitSwitch> right_sw;

    DeviceFixture() : p(load_hw_test_config()) {
        using namespace robot;
        can_bus = std::make_shared<driver::LinuxCanSocket>(p.can_iface);
        walk_group = std::make_shared<device::WalkMotorGroup>(can_bus,
                                                              p.motor_id_base,
                                                              p.comm_timeout_ms,
                                                              p.termination_init_enabled,
                                                              p.termination_init_retry_count,
                                                              p.termination_motor_id);
        imu_serial =
            std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial =
            std::make_shared<driver::LibSerialPort>(p.bms_port, hal::UartConfig{p.bms_baud});
        bms = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        left_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.left_limit_line);
        right_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.right_limit_line);
        left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);
        right_sw = std::make_shared<device::LimitSwitch>(right_gpio, device::LimitSide::RIGHT);
    }

    ~DeviceFixture() {
        if (left_sw)
            left_sw->close();
        if (right_sw)
            right_sw->close();
        if (imu)
            imu->close();
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        // bms_serial 由 shared_ptr 析构时自动关闭
    }
};

// ── FullSystemFixture：全层栈（集成测试使用）─────────────────────────────────
struct FullSystemFixture : DeviceFixture {
    robot::middleware::EventBus event_bus;
    bool use_real_brush_{false};
    std::shared_ptr<robot::driver::LibSerialPort> real_brush_serial;
    std::shared_ptr<MockSerialPort> mock_brush_serial;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<robot::device::GpsDevice> gps;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::shared_ptr<robot::service::NavService> nav;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::service::FaultService> fault;
    std::shared_ptr<robot::service::HealthService> health;  ///< null cloud，本地 JSONL 落盘
    std::unique_ptr<robot::app::WatchdogMgr> watchdog;
    std::shared_ptr<robot::app::RobotFsm> fsm;
    std::shared_ptr<robot::app::FaultHandler> fault_handler;
    std::vector<robot::service::FaultService::FaultEvent> dispatched_faults;

    static robot::hal::UartConfig build_brush_uart_config_(const HwParams& p) {
        robot::hal::UartConfig cfg;
        cfg.baudrate = p.brush_baud;
        cfg.data_bits = 8;
        cfg.parity = 'N';
        cfg.stop_bits = 1;
        cfg.write_timeout_ms = 500;
        return cfg;
    }

    /// @param pid_enabled 是否开启航向 PID（clean_cycle 测试按场景传入）
    /// @param use_real_brush 是否接入真实 ODrive 滚刷链路
    explicit FullSystemFixture(bool pid_enabled = false, bool use_real_brush = false)
        : DeviceFixture()
        , use_real_brush_(use_real_brush) {
        using namespace robot;

        std::shared_ptr<hal::ISerialPort> brush_serial;
        if (use_real_brush_) {
            real_brush_serial = std::make_shared<driver::LibSerialPort>(
                p.brush_port, build_brush_uart_config_(p));
            brush_serial = real_brush_serial;
        } else {
            mock_brush_serial = std::make_shared<MockSerialPort>();
            brush_serial = mock_brush_serial;
        }
        brush = std::make_shared<device::BrushMotor>(brush_serial,
                                                     p.brush_axis,
                                                     p.brush_counts_per_rev,
                                                     p.brush_watchdog_enabled,
                                                     p.brush_watchdog_timeout_s);

        // SafetyMonitor 构造时内部绑定 LimitSwitch 回调
        safety =
            std::make_unique<middleware::SafetyMonitor>(walk_group, left_sw, right_sw, event_bus);

        robot::device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = p.gpsd_host;
        gpsd_cfg.port = p.gpsd_port;
        gpsd_cfg.watch = p.gpsd_watch;
        gps = device::GpsDevice::create_gpsd(gpsd_cfg);

        nav = std::make_shared<service::NavService>(walk_group, imu, gps, 0.3f);

        service::MotionService::Config motion_cfg;
        motion_cfg.clean_speed_rpm = p.test_speed_rpm;
        motion_cfg.return_speed_rpm = p.test_return_rpm;
        motion_cfg.brush_rpm = use_real_brush_ ? static_cast<int>(std::lround(p.brush_test_rpm)) : 0;
        motion_cfg.return_brush_rpm =
            use_real_brush_ ? static_cast<int>(std::lround(p.brush_test_rpm)) : 0;
        motion_cfg.edge_reverse_rpm = 0.0f;
        motion_cfg.heading_pid_en = pid_enabled;
        // 将配置文件中的 PID 参数传入（无论是否使能，均写入供 start_cleaning 时生效）
        motion_cfg.pid.kp = p.pid.kp;
        motion_cfg.pid.ki = p.pid.ki;
        motion_cfg.pid.kd = p.pid.kd;
        motion_cfg.pid.max_output = p.pid.max_output;
        motion_cfg.pid.integral_limit = p.pid.integral_limit;
        motion_cfg.pid.deadband_rate_dps = p.pid.deadband_rate_dps;

        motion =
            std::make_shared<service::MotionService>(walk_group, brush, imu, event_bus, motion_cfg);
        fault = std::make_shared<service::FaultService>(event_bus);

        // WatchdogMgr：路径为空 = 不操作 /dev/watchdog
        watchdog = std::make_unique<app::WatchdogMgr>("");

        fsm = std::make_shared<app::RobotFsm>(motion, nav, fault, event_bus);

        // SafetyMonitor LimitSettledEvent → RobotFsm
        event_bus.subscribe<middleware::SafetyMonitor::LimitSettledEvent>(
            [this](const middleware::SafetyMonitor::LimitSettledEvent& evt) {
                if (evt.side == device::LimitSide::LEFT)
                    fsm->dispatch(app::EvFarEndLimitSettled{});
                else
                    fsm->dispatch(app::EvParkingSideLimitSettled{});
            });

        // FaultHandler: P0→emergency_stop+EvFaultP0，P1→stop+EvFaultP1；同时记录 dispatched_faults
        fault_handler = std::make_shared<app::FaultHandler>(
            motion, event_bus, [this](service::FaultService::FaultEvent e) {
                dispatched_faults.push_back(e);
                using Level = service::FaultService::FaultEvent::Level;
                if (e.level == Level::P0)
                    fsm->dispatch(app::EvFaultP0{});
                else if (e.level == Level::P1)
                    fsm->dispatch(app::EvFaultP1{});
            });
    }

    /// 初始化所有硬件并启动后台线程，dispatch EvInitDone → FSM = "Idle"
    /// @param health_jsonl_path  HealthService JSONL 落盘路径（空字符串=不落盘）
    /// @return false 表示关键硬件（walk_group）初始化失败
    bool init(const std::string& health_jsonl_path = "") {
        using robot::device::DeviceError;
        if (walk_group->open() != DeviceError::OK) {
            spdlog::error("[FullSystemFixture] walk_group open 失败");
            return false;
        }
        walk_group->set_feedback_mode_all(10u);  // 10ms 主动上报

        if (!imu->open())
            spdlog::warn("[FullSystemFixture] IMU open 失败（非致命）");

        if (bms->open() != DeviceError::OK)
            spdlog::warn("[FullSystemFixture] BMS open 失败（非致命）");

        if (!gps || !gps->open())
            spdlog::warn("[FullSystemFixture] GPS(gpsd) open 失败（非致命）");

        if (!brush->open()) {
            if (use_real_brush_) {
                spdlog::error("[FullSystemFixture] 真实滚刷串口打开失败: {}", p.brush_port);
                return false;
            }
            spdlog::warn("[FullSystemFixture] mock brush open 失败（非致命）");
        } else if (use_real_brush_) {
            if (brush->clear_fault() != DeviceError::OK)
                spdlog::warn("[FullSystemFixture] 真实滚刷 clear_fault 失败");
            if (brush->set_mode_speed() != DeviceError::OK)
                spdlog::warn("[FullSystemFixture] 真实滚刷 set_mode_speed 失败");
        }

        // 限位开关：gpiochip5 不支持 IRQ，使用 1ms 软件轮询；测试中不设 RT 优先级，无 CPU 绑定
        if (!left_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] left_sw open 失败");
        if (!right_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] right_sw open 失败");

        if (!safety->start()) {
            spdlog::error("[FullSystemFixture] safety monitor start 失败");
            return false;
        }
        watchdog->start();
        fault_handler->start_listening();
        start_loops_();

        // HealthService 在硬件 open 之后构造，保证设备状态缓存已就绪
        if (!health_jsonl_path.empty()) {
            std::filesystem::remove(health_jsonl_path);  // 清旧文件，open() 创建新文件

            health = std::make_shared<robot::service::HealthService>(
                walk_group,
                brush,
                bms,
                imu,
                gps,
                nullptr,  // cloud = null，不需要 MQTT/LoRaWAN
                robot::service::HealthService::Mode::DIAGNOSTICS,
                health_jsonl_path,
                p.health_log_max_bytes,
                p.health_log_max_files);
            spdlog::info("[FullSystemFixture] HealthService 已创建: {}", health_jsonl_path);
        }

        // FSM: StateInit → Idle
        fsm->dispatch(robot::app::EvInitDone{});
        return true;
    }

    /// 轮询等待 FSM 进入指定状态，超时返回 false
    bool wait_state(const std::string& expected,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (fsm->current_state() == expected)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        spdlog::warn(
            "[FullSystemFixture] wait_state 超时: 期望={} 实际={}", expected, fsm->current_state());
        return false;
    }

    ~FullSystemFixture() {
        stop_loops_();
        if (safety)
            safety->stop();
        if (motion)
            motion->emergency_stop();
        if (watchdog)
            watchdog->stop();
    }

   private:
    std::thread walk_ctrl_thread_;
    std::thread nav_exec_thread_;
    std::thread brush_poll_thread_;
    std::atomic<bool> loops_running_{false};

    void start_loops_() {
        loops_running_.store(true);
        walk_ctrl_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                motion->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        nav_exec_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                nav->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        if (use_real_brush_) {
            brush_poll_thread_ = std::thread([this] {
                while (loops_running_.load()) {
                    brush->update();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            });
        }
    }

    void stop_loops_() {
        loops_running_.store(false);
        if (walk_ctrl_thread_.joinable())
            walk_ctrl_thread_.join();
        if (nav_exec_thread_.joinable())
            nav_exec_thread_.join();
        if (brush_poll_thread_.joinable())
            brush_poll_thread_.join();
    }
};

// ── ImuGpsHealthFixture：仅 IMU + GPS + Health 本地落盘 ─────────────────────
struct ImuGpsHealthFixture {
    HwParams p;
    std::shared_ptr<MockCanBus> mock_can;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<MockSerialPort> mock_brush_serial;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<MockSerialPort> mock_bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::device::GpsDevice> gps;
    std::shared_ptr<robot::service::HealthService> health;

    ImuGpsHealthFixture() : p(load_hw_test_config()) {
        using namespace robot;

        // HealthService 仍要求 walk / brush / bms 依赖，这里用最小 mock 底座承载，
        // 不打开真实 CAN / BMS / Brush 硬件，仅为生成完整 JSON 提供静态状态。
        mock_can = std::make_shared<MockCanBus>();
        walk_group = std::make_shared<device::WalkMotorGroup>(mock_can, 1u, 200u, false, 1u, 2u);

        mock_brush_serial = std::make_shared<MockSerialPort>();
        brush = std::make_shared<device::BrushMotor>(mock_brush_serial, 0u, 8192.0f, false, 0.5f);

        mock_bms_serial = std::make_shared<MockSerialPort>();
        bms = std::make_shared<device::BMS>(mock_bms_serial, 95.0f, 15.0f);

        imu_serial =
            std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu = std::make_shared<device::ImuDevice>(imu_serial);

        device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = p.gpsd_host;
        gpsd_cfg.port = p.gpsd_port;
        gpsd_cfg.watch = p.gpsd_watch;
        gps = device::GpsDevice::create_gpsd(gpsd_cfg);
    }

    bool init(const std::string& health_jsonl_path) {
        if (!imu || !imu->open()) {
            spdlog::error("[ImuGpsHealthFixture] IMU open 失败");
            return false;
        }
        if (!gps || !gps->open()) {
            spdlog::error("[ImuGpsHealthFixture] GPS(gpsd) open 失败");
            return false;
        }

        health = std::make_shared<robot::service::HealthService>(
            walk_group,
            brush,
            bms,
            imu,
            gps,
            nullptr,
            robot::service::HealthService::Mode::DIAGNOSTICS,
            health_jsonl_path,
            p.health_log_max_bytes,
            p.health_log_max_files);
        return true;
    }

    ~ImuGpsHealthFixture() {
        if (gps)
            gps->close();
        if (imu)
            imu->close();
    }
};

// ── ThingsBoardRuntimeFixture：真实硬件 + 真实 ThingsBoard 运行时联调 ─────────
struct ThingsBoardRuntimeFixture : FullSystemFixture {
    std::optional<tb_test_support::RepoPaths> tb_paths;
    std::string tb_cache_path{"/tmp/hw_tb_runtime_cache.jsonl"};
    std::unique_ptr<robot::service::ConfigService> tb_cfg_file;
    robot::service::SchedulerService scheduler;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net_mgr;
    std::shared_ptr<robot::middleware::DataCache> data_cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::shared_ptr<robot::service::ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<robot::service::CommandTracker> command_tracker;
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;
    std::shared_ptr<robot::service::ThingsBoardControlPlane> tb_control;

    ThingsBoardRuntimeFixture() : FullSystemFixture(false) {}

    bool right_limit_active() const {
        return right_sw && !right_sw->read_current_level();
    }

    bool left_limit_active() const {
        return left_sw && !left_sw->read_current_level();
    }

    robot::app::ParkingSideFacts parking_facts() const {
        const bool left_sensor_active = left_limit_active();
        const bool right_sensor_active = right_limit_active();
        return robot::app::ParkingSideRuntime::from_physical_limits(
            tb_cfg ? tb_cfg->active_config().parking_side : robot::service::ParkingSide::Left,
            left_sensor_active,
            right_sensor_active);
    }

    bool is_at_parking_side() const {
        return parking_facts().at_parking_side;
    }

    bool is_at_far_end() const {
        return parking_facts().at_far_end;
    }

    bool init_thingsboard_runtime(const std::string& health_jsonl_path = "") {
        tb_paths = tb_test_support::find_repo_paths();
        if (!tb_paths.has_value()) {
            spdlog::error(
                "[ThingsBoardRuntimeFixture] 未找到 config.runtime.json/config.fixed.json 或旧 "
                "config.json");
            return false;
        }

        tb_cfg_file = std::make_unique<robot::service::ConfigService>(
            tb_paths->runtime_config_path.string(), tb_paths->fixed_config_path.string());
        if (!tb_cfg_file->load()) {
            spdlog::error("[ThingsBoardRuntimeFixture] 配置加载失败: {}",
                          tb_paths->runtime_config_path.string());
            return false;
        }

        if (!FullSystemFixture::init(health_jsonl_path)) {
            return false;
        }

        std::filesystem::remove(tb_cache_path);
        data_cache = std::make_shared<robot::middleware::DataCache>(tb_cache_path);
        if (!data_cache->open()) {
            spdlog::error("[ThingsBoardRuntimeFixture] DataCache open 失败");
            return false;
        }

        auto mqtt_cfg = build_tb_mqtt_config(*tb_cfg_file, tb_paths->repo_root);
        if (mqtt_cfg.broker_uri.empty()) {
            spdlog::error("[ThingsBoardRuntimeFixture] network.mqtt.broker_uri 为空");
            return false;
        }

        mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);
        net_mgr = std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
        cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
        tb_cfg =
            std::make_shared<robot::service::ThingsBoardConfigManager>(*tb_cfg_file, scheduler);
        command_tracker = std::make_shared<robot::service::CommandTracker>();
        if (motion) {
            motion->set_parking_side_provider(
                [this]() { return tb_cfg->active_config().parking_side; });
            motion->set_runtime_config_provider([this]() { return tb_cfg->active_config(); });
        }
        supervisor =
            std::make_shared<robot::app::RobotSupervisor>(fsm, tb_cfg, command_tracker, fault, nav);
        tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
            *tb_cfg_file, cloud, tb_cfg, command_tracker, supervisor);

        tb_control->subscribe_shared_attributes();
        tb_control->register_rpc_handlers(
            [this] { return parking_facts().is_valid_start_position(); },
            [this] { return is_at_parking_side(); },
            [this] { return is_at_parking_side(); },
            [this] { return 80.0f; },
            []() {});

        if (!net_mgr->connect()) {
            return false;
        }
        tb_control->request_shared_attributes_snapshot();
        return true;
    }

    bool wait_state_with_thingsboard(
        const std::vector<std::string>& expected,
        std::chrono::seconds timeout,
        std::chrono::milliseconds poll_period = std::chrono::milliseconds(500)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto state = fsm->current_state();
            for (const auto& item : expected) {
                if (state == item) {
                    return true;
                }
            }
            if (bms) {
                bms->update();
            }
            if (health) {
                health->update();
            }
            if (tb_control) {
                tb_control->publish_business_telemetry();
            }
            std::this_thread::sleep_for(poll_period);
        }

        const auto final_state = fsm->current_state();
        for (const auto& item : expected) {
            if (final_state == item) {
                return true;
            }
        }
        return false;
    }

    ~ThingsBoardRuntimeFixture() {
        if (net_mgr) {
            net_mgr->disconnect();
        }
        if (data_cache) {
            data_cache->close();
        }
        std::filesystem::remove(tb_cache_path);
    }
};

}  // namespace hw
