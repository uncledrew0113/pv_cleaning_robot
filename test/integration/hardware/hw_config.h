// test/integration/hardware/hw_config.h
#pragma once
/**
 * @file hw_config.h
 * @brief 硬件测试公共 Fixture
 *
 * DeviceFixture       — Driver + Device 层，用于限位和电机单元测试
 * FullSystemFixture   — 全层栈，用于集成测试（BrushMotor 用 MockModbusMaster）
 *
 * 硬件接线（默认值与 config/config.json 对齐，可通过 hw_test_config.json 覆盖）：
 *   CAN      : can0（默认），行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1（默认），WIT Motion，9600 baud
 *   BMS      : /dev/ttyS8（默认），嘉佰达通用协议 V4，9600 baud
 *   距离传感器: /dev/ttyS9（默认），RS485 Modbus RTU，9600 baud
 *   GPIO     : gpiochip5 line0=前限位，line1=后限位（默认）
 */
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
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
#include "pv_cleaning_robot/device/distance_sensor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Middleware
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

// Service
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

// App
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// Mock（滚刷电机未安装）
#include "mock/mock_modbus_master.h"
#include "pv_cleaning_robot/service/config_service.h"

namespace hw {

// ── 运行时硬件测试参数（从 hw_test_config.json 加载，缺失则用内嵌默认值）──────
/// 所有参数均可通过 hw_test_config.json 覆盖，无需重新编译
struct HwParams {
    // hardware mapping
    std::string can_iface = "can0";
    uint8_t motor_id_base = 1u;
    uint16_t comm_timeout_ms = 500u;  ///< update 50ms × 10 倍余量
    std::string imu_port = "/dev/ttyS1";
    int imu_baud = 9600;
    std::string bms_port = "/dev/ttyS8";
    int bms_baud = 9600;
    std::string dist_port = "/dev/ttyS9";  ///< 距离传感器 RS485 串口
    int dist_baud = 9600;
    uint8_t dist_slave_id = 1u;
    uint8_t dist_channel_count = 2u;
    std::string gpio_chip = "gpiochip5";
    unsigned front_limit_line = 0u;
    unsigned rear_limit_line = 1u;
    // timing
    int limit_timeout_sec = 60;   ///< 每段（单一限位）等待最大秒数
    int online_timeout_ms = 600;  ///< 等待电机上线最大毫秒数
    int sweep_duration_ms = 5000;
    int loop_period_ms = 50;
    // behavior
    float test_speed_rpm = 10.0f;  ///< 安全低速（测试专用）
    float test_return_rpm = 10.0f;
    float sweep_rpm = 20.0f;
    float limit_test_rpm = 10.0f;
    float combined_passes = 50.0f;  ///< combined 测试趟数（1=一来回，2=两来回…）
    std::string health_jsonl_path = "/tmp/hw_system_test_health.jsonl";
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
        p.imu_port = cfg.get<std::string>("hardware.imu_port", p.imu_port);
        p.imu_baud = cfg.get<int>("hardware.imu_baud", p.imu_baud);
        p.bms_port = cfg.get<std::string>("hardware.bms_port", p.bms_port);
        p.bms_baud = cfg.get<int>("hardware.bms_baud", p.bms_baud);
        p.dist_port = cfg.get<std::string>("hardware.dist_port", p.dist_port);
        p.dist_baud = cfg.get<int>("hardware.dist_baud", p.dist_baud);
        p.dist_slave_id =
            static_cast<uint8_t>(cfg.get<int>("hardware.dist_slave_id", (int)p.dist_slave_id));
        p.dist_channel_count = static_cast<uint8_t>(
            cfg.get<int>("hardware.dist_channel_count", (int)p.dist_channel_count));
        p.gpio_chip = cfg.get<std::string>("hardware.gpio_chip", p.gpio_chip);
        p.front_limit_line = static_cast<unsigned>(
            cfg.get<int>("hardware.front_limit_line", (int)p.front_limit_line));
        p.rear_limit_line =
            static_cast<unsigned>(cfg.get<int>("hardware.rear_limit_line", (int)p.rear_limit_line));
        p.limit_timeout_sec = cfg.get<int>("timing.limit_timeout_sec", p.limit_timeout_sec);
        p.online_timeout_ms = cfg.get<int>("timing.online_timeout_ms", p.online_timeout_ms);
        p.sweep_duration_ms = cfg.get<int>("timing.sweep_duration_ms", p.sweep_duration_ms);
        p.loop_period_ms = cfg.get<int>("timing.loop_period_ms", p.loop_period_ms);
        p.test_speed_rpm = cfg.get<float>("behavior.test_speed_rpm", p.test_speed_rpm);
        p.test_return_rpm = cfg.get<float>("behavior.test_return_rpm", p.test_return_rpm);
        p.sweep_rpm = cfg.get<float>("behavior.sweep_rpm", p.sweep_rpm);
        p.limit_test_rpm = cfg.get<float>("behavior.limit_test_rpm", p.limit_test_rpm);
        p.combined_passes = cfg.get<float>("behavior.combined_passes", p.combined_passes);
        p.health_jsonl_path =
            cfg.get<std::string>("behavior.health_jsonl_path", p.health_jsonl_path);
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

// ── DeviceFixture：Driver + Device 层（限位 / 电机单元测试使用）────────────
struct DeviceFixture {
    HwParams p;  ///< 运行时硬件参数，必须为第一成员（构造时初始化列表先于 ctor 体）
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::driver::LibSerialPort> bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibGpiodPin> front_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> rear_gpio;
    std::shared_ptr<robot::device::LimitSwitch> front_sw;
    std::shared_ptr<robot::device::LimitSwitch> rear_sw;

    DeviceFixture() : p(load_hw_test_config()) {
        using namespace robot;
        can_bus = std::make_shared<driver::LinuxCanSocket>(p.can_iface);
        walk_group =
            std::make_shared<device::WalkMotorGroup>(can_bus, p.motor_id_base, p.comm_timeout_ms);
        imu_serial =
            std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial =
            std::make_shared<driver::LibSerialPort>(p.bms_port, hal::UartConfig{p.bms_baud});
        bms = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        front_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.front_limit_line);
        rear_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.rear_limit_line);
        front_sw = std::make_shared<device::LimitSwitch>(front_gpio, device::LimitSide::FRONT);
        rear_sw = std::make_shared<device::LimitSwitch>(rear_gpio, device::LimitSide::REAR);
    }

    ~DeviceFixture() {
        if (front_sw)
            front_sw->close();
        if (rear_sw)
            rear_sw->close();
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
    std::shared_ptr<MockModbusMaster> mock_modbus;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<robot::device::GpsDevice> gps_dummy;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::shared_ptr<robot::service::NavService> nav;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::service::FaultService> fault;
    std::shared_ptr<robot::service::HealthService> health;  ///< null cloud，本地 JSONL 落盘
    std::shared_ptr<robot::driver::LibModbusMaster> dist_modbus;  ///< 距离传感器 Modbus RTU 主站
    std::shared_ptr<robot::device::DistanceSensor> dist_sensor;  ///< 距离传感器（可选）
    std::unique_ptr<robot::app::WatchdogMgr> watchdog;
    std::shared_ptr<robot::app::RobotFsm> fsm;
    std::shared_ptr<robot::app::FaultHandler> fault_handler;
    std::vector<robot::service::FaultService::FaultEvent> dispatched_faults;

    /// @param pid_enabled 是否开启航向 PID（clean_cycle 测试按场景传入）
    explicit FullSystemFixture(bool pid_enabled = false) : DeviceFixture() {
        using namespace robot;

        mock_modbus = std::make_shared<MockModbusMaster>();
        brush = std::make_shared<device::BrushMotor>(mock_modbus, 1);

        // SafetyMonitor 构造时内部绑定 LimitSwitch 回调
        safety =
            std::make_unique<middleware::SafetyMonitor>(walk_group, front_sw, rear_sw, event_bus);

        // GPS 未安装：创建占位对象，不 open，NavService 会跳过 GPS 校正
        auto gps_serial_dummy =
            std::make_shared<driver::LibSerialPort>("/dev/null", hal::UartConfig{9600});
        gps_dummy = std::make_shared<device::GpsDevice>(gps_serial_dummy);

        nav = std::make_shared<service::NavService>(walk_group, imu, gps_dummy, 0.3f);

        service::MotionService::Config motion_cfg;
        motion_cfg.clean_speed_rpm = p.test_speed_rpm;
        motion_cfg.return_speed_rpm = p.test_return_rpm;
        motion_cfg.brush_rpm = 0;  // MockModbus，不实际驱动滚刷
        motion_cfg.return_brush_rpm = 0;
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
                if (evt.side == device::LimitSide::FRONT)
                    fsm->dispatch(app::EvFrontLimitSettled{});
                else
                    fsm->dispatch(app::EvRearLimitSettled{});
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

        // 限位开关：gpiochip5 不支持 IRQ，使用 1ms 软件轮询；测试中不设 RT 优先级，无 CPU 绑定
        if (!front_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] front_sw open 失败");
        if (!rear_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] rear_sw open 失败");

        if (!safety->start()) {
            spdlog::error("[FullSystemFixture] safety monitor start 失败");
            return false;
        }
        watchdog->start();
        fault_handler->start_listening();
        start_loops_();

        // HealthService 在硬件 open 之后构造，保证传感器缓存已就绪
        if (!health_jsonl_path.empty()) {
            std::filesystem::remove(health_jsonl_path);  // 清旧文件，open() 创建新文件

            // 距离传感器（Modbus RTU 9600-8-N-1，可选，打开失败则 nullptr 传入 HealthService）
            dist_modbus = std::make_shared<robot::driver::LibModbusMaster>(
                p.dist_port, robot::hal::ModbusConfig{p.dist_baud, 'N', 8, 1});
            if (dist_modbus->open()) {
                robot::device::DistanceSensorConfig dist_cfg;
                dist_cfg.slave_id = p.dist_slave_id;
                dist_cfg.channel_count = p.dist_channel_count;
                dist_sensor =
                    std::make_shared<robot::device::DistanceSensor>(dist_modbus, dist_cfg);
                if (!dist_sensor->open())
                    spdlog::warn("[FullSystemFixture] 距离传感器 open 失败，数据不可用");
                else {
                    spdlog::info("[FullSystemFixture] 距离传感器已打开: {}", p.dist_port);
                    // 启动 100ms 轮询线程更新距离传感器缓存
                    dist_update_thread_ = std::thread([this] {
                        while (loops_running_.load()) {
                            dist_sensor->update();
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                    });
                }
            } else {
                spdlog::warn("[FullSystemFixture] 距离传感器 Modbus 打开失败，传感器不可用");
            }

            health = std::make_shared<robot::service::HealthService>(
                walk_group,
                brush,
                bms,
                imu,
                gps_dummy,
                nullptr,  // cloud = null，不需要 MQTT/LoRaWAN
                robot::service::HealthService::Mode::DIAGNOSTICS,
                health_jsonl_path,
                dist_sensor);  // 距离传感器（可选）
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
    std::thread dist_update_thread_;  ///< 距离传感器轮询线程（可选）
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
    }

    void stop_loops_() {
        loops_running_.store(false);
        if (walk_ctrl_thread_.joinable())
            walk_ctrl_thread_.join();
        if (nav_exec_thread_.joinable())
            nav_exec_thread_.join();
        if (dist_update_thread_.joinable())
            dist_update_thread_.join();
    }
};

}  // namespace hw
