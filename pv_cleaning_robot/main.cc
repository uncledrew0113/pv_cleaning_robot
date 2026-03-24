/**
 * @file main.cc
 * @brief PV 清扫机器人主程序入口（配置文件驱动依赖注入）
 *
 * 启动流程：
 *   1. 加载 config/config.json
 *   2. 初始化日志
 *   3. 构造 HAL / Driver / Device 对象
 *   4. 构造 Middleware / Service / App 对象
 *   5. 启动各执行线程
 *   6. 进入主循环（轮询 FSM 状态与调度服务）
 *   7. 捕获 SIGINT/SIGTERM，优雅关闭
 */
#include "pv_cleaning_robot/middleware/logger.h"
#include "pv_cleaning_robot/service/config_service.h"

// HAL / Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"

// Protocol
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

// Device
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor.h"

// Middleware
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/lorawan_transport.h"
#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

// Service
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/diagnostics_collector.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

// App
#include <atomic>
#include <csignal>
#include <memory>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

#include "pv_cleaning_robot/app/clean_task.h"
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// ── 优雅退出信号 ──────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

int main() {
    // ── 1. 配置服务 ────────────────────────────────────────────────────
    // 优先尝试部署路径，失败则回退到当前目录（开发环境）
    auto cfg_ptr = std::make_unique<robot::service::ConfigService>("/opt/robot/config/config.json");
    if (!cfg_ptr->load()) {
        cfg_ptr = std::make_unique<robot::service::ConfigService>("config/config.json");
        if (!cfg_ptr->load()) {
            fprintf(stderr, "[FATAL] 无法加载 config.json\n");
            return 1;
        }
    }
    auto& cfg = *cfg_ptr;

    // ── 2. 日志初始化 ──────────────────────────────────────────────────
    robot::middleware::Logger::Config log_cfg;
    log_cfg.log_dir = cfg.get<std::string>("logging.log_dir", "logs");
    log_cfg.level = cfg.get<std::string>("logging.level", "info");
    log_cfg.console_output = cfg.get<bool>("logging.console", true);
    robot::middleware::Logger::init(log_cfg);
    auto log = robot::middleware::Logger::get();
    log->info("[Main] 配置加载完成，日志已初始化");

    // ── 3. 信号处理 ────────────────────────────────────────────────────
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── 4. EventBus ───────────────────────────────────────────────────
    robot::middleware::EventBus event_bus;

    // ── 5. CAN 驱动 ────────────────────────────────────────────────────
    auto can_bus = std::make_shared<robot::driver::LinuxCanSocket>(
        cfg.get<std::string>("can.interface", "can0"));

    auto walk_motor = std::make_shared<robot::device::WalkMotor>(
        can_bus, cfg.get<uint8_t>("can.walk_motor.motor_id", 1u));

    // ── 6. RS485 串口驱动 ──────────────────────────────────────────────
    // libmodbus 直接管理串口，无需 LibSerialPort 包装
    auto brush_modbus = std::make_shared<robot::driver::LibModbusMaster>(
        cfg.get<std::string>("serial.brush.port", "/dev/ttyS3"),
        robot::hal::ModbusConfig{cfg.get<int>("serial.brush.baudrate", 115200)});
    // BMS 使用嘉佰达通用协议 V4（UART，9600 bps，8-N-1）
    auto bms_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.bms.port", "/dev/ttyS4"),
        robot::hal::UartConfig{cfg.get<int>("serial.bms.baudrate", 9600)});

    auto brush_motor = std::make_shared<robot::device::BrushMotor>(
        brush_modbus, cfg.get<int>("serial.brush.slave_id", 1));

    auto bms = std::make_shared<robot::device::BMS>(bms_serial,
                                                    cfg.get<float>("robot.battery_full_soc", 95.0f),
                                                    cfg.get<float>("robot.battery_low_soc", 15.0f));

    // ── 7. UART 驱动 ──────────────────────────────────────────────────
    auto imu_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.imu.port", "/dev/ttyS1"),
        robot::hal::UartConfig{cfg.get<int>("serial.imu.baudrate", 921600)});
    auto imu = std::make_shared<robot::device::ImuDevice>(imu_serial);

    auto gps_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.gps.port", "/dev/ttyS2"),
        robot::hal::UartConfig{cfg.get<int>("serial.gps.baudrate", 9600)});
    auto gps = std::make_shared<robot::device::GpsDevice>(gps_serial);

    // ── 8. GPIO 限位开关 ───────────────────────────────────────────────
    auto front_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.front_limit.chip", "gpiochip0"),
        cfg.get<int>("gpio.front_limit.line", 10));
    auto rear_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.rear_limit.chip", "gpiochip0"),
        cfg.get<int>("gpio.rear_limit.line", 11));

    auto front_switch =
        std::make_shared<robot::device::LimitSwitch>(front_gpio, robot::device::LimitSide::FRONT);
    auto rear_switch =
        std::make_shared<robot::device::LimitSwitch>(rear_gpio, robot::device::LimitSide::REAR);

    // ── 9. 初始化设备 ──────────────────────────────────────────────────
    if (walk_motor->open() != robot::device::DeviceError::OK) {
        log->error("[Main] 行走电机 CAN 初始化失败");
        return 1;
    }
    if (!imu->open())
        log->warn("[Main] IMU 初始化失败");
    if (!gps->open())
        log->warn("[Main] GPS 初始化失败");
    if (!front_switch->open())
        log->warn("[Main] 前限位开关初始化失败");
    if (!rear_switch->open())
        log->warn("[Main] 后限位开关初始化失败");
    if (!brush_motor->open())
        log->warn("[Main] 辊刷电机 RS485 初始化失败");
    if (bms->open() != robot::device::DeviceError::OK)
        log->warn("[Main] BMS RS485 初始化失败");

    // ── 10. 安全监控器 ─────────────────────────────────────────────────
    robot::middleware::SafetyMonitor safety_monitor(
        walk_motor, front_switch, rear_switch, event_bus);
    safety_monitor.start();

    // ── 11. 网络传输 ───────────────────────────────────────────────────
    robot::middleware::MqttTransport::Config mqtt_cfg;
    mqtt_cfg.broker_uri = cfg.get<std::string>("network.mqtt.broker_uri", "tcp://localhost:1883");
    mqtt_cfg.client_id = cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001");
    mqtt_cfg.tls_enabled = cfg.get<bool>("network.mqtt.tls_enabled", false);
    auto mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);

    std::string transport_mode = cfg.get<std::string>("network.transport_mode", "mqtt_only");
    robot::middleware::NetworkManager::Mode net_mode =
        robot::middleware::NetworkManager::Mode::MQTT_ONLY;
    if (transport_mode == "lorawan_only")
        net_mode = robot::middleware::NetworkManager::Mode::LORAWAN_ONLY;
    if (transport_mode == "dual_parallel")
        net_mode = robot::middleware::NetworkManager::Mode::DUAL_PARALLEL;

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
    net_mgr->connect();

    // ── 12. 数据缓存 ───────────────────────────────────────────────────
    auto data_cache = std::make_shared<robot::middleware::DataCache>(
        cfg.get<std::string>("storage.cache_db", "/var/robot/telemetry.db"));
    data_cache->open();

    // ── 13. 服务层 ─────────────────────────────────────────────────────
    robot::service::MotionService::Config motion_cfg;
    motion_cfg.clean_speed_rpm = cfg.get<float>("robot.clean_speed_rpm", 300.0f);
    motion_cfg.return_speed_rpm = cfg.get<float>("robot.return_speed_rpm", 500.0f);
    motion_cfg.brush_rpm = cfg.get<int>("robot.brush_rpm", 1200);

    auto motion = std::make_shared<robot::service::MotionService>(
        walk_motor, brush_motor, event_bus, motion_cfg);
    auto nav = std::make_shared<robot::service::NavService>(walk_motor, imu, gps);
    auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);

    // 上电首次发布设备静态属性（云端连接后立即通知平台本机信息）
    if (net_mgr->is_connected()) {
        cloud->publish_attributes(nlohmann::json{
            {"fw_version", cfg.get<std::string>("device.fw_version", "1.0.0")},
            {"hw_version", cfg.get<std::string>("device.hw_version", "1.0")},
            {"device_id", cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001")}}
                                      .dump());
        log->info("[Main] 设备静态属性已发布至云端");
    }

    auto fault = std::make_shared<robot::service::FaultService>(event_bus);

    // ── 14. 应用层 ─────────────────────────────────────────────────────
    auto fsm = std::make_shared<robot::app::RobotFsm>(motion, nav, fault, event_bus);
    fsm->dispatch(robot::app::EvInitDone{});

    robot::app::CleanTask::Config task_cfg;
    task_cfg.track_length_m = cfg.get<float>("robot.track_length_m", 1000.0f);
    task_cfg.passes = cfg.get<int>("robot.passes", 1);
    auto clean_task = std::make_shared<robot::app::CleanTask>(motion, nav, task_cfg);

    robot::app::FaultHandler fault_handler(
        motion, event_bus, [fsm](const robot::service::FaultService::FaultEvent& evt) {
            using Level = robot::service::FaultService::FaultEvent::Level;
            if (evt.level == Level::P0)
                fsm->dispatch(robot::app::EvFaultP0{});
            else if (evt.level == Level::P1)
                fsm->dispatch(robot::app::EvFaultP1{});
        });
    fault_handler.start_listening();

    robot::app::WatchdogMgr watchdog(cfg.get<std::string>("system.hw_watchdog", "/dev/watchdog"));
    watchdog.set_timeout_callback([&fault](const std::string& thread_name) {
        fault->report(robot::service::FaultService::FaultEvent::Level::P0,
                      0xDEAD,
                      "watchdog timeout: " + thread_name);
    });
    watchdog.start();

    // ── 15. 上报服务 ───────────────────────────────────────────────────
    std::string diag_mode = cfg.get<std::string>("diagnostics.mode", "production");
    std::shared_ptr<robot::middleware::IRunnable> reporter;
    if (diag_mode == "development") {
        reporter = std::make_shared<robot::service::DiagnosticsCollector>(
            walk_motor, brush_motor, bms, imu, gps, cloud);
    } else {
        reporter = std::make_shared<robot::service::HealthService>(
            walk_motor, brush_motor, bms, imu, gps, cloud);
    }

    // ── 16. 线程执行器 ────────────────────────────────────────────────
    // 行走控制线程：SCHED_FIFO 80, 10ms
    robot::middleware::ThreadExecutor walk_exec({"walk_ctrl", 10, SCHED_FIFO, 80});
    walk_exec.add_runnable(motion);
    int walk_wd = watchdog.register_thread("walk_ctrl", 500);

    // 导航线程：10ms
    robot::middleware::ThreadExecutor nav_exec({"nav", 10});
    nav_exec.add_runnable(nav);

    // BMS 采集线程：500ms
    robot::middleware::ThreadExecutor bms_exec({"bms", 500});
    bms_exec.add_runnable(
        std::make_shared<robot::middleware::RunnableAdapter>([&bms]() { bms->update(); }));

    // 云端上报线程
    int report_period = cfg.get<int>("diagnostics.publish_interval_ms", 1000);
    robot::middleware::ThreadExecutor cloud_exec({"cloud", report_period});
    cloud_exec.add_runnable(reporter);
    cloud_exec.add_runnable(cloud);

    walk_exec.start();
    nav_exec.start();
    bms_exec.start();
    cloud_exec.start();

    log->info("[Main] 所有线程启动完成，进入主循环...");

    // ── 17. 主循环 ───────────────────────────────────────────────────
    while (g_running.load()) {
        // 看门狗心跳
        watchdog.heartbeat(walk_wd);

        // BMS 低电量检测
        if (bms->is_low_battery() && fsm->current_state() != "Returning" &&
            fsm->current_state() != "Charging") {
            log->warn("[Main] 电量不足，触发返回");
            fsm->dispatch(robot::app::EvLowBattery{});
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ── 18. 优雅关闭 ──────────────────────────────────────────────────
    log->info("[Main] 收到退出信号，正在关闭...");
    walk_exec.stop();
    nav_exec.stop();
    bms_exec.stop();
    cloud_exec.stop();
    safety_monitor.stop();
    watchdog.stop();
    motion->emergency_stop();
    walk_motor->close();
    net_mgr->disconnect();
    data_cache->close();
    log->info("[Main] 正常退出");
    return 0;
}
