/**
 * @file main.cc
 * @brief PV 清扫机器人主程序入口（配置文件驱动依赖注入）
 *
 * 启动流程：
 *   1. 加载 split config（runtime + fixed）
 *   2. 初始化日志
 *   3. 构造 HAL / Driver / Device 对象
 *   4. 构造 Middleware / Service / App 对象
 *   5. 启动各执行线程
 *   6. 进入主循环（轮询 FSM 状态与调度服务）
 *   7. 捕获 SIGINT/SIGTERM，优雅关闭
 */
#include <cmath>
#include <sys/mman.h>
#include <unistd.h>

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
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
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
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/recovery_motion.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

// App
#include <algorithm>
#include <atomic>
#include <csignal>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <string_view>
#include <thread>

#include "pv_cleaning_robot/app/robot_controller.h"
#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// ── 优雅退出信号 ──────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

namespace {

void publish_startup_position_status(
    const std::shared_ptr<spdlog::logger>& log,
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

}  // namespace

int main() {
    // ── 1. 配置服务 ────────────────────────────────────────────────────
    // ConfigService::get() 会先读 runtime，再回退到 fixed。
    auto cfg_ptr = std::make_unique<robot::service::ConfigService>(
        "/opt/robot/config/config.runtime.json", "/opt/robot/config/config.fixed.json");
    if (!cfg_ptr->load()) {
        cfg_ptr = std::make_unique<robot::service::ConfigService>("config/config.runtime.json",
                                                                  "config/config.fixed.json");
        if (!cfg_ptr->load()) {
            fprintf(stderr, "[FATAL] 无法加载 config.runtime.json/config.fixed.json\n");
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

    // ── 2.1 锁定所有内存页（消除运行期缺页中断，降低 RT 延迟抖动）──────
    // MCL_CURRENT: 锁定当前已映射页；MCL_FUTURE: 锁定后续 mmap/堆增长页
    // 需要 CAP_IPC_LOCK 或运行为 root；失败仅警告，不阻止启动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        log->warn(
            "[Main] mlockall(MCL_CURRENT|MCL_FUTURE) 失败: {} "
            "（内存分页中断可能增加 RT 延迟抖动，建议以 root 运行或设置 RLIMIT_MEMLOCK）",
            strerror(errno));
    } else {
        log->info("[Main] 内存已全部锁定，RT 延迟抖动最小化");
    }

    // ── 3. 信号处理 ────────────────────────────────────────────────────
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── 4. EventBus ───────────────────────────────────────────────────
    robot::middleware::EventBus event_bus;

    // ── 5. CAN 驱动 ────────────────────────────────────────────────────
    auto can_bus = std::make_shared<robot::driver::LinuxCanSocket>(
        cfg.get<std::string>("can.interface", "can0"));

    // WalkMotorGroup：4轮统一控制，含通信超时与锁存式 emergency override；
    // 视觉纠偏由 MotionService 持有的 HeadingCorrector 负责
    // comm_timeout=200ms（update() 20ms 周期的 10 倍），允许错过 9 帧不停车
    auto walk_group = std::make_shared<robot::device::WalkMotorGroup>(
        can_bus,
        cfg.get<uint8_t>("can.walk_motor.motor_id", 1u),
        cfg.get<uint16_t>("can.walk_motor.comm_timeout_ms", 200u),
        cfg.get<bool>("can.walk_motor.termination_init_enabled", true),
        cfg.get<uint8_t>("can.walk_motor.termination_init_retry_count", 3u),
        cfg.get<uint8_t>("can.walk_motor.termination_motor_id", 2u));

    // ── 6. 串口驱动 ───────────────────────────────────────────────────
    auto brush_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.brush.port", "/dev/ttyS3"),
        robot::hal::UartConfig{cfg.get<int>("serial.brush.baudrate", 115200)});
    // BMS 使用嘉佰达通用协议 V4（UART，9600 bps，8-N-1）
    auto bms_serial = std::make_shared<robot::driver::LibSerialPort>(
        cfg.get<std::string>("serial.bms.port", "/dev/ttyS8"),
        robot::hal::UartConfig{cfg.get<int>("serial.bms.baudrate", 9600)});

    auto brush_motor = std::make_shared<robot::device::BrushMotor>(
        brush_serial,
        cfg.get<uint8_t>("serial.brush.axis", 0u));

    auto bms = std::make_shared<robot::device::BMS>(
        bms_serial,
        cfg.get<float>("robot.charge_stop_soc", 95.0f),
        cfg.get<float>("robot.min_battery_soc", 30.0f));

    // ── 7. UART 驱动 ──────────────────────────────────────────────────
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

    // ── 8. GPIO 限位开关 ───────────────────────────────────────────────
    auto left_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.left_limit.chip", "gpiochip0"),
        cfg.get<int>("gpio.left_limit.line", 10));
    auto right_gpio = std::make_shared<robot::driver::LibGpiodPin>(
        cfg.get<std::string>("gpio.right_limit.chip", "gpiochip0"),
        cfg.get<int>("gpio.right_limit.line", 11));

    auto left_switch =
        std::make_shared<robot::device::LimitSwitch>(left_gpio, robot::device::LimitSide::LEFT);
    auto right_switch =
        std::make_shared<robot::device::LimitSwitch>(right_gpio, robot::device::LimitSide::RIGHT);

    // ── 9. 初始化设备 ──────────────────────────────────────────────────
    // 行走电机组是运动控制核心（open 失败则整体无法运行）
    if (walk_group->open() != robot::device::DeviceError::OK) {
        log->error("[Main] walk_group CAN 初始化失败，退出");
        return 1;
    }
    // 主动配置电机反馈方式：10ms 主动上报（100Hz），与 nav_exec 10ms 采样对齐。
    // 不依赖上次写入 EEPROM 的值，确保每次上电行为确定性。
    if (walk_group->set_feedback_mode_all(10u) != robot::device::DeviceError::OK)
        log->warn("[Main] 电机反馈模式配置失败，将使用硬件保存值");
    else
        log->info("[Main] 电机反馈配置：10ms 主动上报 (100Hz)");

    if (!imu->open())
        log->warn("[Main] IMU 初始化失败");
    else {
        // 主动配置 IMU 输出频率为 100Hz（RRATE=0x09），与 imu_read 线程和姿态纠偏周期对齐。
        // 不依赖硬件 EEPROM 保存值，确保上电后频率确定。
        if (imu->set_output_rate(100) != robot::device::DeviceError::OK)
            log->warn("[Main] IMU 频率配置失败，将使用硬件保存值");
        else
            log->info("[Main] IMU 输出频率配置：100Hz");
    }
    if (!gps->open())
        log->warn("[Main] GPS 初始化失败");
    // gpio.use_irq: 若 GPIO 控制器不支持硬件 IRQ（如 RK3576 gpiochip5），配置为 false 使用 1ms
    // 软件轮询
    const bool gpio_use_irq = cfg.get<bool>("gpio.use_irq", false);
    const bool left_open_ok = left_switch->open(95,  // SCHED_FIFO 95: GPIO 边缘最高硬件响应优先级
                                                2,       // 2ms 软件消抖
                                                1 << 4,  // 绑定大核 CPU 4（安全关键专用）
                                                gpio_use_irq);
    const bool right_open_ok =
        right_switch->open(95,
                           2,
                           1 << 4,  // 绑定大核 CPU 4（与 safety_monitor 同核，减少跨核缓存失效）
                           gpio_use_irq);
    if (!left_open_ok)
        log->warn("[Main] 左限位开关初始化失败");
    if (!right_open_ok)
        log->warn("[Main] 右限位开关初始化失败");

    if (!brush_motor->open())
        log->warn("[Main] 滚刷电机 RS485 初始化失败");
    if (bms->open() != robot::device::DeviceError::OK)
        log->warn("[Main] BMS RS485 初始化失败");

    // ── 10. 安全监控器 ─────────────────────────────────────────────────
    robot::middleware::SafetyMonitor safety_monitor(
        [walk_group]() { walk_group->emergency_override(0.0f); },
        left_switch,
        right_switch,
        event_bus);
    safety_monitor.start();

    // ── 11. 网络传输 ───────────────────────────────────────────────────
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

    // ── 12. 数据缓存 ───────────────────────────────────────────────────
    auto data_cache = std::make_shared<robot::middleware::DataCache>(
        cfg.get<std::string>("storage.cache_path", "/var/robot/telemetry_cache.jsonl"));
    data_cache->open();

    // ── 13. 服务层 ─────────────────────────────────────────────────────
    robot::service::MotionService::Config motion_cfg;
    motion_cfg.clean_speed_rpm = cfg.get<float>("robot.clean_speed_rpm", 300.0f);
    motion_cfg.return_speed_rpm = cfg.get<float>("robot.return_speed_rpm", 300.0f);
    motion_cfg.brush_rpm = cfg.get<int>("robot.brush_rpm", 1000);
    motion_cfg.heading_pid_en = cfg.get<bool>("robot.heading_pid_en", false);
    // 视觉纠偏参数从 robot.pid.* 读取；未配置时使用控制器头文件默认值。
    motion_cfg.pid.uds_path = cfg.get<std::string>("robot.pid.uds_path", motion_cfg.pid.uds_path);
    motion_cfg.pid.reconnect_interval_ms =
        cfg.get<int>("robot.pid.reconnect_interval_ms", motion_cfg.pid.reconnect_interval_ms);
    motion_cfg.pid.result_timeout_ms =
        cfg.get<int>("robot.pid.result_timeout_ms", motion_cfg.pid.result_timeout_ms);
    motion_cfg.pid.min_confidence =
        cfg.get<float>("robot.pid.min_confidence", motion_cfg.pid.min_confidence);
    motion_cfg.pid.deadband_yaw_deg =
        cfg.get<float>("robot.pid.deadband_yaw_deg",
                       cfg.get<float>("robot.pid.deadband_slope",
                                      cfg.get<float>("robot.pid.deadband_norm",
                                                     motion_cfg.pid.deadband_yaw_deg)));
    motion_cfg.pid.kp = cfg.get<float>("robot.pid.kp", motion_cfg.pid.kp);
    motion_cfg.pid.ki = cfg.get<float>("robot.pid.ki", motion_cfg.pid.ki);
    motion_cfg.pid.kd = cfg.get<float>("robot.pid.kd", motion_cfg.pid.kd);
    motion_cfg.pid.integral_limit =
        cfg.get<float>("robot.pid.integral_limit", motion_cfg.pid.integral_limit);
    motion_cfg.pid.max_output = cfg.get<float>("robot.pid.max_output", motion_cfg.pid.max_output);
    motion_cfg.pid.min_effective_output =
        cfg.get<float>("robot.pid.min_effective_output", motion_cfg.pid.min_effective_output);
    motion_cfg.pid.yaw_alpha =
        cfg.get<float>("robot.pid.yaw_alpha",
                       cfg.get<float>("robot.pid.slope_alpha",
                                      cfg.get<float>("robot.pid.offset_alpha",
                                                     motion_cfg.pid.yaw_alpha)));
    motion_cfg.pid.output_sign =
        cfg.get<float>("robot.pid.output_sign", motion_cfg.pid.output_sign);

    auto motion = std::make_shared<robot::service::MotionService>(
        walk_group, brush_motor, imu, event_bus, motion_cfg);
    auto nav = std::make_shared<robot::service::NavService>(walk_group, imu, gps);
    auto recovery = std::make_shared<robot::service::RecoveryMotion>();
    robot::service::SchedulerService scheduler;
    auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
    auto command_tracker = std::make_shared<robot::service::CommandTracker>();

    auto fault = std::make_shared<robot::service::FaultService>(event_bus);

    // ── 14. 应用层 ─────────────────────────────────────────────────────
    auto controller = std::make_shared<robot::app::RobotController>(
        robot::app::RobotController::ActionPorts{
            [motion](const robot::domain::MissionSegment& segment) {
                return motion->start_segment(segment);
            },
            [motion] { motion->stop_cleaning(); },
            [motion] { motion->emergency_stop(); },
            [recovery] { recovery->start(); },
            [fault] { fault->clear_active_fault(); },
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
            runtime.active_config = snap.active_config;
            runtime.pending_config = snap.pending_config;
            return runtime;
        }};
    auto tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
        cfg, &scheduler, cloud, command_tracker, command_port);
    cfg.apply_active_runtime_schedules(scheduler);
    motion->set_primary_dock_query(
        [&cfg]() { return cfg.active_runtime_config().primary_dock; });
    motion->set_runtime_config_query([&cfg]() { return cfg.active_runtime_config(); });
    const auto current_battery_soc = [&bms]() { return bms->get_data().soc_pct; };
    const auto current_limit_levels =
        [&left_switch, &right_switch, left_open_ok, right_open_ok]() -> std::pair<bool, bool> {
            const bool left_sensor_active = left_open_ok && !left_switch->read_current_level();
            const bool right_sensor_active = right_open_ok && !right_switch->read_current_level();
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
    });
    controller->start();
    const auto configured_primary_dock_text = robot::domain::endpoint_config_string(
        cfg.active_runtime_config().primary_dock);
    const auto [startup_left_limit_active, startup_right_limit_active] = current_limit_levels();
    const auto startup_state = robot::domain::estimate_position(
        robot::domain::LimitState{startup_left_limit_active, startup_right_limit_active});
    if (!robot::domain::is_at_target(startup_state, cfg.active_runtime_config().primary_dock)) {
        log->warn(
            "[Main] [自检警告] 当前未检测到主停机端限位触发，"
            "设备可能不在配置主停机端（primary_dock={}），请手动归位再启动",
            configured_primary_dock_text);
    }
    const bool dual_dock_mode = cfg.get<std::string>("robot.dock_mode", "single_dock") == "dual_dock";
    const auto opposite_dock = robot::domain::opposite_endpoint(cfg.active_runtime_config().primary_dock);
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
    tb_control->subscribe_shared_attributes();
    // 在 connect() 前完成 shared attributes / RPC 注册，避免首个云端下行消息丢失。
    tb_control->register_rpc_handlers();

    net_mgr->connect();

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
            robot::domain::LaneConfig{
                dual_dock_mode ? robot::domain::DockMode::DualDock
                               : robot::domain::DockMode::SingleDock,
                cfg.active_runtime_config().primary_dock},
            startup_state));

    // ── 应用层桥接：限位业务事件 / 调度窗口启动 ──────────────────────────
    safety_monitor.set_limit_settled_callback(
        [controller](robot::domain::Endpoint endpoint) {
            controller->post_limit_settled(endpoint);
        });
    safety_monitor.set_limit_unstable_callback(
        [controller](robot::domain::Endpoint endpoint) {
            controller->post_limit_unstable(endpoint);
        });

    // ── 调度服务：定时触发清扫任务（读取 active runtime 的 scheduler.windows） ─────
    scheduler.set_on_window_hit([controller] { controller->post_schedule_window_hit(); });

    robot::app::WatchdogMgr watchdog(cfg.get<std::string>("system.hw_watchdog", "/dev/watchdog"));
    watchdog.set_timeout_callback([controller, motion](const std::string& thread_name) {
        motion->emergency_stop();
        controller->post_watchdog_timeout(thread_name);
    });
    watchdog.start();

    // ── 15. 上报服务 ───────────────────────────────────────────────────
    std::string diag_mode = cfg.get<std::string>("diagnostics.mode", "production");
    const bool cloud_upload = cfg.get<bool>("diagnostics.cloud_upload", true);
    const bool local_log = cfg.get<bool>("diagnostics.local_log", true);
    // HealthService 已内嵌 DiagnosticsCollector 逻辑：
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
    std::shared_ptr<robot::service::CloudService> health_cloud = cloud_upload ? cloud : nullptr;
    auto reporter = std::make_shared<robot::service::HealthService>(
        walk_group,
        brush_motor,
        bms,
        imu,
        gps,
        health_cloud,
        diag_mode == "development" ? robot::service::HealthService::Mode::DIAGNOSTICS
                                   : robot::service::HealthService::Mode::HEALTH,
        local_tel_path,
        local_log_max_bytes,
        local_log_max_files);

    // ── 16. 线程执行器 ────────────────────────────────────────────────
    // ── RK3576 CPU 拓扑 ───────────────────────────────────────────────
    //   CPU 0-3: Cortex-A55 (LITTLE 核)  → 非 RT 后台任务
    //   CPU 4-7: Cortex-A76 (BIG 核)     → RT 任务专用
    //
    // 线程绑定策略（与 safety_monitor/gpio 线程配合，不竞争同一核心）：
    //   CPU 4: gpio_left/right(95) + safety_monitor(94)  → GPIO 边沿优先，轮询兜底
    //   CPU 5: group_recv(82) + walk_ctrl(80)            → 先接收后控制，降低读旧数据概率
    //   CPU 6: nav(65) + imu_read(68)                    → 导航路径，专用核
    //   CPU 7: watchdog(50) + main(SCHED_OTHER)          → 管理任务
    //   CPU 0-3: bms + cloud + gps（LITTLE 核，低功耗后台）
    //
    // 行走控制线程：SCHED_FIFO 80, 20ms (50Hz)，绑定 CPU 5
    // 50Hz 姿态纠偏与 100Hz IMU 组合，采样/控制比 2:1，路align 速度 1.5m/s 下 20ms 塔偏跨度 ≤ 30mm
    robot::middleware::ThreadExecutor walk_exec({"walk_ctrl", 50, SCHED_FIFO, 80, 1 << 5});
    walk_exec.add_runnable(motion);
    // 心跳在 walk_ctrl 线程自身内汇报（超时 = 该线程死锁，而非主线程死锁）
    int walk_wd = watchdog.register_thread("walk_ctrl", 500);
    walk_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, walk_wd]() { watchdog.heartbeat(walk_wd); }));

    // 导航线程：SCHED_FIFO 65, 10ms，绑定 CPU 6
    // 里程计+IMU 融合有 10ms 截止时间约束，需要 RT 保证（旧版为 SCHED_OTHER）
    robot::middleware::ThreadExecutor nav_exec({"nav", 10, SCHED_FIFO, 65, 1 << 6});
    nav_exec.add_runnable(nav);
    int nav_wd = watchdog.register_thread("nav", 100);
    nav_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, nav_wd]() { watchdog.heartbeat(nav_wd); }));

    // BMS 采集线程：500ms，绑定 LITTLE 核 CPU 0-3（低功耗后台）
    robot::middleware::ThreadExecutor bms_exec({"bms", 500, SCHED_OTHER, 0, 0x0F});
    bms_exec.add_runnable(
        std::make_shared<robot::middleware::RunnableAdapter>([&bms]() { bms->update(); }));
    // BrushMotor 状态不需要实时性，500ms 周期足够。
    // 将 brush_->update() 安排在此线程，避免 Modbus RTU 阻塞 I/O
    // 占用 walk_ctrl(FIFO 80, 20ms) 的控制周期时间预算。
    bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&brush_motor]() { brush_motor->update(); }));
    int bms_wd = watchdog.register_thread("bms", 2000);
    bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, bms_wd]() { watchdog.heartbeat(bms_wd); }));

    // 云端上报线程：绑定 LITTLE 核 CPU 0-3（低功耗后台）
    const int active_report_period =
        std::max(1, cfg.get<int>("diagnostics.publish_interval_active_ms", 1000));
    const int idle_report_period =
        std::max(1, cfg.get<int>("diagnostics.publish_interval_idle_ms", 300000));
    robot::middleware::ThreadExecutor cloud_exec(
        {"cloud", active_report_period, SCHED_OTHER, 0, 0x0F});
    if (cloud_upload || local_log) {
        cloud_exec.add_runnable(reporter);
    }
    if (cloud_upload) {
        cloud_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
            [tb_control]() { tb_control->publish_business_telemetry(); }));
        cloud_exec.add_runnable(cloud);
    }
    int cloud_wd = watchdog.register_thread("cloud", 310000);
    cloud_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, cloud_wd]() { watchdog.heartbeat(cloud_wd); }));

    walk_exec.start();
    nav_exec.start();
    bms_exec.start();
    cloud_exec.start();

    log->info("[Main] 所有线程启动完成，进入主循环...");

    // ── 17. 主循环 ───────────────────────────────────────────────────
    while (g_running.load()) {
        // 调度器 tick：切换到从 SCHED_OTHER 主循环调用，精度 100ms（调度窪口最小误微差 1min）
        scheduler.tick();

        const std::string current_state = controller->snapshot().state;
        if (current_state == "SelfChecking") {
            controller->complete_self_check(true);
        }
        const bool active_motion_state =
            current_state == "ExecutingMission" || current_state == "Recovering";
        const int desired_report_period =
            active_motion_state ? active_report_period : idle_report_period;
        if (cloud_exec.period_ms() != desired_report_period) {
                cloud_exec.set_period_ms(desired_report_period);
        }
        if (current_state == "ExecutingMission" && nav->get_pose().spin_free_detected) {
            controller->post_fault(robot::app::FaultFact{
                robot::app::FaultSource::FaultDetector,
                robot::domain::FaultCode::kWheelSpinFree,
                "wheel_spin_free"});
            nav->clear_spin_detection();
        }
        if (current_state == "Recovering") {
            const auto recovery_result = recovery->step();
            if (recovery_result == robot::service::RecoveryMotion::Result::Done) {
                controller->post_recovery_finished(true);
            } else if (recovery_result == robot::service::RecoveryMotion::Result::Failed) {
                controller->post_recovery_finished(false);
            }
        }
        controller->post_tick();
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
    controller->stop();
    motion->emergency_stop();
    brush_motor->close();
    walk_group->close();
    imu->close();
    gps->close();
    bms->close();
    left_switch->close();
    right_switch->close();
    net_mgr->disconnect();
    data_cache->close();
    log->info("[Main] 正常退出");
    return 0;
}
