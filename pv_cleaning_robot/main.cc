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
#include <sys/mman.h>

#include <cmath>

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
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

// App
#include <atomic>
#include <csignal>
#include <algorithm>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/parking_side_runtime.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
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

    // WalkMotorGroup：4轮统一控制，含通信超时/航向PID/边缘覆盖功能
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
        cfg.get<uint8_t>("serial.brush.axis", 0u),
        cfg.get<float>("serial.brush.counts_per_rev", 8192.0f),
        cfg.get<bool>("serial.brush.watchdog_enabled", true),
        cfg.get<float>("serial.brush.watchdog_timeout_s", 0.5f));

    auto bms = std::make_shared<robot::device::BMS>(bms_serial,
                                                    cfg.get<float>("robot.battery_full_soc", 95.0f),
                                                    cfg.get<float>("robot.battery_low_soc", 15.0f));

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
            cfg.get<std::string>("gps.serial.port",
                                 cfg.get<std::string>("serial.gps.port", "/dev/ttyS2")),
            robot::hal::UartConfig{
                cfg.get<int>("gps.serial.baudrate",
                             cfg.get<int>("serial.gps.baudrate", 9600))});
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
        // 主动配置 IMU 输出频率为 100Hz（RRATE=0x09），与 imu_read 线程和 PID 对齐。
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

    // ── 上电自检：确认设备在当前配置的停机位，且对侧无遮挡 ───────────────────────
    // left_limit 实际对应左侧传感器，right_limit 实际对应右侧传感器。
    const auto configured_parking_side_text =
        cfg.get<std::string>("robot.parking_side", "left");
    const auto configured_parking_side =
        configured_parking_side_text == "right" ? robot::service::ParkingSide::Right
                                                : robot::service::ParkingSide::Left;
    const bool left_sensor_active = left_open_ok && !left_switch->read_current_level();
    const bool right_sensor_active = right_open_ok && !right_switch->read_current_level();
    const auto startup_facts = robot::app::ParkingSideRuntime::from_physical_limits(
        configured_parking_side, left_sensor_active, right_sensor_active);
    if (!startup_facts.at_parking_side) {
        log->warn(
            "[Main] [自检警告] 当前未检测到停机位一侧限位触发，"
            "设备可能不在配置停机位（parking_side={}），请手动归位再启动",
            configured_parking_side_text);
    }
    if (startup_facts.at_far_end) {
        log->warn(
            "[Main] [自检警告] 对侧限位当前处于触发状态，"
            "可能有遮挡或传感器异常，请检查环境");
    }
    log->info(
        "[Main] 上电自检：parking_side={} at_parking_side={} at_far_end={} (left_sensor={} right_sensor={})",
        configured_parking_side_text,
        startup_facts.at_parking_side,
        startup_facts.at_far_end,
        left_sensor_active,
        right_sensor_active);
    if (!brush_motor->open())
        log->warn("[Main] 滚刷电机 RS485 初始化失败");
    if (bms->open() != robot::device::DeviceError::OK)
        log->warn("[Main] BMS RS485 初始化失败");

    // ── 10. 安全监控器 ─────────────────────────────────────────────────
    robot::middleware::SafetyMonitor safety_monitor(
        walk_group, left_switch, right_switch, event_bus);
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
    motion_cfg.return_brush_rpm = cfg.get<int>("robot.return_brush_rpm", 1000);
    motion_cfg.edge_reverse_rpm = cfg.get<float>("robot.edge_reverse_rpm", 0.0f);
    motion_cfg.heading_pid_en = cfg.get<bool>("robot.heading_pid_en", true);
    // PID 参数从 robot.pid.* 读取；未配置时使用头文件默认值（kp=1.5, ki=0.05, kd=0.3）
    motion_cfg.pid.kp                    = cfg.get<float>("robot.pid.kp",                    1.5f);
    motion_cfg.pid.ki                    = cfg.get<float>("robot.pid.ki",                    0.05f);
    motion_cfg.pid.kd                    = cfg.get<float>("robot.pid.kd",                    0.3f);
    motion_cfg.pid.max_output            = cfg.get<float>("robot.pid.max_output",            30.0f);
    motion_cfg.pid.integral_limit        = cfg.get<float>("robot.pid.integral_limit",        5.0f);
    motion_cfg.pid.deadband_rate_dps     = cfg.get<float>("robot.pid.deadband_rate_dps",     2.0f);

    auto motion = std::make_shared<robot::service::MotionService>(
        walk_group, brush_motor, imu, event_bus, motion_cfg);
    auto nav = std::make_shared<robot::service::NavService>(walk_group, imu, gps);
    robot::service::SchedulerService scheduler;
    auto cloud = std::make_shared<robot::service::CloudService>(net_mgr, data_cache);
    auto tb_cfg =
        std::make_shared<robot::service::ThingsBoardConfigManager>(cfg, scheduler);
    auto command_tracker = std::make_shared<robot::service::CommandTracker>();

    auto fault = std::make_shared<robot::service::FaultService>(event_bus);

    // ── 14. 应用层 ─────────────────────────────────────────────────────
    auto fsm = std::make_shared<robot::app::RobotFsm>(motion, nav, fault, event_bus);
    fsm->dispatch(robot::app::EvInitDone{});
    auto supervisor =
        std::make_shared<robot::app::RobotSupervisor>(fsm, tb_cfg, command_tracker, fault, nav);
    auto tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
        cfg,
        cloud,
        tb_cfg,
        command_tracker,
        supervisor);
    const auto physical_limit_facts_for = [&left_switch, &right_switch, &left_open_ok, &right_open_ok](
                                              robot::service::ParkingSide parking_side) {
        const bool left_sensor_active = left_open_ok && !left_switch->read_current_level();
        const bool right_sensor_active = right_open_ok && !right_switch->read_current_level();
        return robot::app::ParkingSideRuntime::from_physical_limits(
            parking_side, left_sensor_active, right_sensor_active);
    };
    const auto current_active_parking_facts = [&tb_cfg, physical_limit_facts_for]() {
        return physical_limit_facts_for(tb_cfg->active_config().parking_side);
    };
    const auto current_start_parking_facts = [&tb_cfg, physical_limit_facts_for]() {
        const auto pending = tb_cfg->pending_config();
        const auto parking_side =
            pending ? pending->parking_side : tb_cfg->active_config().parking_side;
        return physical_limit_facts_for(parking_side);
    };
    tb_control->subscribe_shared_attributes();
    // 在 connect() 前完成 shared attributes / RPC 注册，避免首个云端下行消息丢失。
    tb_control->register_rpc_handlers(
        [current_start_parking_facts]() {
            return current_start_parking_facts().at_parking_side;
        },
        [current_start_parking_facts]() {
            return current_start_parking_facts().at_far_end;
        },
        [current_active_parking_facts]() {
            return current_active_parking_facts().at_parking_side;
        });

    net_mgr->connect();

    if (net_mgr->is_connected()) {
        tb_control->request_shared_attributes_snapshot();
        if (cfg.last_load_used_backup()) {
            tb_control->publish_backup_fallback_event();
        }
        tb_control->publish_startup_attributes();
        log->info("[Main] 设备静态属性已发布至云端");
    }

    // ── 限位开关防抖事件订阅：LimitSettledEvent → FSM ──────────────────
    // SafetyMonitor::monitor_loop (SCHED_FIFO 94) 在急停后延迟 180ms 发布此事件。
    // 回调在 monitor_loop 线程上执行，必须极短（不阻塞，不持锁调 I/O）。
    event_bus.subscribe<robot::middleware::SafetyMonitor::LimitSettledEvent>(
        [&fsm, &log, &tb_cfg](const robot::middleware::SafetyMonitor::LimitSettledEvent& evt) {
            const bool parking_left =
                tb_cfg->active_config().parking_side == robot::service::ParkingSide::Left;
            const bool parking_side_hit =
                (parking_left && evt.side == robot::device::LimitSide::LEFT) ||
                (!parking_left && evt.side == robot::device::LimitSide::RIGHT);

            if (parking_side_hit) {
                log->info("[Limit] 停机侧防抖完成，调度 EvParkingSideLimitSettled");
                fsm->dispatch(robot::app::EvParkingSideLimitSettled{});
            } else {
                log->info("[Limit] 对侧防抖完成，调度 EvFarEndLimitSettled");
                fsm->dispatch(robot::app::EvFarEndLimitSettled{});
            }
        });

    // ── 调度服务：定时触发清扫任务（读取 config.json 的 scheduler.windows） ─────
    scheduler.set_on_window_hit(
        [current_start_parking_facts, &log, &supervisor]() {
            const auto facts = current_start_parking_facts();
            if (!supervisor->start_task(facts.at_parking_side)) {
                log->warn("[Main] 调度启动被拒绝");
            }
        });

    robot::app::FaultHandler fault_handler(
        motion, event_bus, [fsm](const robot::service::FaultService::FaultEvent& evt) {
            using Level = robot::service::FaultService::FaultEvent::Level;
            if (evt.level == Level::P0)
                fsm->dispatch(robot::app::EvFaultP0{});
            else if (evt.level == Level::P1)
                fsm->dispatch(robot::app::EvFaultP1{});
            else if (evt.level == Level::P2)
                fsm->dispatch(robot::app::EvFaultP2{});
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
    // HealthService 已内嵌 DiagnosticsCollector 逻辑：
    //   production  -> Mode::HEALTH      （精简状态字段）
    //   development -> Mode::DIAGNOSTICS （完整诊断字段）
    // local_log_path 非空时额外把每帧 JSON payload 以 JSONL 追加到本地文件，
    // 离线调试阶段可直接 cat/grep 查看，完全独立于 MQTT/LoRaWAN。
    std::string local_tel_path = cfg.get<std::string>(
        "diagnostics.local_log_path",
        cfg.get<std::string>("diagnostics.local_path", ""));
    const size_t local_log_max_bytes = static_cast<size_t>(std::max(
        1, cfg.get<int>("diagnostics.local_log_max_bytes", 10 * 1024 * 1024)));
    const size_t local_log_max_files =
        static_cast<size_t>(std::max(1, cfg.get<int>("diagnostics.local_log_max_files", 3)));
    auto reporter = std::make_shared<robot::service::HealthService>(
        walk_group,
        brush_motor,
        bms,
        imu,
        gps,
        cloud,
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
    // 50Hz PID 与 100Hz IMU 组合，采样/控制比 2:1，路align 速度 1.5m/s 下 20ms 塔偏跨度 ≤ 30mm
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
    int report_period = cfg.get<int>("diagnostics.publish_interval_ms", 1000);
    const int active_report_period = std::max(
        1, cfg.get<int>("diagnostics.publish_interval_active_ms", report_period));
    const int idle_report_period =
        std::max(1, cfg.get<int>("diagnostics.publish_interval_idle_ms", 300000));
    robot::middleware::ThreadExecutor cloud_exec({"cloud", report_period, SCHED_OTHER, 0, 0x0F});
    cloud_exec.add_runnable(reporter);
    cloud_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>([tb_control]() {
        tb_control->publish_business_telemetry();
    }));
    cloud_exec.add_runnable(cloud);
    int cloud_wd = watchdog.register_thread("cloud", 5000);
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

        const std::string current_state = supervisor->current_state();
        const bool active_task_state = current_state == "CleanFwd" ||
                                       current_state == "CleanReturn" ||
                                       current_state == "Returning" ||
                                       current_state == "Paused";
        const int desired_report_period =
            active_task_state ? active_report_period : idle_report_period;
        if (cloud_exec.period_ms() != desired_report_period) {
            cloud_exec.set_period_ms(desired_report_period);
        }
        supervisor->tick_safety(bms->is_low_battery());

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
    walk_group->close();
    net_mgr->disconnect();
    data_cache->close();
    log->info("[Main] 正常退出");
    return 0;
}
