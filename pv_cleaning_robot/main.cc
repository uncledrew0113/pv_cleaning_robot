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
        cfg.get<uint16_t>("can.walk_motor.comm_timeout_ms", 200u));

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
    const bool front_open_ok = front_switch->open(95,  // SCHED_FIFO 95: GPIO 边缘最高硬件响应优先级
                                                  2,        // 2ms 软件消抖
                                                  1 << 4);  // 绑定大核 CPU 4（安全关键专用）
    const bool rear_open_ok =
        rear_switch->open(95,
                          2,
                          1 << 4);  // 绑定大核 CPU 4（与 safety_monitor 同核，减少跨核缓存失效）
    if (!front_open_ok)
        log->warn("[Main] 前限位开关初始化失败");
    if (!rear_open_ok)
        log->warn("[Main] 后限位开关初始化失败");

    // ── 上电自检：确认设备在停机位（尾0前1）────────────────────────────────────
    // 感应式接近开关：接近边框时输出低电平（ read_value()=false=0 ）
    //   尾部 false（低/0） = 在停机位边框内  → 正常
    //   前部 true （高/1） = 前方无遮挡       → 正常
    const bool rear_at_home = rear_open_ok && !rear_switch->read_current_level();
    const bool front_clear = front_open_ok && front_switch->read_current_level();
    if (!rear_at_home) {
        log->warn(
            "[Main] [自检警告] 尾部限位开关读数不为 0，"
            "设备可能不在停机位，请手动归位再启动");
    }
    if (!front_clear) {
        log->warn(
            "[Main] [自检警告] 前部限位开关读数不为 1，"
            "前方有遮挡或传感器异常，请检查环境");
    }
    log->info("[Main] 上电自检：尾部={}（期望 0/false），前部={}（期望 1/true）",
              rear_at_home ? "OK(0)" : "FAIL(1)",
              front_clear ? "OK(1)" : "FAIL(0)");
    if (!brush_motor->open())
        log->warn("[Main] 辊刷电机 RS485 初始化失败");
    if (bms->open() != robot::device::DeviceError::OK)
        log->warn("[Main] BMS RS485 初始化失败");

    // ── 10. 安全监控器 ─────────────────────────────────────────────────
    robot::middleware::SafetyMonitor safety_monitor(
        walk_group, front_switch, rear_switch, event_bus);
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
        cfg.get<std::string>("storage.cache_path", "/var/robot/telemetry_cache.jsonl"));
    data_cache->open();

    // ── 13. 服务层 ─────────────────────────────────────────────────────
    robot::service::MotionService::Config motion_cfg;
    motion_cfg.clean_speed_rpm = cfg.get<float>("robot.clean_speed_rpm", 300.0f);
    motion_cfg.return_speed_rpm = cfg.get<float>("robot.return_speed_rpm", 500.0f);
    motion_cfg.brush_rpm = cfg.get<int>("robot.brush_rpm", 1200);
    motion_cfg.edge_reverse_rpm = cfg.get<float>("robot.edge_reverse_rpm", 0.0f);
    motion_cfg.heading_pid_en = cfg.get<bool>("robot.heading_pid_en", true);

    auto motion = std::make_shared<robot::service::MotionService>(
        walk_group, brush_motor, imu, event_bus, motion_cfg);
    auto nav = std::make_shared<robot::service::NavService>(walk_group, imu, gps);
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

    // ── 限位开关事件订阅：将 LimitTriggerEvent 映射到 FSM 事件 ──────────
    // 此订阅由 EventBus 在 GPIO 监控线程中调用，必须极短不得阐塞。
    //   FRONT 下降沿 → EvReachEnd  → FSM 进入 CleanReturn，运动服务反向
    //   REAR  下降沿 → EvReachHome → FSM 进入 CleanFwd/Charging，运动服务停止或再前进
    event_bus.subscribe<robot::middleware::SafetyMonitor::LimitTriggerEvent>(
        [&fsm, &log](const robot::middleware::SafetyMonitor::LimitTriggerEvent& evt) {
            if (evt.side == robot::device::LimitSide::FRONT) {
                log->info("[Limit] 前端到达，调度 EvReachEnd");
                fsm->dispatch(robot::app::EvReachEnd{});
            } else {
                log->info("[Limit] 尾端到达停机位，调度 EvReachHome");
                fsm->dispatch(robot::app::EvReachHome{});
            }
        });

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
    // HealthService 已内嵌 DiagnosticsCollector 逻辑：
    //   production  -> Mode::HEALTH      （精简状态字段）
    //   development -> Mode::DIAGNOSTICS （完整诊断字段）
    auto reporter = std::make_shared<robot::service::HealthService>(
        walk_group,
        brush_motor,
        bms,
        imu,
        gps,
        cloud,
        diag_mode == "development" ? robot::service::HealthService::Mode::DIAGNOSTICS
                                   : robot::service::HealthService::Mode::HEALTH);

    // ── 16. 线程执行器 ────────────────────────────────────────────────
    // ── RK3576 CPU 拓扑 ───────────────────────────────────────────────
    //   CPU 0-3: Cortex-A55 (LITTLE 核)  → 非 RT 后台任务
    //   CPU 4-7: Cortex-A76 (BIG 核)     → RT 任务专用
    //
    // 线程绑定策略（与 safety_monitor/gpio 线程配合，不竞争同一核心）：
    //   CPU 4: gpio_front/rear(95) + safety_monitor(94)  → GPIO 边沿优先，轮询兜底
    //   CPU 5: group_recv(82) + walk_ctrl(80)            → 先接收后控制，降低读旧数据概率
    //   CPU 6: nav(65) + imu_read(68)                    → 导航路径，专用核
    //   CPU 7: watchdog(50) + main(SCHED_OTHER)          → 管理任务
    //   CPU 0-3: bms + cloud + gps（LITTLE 核，低功耗后台）
    //
    // 行走控制线程：SCHED_FIFO 80, 20ms (50Hz)，绑定 CPU 5
    // 50Hz PID 与 100Hz IMU 组合，采样/控制比 2:1，路align 速度 1.5m/s 下 20ms 塔偏跨度 ≤ 30mm
    robot::middleware::ThreadExecutor walk_exec({"walk_ctrl", 20, SCHED_FIFO, 80, 1 << 5});
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
    int bms_wd = watchdog.register_thread("bms", 2000);
    bms_exec.add_runnable(std::make_shared<robot::middleware::RunnableAdapter>(
        [&watchdog, bms_wd]() { watchdog.heartbeat(bms_wd); }));

    // 云端上报线程：绑定 LITTLE 核 CPU 0-3（低功耗后台）
    int report_period = cfg.get<int>("diagnostics.publish_interval_ms", 1000);
    robot::middleware::ThreadExecutor cloud_exec({"cloud", report_period, SCHED_OTHER, 0, 0x0F});
    cloud_exec.add_runnable(reporter);
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
        // BMS 低电量检测
        if (bms->is_low_battery() && fsm->current_state() != "Returning" &&
            fsm->current_state() != "Charging") {
            log->warn("[Main] 电量不足，触发返回");
            fsm->dispatch(robot::app::EvLowBattery{});
        }

        // 悬空检测：轮子空转 > 500ms 触发 P0 级故障
        // 排除 Idle/Charging（静止状态）和 Fault（已居存故障，避免重复上报）
        const auto cur_state = fsm->current_state();
        if (cur_state != "Idle" && cur_state != "Charging" && cur_state != "Fault") {
            if (nav->get_pose().spin_free_detected) {
                log->error("[Main] 悬空检测触发——立即停机");
                fault->report(robot::service::FaultService::FaultEvent::Level::P0,
                              0x0002, "wheel spin-free detected");
                nav->clear_spin_detection();  // 仅清除计数器，不重置里程计
            }
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
    walk_group->close();
    net_mgr->disconnect();
    data_cache->close();
    log->info("[Main] 正常退出");
    return 0;
}
