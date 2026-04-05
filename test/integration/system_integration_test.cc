/**
 * 系统集成测试（类 main 全链路）
 * [integration][system]
 *
 * 覆盖场景：
 *   TC1  — ConfigService 加载 + 字段验证
 *   TC2  — Logger 初始化
 *   TC3  — 系统启动 FSM 进入 Idle
 *   TC4  — HealthService DIAGNOSTICS 模式 JSONL 本地落盘（含 JSON 合法性校验）
 *   TC5  — WatchdogMgr 正常心跳——不触发超时
 *   TC6  — WatchdogMgr 超时检测——漏心跳后回调触发
 *   TC7  — SafetyMonitor GPIO → emergency_override → LimitSettledEvent → FSM 转换
 *   TC8  — P0 故障链（FaultService → FaultHandler → FSM Fault → Reset → Idle）
 *   TC9  — P1 故障链 + EvLowBattery（独立场景）
 *   TC10 — N=1 完整任务链（ThreadExecutor-driven MotionService 心跳帧验证）
 *
 * 设计约束：
 *   - 全部 I/O 经 Mock（MockCanBus / MockGpioPin / MockModbusMaster / MockSerialPort）
 *   - NullTransport 替代 MQTT/LoRaWAN，不建立真实网络连接
 *   - WatchdogMgr 路径传空字符串，不需要 /dev/watchdog
 *   - 不验证实时时序，仅验证状态和帧存在性
 */
#include <catch2/catch.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_gpio_pin.h"
#include "../mock/mock_modbus_master.h"
#include "../mock/mock_serial_port.h"

#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/logger.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

using namespace robot::app;
using robot::device::BMS;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::LimitSide;
using robot::device::LimitSwitch;
using robot::device::WalkMotorGroup;
using robot::middleware::DataCache;
using robot::middleware::EventBus;
using robot::middleware::NetworkManager;
using robot::middleware::SafetyMonitor;
using robot::middleware::ThreadExecutor;
using robot::service::CloudService;
using robot::service::ConfigService;
using robot::service::FaultService;
using robot::service::HealthService;
using robot::service::MotionService;
using robot::service::NavService;
using FaultEvent = FaultService::FaultEvent;
using FaultLevel = FaultEvent::Level;

// ─────────────────────────────────────────────────────────────────────────────
// 辅助常量
// ─────────────────────────────────────────────────────────────────────────────
static const char* kCfgPath      = "/tmp/pv_sys_test_config.json";
static const char* kHealthPath   = "/tmp/pv_sys_test_health.jsonl";
static const char* kCachePath    = "/tmp/pv_sys_test_cache.jsonl";

// ─────────────────────────────────────────────────────────────────────────────
// NullTransport：INetworkTransport 空实现，不建立任何真实网络连接
// ─────────────────────────────────────────────────────────────────────────────
struct NullTransport : robot::middleware::INetworkTransport {
    bool connect()    override { return false; }
    void disconnect() override {}
    bool is_connected() const override { return false; }
    bool publish(const std::string&, const std::string&) override { return false; }
    bool subscribe(const std::string&, MessageCallback) override { return false; }
};

// ─────────────────────────────────────────────────────────────────────────────
// 测试专用 config.json（写入临时文件，ConfigService 从此文件加载）
// ─────────────────────────────────────────────────────────────────────────────
static void write_test_config() {
    nlohmann::json cfg = {
        {"logging",     {{"log_dir", "/tmp/pv_sys_test_logs"}, {"level", "debug"}, {"console", false}}},
        {"robot",       {{"clean_speed_rpm", 30.0f}, {"return_speed_rpm", 30.0f},
                         {"brush_rpm", 1000}, {"return_brush_rpm", 1000},
                         {"heading_pid_en", false}, {"edge_reverse_rpm", 0.0f},
                         {"battery_full_soc", 95.0f}, {"battery_low_soc", 15.0f},
                         {"wheel_circ_m", 0.3f}, {"track_length_m", 100.0f}, {"passes", 1.0f}}},
        {"diagnostics", {{"mode", "development"}, {"local_path", kHealthPath},
                         {"cloud_upload", false}, {"local_log", true},
                         {"publish_interval_ms", 1000}}},
        {"storage",     {{"cache_path", kCachePath}}},
        {"system",      {{"hw_watchdog", ""}}},
        {"can",         {{"interface", "can0"},
                         {"walk_motor", {{"motor_id", 1}, {"comm_timeout_ms", 200}}}}},
        {"gpio",        {{"front_limit", {{"chip", "gpiochip5"}, {"line", 0}}},
                         {"rear_limit",  {{"chip", "gpiochip5"}, {"line", 1}}}}}
    };
    std::filesystem::create_directories("/tmp");
    std::ofstream f(kCfgPath);
    f << cfg.dump(2);
}

// ─────────────────────────────────────────────────────────────────────────────
// SystemFixture：组装完整系统（类 main），全部层次均使用真实类 + Mock 硬件
// ─────────────────────────────────────────────────────────────────────────────
struct SystemFixture {
    // ── Mock 硬件层 ────────────────────────────────────────────────
    std::shared_ptr<MockCanBus>         can      {std::make_shared<MockCanBus>()};
    std::shared_ptr<MockModbusMaster>   modbus   {std::make_shared<MockModbusMaster>()};
    std::shared_ptr<MockSerialPort>     imu_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort>     gps_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort>     bms_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockGpioPin>        front_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin>        rear_pin {std::make_shared<MockGpioPin>()};

    // ── 设备层 ────────────────────────────────────────────────────
    std::shared_ptr<WalkMotorGroup>     group  {std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<BrushMotor>         brush  {std::make_shared<BrushMotor>(modbus, 1)};
    std::shared_ptr<ImuDevice>          imu    {std::make_shared<ImuDevice>(imu_sp)};
    std::shared_ptr<GpsDevice>          gps    {std::make_shared<GpsDevice>(gps_sp)};
    std::shared_ptr<BMS>                bms    {std::make_shared<BMS>(bms_sp, 95.0f, 15.0f)};
    std::shared_ptr<LimitSwitch>        front_sw
        {std::make_shared<LimitSwitch>(front_pin, LimitSide::FRONT)};
    std::shared_ptr<LimitSwitch>        rear_sw
        {std::make_shared<LimitSwitch>(rear_pin, LimitSide::REAR)};

    // ── 配置层 ────────────────────────────────────────────────────
    ConfigService cfg{kCfgPath};

    // ── 中间件层 ──────────────────────────────────────────────────
    EventBus bus;
    std::shared_ptr<DataCache> cache
        {std::make_shared<DataCache>(kCachePath)};
    std::shared_ptr<NetworkManager> net_mgr
        {std::make_shared<NetworkManager>(
            std::make_shared<NullTransport>(),
            std::make_shared<NullTransport>(),
            NetworkManager::Mode::MQTT_ONLY)};

    // ── 服务层 ────────────────────────────────────────────────────
    std::shared_ptr<CloudService> cloud
        {std::make_shared<CloudService>(net_mgr, cache)};
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService>    nav;
    std::shared_ptr<FaultService>  fault_svc{std::make_shared<FaultService>(bus)};
    std::shared_ptr<HealthService> health;

    // ── App 层 ────────────────────────────────────────────────────
    WatchdogMgr  watchdog{""};   // 空路径 = 不打开 /dev/watchdog
    SafetyMonitor safety_mon;
    RobotFsm     fsm;
    std::vector<FaultEvent> dispatched_faults;
    FaultHandler fault_handler;

    // ─────────────────────────────────────────────────────────────
    SystemFixture()
        : motion(std::make_shared<MotionService>(
              group, brush, nullptr, bus,
              MotionService::Config{
                  .clean_speed_rpm  = 30.0f,
                  .return_speed_rpm = 30.0f,
                  .brush_rpm        = 1000,
                  .return_brush_rpm = 1000,
                  .edge_reverse_rpm = 0.0f,
                  .heading_pid_en   = false}))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , health(std::make_shared<HealthService>(
              group, brush, bms, imu, gps,
              cloud,
              HealthService::Mode::DIAGNOSTICS,
              kHealthPath))
        , safety_mon(group, front_sw, rear_sw, bus)
        , fsm(motion, nav, fault_svc, bus)
        , fault_handler(motion, bus, [this](FaultEvent e) {
              dispatched_faults.push_back(e);
              if (e.level == FaultLevel::P0)
                  fsm.dispatch(EvFaultP0{});
              else if (e.level == FaultLevel::P1)
                  fsm.dispatch(EvFaultP1{});
          })
    {
        // 配置 Mock 返回值
        can->open_result     = true;
        can->send_result     = true;
        can->opened          = true;   // 允许 send_ctrl() is_open() 检查通过
        modbus->open_result  = true;
        modbus->write_reg_return = 0;
        front_pin->open_result = true;
        rear_pin->open_result  = true;

        // 打开设备（仅必要设备）
        cfg.load();
        brush->open();
        front_sw->open();
        rear_sw->open();

        // 预先清空落盘文件
        std::filesystem::remove(kHealthPath);
        std::filesystem::remove(kCachePath);

        fault_handler.start_listening();
        fsm.dispatch(EvInitDone{});
    }
};

// =============================================================================
// TC1：ConfigService 加载与字段验证
// =============================================================================
TEST_CASE("System: ConfigService 加载测试配置并正确返回字段值",
          "[integration][system][config]") {
    write_test_config();
    ConfigService cfg{kCfgPath};
    REQUIRE(cfg.load());
    REQUIRE(cfg.is_loaded());

    REQUIRE(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(30.0f));
    REQUIRE(cfg.get<float>("robot.battery_full_soc", 0.0f) == Approx(95.0f));
    REQUIRE(cfg.get<float>("robot.battery_low_soc", 0.0f)  == Approx(15.0f));
    REQUIRE(cfg.get<bool>("robot.heading_pid_en", true) == false);
    REQUIRE(cfg.get<std::string>("diagnostics.mode", "") == "development");
    REQUIRE(cfg.get<std::string>("diagnostics.local_path", "") == kHealthPath);
    REQUIRE(cfg.get<std::string>("system.hw_watchdog", "x") == "");
    REQUIRE(cfg.get<int>("can.walk_motor.motor_id", 0) == 1);
    REQUIRE(cfg.get<int>("can.walk_motor.comm_timeout_ms", 0) == 200);
}

// =============================================================================
// TC2：Logger 初始化
// =============================================================================
TEST_CASE("System: Logger 初始化后可获取合法 spdlog 实例",
          "[integration][system][logger]") {
    robot::middleware::Logger::init({
        .log_dir        = "/tmp/pv_sys_test_logs",
        .file_name      = "sys_test",
        .console_output = false,
        .level          = "debug"
    });
    auto logger = robot::middleware::Logger::get();
    REQUIRE(logger != nullptr);
    // 验证日志调用不崩溃
    logger->info("[system_test] Logger 初始化正常");
    logger->debug("[system_test] debug 级别日志");
}

// =============================================================================
// TC3：系统启动序列——FSM 进入 Idle
// =============================================================================
TEST_CASE("System: 系统启动后 FSM 处于 Idle 状态",
          "[integration][system][fsm]") {
    write_test_config();
    SystemFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");
}

// =============================================================================
// TC4：HealthService DIAGNOSTICS JSONL 本地落盘
// =============================================================================
TEST_CASE("System: HealthService DIAGNOSTICS 模式落盘 JSONL 文件并验证 JSON 合法性",
          "[integration][system][health]") {
    write_test_config();
    std::filesystem::remove(kHealthPath);

    SystemFixture f;
    // 连续调用 3 次 update，期望产生 3 行 JSONL
    f.health->update();
    f.health->update();
    f.health->update();

    // 文件必须存在
    REQUIRE(std::filesystem::exists(kHealthPath));

    // 读取行数并验证每行是合法 JSON
    std::ifstream ifs(kHealthPath);
    REQUIRE(ifs.is_open());

    int line_count = 0;
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        ++line_count;
        nlohmann::json j;
        REQUIRE_NOTHROW(j = nlohmann::json::parse(line));

        // DIAGNOSTICS 模式必须包含所有顶级键
        REQUIRE(j.contains("walk"));
        REQUIRE(j.contains("brush"));
        REQUIRE(j.contains("bms"));
        REQUIRE(j.contains("imu"));
        REQUIRE(j.contains("gps"));

        // walk 子键：lt/rt/lb/rb 每轮独立诊断 + ctrl_frames
        REQUIRE(j["walk"].contains("lt"));
        REQUIRE(j["walk"].contains("rt"));
        REQUIRE(j["walk"].contains("lb"));
        REQUIRE(j["walk"].contains("rb"));
        REQUIRE(j["walk"].contains("ctrl_frames"));

        // bms 子键
        REQUIRE(j["bms"].contains("soc"));
        REQUIRE(j["bms"].contains("voltage"));
    }
    REQUIRE(line_count == 3);

    std::filesystem::remove(kHealthPath);
}

// =============================================================================
// TC5：WatchdogMgr 正常心跳——不触发超时
// =============================================================================
TEST_CASE("System: WatchdogMgr 正常心跳时不触发超时回调",
          "[integration][system][watchdog]") {
    WatchdogMgr watchdog{""};    // 不需要真实 /dev/watchdog

    std::atomic<bool> timeout_fired{false};
    watchdog.set_timeout_callback([&](const std::string&) {
        timeout_fired.store(true);
    });

    REQUIRE(watchdog.start());
    int tid = watchdog.register_thread("ctrl_normal", 500);  // 500ms 超时

    // 每 80ms 喂狗，共 5 次 = 400ms < 500ms 超时
    for (int i = 0; i < 5; ++i) {
        watchdog.heartbeat(tid);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    watchdog.stop();
    REQUIRE(timeout_fired.load() == false);
}

// =============================================================================
// TC6：WatchdogMgr 超时检测——漏心跳后回调触发
// =============================================================================
TEST_CASE("System: WatchdogMgr 超时后回调触发并包含正确线程名",
          "[integration][system][watchdog]") {
    WatchdogMgr watchdog{""};

    std::string fired_name;
    watchdog.set_timeout_callback([&](const std::string& name) {
        fired_name = name;
    });

    REQUIRE(watchdog.start());
    // 注册 50ms 超时但从不喂狗
    watchdog.register_thread("dead_thread", 50);

    // 等待 350ms，远超 50ms 超时 + 200ms watchdog 监控周期
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    watchdog.stop();

    REQUIRE(fired_name == "dead_thread");
}

// =============================================================================
// TC7：SafetyMonitor GPIO → emergency_override → LimitSettledEvent → FSM
// =============================================================================
TEST_CASE("System: SafetyMonitor 前端 GPIO 触发 → emergency_override → FSM CleanReturn",
          "[integration][system][safety]") {
    write_test_config();
    SystemFixture f;

    // 启动任务，进入 CleanFwd
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 记录触发前 CAN 帧数
    const size_t frames_before = f.can->sent_frames.size();

    // 订阅 LimitSettledEvent → 转发给 FSM（模拟 main.cc 中的 EventBus 桥接）
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::FRONT)
                f.fsm.dispatch(EvFrontLimitSettled{});
            else
                f.fsm.dispatch(EvRearLimitSettled{});
        });

    f.safety_mon.start();

    // 模拟前端 GPIO 边沿触发
    if (f.front_pin->registered_cb) {
        f.front_pin->simulate_edge();
    }

    // 等待 SafetyMonitor monitor_loop 180ms 防抖完成后发布 LimitSettledEvent
    std::this_thread::sleep_for(std::chrono::milliseconds(280));
    f.safety_mon.stop();

    // emergency_override(0.0f) 应向 CAN 总线发送零速停车帧
    REQUIRE(f.can->sent_frames.size() > frames_before);

    // FSM 已转换 → CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");
}

// =============================================================================
// TC8：P0 故障链——FaultService → FaultHandler → FSM Fault → Reset → Idle
// =============================================================================
TEST_CASE("System: P0 故障链 FaultService→FaultHandler→FSM Fault，复位后回 Idle",
          "[integration][system][fault]") {
    write_test_config();
    SystemFixture f;

    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 通过 FaultService 上报 P0（FaultHandler → dispatch EvFaultP0 → FSM Fault）
    f.fault_svc->report(FaultLevel::P0, 0x1001, "CAN_comm_lost");
    REQUIRE(f.fsm.current_state() == "Fault");

    // FaultHandler 应记录到 dispatched_faults
    REQUIRE(!f.dispatched_faults.empty());
    REQUIRE(f.dispatched_faults[0].level == FaultLevel::P0);
    REQUIRE(f.dispatched_faults[0].code  == 0x1001u);

    // 人工复位 → Idle
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");

    // 复位后可重新启动任务
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// =============================================================================
// TC9-A：P1 故障链——CleanReturn → Returning → Charging
// =============================================================================
TEST_CASE("System: P1 故障发生在清扫中 → Returning → Charging",
          "[integration][system][fault]") {
    write_test_config();
    SystemFixture f;

    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    f.fsm.dispatch(EvFrontLimitSettled{});  // → CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fault_svc->report(FaultLevel::P1, 0x2001, "slope_too_steep");
    REQUIRE(f.fsm.current_state() == "Returning");

    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// =============================================================================
// TC9-B：低电量触发 Returning 路径
// =============================================================================
TEST_CASE("System: EvLowBattery 在 CleanFwd 触发 → Returning → Charging",
          "[integration][system][fault]") {
    write_test_config();
    SystemFixture f;

    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.fsm.dispatch(EvLowBattery{});
    REQUIRE(f.fsm.current_state() == "Returning");

    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// =============================================================================
// TC10：N=1 完整任务链 + ThreadExecutor-driven MotionService 心跳帧验证
// =============================================================================
TEST_CASE("System: N=1 完整任务链 + ThreadExecutor 驱动 MotionService 产生心跳帧",
          "[integration][system][full_chain]") {
    write_test_config();
    SystemFixture f;

    // 启动 ThreadExecutor（SCHED_OTHER，不需 root；50ms 周期）
    ThreadExecutor exec({
        .name          = "test_ctrl",
        .period_ms     = 50,
        .sched_policy  = 0,
        .sched_priority = 0,
        .cpu_affinity  = 0
    });
    exec.add_runnable(f.motion);
    REQUIRE(exec.start());

    // N=1 往返任务链：CleanFwd → CleanReturn → Charging
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE(!f.can->sent_frames.empty());   // start_cleaning() 已投帧

    // 等待 MotionService update() 至少运行 2 个周期（100ms），产生心跳帧
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    const size_t frames_after_start = f.can->sent_frames.size();

    // 前端限位 → CleanReturn
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // 等待 start_returning() 心跳帧
    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    // 尾端限位 → Charging（N=1 完成）
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");

    exec.stop();

    // stop_cleaning() 后 has_ctrl_frame_=false，心跳帧停止，但之前应已产生足够帧
    REQUIRE(f.can->sent_frames.size() > frames_after_start);
}

// =============================================================================
// TC11：SafetyMonitor + HealthService + WatchdogMgr 联合启动——无崩溃
// =============================================================================
TEST_CASE("System: SafetyMonitor + HealthService + WatchdogMgr 联合启动 300ms 无崩溃",
          "[integration][system][combined]") {
    write_test_config();
    SystemFixture f;

    // 启动看门狗（线程化）
    int tid = -1;
    f.watchdog.set_timeout_callback([](const std::string& n) {
        spdlog::warn("[system_test] watchdog timeout: {}", n);
    });
    REQUIRE(f.watchdog.start());
    tid = f.watchdog.register_thread("test_combined", 2000);  // 2s 超时，足够测试用

    // 启动安全监控
    REQUIRE(f.safety_mon.start());

    // FSM 进入 CleanFwd，MotionService 开始发帧
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // HealthService 落盘 3 次
    f.health->update();
    f.watchdog.heartbeat(tid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    f.health->update();
    f.watchdog.heartbeat(tid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    f.health->update();
    f.watchdog.heartbeat(tid);

    f.safety_mon.stop();
    f.watchdog.stop();

    // 验证 JSONL 有内容
    REQUIRE(std::filesystem::exists(kHealthPath));

    // 验证 CAN 总线收到帧（MotionService 已调 set_speeds）
    REQUIRE(!f.can->sent_frames.empty());

    std::filesystem::remove(kHealthPath);
}
