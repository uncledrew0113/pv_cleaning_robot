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
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <string>
#include <thread>
#include <spdlog/spdlog.h>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_gpio_pin.h"
#include "../mock/mock_modbus_master.h"
#include "../mock/mock_serial_port.h"
#include "integration/thingsboard_test_support.h"

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

namespace {

namespace fs = std::filesystem;

rapidjson::Document parse_json_line(const std::string& line)
{
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

MotionService::Config make_motion_config()
{
    MotionService::Config cfg;
    cfg.clean_speed_rpm = 30.0f;
    cfg.return_speed_rpm = 30.0f;
    cfg.brush_rpm = 1000;
    cfg.return_brush_rpm = 1000;
    cfg.edge_reverse_rpm = 0.0f;
    cfg.heading_pid_en = false;
    return cfg;
}

EvScheduleStart make_schedule_start(bool at_parking_side, bool at_far_end, float passes)
{
    EvScheduleStart evt;
    evt.at_parking_side = at_parking_side;
    evt.at_far_end = at_far_end;
    evt.passes = passes;
    return evt;
}

EvScheduleStart start_from_parking_side(float passes)
{
    return make_schedule_start(true, false, passes);
}

ThreadExecutor::Config make_executor_config()
{
    ThreadExecutor::Config cfg;
    cfg.name = "test_ctrl";
    cfg.period_ms = 50;
    cfg.sched_policy = 0;
    cfg.sched_priority = 0;
    cfg.cpu_affinity = 0;
    return cfg;
}

robot::middleware::Logger::Config make_logger_config()
{
    robot::middleware::Logger::Config cfg;
    cfg.log_dir = "/tmp/pv_sys_test_logs";
    cfg.file_name = "sys_test";
    cfg.console_output = false;
    cfg.level = "debug";
    return cfg;
}

std::string build_test_runtime_config_json()
{
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();

    writer.Key("robot");
    writer.StartObject();
    writer.Key("clean_speed_rpm");
    writer.Double(30.0);
    writer.Key("return_speed_rpm");
    writer.Double(30.0);
    writer.Key("brush_rpm");
    writer.Int(1000);
    writer.Key("return_brush_rpm");
    writer.Int(1000);
    writer.Key("heading_pid_en");
    writer.Bool(false);
    writer.Key("edge_reverse_rpm");
    writer.Double(0.0);
    writer.Key("start_battery_soc");
    writer.Double(30.0);
    writer.Key("charge_start_soc");
    writer.Double(15.0);
    writer.Key("charge_stop_soc");
    writer.Double(95.0);
    writer.Key("wheel_circ_m");
    writer.Double(0.3);
    writer.Key("track_length_m");
    writer.Double(100.0);
    writer.Key("passes");
    writer.Double(1.0);
    writer.Key("parking_side");
    writer.String("left");
    writer.EndObject();

    writer.Key("scheduler");
    writer.StartObject();
    writer.Key("windows");
    writer.StartArray();
    writer.StartObject();
    writer.Key("hour");
    writer.Int(8);
    writer.Key("minute");
    writer.Int(0);
    writer.EndObject();
    writer.EndArray();
    writer.EndObject();

    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_test_fixed_config_json(const std::string& health_path,
                                         const std::string& cache_path)
{
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();

    writer.Key("logging");
    writer.StartObject();
    writer.Key("log_dir");
    writer.String("/tmp/pv_sys_test_logs");
    writer.Key("level");
    writer.String("debug");
    writer.Key("console");
    writer.Bool(false);
    writer.EndObject();

    writer.Key("diagnostics");
    writer.StartObject();
    writer.Key("mode");
    writer.String("development");
    writer.Key("local_path");
    writer.String(health_path.c_str());
    writer.Key("cloud_upload");
    writer.Bool(false);
    writer.Key("local_log");
    writer.Bool(true);
    writer.Key("publish_interval_ms");
    writer.Int(1000);
    writer.Key("publish_interval_active_ms");
    writer.Int(1000);
    writer.Key("publish_interval_idle_ms");
    writer.Int(300000);
    writer.EndObject();

    writer.Key("storage");
    writer.StartObject();
    writer.Key("cache_path");
    writer.String(cache_path.c_str());
    writer.EndObject();

    writer.Key("system");
    writer.StartObject();
    writer.Key("hw_watchdog");
    writer.String("");
    writer.EndObject();

    writer.Key("device");
    writer.StartObject();
    writer.Key("software_version");
    writer.String("1.0.0");
    writer.Key("hardware_version");
    writer.String("1.0");
    writer.Key("model");
    writer.String("pv_cleaning_robot");
    writer.EndObject();

    writer.Key("can");
    writer.StartObject();
    writer.Key("interface");
    writer.String("can0");
    writer.Key("walk_motor");
    writer.StartObject();
    writer.Key("motor_id");
    writer.Int(1);
    writer.Key("comm_timeout_ms");
    writer.Int(200);
    writer.EndObject();
    writer.EndObject();

    writer.Key("gpio");
    writer.StartObject();
    writer.Key("left_limit");
    writer.StartObject();
    writer.Key("chip");
    writer.String("gpiochip5");
    writer.Key("line");
    writer.Int(0);
    writer.EndObject();
    writer.Key("right_limit");
    writer.StartObject();
    writer.Key("chip");
    writer.String("gpiochip5");
    writer.Key("line");
    writer.Int(1);
    writer.EndObject();
    writer.EndObject();

    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

struct SystemTestFiles {
    tb_test_support::TempSplitConfigPaths split_config{
        tb_test_support::make_temp_split_config_paths("pv_sys_test_config")};
    fs::path health_path{"/tmp/pv_sys_test_health.jsonl"};
    fs::path log_dir{"/tmp/pv_sys_test_logs"};

    void write_config() const
    {
        tb_test_support::write_split_config(split_config,
                                            build_test_runtime_config_json(),
                                            build_test_fixed_config_json(health_path.string(),
                                                                         split_config.cache_path.string()));
    }

    void cleanup_runtime_files() const
    {
        fs::remove(health_path);
        fs::remove(split_config.cache_path);
    }

    void cleanup_all() const
    {
        cleanup_runtime_files();
        tb_test_support::cleanup_split_config_paths(split_config);
    }
};

const SystemTestFiles& system_test_files()
{
    static const SystemTestFiles files{};
    return files;
}

}  // namespace

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
// SystemFixture：组装完整系统（类 main），全部层次均使用真实类 + Mock 硬件
// ─────────────────────────────────────────────────────────────────────────────
struct SystemFixture {
    SystemTestFiles files{system_test_files()};

    // ── Mock 硬件层 ────────────────────────────────────────────────
    std::shared_ptr<MockCanBus>         can      {std::make_shared<MockCanBus>()};
    std::shared_ptr<MockSerialPort>     brush_sp {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort>     imu_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort>     gps_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort>     bms_sp   {std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockGpioPin>        left_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin>        right_pin {std::make_shared<MockGpioPin>()};

    // ── 设备层 ────────────────────────────────────────────────────
    std::shared_ptr<WalkMotorGroup>     group  {std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<BrushMotor>         brush  {std::make_shared<BrushMotor>(brush_sp, 0, 8192.0f, true, 0.5f)};
    std::shared_ptr<ImuDevice>          imu    {std::make_shared<ImuDevice>(imu_sp)};
    std::shared_ptr<GpsDevice>          gps    {std::make_shared<GpsDevice>(gps_sp)};
    std::shared_ptr<BMS>                bms    {std::make_shared<BMS>(bms_sp, 95.0f, 15.0f)};
    std::shared_ptr<LimitSwitch>        left_sw
        {std::make_shared<LimitSwitch>(left_pin, LimitSide::LEFT)};
    std::shared_ptr<LimitSwitch>        right_sw
        {std::make_shared<LimitSwitch>(right_pin, LimitSide::RIGHT)};

    // ── 配置层 ────────────────────────────────────────────────────
    ConfigService cfg{files.split_config.runtime_path.string(), files.split_config.fixed_path.string()};

    // ── 中间件层 ──────────────────────────────────────────────────
    EventBus bus;
    std::shared_ptr<DataCache> cache
        {std::make_shared<DataCache>(files.split_config.cache_path.string())};
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
              make_motion_config()))
        , nav(std::make_shared<NavService>(group, imu, gps))
        // 注意：health 不在此处初始化，延迟到构造体内，
        // 必须在清理 health JSONL 之后创建，
        // 否则 HealthService 打开文件后被 remove 删掉目录项，
        // 写入将走未链接的 inode，exists() 永远返回 false
        , safety_mon(group, left_sw, right_sw, bus)
        , fsm(motion, nav, fault_svc, bus)
        , fault_handler(motion, bus, [this](FaultEvent e) {
              dispatched_faults.push_back(e);
              if (e.level == FaultLevel::P0)
                  fsm.dispatch(EvFaultP0{});
              else if (e.level == FaultLevel::P1)
                  fsm.dispatch(EvFaultP1{});
          })
    {
        files.write_config();
        // 配置 Mock 返回值
        can->open_result     = true;
        can->send_result     = true;
        can->opened          = true;   // 允许 send_ctrl() is_open() 检查通过
        brush_sp->open_result = true;
        left_pin->open_result = true;
        right_pin->open_result  = true;

        // 打开设备（仅必要设备）
        cfg.load();
        brush->open();
        left_sw->open();
        right_sw->open();

        // 预先清空落盘文件（必须在 HealthService 构造之前执行！）
        files.cleanup_runtime_files();

        // HealthService 在文件清理之后才构造，保证打开的是新文件
        health = std::make_shared<HealthService>(
            group, brush, bms, imu, gps,
            cloud,
            HealthService::Mode::DIAGNOSTICS,
            files.health_path.string());

        fault_handler.start_listening();
        fsm.dispatch(EvInitDone{});
    }

    ~SystemFixture()
    {
        files.cleanup_all();
    }
};

// =============================================================================
// TC1：ConfigService 加载与字段验证
// =============================================================================
TEST_CASE("System: ConfigService 加载测试配置并正确返回字段值",
          "[integration][system][config]") {
    const auto& files = system_test_files();
    files.write_config();
    ConfigService cfg(files.split_config.runtime_path.string(),
                      files.split_config.fixed_path.string());
    REQUIRE(cfg.load());
    REQUIRE(cfg.is_loaded());

    REQUIRE(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(30.0f));
    REQUIRE(cfg.get<float>("robot.start_battery_soc", 0.0f) == Approx(30.0f));
    REQUIRE(cfg.get<float>("robot.charge_start_soc", 0.0f)  == Approx(15.0f));
    REQUIRE(cfg.get<float>("robot.charge_stop_soc", 0.0f)  == Approx(95.0f));
    REQUIRE(cfg.get<bool>("robot.heading_pid_en", true) == false);
    REQUIRE(cfg.get<std::string>("diagnostics.mode", "") == "development");
    REQUIRE(cfg.get<std::string>("diagnostics.local_path", "") == files.health_path.string());
    REQUIRE(cfg.get<std::string>("system.hw_watchdog", "x") == "");
    REQUIRE(cfg.get<int>("can.walk_motor.motor_id", 0) == 1);
    REQUIRE(cfg.get<int>("can.walk_motor.comm_timeout_ms", 0) == 200);
    files.cleanup_all();
}

// =============================================================================
// TC2：Logger 初始化
// =============================================================================
TEST_CASE("System: Logger 初始化后可获取合法 spdlog 实例",
          "[integration][system][logger]") {
    robot::middleware::Logger::init(make_logger_config());
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
    SystemFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");
}

// =============================================================================
// TC4：HealthService DIAGNOSTICS JSONL 本地落盘
// =============================================================================
TEST_CASE("System: HealthService DIAGNOSTICS 模式落盘 JSONL 文件并验证 JSON 合法性",
          "[integration][system][health]") {
    SystemFixture f;
    // 连续调用 3 次 update，期望产生 3 行 JSONL
    f.health->update();
    f.health->update();
    f.health->update();

    // 文件必须存在
    REQUIRE(std::filesystem::exists(f.files.health_path));

    // 读取行数并验证每行是合法 JSON
    std::ifstream ifs(f.files.health_path);
    REQUIRE(ifs.is_open());

    int line_count = 0;
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        ++line_count;
        auto j = parse_json_line(line);

        // DIAGNOSTICS 模式必须包含所有顶级键
        REQUIRE(j.HasMember("walk"));
        REQUIRE(j.HasMember("brush"));
        REQUIRE(j.HasMember("bms"));
        REQUIRE(j.HasMember("imu"));
        REQUIRE(j.HasMember("gps"));

        // walk 子键：lt/rt/lb/rb 每轮独立诊断 + ctrl_frames
        REQUIRE(j["walk"].HasMember("lt"));
        REQUIRE(j["walk"].HasMember("rt"));
        REQUIRE(j["walk"].HasMember("lb"));
        REQUIRE(j["walk"].HasMember("rb"));
        REQUIRE(j["walk"].HasMember("ctrl_frames"));

        // bms 子键
        REQUIRE(j["bms"].HasMember("soc"));
        REQUIRE(j["bms"].HasMember("voltage"));
    }
    REQUIRE(line_count == 3);

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
    SystemFixture f;

    // 启动任务，进入 CleanFwd
    f.fsm.dispatch(start_from_parking_side(2.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 记录触发前 CAN 帧数
    const size_t frames_before = f.can->sent_frames.size();

    // 订阅 LimitSettledEvent → 转发给 FSM（模拟 main.cc 中的 EventBus 桥接）
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::LEFT)
                f.fsm.dispatch(EvFarEndLimitSettled{});
            else
                f.fsm.dispatch(EvParkingSideLimitSettled{});
        });

    f.safety_mon.start();

    // 模拟前端 GPIO 边沿触发
    if (f.left_pin->registered_cb) {
        f.left_pin->simulate_edge();
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
    SystemFixture f;

    f.fsm.dispatch(start_from_parking_side(1.0f));
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
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// =============================================================================
// TC9-A：P1 故障链——CleanReturn → Fault
// =============================================================================
TEST_CASE("System: P1 故障发生在清扫中 → Fault",
          "[integration][system][fault]") {
    SystemFixture f;

    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});  // → CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fault_svc->report(FaultLevel::P1, 0x2001, "slope_too_steep");
    REQUIRE(f.fsm.current_state() == "Fault");
}

// =============================================================================
// TC10：N=1 完整任务链 + ThreadExecutor-driven MotionService 心跳帧验证
// =============================================================================
TEST_CASE("System: N=1 完整任务链 + ThreadExecutor 驱动 MotionService 产生心跳帧",
          "[integration][system][full_chain]") {
    SystemFixture f;

    // 启动 ThreadExecutor（SCHED_OTHER，不需 root；50ms 周期）
    ThreadExecutor exec(make_executor_config());
    exec.add_runnable(f.motion);
    REQUIRE(exec.start());

    // N=1 往返任务链：CleanFwd → CleanReturn → Charging
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE(!f.can->sent_frames.empty());   // start_cleaning() 已投帧

    // 等待 MotionService update() 至少运行 2 个周期（100ms），产生心跳帧
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    const size_t frames_after_start = f.can->sent_frames.size();

    // 对侧限位 → CleanReturn
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // 等待 start_returning() 心跳帧
    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    // 停机侧限位 → Charging（N=1 完成）
    f.fsm.dispatch(EvParkingSideLimitSettled{});
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
    f.fsm.dispatch(start_from_parking_side(1.0f));
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
    REQUIRE(std::filesystem::exists(f.files.health_path));

    // 验证 CAN 总线收到帧（MotionService 已调 set_speeds）
    REQUIRE(!f.can->sent_frames.empty());

}
