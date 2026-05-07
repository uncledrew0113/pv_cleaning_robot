#include <catch2/catch.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace fs = std::filesystem;

namespace {

bool real_tb_test_enabled()
{
    const char* value = std::getenv("TB_REAL_TEST");
    if (!value) {
        return false;
    }
    const std::string env_value(value);
    return !(env_value.empty() || env_value == "0" || env_value == "false" ||
             env_value == "FALSE");
}

struct RepoPaths {
    fs::path repo_root;
    fs::path config_path;
};

std::optional<RepoPaths> find_repo_paths()
{
    fs::path current = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        const auto candidate = current / "config" / "config.json";
        if (fs::exists(candidate)) {
            return RepoPaths{current, candidate};
        }
        if (!current.has_parent_path()) {
            break;
        }
        current = current.parent_path();
    }
    return std::nullopt;
}

fs::path resolve_repo_relative(const fs::path& repo_root, const std::string& path)
{
    if (path.empty()) {
        return {};
    }
    fs::path p(path);
    if (p.is_absolute()) {
        return p;
    }
    return repo_root / p;
}

robot::middleware::MqttTransport::Config build_mqtt_config(
    robot::service::ConfigService& cfg, const fs::path& repo_root)
{
    robot::middleware::MqttTransport::Config mqtt_cfg;
    mqtt_cfg.broker_uri = cfg.get<std::string>("network.mqtt.broker_uri", "");
    mqtt_cfg.client_id = cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001");
    mqtt_cfg.username = cfg.get<std::string>("network.mqtt.username", "");
    mqtt_cfg.password = cfg.get<std::string>("network.mqtt.password", "");
    mqtt_cfg.tls_enabled = cfg.get<bool>("network.mqtt.tls_enabled", false);
    mqtt_cfg.ca_cert_path =
        resolve_repo_relative(repo_root, cfg.get<std::string>("network.mqtt.ca_cert_path", ""))
            .string();
    mqtt_cfg.client_cert_path = resolve_repo_relative(
                                    repo_root,
                                    cfg.get<std::string>("network.mqtt.client_cert_path", ""))
                                    .string();
    mqtt_cfg.client_key_path = resolve_repo_relative(
                                   repo_root,
                                   cfg.get<std::string>("network.mqtt.client_key_path", ""))
                                   .string();
    mqtt_cfg.insecure_skip_server_name_check =
        cfg.get<bool>("network.mqtt.insecure_skip_server_name_check", false);
    mqtt_cfg.keep_alive_sec = cfg.get<int>("network.mqtt.keep_alive_s", 60);
    mqtt_cfg.qos = cfg.get<int>("network.mqtt.qos", 1);
    return mqtt_cfg;
}

template <typename Pred>
bool wait_until(Pred pred,
                std::chrono::seconds timeout,
                std::chrono::milliseconds poll = std::chrono::milliseconds(500))
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(poll);
    }
    return pred();
}

struct RealThingsBoardRuntimeMockFixture {
    fs::path repo_root;
    fs::path temp_config_path{"/tmp/tb_runtime_mock_config.json"};
    fs::path temp_pending_path{"/tmp/tb_runtime_mock_config.pending.json"};
    fs::path temp_backup_path{"/tmp/tb_runtime_mock_config.backup.json"};
    fs::path cache_path{"/tmp/tb_runtime_mock_cache.jsonl"};

    robot::service::ConfigService cfg;
    robot::service::SchedulerService scheduler;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::shared_ptr<robot::middleware::DataCache> cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::shared_ptr<robot::service::ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<robot::service::CommandTracker> command_tracker;

    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<robot::device::WalkMotorGroup> group{
        std::make_shared<robot::device::WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::BrushMotor> brush{
        std::make_shared<robot::device::BrushMotor>(brush_serial, 0, 8192.0f, true, 0.5f)};
    std::shared_ptr<MockSerialPort> imu_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::ImuDevice> imu{
        std::make_shared<robot::device::ImuDevice>(imu_serial)};
    std::shared_ptr<MockSerialPort> gps_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::GpsDevice> gps{
        std::make_shared<robot::device::GpsDevice>(gps_serial)};
    robot::middleware::EventBus bus;
    std::shared_ptr<robot::service::MotionService> motion;
    std::shared_ptr<robot::service::NavService> nav;
    std::shared_ptr<robot::service::FaultService> fault{std::make_shared<robot::service::FaultService>(bus)};
    std::shared_ptr<robot::app::RobotFsm> fsm;
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;
    std::shared_ptr<robot::service::ThingsBoardControlPlane> tb_control;

    bool at_home{true};
    bool at_front{false};

    explicit RealThingsBoardRuntimeMockFixture(const RepoPaths& paths)
        : repo_root(paths.repo_root)
        , cfg(temp_config_path.string())
        , scheduler()
    {
        fs::copy_file(paths.config_path, temp_config_path, fs::copy_options::overwrite_existing);
        fs::remove(temp_pending_path);
        fs::remove(temp_backup_path);
        fs::remove(cache_path);

        REQUIRE(cfg.load());
        cache = std::make_shared<robot::middleware::DataCache>(cache_path.string());
        REQUIRE(cache->open());

        auto mqtt_cfg = build_mqtt_config(cfg, repo_root);
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
        spdlog::info(
            "[TB runtime mock] broker='{}' tls_enabled={} ca='{}' cert='{}' key='{}' skip_server_name_check={}",
            mqtt_cfg.broker_uri,
            mqtt_cfg.tls_enabled,
            mqtt_cfg.ca_cert_path,
            mqtt_cfg.client_cert_path,
            mqtt_cfg.client_key_path,
            mqtt_cfg.insecure_skip_server_name_check);
        mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);
        net = std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
        cloud = std::make_shared<robot::service::CloudService>(net, cache);
        tb_cfg = std::make_shared<robot::service::ThingsBoardConfigManager>(cfg, scheduler);
        command_tracker = std::make_shared<robot::service::CommandTracker>();

        robot::service::MotionService::Config motion_cfg;
        motion_cfg.heading_pid_en = false;
        motion = std::make_shared<robot::service::MotionService>(group, brush, nullptr, bus, motion_cfg);
        nav = std::make_shared<robot::service::NavService>(group, imu, gps);
        fsm = std::make_shared<robot::app::RobotFsm>(motion, nav, fault, bus);
        supervisor = std::make_shared<robot::app::RobotSupervisor>(
            fsm, tb_cfg, command_tracker, fault, nav);

        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();
        fsm->dispatch(robot::app::EvInitDone{});

        tb_control = std::make_shared<robot::service::ThingsBoardControlPlane>(
            cfg, cloud, tb_cfg, command_tracker, supervisor);
        tb_control->subscribe_shared_attributes();
        tb_control->register_rpc_handlers(
            [this]() { return at_home; }, [this]() { return at_front; });
    }

    ~RealThingsBoardRuntimeMockFixture()
    {
        if (net) {
            net->disconnect();
        }
        if (cache) {
            cache->close();
        }
        fs::remove(temp_config_path);
        fs::remove(temp_pending_path);
        fs::remove(temp_backup_path);
        fs::remove(cache_path);
    }

    bool connect()
    {
        if (!net->connect()) {
            return false;
        }
        return true;
    }

    bool wait_state_with_cloud(const std::vector<std::string>& expected,
                               std::chrono::seconds timeout,
                               std::chrono::milliseconds poll = std::chrono::milliseconds(500))
    {
        return wait_until(
            [&]() {
                const auto state = fsm->current_state();
                for (const auto& candidate : expected) {
                    if (state == candidate) {
                        return true;
                    }
                }
                return false;
            },
            timeout,
            poll);
    }
};

}  // namespace

TEST_CASE("Real ThingsBoard with mock runtime RPC start/stop/return changes state",
          "[integration][thingsboard][real][runtime_mock]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard mock runtime RPC test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    RealThingsBoardRuntimeMockFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn("[TB runtime mock] ACTION REQUIRED: send RPC start -> stop -> return");
    spdlog::warn("[TB runtime mock] mock state precondition: at_home={} at_front={}",
                 f.at_home,
                 f.at_front);

    REQUIRE(f.wait_state_with_cloud({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    auto snap = f.supervisor->snapshot();
    CHECK((snap.device_state == "CleanFwd" || snap.device_state == "CleanReturn"));
    CHECK(snap.task_state == "RunningTask");

    REQUIRE(f.wait_state_with_cloud({"Paused"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Paused");
    CHECK(snap.task_state == "PausedTask");

    REQUIRE(f.wait_state_with_cloud({"Returning"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Returning");
    CHECK(snap.task_state == "ReturningHome");
}

TEST_CASE("Real ThingsBoard with mock runtime RPC terminate/reset changes state",
          "[integration][thingsboard][real][runtime_mock][terminate_reset]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard mock runtime terminate/reset test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    RealThingsBoardRuntimeMockFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn("[TB runtime mock] ACTION REQUIRED: send RPC start -> stop -> terminate -> reset");
    spdlog::warn("[TB runtime mock] mock state precondition: at_home={} at_front={}",
                 f.at_home,
                 f.at_front);

    REQUIRE(f.wait_state_with_cloud({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    REQUIRE(f.wait_state_with_cloud({"Paused"}, std::chrono::seconds(120)));
    REQUIRE(f.wait_state_with_cloud({"Terminated"}, std::chrono::seconds(120)));
    auto snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Terminated");
    CHECK(snap.task_state == "IdleTask");

    f.at_home = true;
    REQUIRE(f.wait_state_with_cloud({"Idle"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Idle");
    CHECK(snap.task_state == "IdleTask");
}
