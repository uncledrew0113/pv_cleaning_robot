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
#include "pv_cleaning_robot/app/parking_side_runtime.h"
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
#include "integration/thingsboard_test_support.h"

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
    fs::path temp_runtime_path{"/tmp/tb_runtime_mock_config.runtime.json"};
    fs::path temp_fixed_path{"/tmp/tb_runtime_mock_config.fixed.json"};
    fs::path temp_pending_path{"/tmp/tb_runtime_mock_config.runtime.pending.json"};
    fs::path temp_backup_path{"/tmp/tb_runtime_mock_config.runtime.backup.json"};
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

    bool left_sensor_active{true};
    bool right_sensor_active{false};

    explicit RealThingsBoardRuntimeMockFixture(const tb_test_support::RepoPaths& paths)
        : repo_root(paths.repo_root)
        , cfg(temp_runtime_path.string(), temp_fixed_path.string())
        , scheduler()
    {
        fs::copy_file(
            paths.runtime_config_path, temp_runtime_path, fs::copy_options::overwrite_existing);
        if (!paths.fixed_config_path.empty()) {
            fs::copy_file(
                paths.fixed_config_path, temp_fixed_path, fs::copy_options::overwrite_existing);
        } else {
            fs::remove(temp_fixed_path);
        }
        fs::remove(temp_pending_path);
        fs::remove(temp_backup_path);
        fs::remove(cache_path);

        REQUIRE(cfg.load());
        cache = std::make_shared<robot::middleware::DataCache>(cache_path.string());
        REQUIRE(cache->open());

        auto mqtt_cfg = tb_test_support::build_mqtt_config(cfg, repo_root, "_runtime_mock_itest");
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
        motion->set_parking_side_provider(
            [this]() { return tb_cfg->active_config().parking_side; });
        motion->set_runtime_config_provider([this]() { return tb_cfg->active_config(); });
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
            [this]() { return parking_facts().is_valid_start_position(); },
            [this]() { return parking_facts().at_parking_side; },
            [this]() { return parking_facts().at_parking_side; },
            [this]() { return 80.0f; },
            []() {});
    }

    ~RealThingsBoardRuntimeMockFixture()
    {
        if (net) {
            net->disconnect();
        }
        if (cache) {
            cache->close();
        }
        fs::remove(temp_runtime_path);
        fs::remove(temp_fixed_path);
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

    robot::app::ParkingSideFacts parking_facts() const
    {
        return robot::app::ParkingSideRuntime::from_physical_limits(
            tb_cfg->active_config().parking_side, left_sensor_active, right_sensor_active);
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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    RealThingsBoardRuntimeMockFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn("[TB runtime mock] ACTION REQUIRED: send RPC start -> stop -> return");
    const auto facts = f.parking_facts();
    spdlog::warn("[TB runtime mock] mock state precondition: at_parking_side={} at_far_end={}",
                 facts.at_parking_side,
                 facts.at_far_end);

    REQUIRE(f.wait_state_with_cloud({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    auto snap = f.supervisor->snapshot();
    CHECK((snap.device_state == "CleanFwd" || snap.device_state == "CleanReturn"));
    CHECK(snap.task_state == "RunningTask");

    REQUIRE(f.wait_state_with_cloud({"Stopped"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Stopped");
    CHECK(snap.task_state == "StoppedTask");

    if (f.tb_cfg->active_config().parking_side == robot::service::ParkingSide::Left) {
        f.left_sensor_active = false;
        f.right_sensor_active = true;
    } else {
        f.left_sensor_active = true;
        f.right_sensor_active = false;
    }

    REQUIRE(f.wait_state_with_cloud({"Returning"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Returning");
    CHECK(snap.task_state == "ReturningTask");
}

TEST_CASE("Real ThingsBoard with mock runtime RPC start/stop/start cycles task state",
          "[integration][thingsboard][real][runtime_mock][restart_cycle]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard mock runtime restart-cycle test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    RealThingsBoardRuntimeMockFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn("[TB runtime mock] ACTION REQUIRED: send RPC start -> stop -> start");
    const auto facts = f.parking_facts();
    spdlog::warn("[TB runtime mock] mock state precondition: at_parking_side={} at_far_end={}",
                 facts.at_parking_side,
                 facts.at_far_end);

    REQUIRE(f.wait_state_with_cloud({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    REQUIRE(f.wait_state_with_cloud({"Stopped"}, std::chrono::seconds(120)));
    auto snap = f.supervisor->snapshot();
    CHECK(snap.device_state == "Stopped");
    CHECK(snap.task_state == "StoppedTask");

    f.left_sensor_active =
        (f.tb_cfg->active_config().parking_side == robot::service::ParkingSide::Left);
    f.right_sensor_active = !f.left_sensor_active;
    REQUIRE(f.wait_state_with_cloud({"CleanFwd", "CleanReturn"}, std::chrono::seconds(120)));
    snap = f.supervisor->snapshot();
    CHECK((snap.device_state == "CleanFwd" || snap.device_state == "CleanReturn"));
    CHECK(snap.task_state == "RunningTask");
}
