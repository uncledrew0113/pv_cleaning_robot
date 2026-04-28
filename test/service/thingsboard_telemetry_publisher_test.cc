#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"
#include "pv_cleaning_robot/service/thingsboard_telemetry_publisher.h"

using robot::middleware::DataCache;
using robot::middleware::INetworkTransport;
using robot::middleware::NetworkManager;
using robot::service::CloudService;
using robot::service::CommandTracker;
using robot::service::ConfigService;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;
using robot::service::SchedulerService;
using robot::service::ThingsBoardConfigManager;
using robot::service::ThingsBoardTelemetryPublisher;
namespace fs = std::filesystem;

namespace {

struct MockTransport final : INetworkTransport {
    std::vector<std::pair<std::string, std::string>> published;
    bool connected{false};

    bool connect() override {
        connected = true;
        return true;
    }
    void disconnect() override { connected = false; }
    bool is_connected() const override { return connected; }
    bool publish(const std::string& topic, const std::string& payload) override {
        published.emplace_back(topic, payload);
        return true;
    }
    bool subscribe(const std::string&, MessageCallback) override { return true; }
};

struct Fixture {
    std::string path{"/tmp/test_tb_telemetry.json"};
    std::string pending_path{"/tmp/test_tb_telemetry.pending.json"};
    std::string backup_path{"/tmp/test_tb_telemetry.backup.json"};
    std::string cache_path{"/tmp/test_tb_telemetry.cache.jsonl"};

    ConfigService cfg{path};
    SchedulerService scheduler;
    std::shared_ptr<MockTransport> mqtt{std::make_shared<MockTransport>()};
    std::shared_ptr<NetworkManager> net{
        std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY)};
    std::shared_ptr<DataCache> cache{std::make_shared<DataCache>(cache_path)};
    std::shared_ptr<CloudService> cloud{std::make_shared<CloudService>(net, cache)};
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<CommandTracker> command_tracker{std::make_shared<CommandTracker>()};

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
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault{std::make_shared<FaultService>(bus)};
    std::shared_ptr<robot::app::RobotFsm> fsm;
    std::shared_ptr<robot::app::RobotSupervisor> supervisor;
    std::shared_ptr<ThingsBoardTelemetryPublisher> publisher;

    Fixture() {
        std::ofstream f(path);
        f << R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  },
  "device": {
    "software_version": "2.0.0",
    "hardware_version": "A1",
    "model": "pv_cleaning_robot_test"
  },
  "network": {
    "mqtt": {
      "client_id": "pv_robot_test_001"
    }
  }
})";
        f.close();

        REQUIRE(cfg.load());
        scheduler.clear_windows();
        scheduler.add_window({8, 0});
        cache->open();
        REQUIRE(net->connect());
        tb_cfg = std::make_shared<ThingsBoardConfigManager>(cfg, scheduler);

        MotionService::Config motion_cfg;
        motion_cfg.heading_pid_en = false;
        motion = std::make_shared<MotionService>(group, brush, nullptr, bus, motion_cfg);
        nav = std::make_shared<NavService>(group, imu, gps);
        fsm = std::make_shared<robot::app::RobotFsm>(motion, nav, fault, bus);
        supervisor = std::make_shared<robot::app::RobotSupervisor>(
            fsm, tb_cfg, command_tracker, fault, nav);

        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();
        fsm->dispatch(robot::app::EvInitDone{});

        publisher = std::make_shared<ThingsBoardTelemetryPublisher>(cfg, cloud, supervisor);
    }

    ~Fixture() {
        cache->close();
        fs::remove(path);
        fs::remove(pending_path);
        fs::remove(backup_path);
        fs::remove(cache_path);
    }

    nlohmann::json last_published_json(const std::string& topic_suffix) const {
        for (auto it = mqtt->published.rbegin(); it != mqtt->published.rend(); ++it) {
            if (it->first.find(topic_suffix) != std::string::npos) {
                return nlohmann::json::parse(it->second);
            }
        }
        FAIL("expected published topic suffix not found");
        return nlohmann::json::object();
    }
};

}  // namespace

TEST_CASE("ThingsBoardTelemetryPublisher publishes startup attributes",
          "[service][tb_telemetry]") {
    Fixture f;

    f.publisher->publish_startup_attributes();

    const auto j = f.last_published_json("attributes");
    CHECK(j.at("software_version").get<std::string>() == "2.0.0");
    CHECK(j.at("hardware_version").get<std::string>() == "A1");
    CHECK(j.at("device_model").get<std::string>() == "pv_cleaning_robot_test");
    CHECK(j.at("device_id").get<std::string>() == "pv_robot_test_001");
    REQUIRE(j.at("supported_rpc_methods").is_array());
    CHECK(j.at("config_schema_version").get<std::string>() == "thingsboard-v1");
}

TEST_CASE("ThingsBoardTelemetryPublisher publishes backup fallback event",
          "[service][tb_telemetry]") {
    Fixture f;

    f.publisher->publish_backup_fallback_event();

    const auto j = f.last_published_json("telemetry");
    CHECK(j.at("event").get<std::string>() == "config_backup_fallback");
    CHECK(j.at("accepted").get<bool>() == true);
    CHECK(j.at("reason").get<std::string>() == "loaded_from_backup");
}

TEST_CASE("ThingsBoardTelemetryPublisher publishes business telemetry from supervisor snapshot",
          "[service][tb_telemetry]") {
    Fixture f;
    f.fsm->dispatch(robot::app::EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(robot::app::EvFrontLimitSettled{});
    f.command_tracker->reject("return", "req-1", "return_not_allowed_in_current_state");
    f.publisher->publish_business_telemetry();

    const auto j = f.last_published_json("telemetry");
    CHECK(j.at("device_state").get<std::string>() == "CleanReturn");
    CHECK(j.at("task_state").get<std::string>() == "RunningTask");
    CHECK(j.at("target_half_passes").get<int>() == 4);
    CHECK(j.at("completed_half_passes").get<int>() == 1);
    REQUIRE(j.contains("last_command"));
    CHECK(j.at("last_command").at("name").get<std::string>() == "return");
}
