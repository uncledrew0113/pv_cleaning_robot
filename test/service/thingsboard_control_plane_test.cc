#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <rapidjson/document.h>
#include <string>
#include <utility>
#include <vector>

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
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

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
using robot::service::ThingsBoardControlPlane;
namespace fs = std::filesystem;

namespace {

rapidjson::Document parse_json(const std::string& text)
{
    rapidjson::Document doc;
    doc.Parse(text.c_str(), text.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

struct MockTransport final : INetworkTransport {
    MessageCallback rpc_cb;
    MessageCallback attr_cb;
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
    bool subscribe(const std::string& topic, MessageCallback cb) override {
        if (topic.find("attributes") != std::string::npos) {
            attr_cb = std::move(cb);
        } else {
            rpc_cb = std::move(cb);
        }
        return true;
    }

    void emit_attributes(const std::string& payload) {
        if (attr_cb) {
            attr_cb("v1/devices/me/attributes", payload);
        }
    }

    void emit_rpc(const std::string& request_id, const std::string& payload) {
        if (rpc_cb) {
            rpc_cb("v1/devices/me/rpc/request/" + request_id, payload);
        }
    }
};

struct Fixture {
    std::string path{"/tmp/test_tb_control_plane.json"};
    std::string pending_path{"/tmp/test_tb_control_plane.pending.json"};
    std::string backup_path{"/tmp/test_tb_control_plane.backup.json"};
    std::string cache_path{"/tmp/test_tb_control_plane.cache.jsonl"};

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
    std::shared_ptr<ThingsBoardControlPlane> control_plane;

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

        control_plane = std::make_shared<ThingsBoardControlPlane>(
            cfg,
            cloud,
            tb_cfg,
            command_tracker,
            supervisor);
    }

    ~Fixture() {
        cache->close();
        fs::remove(path);
        fs::remove(pending_path);
        fs::remove(backup_path);
        fs::remove(cache_path);
    }

    rapidjson::Document last_published_json(const std::string& topic_suffix) const {
        for (auto it = mqtt->published.rbegin(); it != mqtt->published.rend(); ++it) {
            if (it->first.find(topic_suffix) != std::string::npos) {
                return parse_json(it->second);
            }
        }
        FAIL("expected published topic suffix not found");
        rapidjson::Document empty;
        empty.SetObject();
        return empty;
    }
};

}  // namespace

TEST_CASE("ThingsBoardControlPlane shared attributes update pending config and emit event",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->subscribe_shared_attributes();

    f.mqtt->emit_attributes(R"({"passes":2.0})");

    const auto pending = f.tb_cfg->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(2.0));

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["event"].GetString()) == "shared_attr_update");
    CHECK(j["accepted"].GetBool() == true);
    CHECK(std::string(j["reason"].GetString()) == "ok");
}

TEST_CASE("ThingsBoardControlPlane publishes startup attributes",
          "[service][tb_control_plane]") {
    Fixture f;

    f.control_plane->publish_startup_attributes();

    const auto j = f.last_published_json("attributes");
    CHECK(std::string(j["software_version"].GetString()) == "2.0.0");
    CHECK(std::string(j["hardware_version"].GetString()) == "A1");
    CHECK(std::string(j["device_model"].GetString()) == "pv_cleaning_robot_test");
    CHECK(std::string(j["device_id"].GetString()) == "pv_robot_test_001");
    REQUIRE(j["supported_rpc_methods"].IsArray());
    CHECK(std::string(j["config_schema_version"].GetString()) == "thingsboard-v1");
}

TEST_CASE("ThingsBoardControlPlane publishes backup fallback event",
          "[service][tb_control_plane]") {
    Fixture f;

    f.control_plane->publish_backup_fallback_event();

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["event"].GetString()) == "config_backup_fallback");
    CHECK(j["accepted"].GetBool() == true);
    CHECK(std::string(j["reason"].GetString()) == "loaded_from_backup");
}

TEST_CASE("ThingsBoardControlPlane start RPC launches new task from idle",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->register_rpc_handlers([]() { return true; }, []() { return false; });

    f.mqtt->emit_rpc("42", R"({"method":"start","params":{}})");

    CHECK(f.fsm->current_state() == "CleanFwd");

    const auto response = f.last_published_json("rpc/response/42");
    CHECK(response["accepted"].GetBool() == true);
    CHECK(std::string(response["result"].GetString()) == "ok");

    bool saw_accepted = false;
    bool saw_completed = false;
    for (const auto& [topic, payload] : f.mqtt->published) {
        if (topic.find("telemetry") == std::string::npos) {
            continue;
        }
        const auto j = parse_json(payload);
        const auto event_it = j.FindMember("event");
        const auto reason_it = j.FindMember("reason");
        const std::string event =
            event_it != j.MemberEnd() && event_it->value.IsString() ? event_it->value.GetString() : "";
        const std::string reason =
            reason_it != j.MemberEnd() && reason_it->value.IsString() ? reason_it->value.GetString() : "";
        if (event == "command_accepted") {
            saw_accepted = true;
        }
        if (event == "command_completed" && reason == "started_new_task") {
            saw_completed = true;
        }
    }
    CHECK(saw_accepted);
    CHECK(saw_completed);
}

TEST_CASE("ThingsBoardControlPlane publishes business telemetry from supervisor snapshot",
          "[service][tb_control_plane]") {
    Fixture f;
    f.fsm->dispatch(robot::app::EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(robot::app::EvFrontLimitSettled{});
    f.command_tracker->reject("return", "req-1", "return_not_allowed_in_current_state");

    f.control_plane->publish_business_telemetry();

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["device_state"].GetString()) == "CleanReturn");
    CHECK(std::string(j["task_state"].GetString()) == "RunningTask");
    CHECK(j["target_half_passes"].GetInt() == 4);
    CHECK(j["completed_half_passes"].GetInt() == 1);
    REQUIRE(j.HasMember("last_command"));
    CHECK(std::string(j["last_command"]["name"].GetString()) == "return");
}
