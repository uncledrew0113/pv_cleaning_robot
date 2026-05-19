#include <catch2/catch.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <rapidjson/document.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
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
using robot::service::ThingsBoardControlPlane;
namespace fs = std::filesystem;

namespace {

rapidjson::Document parse_json(const std::string& text) {
    rapidjson::Document doc;
    doc.Parse(text.c_str(), text.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

struct MockTransport final : INetworkTransport {
    MessageCallback rpc_cb;
    MessageCallback attr_update_cb;
    MessageCallback attr_response_cb;
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
        if (topic == "v1/devices/me/attributes") {
            attr_update_cb = std::move(cb);
        } else if (topic == "v1/devices/me/attributes/response/+") {
            attr_response_cb = std::move(cb);
        } else {
            rpc_cb = std::move(cb);
        }
        return true;
    }

    void emit_attributes(const std::string& payload) {
        if (attr_update_cb) {
            attr_update_cb("v1/devices/me/attributes", payload);
        }
    }

    void emit_rpc(const std::string& request_id, const std::string& payload) {
        if (rpc_cb) {
            rpc_cb("v1/devices/me/rpc/request/" + request_id, payload);
        }
    }
};

struct Fixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_tb_control_plane")};

    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    std::shared_ptr<MockTransport> mqtt{std::make_shared<MockTransport>()};
    std::shared_ptr<NetworkManager> net{
        std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY)};
    std::shared_ptr<DataCache> cache{std::make_shared<DataCache>(paths.cache_path.string())};
    std::shared_ptr<CloudService> cloud{std::make_shared<CloudService>(net, cache)};
    std::shared_ptr<CommandTracker> command_tracker{std::make_shared<CommandTracker>()};

    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<robot::device::WalkMotorGroup> group{
        std::make_shared<robot::device::WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<robot::device::BrushMotor> brush{
        std::make_shared<robot::device::BrushMotor>(brush_serial, 0)};
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
    bool reboot_requested{false};

    Fixture() {
        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "parking_side": "left",
    "start_battery_soc": 30.0,
    "charge_start_soc": 15.0,
    "charge_stop_soc": 95.0
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})",
                                            R"({
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
})");

        REQUIRE(cfg.load());
        cache->open();
        REQUIRE(net->connect());

        cfg.apply_active_runtime_schedules(scheduler);

        MotionService::Config motion_cfg;
        motion_cfg.heading_pid_en = false;
        motion = std::make_shared<MotionService>(group, brush, nullptr, bus, motion_cfg);
        motion->set_parking_side_query(
            [this]() { return cfg.active_runtime_config().parking_side; });
        motion->set_runtime_config_query([this]() { return cfg.active_runtime_config(); });
        nav = std::make_shared<NavService>(group, imu, gps);
        fsm = std::make_shared<robot::app::RobotFsm>(motion, nav, fault, bus);
        supervisor = std::make_shared<robot::app::RobotSupervisor>(
            fsm, cfg, command_tracker, fault, nav);

        can->open_result = true;
        can->send_result = true;
        can->opened = true;
        brush_serial->open_result = true;
        brush->open();
        fsm->dispatch(robot::app::EvInitDone{});

        control_plane = std::make_shared<ThingsBoardControlPlane>(
            cfg, &scheduler, cloud, command_tracker, supervisor);
    }

    ~Fixture() {
        cache->close();
        tb_test_support::cleanup_split_config_paths(paths);
    }

    void register_handlers(bool position_valid = true,
                           bool at_start_parking_side = true,
                           bool at_active_parking_side = false,
                           float battery_soc = 80.0f) {
        control_plane->register_rpc_handlers(
            [position_valid]() { return position_valid; },
            [at_start_parking_side]() { return at_start_parking_side; },
            [at_active_parking_side]() { return at_active_parking_side; },
            [battery_soc]() { return battery_soc; },
            [this]() { reboot_requested = true; });
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

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(2.0));

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["event"].GetString()) == "shared_attr_update");
    CHECK(j["accepted"].GetBool() == true);
}

TEST_CASE("ThingsBoardControlPlane publishes minimal startup attributes",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->publish_startup_attributes();

    const auto j = f.last_published_json("attributes");
    CHECK(std::string(j["software_version"].GetString()) == "2.0.0");
    CHECK(std::string(j["hardware_version"].GetString()) == "A1");
    CHECK(std::string(j["device_model"].GetString()) == "pv_cleaning_robot_test");
    CHECK(std::string(j["device_id"].GetString()) == "pv_robot_test_001");
}

TEST_CASE("ThingsBoardControlPlane requests current release shared attribute snapshot",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->request_shared_attributes_snapshot();

    REQUIRE_FALSE(f.mqtt->published.empty());
    const auto& [topic, payload] = f.mqtt->published.back();
    CHECK(topic.find("v1/devices/me/attributes/request/") == 0);
    CHECK(payload ==
          R"({"sharedKeys":"passes,clean_speed_rpm,return_speed_rpm,brush_rpm,return_brush_rpm,parking_side,start_battery_soc,charge_start_soc,charge_stop_soc,schedules"})");
}

TEST_CASE("ThingsBoardControlPlane start RPC launches a new task", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers(true, true, false, 80.0f);

    f.mqtt->emit_rpc("42", R"({"method":"start","params":{}})");

    CHECK(f.fsm->current_state() == "CleanFwd");
    const auto response = f.last_published_json("rpc/response/42");
    CHECK(response["accepted"].GetBool() == true);
}

TEST_CASE("ThingsBoardControlPlane start RPC can launch away from parking side",
          "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers(true, false, false, 80.0f);

    f.mqtt->emit_rpc("43", R"({"method":"start","params":{}})");

    CHECK(f.fsm->current_state() == "CleanFwd");
    const auto response = f.last_published_json("rpc/response/43");
    CHECK(response["accepted"].GetBool() == true);
}

TEST_CASE("ThingsBoardControlPlane start RPC rejects invalid position or low battery",
          "[service][tb_control_plane]") {
    SECTION("invalid position") {
        Fixture f;
        f.register_handlers(false, false, false, 80.0f);
        f.mqtt->emit_rpc("44", R"({"method":"start","params":{}})");
        const auto response = f.last_published_json("rpc/response/44");
        CHECK(response["accepted"].GetBool() == false);
        CHECK(std::string(response["reason"].GetString()) == "robot_position_invalid");
    }

    SECTION("battery below threshold") {
        Fixture f;
        f.register_handlers(true, true, false, 10.0f);
        f.mqtt->emit_rpc("45", R"({"method":"start","params":{}})");
        const auto response = f.last_published_json("rpc/response/45");
        CHECK(response["accepted"].GetBool() == false);
        CHECK(std::string(response["reason"].GetString()) == "battery_below_start_threshold");
    }
}

TEST_CASE("ThingsBoardControlPlane stop RPC stops active task", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();
    f.mqtt->emit_rpc("45", R"({"method":"start","params":{}})");
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    f.mqtt->emit_rpc("46", R"({"method":"stop","params":{}})");
    CHECK(f.fsm->current_state() == "Stopped");

    const auto response = f.last_published_json("rpc/response/46");
    CHECK(response["accepted"].GetBool() == true);
}

TEST_CASE("ThingsBoardControlPlane return RPC sends robot back to parking side",
          "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();
    f.mqtt->emit_rpc("47", R"({"method":"start","params":{}})");
    REQUIRE(f.fsm->current_state() == "CleanFwd");

    f.control_plane->register_rpc_handlers(
        []() { return true; },
        []() { return true; },
        []() { return false; },
        []() { return 80.0f; },
        []() {});

    f.mqtt->emit_rpc("48", R"({"method":"return","params":{}})");
    CHECK(f.fsm->current_state() == "Returning");
}

TEST_CASE("ThingsBoardControlPlane reset RPC requests reboot", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();

    f.mqtt->emit_rpc("49", R"({"method":"reset","params":{}})");
    const auto response = f.last_published_json("rpc/response/49");
    CHECK(response["accepted"].GetBool() == true);

    for (int i = 0; i < 10 && !f.reboot_requested; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    CHECK(f.reboot_requested);
}

TEST_CASE("ThingsBoardControlPlane publishes business telemetry from supervisor snapshot",
          "[service][tb_control_plane]") {
    Fixture f;
    f.fsm->dispatch(robot::app::EvScheduleStart{true, false, 2.0f});
    f.fsm->dispatch(robot::app::EvFarEndLimitSettled{});
    f.command_tracker->reject("return", "req-1", "return_not_allowed_in_current_state");

    f.control_plane->publish_business_telemetry();

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["device_state"].GetString()) == "CleanReturn");
    CHECK(std::string(j["task_state"].GetString()) == "RunningTask");
    REQUIRE(j.HasMember("active_config_version"));
    CHECK(j["active_config_version"].GetUint64() > 0u);
    CHECK(j.MemberCount() == 3);
}
