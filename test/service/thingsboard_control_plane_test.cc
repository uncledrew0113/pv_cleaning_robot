#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <rapidjson/document.h>
#include <string>
#include <utility>
#include <vector>

#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace {

rapidjson::Document parse_json(const std::string& text) {
    rapidjson::Document doc;
    doc.Parse(text.c_str(), text.size());
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

struct MockTransport final : robot::middleware::INetworkTransport {
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

    robot::service::ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    robot::service::SchedulerService scheduler;
    std::shared_ptr<MockTransport> mqtt{std::make_shared<MockTransport>()};
    std::shared_ptr<robot::middleware::NetworkManager> net{
        std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY)};
    std::shared_ptr<robot::middleware::DataCache> cache{
        std::make_shared<robot::middleware::DataCache>(paths.cache_path.string())};
    std::shared_ptr<robot::service::CloudService> cloud{
        std::make_shared<robot::service::CloudService>(net, cache)};
    std::shared_ptr<robot::service::CommandTracker> command_tracker{
        std::make_shared<robot::service::CommandTracker>()};
    std::optional<robot::domain::RobotCommand> last_command;
    robot::service::RobotCommandResult command_result{true, "accepted"};
    std::shared_ptr<robot::service::ThingsBoardControlPlane> control_plane;

    Fixture() {
        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "repeat_count": 1,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "primary_dock": "A",
    "min_battery_soc": 30.0,
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

        control_plane = std::make_shared<robot::service::ThingsBoardControlPlane>(
            cfg,
            &scheduler,
            cloud,
            command_tracker,
            robot::service::RobotCommandPort{
                [this](const robot::domain::RobotCommand& command) {
                    last_command = command;
                    return command_result;
                },
                [this]() {
                    robot::domain::RobotRuntimeSnapshot snap;
                    snap.state = "Idle";
                    snap.repeat_count = 2;
                    snap.completed_cycles = 1;
                    snap.active_config = cfg.active_runtime_config();
                    snap.cfg_ver = cfg.runtime_config_version(*snap.active_config);
                    return snap;
                }});
    }

    ~Fixture() {
        cache->close();
        tb_test_support::cleanup_split_config_paths(paths);
    }

    void register_handlers() {
        control_plane->register_rpc_handlers();
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

    f.mqtt->emit_attributes(R"({"repeat_count":2})");

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->repeat_count == 2u);

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["event"].GetString()) == "shared_attr_update");
    CHECK(std::string(j["code"].GetString()) == "ok");
}

TEST_CASE("ThingsBoardControlPlane publishes startup attributes", "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->publish_startup_attributes();

    const auto j = f.last_published_json("attributes");
    CHECK(std::string(j["software_version"].GetString()) == "2.0.0");
    CHECK(std::string(j["hardware_version"].GetString()) == "A1");
    CHECK(std::string(j["device_model"].GetString()) == "pv_cleaning_robot_test");
    CHECK(std::string(j["device_id"].GetString()) == "pv_robot_test_001");
}

TEST_CASE("ThingsBoardControlPlane requests release shared attribute snapshot",
          "[service][tb_control_plane]") {
    Fixture f;
    f.control_plane->request_shared_attributes_snapshot();

    REQUIRE_FALSE(f.mqtt->published.empty());
    const auto& [topic, payload] = f.mqtt->published.back();
    CHECK(topic.find("v1/devices/me/attributes/request/") == 0);
    CHECK(payload ==
          R"({"sharedKeys":"repeat_count,clean_speed_rpm,return_speed_rpm,brush_rpm,primary_dock,min_battery_soc,charge_stop_soc,schedules"})");
}

TEST_CASE("ThingsBoard RPC maps robot commands", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();

    f.mqtt->emit_rpc("301", R"({"method":"clean_to_return","params":{}})");
    REQUIRE(f.last_command.has_value());
    CHECK(f.last_command->kind == robot::domain::RobotCommandKind::CleanTowardOppositeEndpoint);
    CHECK(f.last_command->source == robot::domain::CommandSource::Rpc);
    CHECK(f.last_command->command_id == "301");
    CHECK(std::string(f.last_published_json("rpc/response/301")["code"].GetString()) ==
          "accepted");

    f.mqtt->emit_rpc("302", R"({"method":"clean_to_parking","params":{}})");
    REQUIRE(f.last_command.has_value());
    CHECK(f.last_command->kind == robot::domain::RobotCommandKind::CleanTowardPrimaryDock);

    f.mqtt->emit_rpc("303", R"({"method":"start_configured","params":{}})");
    REQUIRE(f.last_command.has_value());
    CHECK(f.last_command->kind == robot::domain::RobotCommandKind::StartConfiguredMission);

    f.mqtt->emit_rpc("304", R"({"method":"stop","params":{}})");
    REQUIRE(f.last_command.has_value());
    CHECK(f.last_command->kind == robot::domain::RobotCommandKind::Stop);

    f.mqtt->emit_rpc("305", R"({"method":"fault_reset","params":{}})");
    REQUIRE(f.last_command.has_value());
    CHECK(f.last_command->kind == robot::domain::RobotCommandKind::FaultReset);
}

TEST_CASE("ThingsBoard RPC reset reboot command is not registered", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();

    f.mqtt->emit_rpc("399", R"({"method":"reset","params":{}})");

    const auto response = f.last_published_json("rpc/response/399");
    CHECK(std::string(response["code"].GetString()) == "method_not_supported");
    REQUIRE_FALSE(f.last_command.has_value());
}

TEST_CASE("ThingsBoard RPC returns app rejection reason", "[service][tb_control_plane]") {
    Fixture f;
    f.register_handlers();
    f.command_result = {false, "robot_position_unknown"};

    f.mqtt->emit_rpc("306", R"({"method":"clean_to_return","params":{}})");

    const auto response = f.last_published_json("rpc/response/306");
    CHECK(std::string(response["code"].GetString()) == "robot_position_unknown");
}

TEST_CASE("ThingsBoardControlPlane publishes business telemetry from app snapshot",
          "[service][tb_control_plane]") {
    Fixture f;

    f.control_plane->publish_business_telemetry();

    const auto j = f.last_published_json("telemetry");
    CHECK(std::string(j["state"].GetString()) == "Idle");
    CHECK(j["fault"].GetUint() == 0u);
    CHECK(j["repeat_count"].GetUint() == 2u);
    CHECK(j["completed_cycles"].GetUint() == 1u);
    REQUIRE(j.HasMember("cfg_ver"));
}
