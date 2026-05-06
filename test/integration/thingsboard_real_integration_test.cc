#include <catch2/catch.hpp>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>

#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"
#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"

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

struct RealThingsBoardFixture {
    fs::path repo_root;
    fs::path config_path;
    fs::path cache_path{"/tmp/tb_real_integration_cache.jsonl"};

    robot::service::ConfigService cfg;
    robot::service::SchedulerService scheduler;
    std::unique_ptr<robot::service::ThingsBoardConfigManager> tb_cfg;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::shared_ptr<robot::middleware::DataCache> cache;
    std::shared_ptr<robot::service::CloudService> cloud;

    RealThingsBoardFixture(const RepoPaths& paths)
        : repo_root(paths.repo_root)
        , config_path(paths.config_path)
        , cfg(config_path.string())
        , scheduler()
    {
        REQUIRE(cfg.load());
        tb_cfg = std::make_unique<robot::service::ThingsBoardConfigManager>(cfg, scheduler);
        fs::remove(cache_path);
        auto mqtt_cfg = build_mqtt_config(cfg, repo_root);
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
        spdlog::info(
            "[TB real test] broker='{}' tls_enabled={} ca='{}' cert='{}' key='{}' skip_server_name_check={}",
            mqtt_cfg.broker_uri,
            mqtt_cfg.tls_enabled,
            mqtt_cfg.ca_cert_path,
            mqtt_cfg.client_cert_path,
            mqtt_cfg.client_key_path,
            mqtt_cfg.insecure_skip_server_name_check);
        mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);
        net = std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
        cache = std::make_shared<robot::middleware::DataCache>(cache_path.string());
        REQUIRE(cache->open());
        cloud = std::make_shared<robot::service::CloudService>(net, cache);
    }

    ~RealThingsBoardFixture()
    {
        if (net) {
            net->disconnect();
        }
        if (cache) {
            cache->close();
        }
        fs::remove(cache_path);
    }
};

}  // namespace

TEST_CASE("Real ThingsBoard mutual TLS connection", "[integration][thingsboard][real]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard connection test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);

    REQUIRE(f.net->connect());
    CHECK(f.net->is_connected());
}

TEST_CASE("Real ThingsBoard publish startup attributes and telemetry",
          "[integration][thingsboard][real]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard publish test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.net->connect());

    const std::string software_version = f.cfg.get<std::string>("device.software_version", "");
    const std::string hardware_version = f.cfg.get<std::string>("device.hardware_version", "");
    const std::string device_model = f.cfg.get<std::string>("device.model", "");
    const std::string device_id = f.cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001");

    std::array<char, 1024> attr_buf{};
    const auto attr_len = robot::service::ThingsBoardJsonCodec::build_startup_attributes(
        {
            software_version.c_str(),
            hardware_version.c_str(),
            device_model.c_str(),
            device_id.c_str(),
        },
        attr_buf.data(),
        attr_buf.size());
    REQUIRE(attr_len > 0u);
    REQUIRE(f.cloud->publish_attributes(std::string(attr_buf.data(), attr_len)));

    robot::app::RobotRuntimeSnapshot snap;
    snap.device_state = "Idle";
    snap.task_state = "IdleTask";
    snap.active_config = f.tb_cfg->active_config();
    snap.active_config_version = 1;

    std::array<char, 4096> telemetry_buf{};
    const auto telemetry_len = robot::service::ThingsBoardJsonCodec::build_business_telemetry(
        snap, telemetry_buf.data(), telemetry_buf.size());
    REQUIRE(telemetry_len > 0u);
    REQUIRE(f.cloud->publish_telemetry(std::string(telemetry_buf.data(), telemetry_len)));
}
