#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include "pv_cleaning_robot/service/config_service.h"

namespace tb_test_support {

namespace fs = std::filesystem;

struct RepoPaths {
    fs::path repo_root;
    fs::path runtime_config_path;
    fs::path fixed_config_path;
};

inline std::optional<RepoPaths> find_repo_paths()
{
    fs::path current = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        const auto runtime_candidate = current / "config" / "config.runtime.json";
        const auto fixed_candidate = current / "config" / "config.fixed.json";
        if (fs::exists(runtime_candidate) && fs::exists(fixed_candidate)) {
            return RepoPaths{current, runtime_candidate, fixed_candidate};
        }
        const auto legacy_candidate = current / "config" / "config.json";
        if (fs::exists(legacy_candidate)) {
            return RepoPaths{current, legacy_candidate, {}};
        }
        if (!current.has_parent_path()) {
            break;
        }
        current = current.parent_path();
    }
    return std::nullopt;
}

inline fs::path resolve_repo_relative(const fs::path& repo_root, const std::string& path)
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

inline robot::middleware::MqttTransport::Config build_mqtt_config(
    robot::service::ConfigService& cfg,
    const fs::path& repo_root,
    const char* client_id_suffix)
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
    mqtt_cfg.connect_timeout_sec = cfg.get<int>("network.mqtt.connect_timeout_s", 10);
    mqtt_cfg.qos = cfg.get<int>("network.mqtt.qos", 1);
    mqtt_cfg.client_id += client_id_suffix;
    return mqtt_cfg;
}

}  // namespace tb_test_support
