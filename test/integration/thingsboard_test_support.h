#pragma once

#include <filesystem>
#include <fstream>
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

struct TempSplitConfigPaths {
    fs::path runtime_path;
    fs::path fixed_path;
    fs::path pending_path;
    fs::path backup_path;
    fs::path cache_path;
};

inline TempSplitConfigPaths make_temp_split_config_paths(const std::string& prefix)
{
    const fs::path base = fs::path("/tmp") / prefix;
    return {
        base.string() + ".runtime.json",
        base.string() + ".fixed.json",
        base.string() + ".runtime.pending.json",
        base.string() + ".runtime.backup.json",
        base.string() + ".cache.jsonl",
    };
}

inline void write_text_file(const fs::path& path, const std::string& text)
{
    std::ofstream out(path);
    out << text;
}

inline void write_split_config(const TempSplitConfigPaths& paths,
                               const std::string& runtime_json,
                               const std::string& fixed_json)
{
    write_text_file(paths.runtime_path, runtime_json);
    write_text_file(paths.fixed_path, fixed_json);
}

// Real/mock ThingsBoard tests use the same split-config file contract as runtime:
// start from runtime/fixed, keep pending/backup/cache isolated under /tmp.
inline void copy_repo_split_config(const RepoPaths& repo_paths, const TempSplitConfigPaths& paths)
{
    fs::copy_file(
        repo_paths.runtime_config_path, paths.runtime_path, fs::copy_options::overwrite_existing);
    if (!repo_paths.fixed_config_path.empty()) {
        fs::copy_file(
            repo_paths.fixed_config_path, paths.fixed_path, fs::copy_options::overwrite_existing);
    } else {
        fs::remove(paths.fixed_path);
    }
    fs::remove(paths.pending_path);
    fs::remove(paths.backup_path);
    fs::remove(paths.cache_path);
}

inline void cleanup_split_config_paths(const TempSplitConfigPaths& paths)
{
    fs::remove(paths.runtime_path);
    fs::remove(paths.fixed_path);
    fs::remove(paths.pending_path);
    fs::remove(paths.backup_path);
    fs::remove(paths.cache_path);
}

inline std::optional<RepoPaths> find_repo_paths()
{
    fs::path current = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        const auto runtime_candidate = current / "config" / "config.runtime.json";
        const auto fixed_candidate = current / "config" / "config.fixed.json";
        if (fs::exists(runtime_candidate) && fs::exists(fixed_candidate)) {
            return RepoPaths{current, runtime_candidate, fixed_candidate};
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
