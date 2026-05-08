#include <catch2/catch.hpp>

#include <array>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

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

struct SharedAttrTarget {
    double passes;
    double clean_speed_rpm;
    robot::service::ParkingSide parking_side;
    const char* parking_side_name;
};

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

template <typename Pred>
bool wait_until(Pred pred, std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return pred();
}

fs::path pending_config_path(const fs::path& config_path)
{
    auto pending = config_path;
    pending.replace_extension(".pending.json");
    return pending;
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

SharedAttrTarget choose_legal_target(const std::optional<robot::service::TbRuntimeConfig>& current)
{
    const SharedAttrTarget target_a{
        2.0, 320.0, robot::service::ParkingSide::Left, "left"};
    const SharedAttrTarget target_b{
        3.0, 330.0, robot::service::ParkingSide::Right, "right"};
    if (!current.has_value()) {
        return target_a;
    }
    if (current->passes == target_a.passes &&
        current->clean_speed_rpm == target_a.clean_speed_rpm &&
        current->parking_side == target_a.parking_side) {
        return target_b;
    }
    return target_a;
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
    std::atomic<int> shared_attr_count{0};
    mutable std::mutex shared_attr_result_mtx;
    std::optional<robot::service::SharedAttrApplyResult> last_shared_attr_result;

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
        mqtt_cfg.client_id += "_real_attr_itest";
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
        // 真实 shared attributes 联调测试需要在 connect() 前把下行回调接好，
        // 否则平台侧更新虽然发送到了设备，测试里的 tb_cfg 不会收到也不会更新。
        cloud->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
            const auto result = tb_cfg->apply_shared_attributes(attrs);
            {
                std::lock_guard<std::mutex> lk(shared_attr_result_mtx);
                last_shared_attr_result = result;
            }
            shared_attr_count.fetch_add(1, std::memory_order_relaxed);
            if (!result.accepted) {
                spdlog::warn("[TB real test] shared attributes rejected: {}", result.reason);
            }
        });
    }

    bool connect_and_request_shared_snapshot()
    {
        if (!net->connect()) {
            return false;
        }
        return cloud->request_shared_attributes_snapshot(
            {"passes",
             "clean_speed_rpm",
             "return_speed_rpm",
             "brush_rpm",
             "parking_side",
             "schedules"});
    }

    bool wait_for_initial_snapshot(std::chrono::seconds timeout = std::chrono::seconds(15))
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        int last_count = shared_attr_updates();
        auto last_change = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() < deadline) {
            const int current_count = shared_attr_updates();
            if (current_count != last_count) {
                last_count = current_count;
                last_change = std::chrono::steady_clock::now();
            }
            if (current_count > 0 &&
                std::chrono::steady_clock::now() - last_change > std::chrono::seconds(1)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        return shared_attr_updates() > 0;
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

    int shared_attr_updates() const
    {
        return shared_attr_count.load(std::memory_order_relaxed);
    }

    std::optional<robot::service::SharedAttrApplyResult> shared_attr_result() const
    {
        std::lock_guard<std::mutex> lk(shared_attr_result_mtx);
        return last_shared_attr_result;
    }
};

struct RawRpcSmokeFixture {
    fs::path repo_root;
    fs::path config_path;
    robot::service::ConfigService cfg;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::atomic<int> rpc_count{0};
    mutable std::mutex rpc_mtx;
    std::string last_topic;
    std::string last_payload;

    explicit RawRpcSmokeFixture(const RepoPaths& paths)
        : repo_root(paths.repo_root)
        , config_path(paths.config_path)
        , cfg(config_path.string())
    {
        REQUIRE(cfg.load());
        auto mqtt_cfg = build_mqtt_config(cfg, repo_root);
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
        mqtt_cfg.client_id += "_raw_rpc_itest";
        spdlog::info(
            "[TB rpc smoke] broker='{}' tls_enabled={} ca='{}' cert='{}' key='{}' skip_server_name_check={}",
            mqtt_cfg.broker_uri,
            mqtt_cfg.tls_enabled,
            mqtt_cfg.ca_cert_path,
            mqtt_cfg.client_cert_path,
            mqtt_cfg.client_key_path,
            mqtt_cfg.insecure_skip_server_name_check);
        mqtt = std::make_shared<robot::middleware::MqttTransport>(mqtt_cfg);
        net = std::make_shared<robot::middleware::NetworkManager>(
            mqtt, nullptr, robot::middleware::NetworkManager::Mode::MQTT_ONLY);
        net->subscribe("v1/devices/me/rpc/request/+",
                       [this](const std::string& topic, const std::string& payload) {
                           std::lock_guard<std::mutex> lk(rpc_mtx);
                           last_topic = topic;
                           last_payload = payload;
                           rpc_count.fetch_add(1, std::memory_order_relaxed);
                           spdlog::info("[TB rpc smoke] raw RPC received: topic='{}' payload={}",
                                        topic,
                                        payload);
                       });
    }

    ~RawRpcSmokeFixture()
    {
        if (net) {
            net->disconnect();
        }
    }

    bool connect() { return net->connect(); }

    int received_count() const
    {
        return rpc_count.load(std::memory_order_relaxed);
    }
};

struct CloudRpcSmokeFixture {
    fs::path repo_root;
    fs::path config_path;
    fs::path cache_path{"/tmp/tb_cloud_rpc_smoke_cache.jsonl"};
    robot::service::ConfigService cfg;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::shared_ptr<robot::middleware::DataCache> cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::atomic<int> rpc_count{0};
    mutable std::mutex rpc_mtx;
    std::string last_params;

    explicit CloudRpcSmokeFixture(const RepoPaths& paths)
        : repo_root(paths.repo_root)
        , config_path(paths.config_path)
        , cfg(config_path.string())
    {
        REQUIRE(cfg.load());
        fs::remove(cache_path);
        auto mqtt_cfg = build_mqtt_config(cfg, repo_root);
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
        mqtt_cfg.client_id += "_cloud_rpc_itest";
        spdlog::info(
            "[TB cloud rpc smoke] broker='{}' tls_enabled={} ca='{}' cert='{}' key='{}' skip_server_name_check={}",
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
        cloud->register_rpc("start", [this](const std::string& /*request_id*/,
                                            const std::string& params) {
            std::lock_guard<std::mutex> lk(rpc_mtx);
            last_params = params;
            rpc_count.fetch_add(1, std::memory_order_relaxed);
            spdlog::info("[TB cloud rpc smoke] CloudService RPC handler invoked: params={}", params);
            return std::string(R"({"accepted":true,"result":"ok"})");
        });
    }

    ~CloudRpcSmokeFixture()
    {
        if (net) {
            net->disconnect();
        }
        if (cache) {
            cache->close();
        }
        fs::remove(cache_path);
    }

    bool connect() { return net->connect(); }

    int received_count() const
    {
        return rpc_count.load(std::memory_order_relaxed);
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

    REQUIRE(f.connect_and_request_shared_snapshot());
    CHECK(f.net->is_connected());
}

TEST_CASE("Real ThingsBoard raw RPC smoke", "[integration][thingsboard][real][rpc_smoke]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to run real ThingsBoard RPC smoke test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB rpc smoke] using config: {}", paths->config_path.string());

    RawRpcSmokeFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn(
        "[TB rpc smoke] ACTION REQUIRED: send any RPC now, for example `start`");
    REQUIRE(wait_until([&] { return f.received_count() > 0; }, std::chrono::seconds(120)));
}

TEST_CASE("Real ThingsBoard CloudService RPC smoke",
          "[integration][thingsboard][real][cloud_rpc_smoke]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to run real ThingsBoard CloudService RPC smoke test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB cloud rpc smoke] using config: {}", paths->config_path.string());

    CloudRpcSmokeFixture f(*paths);
    REQUIRE(f.connect());

    spdlog::warn(
        "[TB cloud rpc smoke] ACTION REQUIRED: send RPC `start` now");
    REQUIRE(wait_until([&] { return f.received_count() > 0; }, std::chrono::seconds(120)));
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
    REQUIRE(f.connect_and_request_shared_snapshot());

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

TEST_CASE("Real ThingsBoard shared attributes update local pending config",
          "[integration][thingsboard][real][shared_attr]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard shared attribute test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.connect_and_request_shared_snapshot());
    REQUIRE(f.wait_for_initial_snapshot());

    const auto pending_cfg_path = pending_config_path(paths->config_path);
    const auto before_pending = f.tb_cfg->pending_config();
    const int before_attr_count = f.shared_attr_updates();
    const auto target = choose_legal_target(before_pending);

    spdlog::warn("[TB real test] ACTION REQUIRED: in ThingsBoard shared attributes, set "
                 "passes={}, clean_speed_rpm={}, parking_side={}",
                 target.passes,
                 target.clean_speed_rpm,
                 target.parking_side_name);

    REQUIRE(wait_until(
        [&] {
            if (f.shared_attr_updates() <= before_attr_count) {
                return false;
            }
            const auto pending = f.tb_cfg->pending_config();
            return pending.has_value() &&
                   pending->passes == Approx(target.passes) &&
                   pending->clean_speed_rpm == Approx(target.clean_speed_rpm) &&
                   pending->parking_side == target.parking_side;
        },
        std::chrono::seconds(120)));

    REQUIRE(fs::exists(pending_cfg_path));
    const auto pending = f.tb_cfg->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(target.passes));
    CHECK(pending->clean_speed_rpm == Approx(target.clean_speed_rpm));
    CHECK(pending->parking_side == target.parking_side);
}

TEST_CASE("Real ThingsBoard rejects unsupported shared attributes",
          "[integration][thingsboard][real][shared_attr][rejected]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard rejected shared attribute test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.connect_and_request_shared_snapshot());
    REQUIRE(f.wait_for_initial_snapshot());

    const auto before_active = f.tb_cfg->active_config();
    const auto before_pending = f.tb_cfg->pending_config();
    const int before_attr_count = f.shared_attr_updates();

    spdlog::warn("[TB real test] ACTION REQUIRED: in ThingsBoard shared attributes, set "
                 "passes=0.5");

    REQUIRE(wait_until(
        [&] {
            if (f.shared_attr_updates() <= before_attr_count) {
                return false;
            }
            return f.tb_cfg->active_config() == before_active &&
                   f.tb_cfg->pending_config() == before_pending;
        },
        std::chrono::seconds(120)));

    const auto result = f.shared_attr_result();
    REQUIRE(result.has_value());
    CHECK_FALSE(result->accepted);
    CHECK(result->reason == "passes must be a positive integer");
    CHECK(f.tb_cfg->active_config() == before_active);
    CHECK(f.tb_cfg->pending_config() == before_pending);
}

TEST_CASE("Real ThingsBoard shared attributes modified while device offline are delivered on reconnect",
          "[integration][thingsboard][real][shared_attr][reconnect]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard offline shared attribute test");
        return;
    }

    const auto paths = find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using config: {}", paths->config_path.string());
    RealThingsBoardFixture f(*paths);

    const auto before_pending = f.tb_cfg->pending_config();
    const int before_attr_count = f.shared_attr_updates();
    const auto target = choose_legal_target(before_pending);

    spdlog::warn("[TB real test] ACTION REQUIRED: while device is still offline, set shared "
                 "attributes to passes={}, clean_speed_rpm={}, parking_side={}; then wait",
                 target.passes,
                 target.clean_speed_rpm,
                 target.parking_side_name);

    REQUIRE(f.connect_and_request_shared_snapshot());

    REQUIRE(wait_until(
        [&] {
            if (f.shared_attr_updates() <= before_attr_count) {
                return false;
            }
            const auto pending = f.tb_cfg->pending_config();
            return pending.has_value() &&
                   pending->passes == Approx(target.passes) &&
                   pending->clean_speed_rpm == Approx(target.clean_speed_rpm) &&
                   pending->parking_side == target.parking_side;
        },
        std::chrono::seconds(120)));

    const auto pending = f.tb_cfg->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(target.passes));
    CHECK(pending->clean_speed_rpm == Approx(target.clean_speed_rpm));
    CHECK(pending->parking_side == target.parking_side);
}
