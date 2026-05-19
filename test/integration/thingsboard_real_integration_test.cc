#include <catch2/catch.hpp>

#include <array>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"
#include "integration/thingsboard_test_support.h"

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

void override_diagnostics_mode(const tb_test_support::TempSplitConfigPaths& paths,
                               const char* mode)
{
    REQUIRE(mode != nullptr);

    std::ifstream in(paths.fixed_path);
    REQUIRE(in.is_open());
    const std::string original((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(original.empty());

    const auto diagnostics_pos = original.find("\"diagnostics\"");
    REQUIRE(diagnostics_pos != std::string::npos);

    const auto mode_key_pos = original.find("\"mode\"", diagnostics_pos);
    REQUIRE(mode_key_pos != std::string::npos);

    const auto colon_pos = original.find(':', mode_key_pos);
    REQUIRE(colon_pos != std::string::npos);

    const auto value_begin = original.find('"', colon_pos + 1);
    REQUIRE(value_begin != std::string::npos);

    const auto value_end = original.find('"', value_begin + 1);
    REQUIRE(value_end != std::string::npos);

    std::string updated = original;
    updated.replace(value_begin + 1, value_end - value_begin - 1, mode);
    tb_test_support::write_text_file(paths.fixed_path, updated);
}

std::string build_sample_health_payload()
{
    std::array<char, 4096> out{};
    robot::service::HealthPayloadBuilder::HealthView view{};
    view.ts_ms = 1778481758995ULL;
    view.walk.wheel[0].torque_a = -0.0141f;
    view.walk.wheel[0].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[1].torque_a = -0.008057f;
    view.walk.wheel[1].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[2].torque_a = 0.024171f;
    view.walk.wheel[2].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[3].torque_a = 0.0141f;
    view.walk.wheel[3].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.imu.pitch_deg = -4.350586f;
    view.imu.roll_deg = -1.049194f;
    view.imu.yaw_deg = 2.971802f;
    view.gps.latitude = 24.683938;
    view.gps.longitude = 118.220434;
    view.gps.fix_quality = 2;

    const size_t len =
        robot::service::HealthPayloadBuilder::build_health(view, out.data(), out.size());
    REQUIRE(len > 0u);
    return std::string(out.data(), len);
}

std::string build_sample_diagnostics_payload()
{
    std::array<char, 8192> out{};
    robot::service::HealthPayloadBuilder::DiagnosticsView view{};
    view.ts_ms = 1778481758995ULL;

    view.walk.wheel[0].torque_a = -0.0141f;
    view.walk.wheel[0].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[0].online = true;
    view.walk.wheel[1].torque_a = -0.008057f;
    view.walk.wheel[1].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[1].online = true;
    view.walk.wheel[2].torque_a = 0.024171f;
    view.walk.wheel[2].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[2].online = true;
    view.walk.wheel[3].torque_a = 0.0141f;
    view.walk.wheel[3].fault = robot::protocol::WalkMotorFault::OVER_VOLTAGE;
    view.walk.wheel[3].online = true;

    view.walk.ctrl_frame_count = 4u;
    view.brush.current_a = 0.100231f;
    view.brush.bus_voltage_v = 26.543409f;
    view.brush.temperature_c = 40.910069f;
    view.imu.accel[0] = 0.746484f;
    view.imu.accel[1] = -0.138770f;
    view.imu.accel[2] = 9.790430f;
    view.imu.gyro[0] = -0.001065f;
    view.imu.gyro[1] = -0.004261f;
    view.imu.pitch_deg = -4.350586f;
    view.imu.roll_deg = -1.049194f;
    view.imu.yaw_deg = 2.971802f;
    view.imu.frame_rate_hz = 81.947891f;
    view.gps.latitude = 24.683938;
    view.gps.longitude = 118.220434;
    view.gps.altitude_m = 122.331001f;
    view.gps.speed_m_s = 0.280000f;
    view.gps.satellites_used = 1u;
    view.gps.hdop = 3.920000f;
    view.gps.fix_quality = 2;
    view.gps.sentence_count = 81u;

    const size_t len =
        robot::service::HealthPayloadBuilder::build_diagnostics(view, out.data(), out.size());
    REQUIRE(len > 0u);
    return std::string(out.data(), len);
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

SharedAttrTarget choose_legal_target(const std::optional<robot::service::RuntimeConfig>& current)
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

struct RealThingsBoardFixture {
    fs::path repo_root;
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("tb_real_integration")};

    robot::service::ConfigService cfg;
    robot::service::SchedulerService scheduler;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::shared_ptr<robot::middleware::DataCache> cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::atomic<int> shared_attr_count{0};
    mutable std::mutex shared_attr_result_mtx;
    std::optional<robot::service::SharedAttrApplyResult> last_shared_attr_result;

    RealThingsBoardFixture(const tb_test_support::RepoPaths& paths,
                           const char* client_id_suffix = "_real_attr_itest",
                           const char* diagnostics_mode_override = nullptr)
        : repo_root(paths.repo_root)
        , cfg(this->paths.runtime_path.string(), this->paths.fixed_path.string())
        , scheduler()
    {
        tb_test_support::copy_repo_split_config(paths, this->paths);
        if (diagnostics_mode_override != nullptr) {
            override_diagnostics_mode(this->paths, diagnostics_mode_override);
        }
        REQUIRE(cfg.load());
        cfg.apply_active_runtime_schedules(scheduler);
        auto mqtt_cfg = tb_test_support::build_mqtt_config(cfg, repo_root, client_id_suffix);
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
        cache = std::make_shared<robot::middleware::DataCache>(this->paths.cache_path.string());
        REQUIRE(cache->open());
        cloud = std::make_shared<robot::service::CloudService>(net, cache);
        // 真实 shared attributes 联调测试需要在 connect() 前把下行回调接好，
        // 否则平台侧更新虽然发送到了设备，测试里的 cfg 不会收到也不会更新。
        cloud->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
            const auto before = cfg.active_runtime_config();
            const auto result = cfg.apply_runtime_patch(attrs);
            if (result.accepted && cfg.active_runtime_config().schedules != before.schedules) {
                cfg.apply_active_runtime_schedules(scheduler);
            }
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
             "return_brush_rpm",
             "parking_side",
             "start_battery_soc",
             "charge_start_soc",
             "charge_stop_soc",
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
        tb_test_support::cleanup_split_config_paths(paths);
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
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("tb_raw_rpc_smoke")};
    robot::service::ConfigService cfg;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::atomic<int> rpc_count{0};
    mutable std::mutex rpc_mtx;
    std::string last_topic;
    std::string last_payload;

    explicit RawRpcSmokeFixture(const tb_test_support::RepoPaths& paths)
        : repo_root(paths.repo_root)
        , cfg(this->paths.runtime_path.string(), this->paths.fixed_path.string())
    {
        tb_test_support::copy_repo_split_config(paths, this->paths);
        REQUIRE(cfg.load());
        auto mqtt_cfg = tb_test_support::build_mqtt_config(cfg, repo_root, "_raw_rpc_itest");
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
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
        tb_test_support::cleanup_split_config_paths(paths);
    }

    bool connect() { return net->connect(); }

    int received_count() const
    {
        return rpc_count.load(std::memory_order_relaxed);
    }
};

struct CloudRpcSmokeFixture {
    fs::path repo_root;
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("tb_cloud_rpc_smoke")};
    robot::service::ConfigService cfg;
    std::shared_ptr<robot::middleware::MqttTransport> mqtt;
    std::shared_ptr<robot::middleware::NetworkManager> net;
    std::shared_ptr<robot::middleware::DataCache> cache;
    std::shared_ptr<robot::service::CloudService> cloud;
    std::atomic<int> rpc_count{0};
    mutable std::mutex rpc_mtx;
    std::string last_params;

    explicit CloudRpcSmokeFixture(const tb_test_support::RepoPaths& paths)
        : repo_root(paths.repo_root)
        , cfg(this->paths.runtime_path.string(), this->paths.fixed_path.string())
    {
        tb_test_support::copy_repo_split_config(paths, this->paths);
        REQUIRE(cfg.load());
        auto mqtt_cfg = tb_test_support::build_mqtt_config(cfg, repo_root, "_cloud_rpc_itest");
        REQUIRE_FALSE(mqtt_cfg.broker_uri.empty());
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
        cache = std::make_shared<robot::middleware::DataCache>(this->paths.cache_path.string());
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
        tb_test_support::cleanup_split_config_paths(paths);
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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using runtime config: {}", paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);

    REQUIRE(f.connect_and_request_shared_snapshot());
    CHECK(f.net->is_connected());
}

TEST_CASE("Real ThingsBoard health and diagnostics modes stay connected after first telemetry publish",
          "[integration][thingsboard][real][mqtt_stability][health_service_mode]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard health mode stability test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());

    SECTION("production health stays connected after first telemetry publish") {
        RealThingsBoardFixture f(*paths, "_health_mode_prod_itest", "production");
        REQUIRE(f.connect_and_request_shared_snapshot());
        REQUIRE(f.net->is_connected());

        const auto payload = build_sample_health_payload();
        REQUIRE(f.cloud->publish_telemetry(payload));

        const auto hold_time = std::chrono::seconds(10);
        const auto sample_period = std::chrono::milliseconds(200);
        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + hold_time;

        while (std::chrono::steady_clock::now() < deadline) {
            INFO("mode=production elapsed_ms="
                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start)
                        .count());
            REQUIRE(f.net->is_connected());
            std::this_thread::sleep_for(sample_period);
        }
    }

    SECTION("development diagnostics stays connected after first telemetry publish") {
        RealThingsBoardFixture f(*paths, "_health_mode_diag_itest", "development");
        REQUIRE(f.connect_and_request_shared_snapshot());
        REQUIRE(f.net->is_connected());

        const auto payload = build_sample_diagnostics_payload();
        REQUIRE(f.cloud->publish_telemetry(payload));

        const auto hold_time = std::chrono::seconds(10);
        const auto sample_period = std::chrono::milliseconds(200);
        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + hold_time;

        while (std::chrono::steady_clock::now() < deadline) {
            INFO("mode=development elapsed_ms="
                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start)
                        .count());
            REQUIRE(f.net->is_connected());
            std::this_thread::sleep_for(sample_period);
        }
    }
}

TEST_CASE("Real ThingsBoard connection stays stable for 60 seconds",
          "[integration][thingsboard][real][mqtt_stability]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard stability test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB stability test] using runtime config: {}",
                 paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);

    REQUIRE(f.connect_and_request_shared_snapshot());
    REQUIRE(f.net->is_connected());

    const auto hold_time = std::chrono::seconds(60);
    const auto sample_period = std::chrono::milliseconds(200);
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + hold_time;

    size_t sample_count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        INFO("elapsed_ms="
             << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start)
                    .count()
             << " sample_count=" << sample_count);
        REQUIRE(f.net->is_connected());
        std::this_thread::sleep_for(sample_period);
        ++sample_count;
    }

    CHECK(f.net->is_connected());
}

TEST_CASE("Real ThingsBoard stays stable when shared attributes callback publishes telemetry",
          "[integration][thingsboard][real][mqtt_stability][attr_callback_publish]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard callback-publish stability test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB callback-publish stability test] using runtime config: {}",
                 paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);

    f.cloud->subscribe_shared_attributes([&f](const rapidjson::Document& attrs) {
        const auto before = f.cfg.active_runtime_config();
        const auto result = f.cfg.apply_runtime_patch(attrs);
        if (result.accepted && f.cfg.active_runtime_config().schedules != before.schedules) {
            f.cfg.apply_active_runtime_schedules(f.scheduler);
        }
        {
            std::lock_guard<std::mutex> lk(f.shared_attr_result_mtx);
            f.last_shared_attr_result = result;
        }
        f.shared_attr_count.fetch_add(1, std::memory_order_relaxed);

        std::array<char, 1024> payload{};
        const auto reason = result.reason.empty() ? "ok" : result.reason.c_str();
        const size_t len = robot::service::ThingsBoardJsonCodec::build_status_event(
            {"shared_attr_update", result.accepted, reason}, payload.data(), payload.size());
        if (len == 0u) {
            spdlog::error("[TB callback-publish stability test] failed to build status event");
            return;
        }
        const bool ok = f.cloud->publish_telemetry(std::string(payload.data(), len));
        spdlog::info(
            "[TB callback-publish stability test] published shared_attr_update from callback: ok={}",
            ok);
    });

    REQUIRE(f.net->connect());
    REQUIRE(f.net->is_connected());

    const std::string software_version = f.cfg.get<std::string>("device.software_version", "");
    const std::string hardware_version = f.cfg.get<std::string>("device.hardware_version", "");
    const std::string device_model = f.cfg.get<std::string>("device.model", "");
    const std::string device_id = f.cfg.get<std::string>("network.mqtt.client_id", "pv_robot_001");

    std::array<char, 1024> startup_attr_buf{};
    const auto startup_attr_len = robot::service::ThingsBoardJsonCodec::build_startup_attributes(
        {
            software_version.c_str(),
            hardware_version.c_str(),
            device_model.c_str(),
            device_id.c_str(),
        },
        startup_attr_buf.data(),
        startup_attr_buf.size());
    REQUIRE(startup_attr_len > 0u);
    REQUIRE(f.cloud->publish_attributes(std::string(startup_attr_buf.data(), startup_attr_len)));

    std::atomic<bool> stop_publishers{false};
    std::atomic<uint32_t> publisher_ticks{0};
    std::thread concurrent_publisher([&]() {
        while (!stop_publishers.load(std::memory_order_relaxed)) {
            robot::app::RobotRuntimeSnapshot snap;
            snap.device_state = "Idle";
            snap.task_state = "IdleTask";
            snap.active_config = f.cfg.active_runtime_config();
            snap.active_config_version = publisher_ticks.fetch_add(1, std::memory_order_relaxed) + 1;

            std::array<char, 4096> telemetry_buf{};
            const auto telemetry_len = robot::service::ThingsBoardJsonCodec::build_business_telemetry(
                snap, telemetry_buf.data(), telemetry_buf.size());
            if (telemetry_len > 0u) {
                f.cloud->publish_telemetry(std::string(telemetry_buf.data(), telemetry_len));
            }

            std::array<char, 1024> event_buf{};
            const auto event_len = robot::service::ThingsBoardJsonCodec::build_status_event(
                {"startup_position_invalid", false, "robot_not_at_any_endpoint"},
                event_buf.data(),
                event_buf.size());
            if (event_len > 0u) {
                f.cloud->publish_telemetry(std::string(event_buf.data(), event_len));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    const auto stop_and_join_publishers = [&]() {
        stop_publishers.store(true, std::memory_order_relaxed);
        if (concurrent_publisher.joinable()) {
            concurrent_publisher.join();
        }
    };

    const std::vector<std::string> shared_keys{
        "passes",
        "clean_speed_rpm",
        "return_speed_rpm",
        "brush_rpm",
        "return_brush_rpm",
        "parking_side",
        "start_battery_soc",
        "charge_start_soc",
        "charge_stop_soc",
        "schedules",
    };
    REQUIRE(f.cloud->request_shared_attributes_snapshot(shared_keys));
    REQUIRE(f.wait_for_initial_snapshot());

    const auto hold_time = std::chrono::seconds(60);
    const auto sample_period = std::chrono::milliseconds(200);
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + hold_time;

    size_t sample_count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        INFO("elapsed_ms="
             << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start)
                    .count()
             << " sample_count=" << sample_count
             << " shared_attr_updates=" << f.shared_attr_updates()
             << " publisher_ticks=" << publisher_ticks.load(std::memory_order_relaxed));
        if (!f.net->is_connected()) {
            stop_and_join_publishers();
        }
        REQUIRE(f.net->is_connected());
        std::this_thread::sleep_for(sample_period);
        ++sample_count;
    }

    stop_and_join_publishers();
    CHECK(f.net->is_connected());
}

TEST_CASE("Real ThingsBoard raw RPC smoke", "[integration][thingsboard][real][rpc_smoke]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to run real ThingsBoard RPC smoke test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB rpc smoke] using runtime config: {}", paths->runtime_config_path.string());

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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB cloud rpc smoke] using runtime config: {}", paths->runtime_config_path.string());

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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using runtime config: {}", paths->runtime_config_path.string());
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
    snap.active_config = f.cfg.active_runtime_config();
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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using runtime config: {}", paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.connect_and_request_shared_snapshot());
    REQUIRE(f.wait_for_initial_snapshot());

    const auto pending_cfg_path = f.paths.pending_path;
    const auto before_pending = f.cfg.pending_runtime_config();
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
            const auto pending = f.cfg.pending_runtime_config();
            return pending.has_value() &&
                   pending->passes == Approx(target.passes) &&
                   pending->clean_speed_rpm == Approx(target.clean_speed_rpm) &&
                   pending->parking_side == target.parking_side;
        },
        std::chrono::seconds(120)));

    REQUIRE(fs::exists(pending_cfg_path));
    const auto pending = f.cfg.pending_runtime_config();
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

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using runtime config: {}", paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);
    REQUIRE(f.connect_and_request_shared_snapshot());
    REQUIRE(f.wait_for_initial_snapshot());

    const auto before_active = f.cfg.active_runtime_config();
    const auto before_pending = f.cfg.pending_runtime_config();
    const int before_attr_count = f.shared_attr_updates();

    spdlog::warn("[TB real test] ACTION REQUIRED: in ThingsBoard shared attributes, set "
                 "passes=0.5");

    REQUIRE(wait_until(
        [&] {
            if (f.shared_attr_updates() <= before_attr_count) {
                return false;
            }
            return f.cfg.active_runtime_config() == before_active &&
                   f.cfg.pending_runtime_config() == before_pending;
        },
        std::chrono::seconds(120)));

    const auto result = f.shared_attr_result();
    REQUIRE(result.has_value());
    CHECK_FALSE(result->accepted);
    CHECK(result->reason == "passes must be a positive integer");
    CHECK(f.cfg.active_runtime_config() == before_active);
    CHECK(f.cfg.pending_runtime_config() == before_pending);
}

TEST_CASE("Real ThingsBoard shared attributes modified while device offline are delivered on reconnect",
          "[integration][thingsboard][real][shared_attr][reconnect]") {
    if (!real_tb_test_enabled()) {
        SUCCEED("Set TB_REAL_TEST=1 to enable real ThingsBoard offline shared attribute test");
        return;
    }

    const auto paths = tb_test_support::find_repo_paths();
    REQUIRE(paths.has_value());
    spdlog::info("[TB real test] using runtime config: {}", paths->runtime_config_path.string());
    RealThingsBoardFixture f(*paths);

    const auto before_pending = f.cfg.pending_runtime_config();
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
            const auto pending = f.cfg.pending_runtime_config();
            return pending.has_value() &&
                   pending->passes == Approx(target.passes) &&
                   pending->clean_speed_rpm == Approx(target.clean_speed_rpm) &&
                   pending->parking_side == target.parking_side;
        },
        std::chrono::seconds(120)));

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(target.passes));
    CHECK(pending->clean_speed_rpm == Approx(target.clean_speed_rpm));
    CHECK(pending->parking_side == target.parking_side);
}
