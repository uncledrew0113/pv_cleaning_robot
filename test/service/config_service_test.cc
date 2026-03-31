/**
 * ConfigService 单元测试
 * [service][config]
 */
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>

#include "pv_cleaning_robot/service/config_service.h"

using robot::service::ConfigService;
namespace fs = std::filesystem;

// ────────────────────────────────────────────────────────────────
// 辅助：写临时 JSON 文件
// ────────────────────────────────────────────────────────────────
struct ConfigFixture {
    std::string path{"/tmp/test_config_service.json"};

    ConfigFixture() {
        std::ofstream f(path);
        f << R"({
  "robot": { "clean_speed_rpm": 300.0, "name": "pv_bot" },
  "network": { "mqtt": { "broker_uri": "tcp://127.0.0.1:1883", "port": 1883 } },
  "diagnostics": { "mode": "production", "enabled": true },
  "system": { "value": 42 }
})";
    }
    ~ConfigFixture() {
        fs::remove(path);
    }
};

// ────────────────────────────────────────────────────────────────
// load()
// ────────────────────────────────────────────────────────────────
TEST_CASE("ConfigService: load() 成功读取合法 JSON 文件", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.is_loaded());
}

TEST_CASE("ConfigService: load() 文件不存在时返回 false", "[service][config]") {
    ConfigService cfg("/tmp/non_existent_config.json");
    REQUIRE_FALSE(cfg.load());
    REQUIRE_FALSE(cfg.is_loaded());
}

// ────────────────────────────────────────────────────────────────
// get<T>() 各类型
// ────────────────────────────────────────────────────────────────
TEST_CASE("ConfigService: get<float>() 读取嵌套路径", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    float spd = cfg.get<float>("robot.clean_speed_rpm", 0.0f);
    REQUIRE(spd == Approx(300.0f).epsilon(0.001f));
}

TEST_CASE("ConfigService: get<std::string>() 读取字符串值", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    auto uri = cfg.get<std::string>("network.mqtt.broker_uri", "");
    REQUIRE(uri == "tcp://127.0.0.1:1883");
}

TEST_CASE("ConfigService: get<int>() 读取整数值", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    int port = cfg.get<int>("network.mqtt.port", 0);
    REQUIRE(port == 1883);
}

TEST_CASE("ConfigService: get<bool>() 读取布尔值", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    bool en = cfg.get<bool>("diagnostics.enabled", false);
    REQUIRE(en == true);
}

// ────────────────────────────────────────────────────────────────
// 默认值
// ────────────────────────────────────────────────────────────────
TEST_CASE("ConfigService: get() 路径不存在时返回默认值", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    auto val = cfg.get<std::string>("non.existent.key", "default_val");
    REQUIRE(val == "default_val");

    int ival = cfg.get<int>("also.missing", 99);
    REQUIRE(ival == 99);
}

TEST_CASE("ConfigService: 未 load() 时 get() 返回默认值", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    // 不调用 load()
    auto val = cfg.get<float>("robot.clean_speed_rpm", 1.0f);
    REQUIRE(val == Approx(1.0f).epsilon(0.001f));
}

// ────────────────────────────────────────────────────────────────
// set() + save() + 重新 load()
// ────────────────────────────────────────────────────────────────
TEST_CASE("ConfigService: set() 修改内存值，get() 立即生效", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    cfg.set<float>("robot.clean_speed_rpm", 500.0f);
    REQUIRE(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(500.0f).epsilon(0.01f));
}

TEST_CASE("ConfigService: save() 后重新 load() 持久化 set() 的值", "[service][config]") {
    ConfigFixture f;
    {
        ConfigService cfg(f.path);
        cfg.load();
        cfg.set<int>("system.value", 999);
        REQUIRE(cfg.save());
    }
    {
        ConfigService cfg2(f.path);
        cfg2.load();
        REQUIRE(cfg2.get<int>("system.value", 0) == 999);
    }
}

// ────────────────────────────────────────────────────────────────
// get_subtree()
// ────────────────────────────────────────────────────────────────
TEST_CASE("ConfigService: get_subtree() 返回子树", "[service][config]") {
    ConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    auto tree = cfg.get_subtree("network.mqtt");
    REQUIRE(tree.contains("port"));
    REQUIRE(tree["port"].get<int>() == 1883);
}
