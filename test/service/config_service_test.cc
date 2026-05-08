/**
 * ConfigService 单元测试
 * [service][config]
 */
#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <rapidjson/document.h>

#include "pv_cleaning_robot/service/config_service.h"

using robot::service::ConfigService;
namespace fs = std::filesystem;

namespace {

rapidjson::Value* require_object_member(rapidjson::Value& parent,
                                        const char* key,
                                        rapidjson::Document::AllocatorType& alloc)
{
    auto it = parent.FindMember(key);
    if (it == parent.MemberEnd()) {
        rapidjson::Value name(key, alloc);
        rapidjson::Value child(rapidjson::kObjectType);
        parent.AddMember(name, child, alloc);
        it = parent.FindMember(key);
    }
    return &it->value;
}

void set_double(rapidjson::Document& doc,
                const char* section,
                const char* key,
                double value)
{
    auto* object = require_object_member(doc, section, doc.GetAllocator());
    auto it = object->FindMember(key);
    if (it == object->MemberEnd()) {
        rapidjson::Value name(key, doc.GetAllocator());
        object->AddMember(name, rapidjson::Value(value), doc.GetAllocator());
    } else {
        it->value.SetDouble(value);
    }
}

void set_int(rapidjson::Document& doc,
             const char* section,
             const char* key,
             int value)
{
    auto* object = require_object_member(doc, section, doc.GetAllocator());
    auto it = object->FindMember(key);
    if (it == object->MemberEnd()) {
        rapidjson::Value name(key, doc.GetAllocator());
        object->AddMember(name, rapidjson::Value(value), doc.GetAllocator());
    } else {
        it->value.SetInt(value);
    }
}

struct LegacyConfigFixture {
    std::string path{"/tmp/test_config_service_legacy.json"};
    std::string pending_path{"/tmp/test_config_service_legacy.pending.json"};
    std::string backup_path{"/tmp/test_config_service_legacy.backup.json"};

    LegacyConfigFixture()
    {
        std::ofstream f(path);
        f << R"({
  "robot": { "clean_speed_rpm": 300.0, "name": "pv_bot" },
  "network": { "mqtt": { "broker_uri": "tcp://127.0.0.1:1883", "port": 1883 } },
  "diagnostics": { "mode": "production", "enabled": true },
  "system": { "value": 42 }
})";
    }

    ~LegacyConfigFixture()
    {
        fs::remove(path);
        fs::remove(pending_path);
        fs::remove(backup_path);
    }
};

struct SplitConfigFixture {
    std::string runtime_path{"/tmp/test_runtime_config_service.runtime.json"};
    std::string fixed_path{"/tmp/test_runtime_config_service.fixed.json"};
    std::string pending_path{"/tmp/test_runtime_config_service.runtime.pending.json"};
    std::string backup_path{"/tmp/test_runtime_config_service.runtime.backup.json"};

    SplitConfigFixture()
    {
        {
            std::ofstream fixed(fixed_path);
            fixed << R"({
  "gpio": {
    "left_limit": { "line": 12 },
    "right_limit": { "line": 13 }
  },
  "network": {
    "mqtt": { "client_id": "fixed_client" }
  }
})";
        }

        {
            std::ofstream runtime(runtime_path);
            runtime << R"({
  "robot": {
    "clean_speed_rpm": 320.0,
    "passes": 1.0,
    "parking_side": "left"
  }
})";
        }

        {
            std::ofstream pending(pending_path);
            pending << R"({
  "robot": {
    "passes": 3.0,
    "parking_side": "right"
  }
})";
        }
    }

    ~SplitConfigFixture()
    {
        fs::remove(runtime_path);
        fs::remove(fixed_path);
        fs::remove(pending_path);
        fs::remove(backup_path);
    }
};

}  // namespace

TEST_CASE("ConfigService: load() 成功读取合法 JSON 文件", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.is_loaded());
}

TEST_CASE("ConfigService: load() 文件不存在时返回 false", "[service][config]")
{
    ConfigService cfg("/tmp/non_existent_config.json");
    REQUIRE_FALSE(cfg.load());
    REQUIRE_FALSE(cfg.is_loaded());
}

TEST_CASE("ConfigService: fixed/runtime/pending 三份配置独立加载", "[service][config]")
{
    SplitConfigFixture f;
    ConfigService cfg(f.runtime_path, f.fixed_path);

    REQUIRE(cfg.load());
    REQUIRE(cfg.load_fixed());

    CHECK(cfg.runtime_path() == f.runtime_path);
    CHECK(cfg.fixed_path() == f.fixed_path);
    CHECK(cfg.pending_path() == f.pending_path);

    CHECK(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(320.0f).epsilon(0.01f));
    CHECK(cfg.get_fixed<int>("gpio.left_limit.line", -1) == 12);
    CHECK(cfg.get_fixed<std::string>("network.mqtt.client_id", "") == "fixed_client");

    const auto pending = cfg.load_pending();
    REQUIRE(pending.has_value());
    const auto robot_it = pending->FindMember("robot");
    REQUIRE(robot_it != pending->MemberEnd());
    CHECK(robot_it->value["passes"].GetDouble() == Approx(3.0).epsilon(0.01));
    CHECK(std::string(robot_it->value["parking_side"].GetString()) == "right");
}

TEST_CASE("ConfigService: clear_pending() 清除 runtime pending 文件", "[service][config]")
{
    SplitConfigFixture f;
    ConfigService cfg(f.runtime_path, f.fixed_path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.clear_pending());
    CHECK_FALSE(fs::exists(f.pending_path));
    CHECK_FALSE(cfg.load_pending().has_value());
}

TEST_CASE("ConfigService: get<float>() 读取嵌套路径", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const float spd = cfg.get<float>("robot.clean_speed_rpm", 0.0f);
    REQUIRE(spd == Approx(300.0f).epsilon(0.001f));
}

TEST_CASE("ConfigService: get<std::string>() 读取字符串值", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const auto uri = cfg.get<std::string>("network.mqtt.broker_uri", "");
    REQUIRE(uri == "tcp://127.0.0.1:1883");
}

TEST_CASE("ConfigService: get<int>() 读取整数值", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const int port = cfg.get<int>("network.mqtt.port", 0);
    REQUIRE(port == 1883);
}

TEST_CASE("ConfigService: get<bool>() 读取布尔值", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const bool en = cfg.get<bool>("diagnostics.enabled", false);
    REQUIRE(en == true);
}

TEST_CASE("ConfigService: get() 路径不存在时返回默认值", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const auto val = cfg.get<std::string>("non.existent.key", "default_val");
    REQUIRE(val == "default_val");

    const int ival = cfg.get<int>("also.missing", 99);
    REQUIRE(ival == 99);
}

TEST_CASE("ConfigService: 未 load() 时 get() 返回默认值", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    const auto val = cfg.get<float>("robot.clean_speed_rpm", 1.0f);
    REQUIRE(val == Approx(1.0f).epsilon(0.001f));
}

TEST_CASE("ConfigService: set() 修改内存值，get() 立即生效", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    cfg.set<float>("robot.clean_speed_rpm", 500.0f);
    REQUIRE(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(500.0f).epsilon(0.01f));
}

TEST_CASE("ConfigService: save() 后重新 load() 持久化 set() 的值", "[service][config]")
{
    LegacyConfigFixture f;
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

TEST_CASE("ConfigService: replace_and_save() 原子替换整份配置并持久化", "[service][config]")
{
    LegacyConfigFixture f;
    {
        ConfigService cfg(f.path);
        REQUIRE(cfg.load());

        auto next = cfg.snapshot();
        set_double(next, "robot", "clean_speed_rpm", 450.0);
        set_double(next, "robot", "passes", 2.0);
        set_int(next, "system", "value", 777);

        REQUIRE(cfg.replace_and_save(next));
        REQUIRE(cfg.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(450.0f).epsilon(0.01f));
        REQUIRE(cfg.get<float>("robot.passes", 0.0f) == Approx(2.0f).epsilon(0.01f));
        REQUIRE(cfg.get<int>("system.value", 0) == 777);
    }
    {
        ConfigService cfg2(f.path);
        REQUIRE(cfg2.load());
        REQUIRE(cfg2.get<float>("robot.clean_speed_rpm", 0.0f) == Approx(450.0f).epsilon(0.01f));
        REQUIRE(cfg2.get<float>("robot.passes", 0.0f) == Approx(2.0f).epsilon(0.01f));
        REQUIRE(cfg2.get<int>("system.value", 0) == 777);
    }
}

TEST_CASE("ConfigService: replace_and_save() 会先写 backup 快照", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    REQUIRE(cfg.load());

    auto next = cfg.snapshot();
    set_int(next, "system", "value", 123);

    REQUIRE(cfg.replace_and_save(next));
    REQUIRE(fs::exists(f.backup_path));

    ConfigService backup_cfg(f.backup_path);
    REQUIRE(backup_cfg.load());
    REQUIRE(backup_cfg.get<int>("system.value", 0) == 42);
}

TEST_CASE("ConfigService: load() 在主配置损坏时自动回退到 backup", "[service][config]")
{
    LegacyConfigFixture f;
    {
        std::ofstream bad_main(f.path, std::ios::trunc);
        bad_main << "{ invalid json";
    }
    {
        std::ofstream good_backup(f.backup_path, std::ios::trunc);
        good_backup << R"({
  "robot": { "clean_speed_rpm": 280.0, "name": "backup_bot" },
  "network": { "mqtt": { "broker_uri": "tcp://127.0.0.1:1883", "port": 1883 } },
  "diagnostics": { "mode": "production", "enabled": true },
  "system": { "value": 314 }
})";
    }

    ConfigService cfg(f.path);
    REQUIRE(cfg.load());
    REQUIRE(cfg.last_load_used_backup());
    REQUIRE(cfg.get<int>("system.value", 0) == 314);
    REQUIRE(cfg.get<std::string>("robot.name", "") == "backup_bot");
}

TEST_CASE("ConfigService: save_pending/load_pending/clear_pending 管理待生效配置",
          "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    REQUIRE(cfg.load());

    auto pending = cfg.snapshot();
    set_double(pending, "robot", "passes", 3.0);
    set_double(pending, "robot", "clean_speed_rpm", 520.0);

    REQUIRE(cfg.save_pending(pending));
    REQUIRE(fs::exists(f.pending_path));

    const auto loaded = cfg.load_pending();
    REQUIRE(loaded.has_value());
    const auto robot_it = loaded->FindMember("robot");
    REQUIRE(robot_it != loaded->MemberEnd());
    REQUIRE(robot_it->value["passes"].GetDouble() == Approx(3.0).epsilon(0.01));
    REQUIRE(robot_it->value["clean_speed_rpm"].GetDouble() == Approx(520.0).epsilon(0.01));

    REQUIRE(cfg.clear_pending());
    REQUIRE_FALSE(fs::exists(f.pending_path));
    REQUIRE_FALSE(cfg.load_pending().has_value());
}

TEST_CASE("ConfigService: get_subtree() 返回子树", "[service][config]")
{
    LegacyConfigFixture f;
    ConfigService cfg(f.path);
    cfg.load();

    const auto tree = cfg.get_subtree("network.mqtt");
    const auto port_it = tree.FindMember("port");
    REQUIRE(port_it != tree.MemberEnd());
    REQUIRE(port_it->value.GetInt() == 1883);
}
