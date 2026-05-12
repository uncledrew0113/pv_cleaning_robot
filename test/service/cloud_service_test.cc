#include <catch2/catch.hpp>

#include <memory>
#include <rapidjson/document.h>
#include <string>

#include "pv_cleaning_robot/service/cloud_service.h"

using robot::middleware::DataCache;
using robot::middleware::INetworkTransport;
using robot::middleware::NetworkManager;
using robot::service::CloudService;

namespace {

struct MockTransport final : INetworkTransport {
    MessageCallback rpc_cb;
    MessageCallback attr_update_cb;
    MessageCallback attr_response_cb;
    std::string last_publish_topic;
    std::string last_publish_payload;

    bool connect() override { return true; }
    void disconnect() override {}
    bool is_connected() const override { return true; }
    bool publish(const std::string& topic, const std::string& payload) override {
        last_publish_topic = topic;
        last_publish_payload = payload;
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

    void emit_attributes_response(const std::string& request_id, const std::string& payload) {
        if (attr_response_cb) {
            attr_response_cb("v1/devices/me/attributes/response/" + request_id, payload);
        }
    }

    void emit_rpc(const std::string& request_id, const std::string& payload) {
        if (rpc_cb) {
            rpc_cb("v1/devices/me/rpc/request/" + request_id, payload);
        }
    }
};

}  // namespace

TEST_CASE("CloudService shared attributes ignore messages before callback registration",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_test.jsonl");
    CloudService cloud(net, cache);

    int call_count = 0;

    mqtt->emit_attributes(R"({"passes":2})");
    cloud.subscribe_shared_attributes([&](const rapidjson::Document& attrs) {
        const auto it = attrs.FindMember("passes");
        REQUIRE(it != attrs.MemberEnd());
        REQUIRE(it->value.GetInt() == 3);
        ++call_count;
    });
    mqtt->emit_attributes(R"({"passes":3})");

    REQUIRE(call_count == 1);
}

TEST_CASE("CloudService exact shared attributes route preserves string and int fields",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_exact_test.jsonl");
    CloudService cloud(net, cache);

    int call_count = 0;
    cloud.subscribe_shared_attributes([&](const rapidjson::Document& attrs) {
        REQUIRE(attrs.IsObject());
        const auto brush_it = attrs.FindMember("brush_rpm");
        const auto side_it = attrs.FindMember("parking_side");
        REQUIRE(brush_it != attrs.MemberEnd());
        REQUIRE(side_it != attrs.MemberEnd());
        REQUIRE(brush_it->value.IsInt());
        REQUIRE(brush_it->value.GetInt() == 30);
        REQUIRE(side_it->value.IsString());
        REQUIRE(std::string(side_it->value.GetString()) == "right");
        ++call_count;
    });

    mqtt->emit_attributes(R"({"brush_rpm":30,"parking_side":"right"})");

    REQUIRE(call_count == 1);
}

TEST_CASE("CloudService RPC parsing preserves params JSON string for handler",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_rpc_test.jsonl");
    CloudService cloud(net, cache);

    std::string seen_request_id;
    std::string handler_params;
    cloud.register_rpc("set_speed", [&](const std::string& request_id, const std::string& params) {
        seen_request_id = request_id;
        handler_params = params;
        return std::string{R"({"ok":true})"};
    });

    mqtt->emit_rpc("42", R"({"method":"set_speed","params":{"speed":80,"mode":"clean"}})");

    REQUIRE(seen_request_id == "42");
    REQUIRE(handler_params == R"({"speed":80,"mode":"clean"})");
    REQUIRE(mqtt->last_publish_topic == "v1/devices/me/rpc/response/42");
    REQUIRE(mqtt->last_publish_payload == R"({"ok":true})");
}

TEST_CASE("CloudService requests shared attributes snapshot using ThingsBoard topics",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_request_test.jsonl");
    CloudService cloud(net, cache);

    REQUIRE(cloud.request_shared_attributes_snapshot({"passes", "clean_speed_rpm", "parking_side"}));
    REQUIRE(mqtt->last_publish_topic.find("v1/devices/me/attributes/request/") == 0);
    REQUIRE(mqtt->last_publish_payload ==
            R"({"sharedKeys":"passes,clean_speed_rpm,parking_side"})");
}

TEST_CASE("CloudService rejects unknown RPC methods with explicit response",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_unknown_rpc_test.jsonl");
    CloudService cloud(net, cache);

    cloud.register_rpc("start", [](const std::string&, const std::string&) {
        return std::string{R"({"accepted":true,"result":"ok"})"};
    });

    mqtt->emit_rpc("99", R"({"method":"stop","params":{}})");

    REQUIRE(mqtt->last_publish_topic == "v1/devices/me/rpc/response/99");
    rapidjson::Document response;
    response.Parse(mqtt->last_publish_payload.c_str());
    REQUIRE_FALSE(response.HasParseError());
    REQUIRE(response.IsObject());
    REQUIRE(response["accepted"].IsBool());
    CHECK_FALSE(response["accepted"].GetBool());
    REQUIRE(response["result"].IsString());
    CHECK(std::string(response["result"].GetString()) == "rejected");
    REQUIRE(response["reason"].IsString());
    CHECK(std::string(response["reason"].GetString()) == "method_not_supported");
}

TEST_CASE("CloudService rejects invalid RPC payload with explicit response",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_invalid_rpc_test.jsonl");
    CloudService cloud(net, cache);

    mqtt->emit_rpc("7", R"({"method":123,"params":{}})");

    REQUIRE(mqtt->last_publish_topic == "v1/devices/me/rpc/response/7");
    rapidjson::Document response;
    response.Parse(mqtt->last_publish_payload.c_str());
    REQUIRE_FALSE(response.HasParseError());
    REQUIRE(response.IsObject());
    REQUIRE(response["accepted"].IsBool());
    CHECK_FALSE(response["accepted"].GetBool());
    REQUIRE(response["result"].IsString());
    CHECK(std::string(response["result"].GetString()) == "rejected");
    REQUIRE(response["reason"].IsString());
    CHECK(std::string(response["reason"].GetString()) == "invalid_request");
}

TEST_CASE("CloudService shared attributes response routes nested shared object to callback",
          "[service][cloud]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto net = std::make_shared<NetworkManager>(mqtt, nullptr, NetworkManager::Mode::MQTT_ONLY);
    auto cache = std::make_shared<DataCache>("/tmp/cloud_service_attr_response_test.jsonl");
    CloudService cloud(net, cache);

    int call_count = 0;
    cloud.subscribe_shared_attributes([&](const rapidjson::Document& attrs) {
        REQUIRE(attrs.IsObject());
        const auto passes_it = attrs.FindMember("passes");
        const auto side_it = attrs.FindMember("parking_side");
        REQUIRE(passes_it != attrs.MemberEnd());
        REQUIRE(side_it != attrs.MemberEnd());
        REQUIRE(passes_it->value.GetInt() == 3);
        REQUIRE(std::string(side_it->value.GetString()) == "right");
        ++call_count;
    });

    mqtt->emit_attributes_response(
        "7", R"({"shared":{"passes":3,"parking_side":"right"}})");

    REQUIRE(call_count == 1);
}
