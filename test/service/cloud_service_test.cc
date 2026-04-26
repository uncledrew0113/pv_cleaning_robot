#include <catch2/catch.hpp>

#include <memory>
#include <string>

#include "pv_cleaning_robot/service/cloud_service.h"

using robot::middleware::DataCache;
using robot::middleware::INetworkTransport;
using robot::middleware::NetworkManager;
using robot::service::CloudService;

namespace {

struct MockTransport final : INetworkTransport {
    MessageCallback rpc_cb;
    MessageCallback attr_cb;

    bool connect() override { return true; }
    void disconnect() override {}
    bool is_connected() const override { return true; }
    bool publish(const std::string&, const std::string&) override { return true; }
    bool subscribe(const std::string& topic, MessageCallback cb) override {
        if (topic.find("attributes") != std::string::npos) {
            attr_cb = std::move(cb);
        } else {
            rpc_cb = std::move(cb);
        }
        return true;
    }

    void emit_attributes(const std::string& payload) {
        if (attr_cb) {
            attr_cb("v1/devices/me/attributes", payload);
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
    cloud.subscribe_shared_attributes([&](const nlohmann::json& attrs) {
        REQUIRE(attrs.at("passes").get<int>() == 3);
        ++call_count;
    });
    mqtt->emit_attributes(R"({"passes":3})");

    REQUIRE(call_count == 1);
}
