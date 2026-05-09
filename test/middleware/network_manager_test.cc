#include <catch2/catch.hpp>

#include "pv_cleaning_robot/middleware/network_manager.h"

using robot::middleware::INetworkTransport;
using robot::middleware::NetworkManager;

namespace {

struct MockTransport : INetworkTransport {
    bool connect_result{true};
    bool publish_result{true};
    bool subscribe_result{true};
    bool connected{false};
    bool connect_called{false};
    bool publish_called{false};
    bool subscribe_called{false};

    bool connect() override {
        connect_called = true;
        connected = connect_result;
        return connect_result;
    }

    void disconnect() override {
        connected = false;
    }

    bool is_connected() const override {
        return connected;
    }

    bool publish(const std::string&, const std::string&) override {
        publish_called = true;
        return publish_result;
    }

    bool subscribe(const std::string&, MessageCallback) override {
        subscribe_called = true;
        return subscribe_result;
    }
};

}  // namespace

TEST_CASE("NetworkManager: LORAWAN_ONLY subscribe returns LoRaWAN result",
          "[middleware][network]") {
    auto lora = std::make_shared<MockTransport>();
    NetworkManager manager(nullptr, lora, NetworkManager::Mode::LORAWAN_ONLY);

    REQUIRE(manager.subscribe("downlink", [](const std::string&, const std::string&) {}));
    REQUIRE(lora->subscribe_called);
}

TEST_CASE("NetworkManager: DUAL_PARALLEL connect succeeds when one transport is alive",
          "[middleware][network]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto lora = std::make_shared<MockTransport>();
    mqtt->connect_result = false;
    lora->connect_result = true;
    NetworkManager manager(mqtt, lora, NetworkManager::Mode::DUAL_PARALLEL);

    REQUIRE(manager.connect());
    REQUIRE(mqtt->connect_called);
    REQUIRE(lora->connect_called);
}

TEST_CASE("NetworkManager: MQTT_ONLY publish does not touch LoRaWAN transport",
          "[middleware][network]") {
    auto mqtt = std::make_shared<MockTransport>();
    auto lora = std::make_shared<MockTransport>();
    NetworkManager manager(mqtt, lora, NetworkManager::Mode::MQTT_ONLY);

    REQUIRE(manager.publish("telemetry", "{}"));
    REQUIRE(mqtt->publish_called);
    REQUIRE_FALSE(lora->publish_called);
}
