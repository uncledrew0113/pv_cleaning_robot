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
    bool subscribe_called{false};

    bool connect() override {
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
