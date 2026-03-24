#pragma once
#include "pv_cleaning_robot/middleware/i_network_transport.h"
#include <memory>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

// 前向声明 paho-mqtt-cpp 类型（避免将 paho 头文件泄漏到全局）
namespace mqtt { class async_client; }

namespace robot::middleware {

/// @brief MQTT 传输层（paho-mqtt-cpp，QoS=1，持久会话）
class MqttTransport : public INetworkTransport {
public:
    struct Config {
        std::string broker_uri;         ///< "tcp://host:port" 或 "ssl://host:8883"
        std::string client_id;
        std::string username;
        std::string password;
        bool        tls_enabled{false};
        std::string ca_cert_path;       ///< TLS CA 文件路径
        int         keep_alive_sec{30};
        int         connect_timeout_sec{10};
        int         qos{1};
    };

    explicit MqttTransport(Config cfg);
    ~MqttTransport() override;

    bool connect()    override;
    void disconnect() override;
    bool is_connected() const override;

    bool publish(const std::string& topic,
                 const std::string& payload) override;
    bool subscribe(const std::string& topic,
                   MessageCallback cb) override;

private:
    friend class MqttCallback;  // 定义在 mqtt_transport.cc，授权访问私有成员

    Config cfg_;
    std::unique_ptr<mqtt::async_client> client_;
    std::unordered_map<std::string, MessageCallback> subscriptions_;
    mutable std::mutex sub_mtx_;
    std::atomic<bool> connected_{false};
};

} // namespace robot::middleware
