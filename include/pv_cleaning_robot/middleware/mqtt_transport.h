#pragma once
#include "pv_cleaning_robot/middleware/i_network_transport.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

// 前向声明 paho-mqtt-cpp 类型（避免将 paho 头文件泄漏到全局）
namespace mqtt { class async_client; }

namespace robot::middleware {

class MqttCallback;  // 定义在 mqtt_transport.cc，此处仅供 unique_ptr 成员声明使用

namespace detail {
/// MQTT topic filter matcher.
/// Supports single-level '+' and multi-level '#' wildcards.
inline bool mqtt_topic_matches(const std::string& filter, const std::string& topic) {
    size_t fi = 0;
    size_t ti = 0;
    while (fi < filter.size()) {
        if (filter[fi] == '#') {
            return fi + 1 == filter.size();
        }

        size_t fnext = filter.find('/', fi);
        size_t tnext = topic.find('/', ti);
        const std::string fseg = filter.substr(fi, fnext - fi);
        const std::string tseg =
            (ti <= topic.size()) ? topic.substr(ti, tnext - ti) : std::string{};

        if (fseg != "+" && fseg != tseg) {
            return false;
        }

        if (fnext == std::string::npos) {
            return tnext == std::string::npos;
        }
        if (tnext == std::string::npos) {
            return false;
        }

        fi = fnext + 1;
        ti = tnext + 1;
    }
    return ti == topic.size();
}
}  // namespace detail

/// @brief MQTT 传输层（paho-mqtt-cpp，QoS=1，持久会话）
///
/// 当前主要用于 ThingsBoard MQTT 接入。
/// 支持：
/// - 明文 TCP
/// - TLS + CA 校验
/// - TLS + 客户端证书/私钥
///
/// 注意：
/// `insecure_skip_server_name_check` 仅用于现场调试。当服务端证书的
/// CN/SAN 与 broker_uri 中使用的 IP/主机名不匹配时，可临时绕过该校验；
/// 正式上线应恢复为 false，并改用匹配证书身份的域名或证书。
class MqttTransport : public INetworkTransport {
public:
    struct Config {
        std::string broker_uri;         ///< "tcp://host:port" 或 "ssl://host:8883"
        std::string client_id;
        std::string username;
        std::string password;
        bool        tls_enabled{false};
        std::string ca_cert_path;       ///< TLS CA 文件路径
        std::string client_cert_path;   ///< TLS 客户端证书路径（PEM）
        std::string client_key_path;    ///< TLS 客户端私钥路径（PEM/KEY）
        bool        insecure_skip_server_name_check{false};  ///< 仅调试用：跳过证书与 broker 主机名/IP 匹配校验
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
    struct PendingDelivery {
        MessageCallback callback;
        std::string topic;
        std::string payload;
    };

    void enqueue_delivery(MessageCallback cb, std::string topic, std::string payload);
    void delivery_loop();
    void subscribe_all_registered_topics(bool wait_for_ack = true);

    Config cfg_;
    std::unique_ptr<mqtt::async_client> client_;
    std::unique_ptr<MqttCallback> callback_;  ///< 替代 raw new，持有 MqttCallback 对象
    std::unordered_map<std::string, MessageCallback> subscriptions_;
    mutable std::mutex sub_mtx_;
    std::mutex delivery_mtx_;
    std::condition_variable delivery_cv_;
    std::deque<PendingDelivery> pending_deliveries_;
    bool stop_delivery_{false};
    std::thread delivery_thread_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> initial_connect_completed_{false};
};

} // namespace robot::middleware
