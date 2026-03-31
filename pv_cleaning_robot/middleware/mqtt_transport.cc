#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include <mqtt/async_client.h>
#include <mqtt/connect_options.h>
#include <mqtt/ssl_options.h>
#include <chrono>
#include <spdlog/spdlog.h>

namespace robot::middleware {

// ── 消息到达回调 ──────────────────────────────────────────────────────────
class MqttCallback : public virtual mqtt::callback {
public:
    explicit MqttCallback(MqttTransport* owner) : owner_(owner) {}

    void connection_lost(const std::string& cause) override
    {
        owner_->connected_.store(false);
        spdlog::warn("[MqttTransport] 连接断开: {}",
                     cause.empty() ? "unknown" : cause);
    }

    void connected(const std::string& /*cause*/) override
    {
        owner_->connected_.store(true);
        // 重连后重新订阅全部 topic（paho 自动重连不保证重订阅）
        std::lock_guard<std::mutex> lk(owner_->sub_mtx_);
        for (auto& [topic, _] : owner_->subscriptions_) {
            try { owner_->client_->subscribe(topic, owner_->cfg_.qos)->wait_for(std::chrono::seconds(5)); }
            catch (...) {}
        }
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        if (!msg) return;
        const std::string& topic   = msg->get_topic();
        const std::string& payload = msg->to_string();

        std::lock_guard<std::mutex> lk(owner_->sub_mtx_);
        // 精确匹配优先
        auto it = owner_->subscriptions_.find(topic);
        if (it != owner_->subscriptions_.end()) {
            it->second(topic, payload);
        }
    }

private:
    MqttTransport* owner_;
};

// ── MqttTransport 实现 ────────────────────────────────────────────────────

MqttTransport::MqttTransport(Config cfg)
    : cfg_(std::move(cfg))
{
    client_ = std::make_unique<mqtt::async_client>(
        cfg_.broker_uri, cfg_.client_id,
        mqtt::create_options(MQTTVERSION_3_1_1));
    callback_ = std::make_unique<MqttCallback>(this);
    client_->set_callback(*callback_);
}

MqttTransport::~MqttTransport()
{
    disconnect();
}

bool MqttTransport::connect()
{
    mqtt::connect_options opts;
    opts.set_keep_alive_interval(cfg_.keep_alive_sec);
    opts.set_connect_timeout(std::chrono::seconds(cfg_.connect_timeout_sec));
    opts.set_clean_session(false);
    opts.set_automatic_reconnect(true);

    if (!cfg_.username.empty())
        opts.set_user_name(cfg_.username);
    if (!cfg_.password.empty())
        opts.set_password(cfg_.password);

    if (cfg_.tls_enabled) {
        mqtt::ssl_options ssl;
        if (!cfg_.ca_cert_path.empty())
            ssl.set_trust_store(cfg_.ca_cert_path);
        ssl.set_verify(true);
        opts.set_ssl(ssl);
    }

    try {
        client_->connect(opts)->wait_for(
            std::chrono::seconds(cfg_.connect_timeout_sec));
        connected_.store(client_->is_connected());
        return connected_.load();
    } catch (...) {
        connected_.store(false);
        return false;
    }
}

void MqttTransport::disconnect()
{
    if (connected_.load()) {
        try { client_->disconnect()->wait(); } catch (...) {}
        connected_.store(false);
    }
}

bool MqttTransport::is_connected() const
{
    return connected_ && client_->is_connected();
}

bool MqttTransport::publish(const std::string& topic,
                            const std::string& payload)
{
    if (!is_connected()) return false;
    try {
        auto msg = mqtt::make_message(topic, payload, cfg_.qos, false);
        client_->publish(msg)->wait_for(std::chrono::seconds(5));
        return true;
    } catch (...) {
        return false;
    }
}

bool MqttTransport::subscribe(const std::string& topic, MessageCallback cb)
{
    {
        std::lock_guard<std::mutex> lk(sub_mtx_);
        subscriptions_[topic] = std::move(cb);
    }
    if (!is_connected()) return false;
    try {
        client_->subscribe(topic, cfg_.qos)->wait_for(std::chrono::seconds(5));
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace robot::middleware
