#include "pv_cleaning_robot/middleware/mqtt_transport.h"
#include <mqtt/async_client.h>
#include <mqtt/connect_options.h>
#include <mqtt/exception.h>
#include <mqtt/ssl_options.h>
#include <chrono>
#include <filesystem>
#include <spdlog/spdlog.h>

namespace robot::middleware {

namespace {

bool subscribe_granted(const mqtt::token_ptr& token)
{
    if (!token) {
        return false;
    }
    const auto reason_codes = token->get_subscribe_response().get_reason_codes();
    if (reason_codes.empty()) {
        return true;
    }
    for (const auto code : reason_codes) {
        if (code > mqtt::GRANTED_QOS_2) {
            return false;
        }
    }
    return true;
}

std::string granted_qos_to_string(const mqtt::token_ptr& token)
{
    if (!token) {
        return "[]";
    }
    const auto qos = token->get_subscribe_response().get_reason_codes();
    std::string out = "[";
    for (size_t i = 0; i < qos.size(); ++i) {
        if (i > 0) {
            out += ",";
        }
        out += std::to_string(qos[i]);
    }
    out += "]";
    return out;
}

}  // namespace

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

    void connected(const std::string& cause) override
    {
        owner_->connected_.store(true);
        if (!owner_->initial_connect_completed_.load()) {
            spdlog::info("[MqttTransport] connected callback during initial connect: cause='{}'",
                         cause.empty() ? "unknown" : cause);
            return;
        }
        spdlog::info("[MqttTransport] connected callback after reconnect: cause='{}'",
                     cause.empty() ? "unknown" : cause);
        // 仅在自动重连后的 connected 回调里重订阅；首次连接由 connect() 成功路径处理。
        owner_->subscribe_all_registered_topics();
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        if (!msg) return;
        const std::string& topic   = msg->get_topic();
        const std::string& payload = msg->to_string();
        spdlog::info("[MqttTransport] message arrived: topic='{}' payload={}", topic, payload);

        std::lock_guard<std::mutex> lk(owner_->sub_mtx_);
        // 精确匹配优先，未命中时回退到 MQTT wildcard 过滤匹配。
        auto it = owner_->subscriptions_.find(topic);
        if (it != owner_->subscriptions_.end()) {
            spdlog::info("[MqttTransport] dispatch exact subscription: topic='{}'", topic);
            it->second(topic, payload);
            return;
        }
        for (auto& [filter, cb] : owner_->subscriptions_) {
            if (detail::mqtt_topic_matches(filter, topic)) {
                spdlog::info("[MqttTransport] dispatch wildcard subscription: filter='{}' topic='{}'",
                             filter,
                             topic);
                cb(topic, payload);
                return;
            }
        }
        spdlog::warn("[MqttTransport] message had no matching subscription: topic='{}'", topic);
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
    initial_connect_completed_.store(false);
    spdlog::info("[MqttTransport] connecting: broker_uri='{}' client_id='{}' tls_enabled={} skip_server_name_check={}",
                 cfg_.broker_uri,
                 cfg_.client_id,
                 cfg_.tls_enabled,
                 cfg_.insecure_skip_server_name_check);
    mqtt::connect_options opts;
    opts.set_keep_alive_interval(cfg_.keep_alive_sec);
    opts.set_connect_timeout(std::chrono::seconds(cfg_.connect_timeout_sec));
    opts.set_clean_session(true);
    opts.set_automatic_reconnect(true);

    if (!cfg_.username.empty())
        opts.set_user_name(cfg_.username);
    if (!cfg_.password.empty())
        opts.set_password(cfg_.password);

    if (cfg_.tls_enabled) {
        mqtt::ssl_options ssl;
        spdlog::info(
            "[MqttTransport] TLS config: ca='{}' exists={} client_cert='{}' exists={} client_key='{}' exists={} skip_server_name_check={}",
            cfg_.ca_cert_path,
            (!cfg_.ca_cert_path.empty() && std::filesystem::exists(cfg_.ca_cert_path)),
            cfg_.client_cert_path,
            (!cfg_.client_cert_path.empty() && std::filesystem::exists(cfg_.client_cert_path)),
            cfg_.client_key_path,
            (!cfg_.client_key_path.empty() && std::filesystem::exists(cfg_.client_key_path)),
            cfg_.insecure_skip_server_name_check);
        if (!cfg_.ca_cert_path.empty())
            ssl.set_trust_store(cfg_.ca_cert_path);
        if (!cfg_.client_cert_path.empty())
            ssl.set_key_store(cfg_.client_cert_path);
        if (!cfg_.client_key_path.empty())
            ssl.set_private_key(cfg_.client_key_path);
        // 始终保持服务端证书链校验开启；调试开关只影响“证书身份与 broker 地址是否匹配”
        // 这一层，不会关闭 CA / client-cert 本身的 TLS 路径。
        ssl.set_enable_server_cert_auth(true);
        ssl.set_verify(!cfg_.insecure_skip_server_name_check);
        opts.set_ssl(ssl);
    }

    try {
        client_->connect(opts)->wait_for(
            std::chrono::seconds(cfg_.connect_timeout_sec));
        connected_.store(client_->is_connected());
        spdlog::info("[MqttTransport] connect result: connected={}", connected_.load());
        if (connected_.load()) {
            subscribe_all_registered_topics();
            initial_connect_completed_.store(true);
        }
        return connected_.load();
    } catch (const mqtt::exception& ex) {
        spdlog::error("[MqttTransport] connect mqtt::exception: what='{}' reason_code={}",
                      ex.what(),
                      ex.get_reason_code());
        connected_.store(false);
        return false;
    } catch (const std::exception& ex) {
        spdlog::error("[MqttTransport] connect std::exception: {}", ex.what());
        connected_.store(false);
        return false;
    } catch (...) {
        spdlog::error("[MqttTransport] connect unknown exception");
        connected_.store(false);
        return false;
    }
}

void MqttTransport::subscribe_all_registered_topics()
{
    std::lock_guard<std::mutex> lk(sub_mtx_);
    for (auto& [topic, _] : subscriptions_) {
        try {
            auto token = client_->subscribe(topic, cfg_.qos);
            if (!token->wait_for(std::chrono::seconds(5))) {
                spdlog::warn("[MqttTransport] subscribe timeout: topic='{}'", topic);
                continue;
            }
            if (!subscribe_granted(token)) {
                spdlog::warn("[MqttTransport] subscribe rejected: topic='{}' granted_qos={}",
                             topic,
                             granted_qos_to_string(token));
                continue;
            }
            spdlog::info("[MqttTransport] subscribed: topic='{}' granted_qos={}",
                         topic,
                         granted_qos_to_string(token));
        } catch (const mqtt::exception& ex) {
            spdlog::warn("[MqttTransport] subscribe mqtt::exception: topic='{}' what='{}' reason_code={}",
                         topic,
                         ex.what(),
                         ex.get_reason_code());
        } catch (const std::exception& ex) {
            spdlog::warn("[MqttTransport] subscribe std::exception: topic='{}' what='{}'",
                         topic,
                         ex.what());
        } catch (...) {
            spdlog::warn("[MqttTransport] subscribe unknown exception: topic='{}'", topic);
        }
    }
}

void MqttTransport::disconnect()
{
    if (connected_.load()) {
        try { client_->disconnect()->wait(); } catch (...) {}
        connected_.store(false);
    }
    initial_connect_completed_.store(false);
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
        // 不在这里同步等待 PUBACK。RPC handler / MQTT callback 也会复用同一 publish 路径，
        // 如果在回调线程里 wait_for(5s)，会把后续下行 RPC/attributes 处理链路一起拖住。
        // 这里以“Paho 已接受异步发送请求”为成功语义，实际 QoS1 发送由库后台完成。
        client_->publish(msg);
        return true;
    } catch (const mqtt::exception& ex) {
        spdlog::warn("[MqttTransport] publish mqtt::exception: topic='{}' what='{}' reason_code={}",
                     topic,
                     ex.what(),
                     ex.get_reason_code());
        return false;
    } catch (const std::exception& ex) {
        spdlog::warn("[MqttTransport] publish std::exception: topic='{}' what='{}'",
                     topic,
                     ex.what());
        return false;
    } catch (...) {
        spdlog::warn("[MqttTransport] publish unknown exception: topic='{}'", topic);
        return false;
    }
}

bool MqttTransport::subscribe(const std::string& topic, MessageCallback cb)
{
    {
        std::lock_guard<std::mutex> lk(sub_mtx_);
        subscriptions_[topic] = std::move(cb);
    }
    if (!is_connected()) {
        spdlog::info("[MqttTransport] deferred subscribe until connected: topic='{}'", topic);
        return true;
    }
    try {
        auto token = client_->subscribe(topic, cfg_.qos);
        if (!token->wait_for(std::chrono::seconds(5))) {
            spdlog::warn("[MqttTransport] subscribe timeout: topic='{}'", topic);
            return false;
        }
        if (!subscribe_granted(token)) {
            spdlog::warn("[MqttTransport] subscribe rejected: topic='{}' granted_qos={}",
                         topic,
                         granted_qos_to_string(token));
            return false;
        }
        spdlog::info("[MqttTransport] subscribed: topic='{}' granted_qos={}",
                     topic,
                     granted_qos_to_string(token));
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace robot::middleware
