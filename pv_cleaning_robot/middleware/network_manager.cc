#include "pv_cleaning_robot/middleware/network_manager.h"

namespace robot::middleware {

bool NetworkManager::uses_mqtt() const
{
    return mode_ == Mode::MQTT_ONLY || mode_ == Mode::DUAL_PARALLEL;
}

bool NetworkManager::uses_lorawan() const
{
    return mode_ == Mode::LORAWAN_ONLY || mode_ == Mode::DUAL_PARALLEL;
}

NetworkManager::NetworkManager(std::shared_ptr<INetworkTransport> mqtt,
                               std::shared_ptr<INetworkTransport> lorawan,
                               Mode                               mode)
    : mqtt_(std::move(mqtt))
    , lorawan_(std::move(lorawan))
    , mode_(mode)
{
}

bool NetworkManager::connect()
{
    return connect(nullptr);
}

bool NetworkManager::connect(const std::atomic<bool>* running)
{
    if (mode_ == Mode::DUAL_PARALLEL) {
        // 两路都尝试，只要还有一路存活就允许上层继续走缓存/重连逻辑。
        const bool mqtt_ok = mqtt_ && mqtt_->connect(running);
        const bool lora_ok = (!running || running->load(std::memory_order_acquire)) &&
                             lorawan_ && lorawan_->connect(running);
        return mqtt_ok || lora_ok;
    }
    if (uses_mqtt()) {
        return mqtt_ && mqtt_->connect(running);
    }
    return lorawan_ && lorawan_->connect(running);
}

void NetworkManager::request_stop()
{
    if (mqtt_)
        mqtt_->request_stop();
    if (lorawan_)
        lorawan_->request_stop();
}

void NetworkManager::disconnect()
{
    if (mqtt_ && uses_mqtt())
        mqtt_->disconnect();
    if (lorawan_ && uses_lorawan())
        lorawan_->disconnect();
}

bool NetworkManager::publish(const std::string& topic,
                             const std::string& payload)
{
    bool ok = false;
    if (uses_mqtt() && mqtt_)
        ok |= mqtt_->publish(topic, payload);
    if (uses_lorawan() && lorawan_)
        ok |= lorawan_->publish(topic, payload);
    return ok;
}

bool NetworkManager::subscribe(const std::string& topic,
                               INetworkTransport::MessageCallback cb)
{
    // MQTT 优先订阅下行；LoRaWAN 同步注册同一回调
    bool ok = false;
    if (uses_mqtt() && mqtt_)
        ok |= mqtt_->subscribe(topic, cb);
    if (uses_lorawan() && lorawan_)
        ok |= lorawan_->subscribe(topic, cb);
    return ok;
}

bool NetworkManager::is_connected() const
{
    if (mode_ == Mode::DUAL_PARALLEL) {
        return (mqtt_ && mqtt_->is_connected()) || (lorawan_ && lorawan_->is_connected());
    }
    if (uses_mqtt()) {
        return mqtt_ && mqtt_->is_connected();
    }
    return lorawan_ && lorawan_->is_connected();
}

} // namespace robot::middleware
