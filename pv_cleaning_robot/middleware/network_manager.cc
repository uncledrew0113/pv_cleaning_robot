#include "pv_cleaning_robot/middleware/network_manager.h"

namespace robot::middleware {

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
    bool ok = true;
    switch (mode_) {
    case Mode::MQTT_ONLY:
        ok = mqtt_ && mqtt_->connect();
        break;
    case Mode::LORAWAN_ONLY:
        ok = lorawan_ && lorawan_->connect();
        break;
    case Mode::DUAL_PARALLEL:
        // 两路均尝试连接，只要其中之一成功即可继续
        {
            bool m = mqtt_ && mqtt_->connect();
            bool l = lorawan_ && lorawan_->connect();
            ok = m || l;
        }
        break;
    }
    return ok;
}

void NetworkManager::disconnect()
{
    if (mqtt_   && (mode_ == Mode::MQTT_ONLY    || mode_ == Mode::DUAL_PARALLEL))
        mqtt_->disconnect();
    if (lorawan_ && (mode_ == Mode::LORAWAN_ONLY || mode_ == Mode::DUAL_PARALLEL))
        lorawan_->disconnect();
}

bool NetworkManager::publish(const std::string& topic,
                             const std::string& payload)
{
    bool ok = false;
    if ((mode_ == Mode::MQTT_ONLY || mode_ == Mode::DUAL_PARALLEL) && mqtt_)
        ok |= mqtt_->publish(topic, payload);
    if ((mode_ == Mode::LORAWAN_ONLY || mode_ == Mode::DUAL_PARALLEL) && lorawan_)
        ok |= lorawan_->publish(topic, payload);
    return ok;
}

bool NetworkManager::subscribe(const std::string& topic,
                               INetworkTransport::MessageCallback cb)
{
    // MQTT 优先订阅下行；LoRaWAN 同步注册同一回调
    bool ok = false;
    if ((mode_ == Mode::MQTT_ONLY || mode_ == Mode::DUAL_PARALLEL) && mqtt_)
        ok |= mqtt_->subscribe(topic, cb);
    if ((mode_ == Mode::LORAWAN_ONLY || mode_ == Mode::DUAL_PARALLEL) && lorawan_)
        lorawan_->subscribe(topic, cb);
    return ok;
}

bool NetworkManager::is_connected() const
{
    switch (mode_) {
    case Mode::MQTT_ONLY:
        return mqtt_ && mqtt_->is_connected();
    case Mode::LORAWAN_ONLY:
        return lorawan_ && lorawan_->is_connected();
    case Mode::DUAL_PARALLEL:
        return (mqtt_ && mqtt_->is_connected()) ||
               (lorawan_ && lorawan_->is_connected());
    }
    return false;
}

} // namespace robot::middleware
