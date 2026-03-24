#pragma once
#include "pv_cleaning_robot/middleware/i_network_transport.h"
#include <memory>
#include <string>

namespace robot::middleware {

/// @brief 网络管理器——根据配置选择传输模式
///
/// 传输模式（由 config.json `network.transport_mode` 决定）：
///   mqtt_only     — 仅使用 MQTT
///   lorawan_only  — 仅使用 LoRaWAN
///   dual_parallel — MQTT + LoRaWAN 同时上报（publish → 同时投递两路）
class NetworkManager {
public:
    enum class Mode { MQTT_ONLY, LORAWAN_ONLY, DUAL_PARALLEL };

    NetworkManager(std::shared_ptr<INetworkTransport> mqtt,
                   std::shared_ptr<INetworkTransport> lorawan,
                   Mode mode);

    /// 连接所有激活的传输层
    bool connect();

    /// 断开所有传输层
    void disconnect();

    /// 向激活的传输层广播消息
    bool publish(const std::string& topic, const std::string& payload);

    /// 在主传输层（MQTT 优先）订阅下行消息
    bool subscribe(const std::string& topic,
                   INetworkTransport::MessageCallback cb);

    bool is_connected() const;

    Mode mode() const { return mode_; }

private:
    std::shared_ptr<INetworkTransport> mqtt_;
    std::shared_ptr<INetworkTransport> lorawan_;
    Mode mode_;
};

} // namespace robot::middleware
