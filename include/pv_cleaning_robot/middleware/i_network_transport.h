/**
 * @file i_network_transport.h
 * @brief 网络传输抽象接口。
 *
 * 本接口统一 MQTT 与 LoRaWAN 等传输实现，供 CloudService 和 NetworkManager 以相同方式发布、
 * 订阅和连接。
 */
#pragma once
#include <atomic>
#include <functional>
#include <string>

namespace robot::middleware {

/// @brief 网络传输抽象接口（MQTT / LoRaWAN 共用）
class INetworkTransport {
public:
    virtual ~INetworkTransport() = default;

    using MessageCallback = std::function<void(const std::string& topic,
                                               const std::string& payload)>;

    /// 建立连接（阻塞直到连接成功或失败）
    virtual bool connect() = 0;

    /// 建立连接；running 变为 false 时应尽快返回 false。
    virtual bool connect(const std::atomic<bool>* running) {
        if (running && !running->load(std::memory_order_acquire)) {
            return false;
        }
        return connect();
    }

    /// 请求正在进行的 connect()/publish()/subscribe 尽快返回；默认实现只断开连接。
    virtual void request_stop() {
        disconnect();
    }

    /// 断开连接
    virtual void disconnect() = 0;

    /// 是否已连接
    virtual bool is_connected() const = 0;

    /// 发布消息（QoS 由实现决定）
    /// @return true=成功发布（或已入队）；false=发布失败
    virtual bool publish(const std::string& topic,
                         const std::string& payload) = 0;

    /// 订阅主题（收到消息时调用 cb）
    virtual bool subscribe(const std::string& topic,
                           MessageCallback cb) = 0;
};

} // namespace robot::middleware
