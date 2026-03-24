#pragma once
#include "pv_cleaning_robot/middleware/i_network_transport.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace robot::middleware {

/// @brief LoRaWAN AT 指令传输层
///
/// 通过串口向 LoRaWAN 模组发送 AT 指令完成数据上行。
/// 受 LoRaWAN 占空比限制，publish() 会强制执行发送间隔。
///
/// 支持用途：可选并行传输（不作主备关系）
class LoRaWANTransport : public INetworkTransport {
public:
    struct Config {
        std::string dev_eui;           ///< Device EUI（16字符十六进制）
        std::string app_key;           ///< App Key（32字符十六进制）
        int         port{2};           ///< LoRaWAN FPort
        int         min_interval_sec{30}; ///< 占空比保护：最小发送间隔（秒）
        int         join_timeout_sec{60}; ///< OTAA 入网超时（秒）
    };

    LoRaWANTransport(std::shared_ptr<hal::ISerialPort> serial, Config cfg);
    ~LoRaWANTransport() override;

    bool connect()    override;  ///< 执行 OTAA 入网（AT+JOIN）
    void disconnect() override;
    bool is_connected() const override;

    /// 发送上行数据（十六进制编码后通过 AT+SEND 上传）
    /// 受占空比限制：若距上次发送不足 min_interval_sec 则返回 false
    bool publish(const std::string& topic,
                 const std::string& payload) override;

    /// LoRaWAN 下行订阅（作为回调注册，当收到下行消息时触发）
    bool subscribe(const std::string& topic, MessageCallback cb) override;

private:
    std::string send_at(const std::string& cmd, int timeout_ms = 2000);
    bool wait_for(const std::string& expected, int timeout_ms);

    std::shared_ptr<hal::ISerialPort> serial_;
    Config                            cfg_;
    bool                              joined_{false};
    std::chrono::steady_clock::time_point last_send_time_{};
    MessageCallback                   downlink_cb_;
    mutable std::mutex                mtx_;
};

} // namespace robot::middleware
