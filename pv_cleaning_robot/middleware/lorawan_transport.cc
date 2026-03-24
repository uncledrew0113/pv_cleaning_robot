#include "pv_cleaning_robot/middleware/lorawan_transport.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

namespace robot::middleware {

LoRaWANTransport::LoRaWANTransport(std::shared_ptr<hal::ISerialPort> serial,
                                   Config                             cfg)
    : serial_(std::move(serial)), cfg_(std::move(cfg))
{
}

LoRaWANTransport::~LoRaWANTransport()
{
    disconnect();
}

bool LoRaWANTransport::connect()
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!serial_->is_open()) {
        if (!serial_->open()) return false;
    }

    // 设置 DevEUI 和 AppKey
    send_at("AT+DEVEUI=" + cfg_.dev_eui);
    send_at("AT+APPKEY=" + cfg_.app_key);

    // 发起 OTAA 入网
    auto resp = send_at("AT+JOIN", 1000);
    if (resp.find("OK") == std::string::npos &&
        resp.find("+JOIN:") == std::string::npos) {
        return false;
    }

    // 等待入网成功（+JOIN: Network joined）
    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(cfg_.join_timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        auto line = send_at("AT+JOIN=?", 500);
        if (line.find("JOINED") != std::string::npos ||
            line.find("Network joined") != std::string::npos) {
            joined_ = true;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return false;
}

void LoRaWANTransport::disconnect()
{
    std::lock_guard<std::mutex> lk(mtx_);
    joined_ = false;
    if (serial_->is_open()) serial_->close();
}

bool LoRaWANTransport::is_connected() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return joined_;
}

bool LoRaWANTransport::publish(const std::string& /*topic*/,
                               const std::string& payload)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!joined_) return false;

    // 占空比保护
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_send_time_).count();
    if (elapsed < cfg_.min_interval_sec) return false;

    // 将 payload 转成十六进制（LoRaWAN 上行为字节流）
    std::ostringstream hex;
    for (unsigned char c : payload) {
        hex << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(c);
    }

    // 限制长度（LoRaWAN SF7 最大约 230 字节）
    std::string hex_str = hex.str();
    if (hex_str.size() > 460) hex_str = hex_str.substr(0, 460);

    std::string cmd = "AT+SEND=" + std::to_string(cfg_.port) + ","
                    + std::to_string(hex_str.size() / 2) + "," + hex_str;
    auto resp = send_at(cmd, 5000);

    if (resp.find("OK") != std::string::npos ||
        resp.find("+SEND:OK") != std::string::npos) {
        last_send_time_ = now;
        return true;
    }
    return false;
}

bool LoRaWANTransport::subscribe(const std::string& /*topic*/,
                                 MessageCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    downlink_cb_ = std::move(cb);
    return true;
}

// ── 私有工具 ──────────────────────────────────────────────────────────────

std::string LoRaWANTransport::send_at(const std::string& cmd, int timeout_ms)
{
    std::string full = cmd + "\r\n";
    serial_->write(reinterpret_cast<const uint8_t*>(full.c_str()), full.size());

    std::string response;
    uint8_t buf[256];
    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        int n = serial_->read(buf, sizeof(buf), 50);
        if (n > 0) {
            response.append(reinterpret_cast<char*>(buf), n);
            if (response.find("\r\n") != std::string::npos) break;
        }
    }
    return response;
}

bool LoRaWANTransport::wait_for(const std::string& expected, int timeout_ms)
{
    auto resp = send_at("", timeout_ms);
    return resp.find(expected) != std::string::npos;
}

} // namespace robot::middleware
