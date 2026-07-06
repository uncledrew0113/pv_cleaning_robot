/**
 * @file lorawan_transport.cc
 * @brief LoRaWAN 网络传输实现。
 *
 * 本文件通过串口 AT 命令驱动 LoRaWAN 模块，适用于低频关键遥测的备用链路。
 */
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
    return connect(nullptr);
}

bool LoRaWANTransport::connect(const std::atomic<bool>* running)
{
    stop_requested_.store(false, std::memory_order_release);
    std::lock_guard<std::mutex> lk(mtx_);
    if (!serial_->is_open()) {
        if (!serial_->open()) return false;
    }

    // 设置 DevEUI 和 AppKey
    if ((running && !running->load(std::memory_order_acquire)) ||
        stop_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    send_at("AT+DEVEUI=" + cfg_.dev_eui);
    if ((running && !running->load(std::memory_order_acquire)) ||
        stop_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    send_at("AT+APPKEY=" + cfg_.app_key);
    if ((running && !running->load(std::memory_order_acquire)) ||
        stop_requested_.load(std::memory_order_acquire)) {
        return false;
    }

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
        if ((running && !running->load(std::memory_order_acquire)) ||
            stop_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        auto line = send_at("AT+JOIN=?", 500);
        if (line.find("JOINED") != std::string::npos ||
            line.find("Network joined") != std::string::npos) {
            joined_ = true;
            return true;
        }
        const auto sleep_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < sleep_deadline) {
            if ((running && !running->load(std::memory_order_acquire)) ||
                stop_requested_.load(std::memory_order_acquire)) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return false;
}

void LoRaWANTransport::request_stop()
{
    stop_requested_.store(true, std::memory_order_release);
}

void LoRaWANTransport::disconnect()
{
    request_stop();
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

// 私有工具。

std::string LoRaWANTransport::send_at(const std::string& cmd, int timeout_ms)
{
    if (stop_requested_.load(std::memory_order_acquire)) {
        return {};
    }
    std::string full = cmd + "\r\n";
    serial_->write(reinterpret_cast<const uint8_t*>(full.c_str()), full.size());

    std::string response;
    uint8_t buf[256];
    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        if (stop_requested_.load(std::memory_order_acquire)) {
            break;
        }
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
