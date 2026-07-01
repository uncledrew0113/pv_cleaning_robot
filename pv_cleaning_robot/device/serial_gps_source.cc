#include "pv_cleaning_robot/device/gps_source.h"

#include "pv_cleaning_robot/hal/i_serial_port.h"

#include <chrono>
#include <cstring>
#include <exception>
#include <spdlog/spdlog.h>

namespace robot::device {

namespace {

DeviceError write_command(const std::shared_ptr<hal::ISerialPort>& serial,
                          const char* cmd,
                          int sleep_ms)
{
    if (!serial || !serial->is_open()) return DeviceError::NOT_OPEN;

    auto* ptr = reinterpret_cast<const uint8_t*>(cmd);
    if (serial->write(ptr, std::strlen(cmd)) < 0) return DeviceError::COMM_TIMEOUT;

    if (sleep_ms > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    return DeviceError::OK;
}

}  // namespace

SerialGpsSource::SerialGpsSource(SerialGpsSourceConfig cfg,
                                 DataCallback on_data,
                                 ParseErrorCallback on_parse_error,
                                 MessageCallback on_message)
    : cfg_(std::move(cfg))
    , on_data_(std::move(on_data))
    , on_parse_error_(std::move(on_parse_error))
    , on_message_(std::move(on_message))
{
}

SerialGpsSource::~SerialGpsSource()
{
    close();
}

bool SerialGpsSource::open()
{
    if (running_.load()) return true;
    if (!cfg_.serial || !cfg_.serial->open()) return false;

    running_.store(true);
    read_thread_ = std::thread(&SerialGpsSource::read_loop, this);
    return true;
}

void SerialGpsSource::request_stop()
{
    running_.store(false);
}

void SerialGpsSource::close()
{
    request_stop();
    if (read_thread_.joinable()) read_thread_.join();
    if (cfg_.serial) cfg_.serial->close();
}

DeviceError SerialGpsSource::set_output_rate(int hz)
{
    int interval_ms = 1000;
    if (hz >= 10) interval_ms = 100;
    else if (hz >= 5) interval_ms = 200;

    char buf[64];
    const int len = std::snprintf(buf, sizeof(buf), "PMTK220,%d", interval_ms);
    uint8_t checksum = 0;
    for (int i = 0; i < len; ++i)
        checksum ^= static_cast<uint8_t>(buf[i]);

    char cmd[80];
    std::snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", buf, checksum);
    return write_command(cfg_.serial, cmd, 100);
}

DeviceError SerialGpsSource::hot_restart()
{
    return write_command(cfg_.serial, "$PMTK101*32\r\n", 500);
}

DeviceError SerialGpsSource::cold_restart()
{
    return write_command(cfg_.serial, "$PMTK103*30\r\n", 1500);
}

void SerialGpsSource::read_loop()
{
    uint8_t byte_buf[128];

    while (running_.load()) {
        const int n = cfg_.serial->read(byte_buf, sizeof(byte_buf), 50);
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            const char c = static_cast<char>(byte_buf[i]);
            if (c == '\n') {
                if (!line_buf_.empty()) {
                    if (on_message_) on_message_();
                    bool parsed = false;
                    try {
                        parsed = parser_.parse_sentence(line_buf_);
                    } catch (const std::exception& ex) {
                        // 现场 GPS 输入可能混入损坏句子；解析异常只代表该句无效，
                        // 不能让后台读线程退出，否则 ErrorManager 会看到 GPS 数据流长期不更新。
                        spdlog::warn("[SerialGpsSource] NMEA parse exception: {}", ex.what());
                    } catch (...) {
                        spdlog::warn("[SerialGpsSource] NMEA parse unknown exception");
                    }
                    if (parsed) {
                        if (on_data_) on_data_(parser_.get_data());
                    } else if (on_parse_error_) {
                        on_parse_error_();
                    }
                    line_buf_.clear();
                }
            } else if (c != '\r') {
                if (line_buf_.size() < 256) {
                    line_buf_ += c;
                } else {
                    line_buf_.clear();
                    if (on_parse_error_) on_parse_error_();
                }
            }
        }
    }
}

}  // namespace robot::device
