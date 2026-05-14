#pragma once

/// @file gps_source.h
/// @brief GPS 数据源抽象和具体实现。
///
/// 支持串口 NMEA 数据源和 gpsd JSON 数据源，
/// 通过回调上报解析后的 GPS 固件数据或解析失败事件。

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/protocol/gpsd_json_parser.h"
#include "pv_cleaning_robot/protocol/nmea_parser.h"

namespace robot::hal {
class ISerialPort;
}

namespace robot::device {

struct SerialGpsSourceConfig {
    /// 串口设备句柄，用于读取 GPS NMEA 数据流。
    std::shared_ptr<hal::ISerialPort> serial;
};

struct GpsdSourceConfig {
    /// gpsd 服务器地址
    std::string host{"127.0.0.1"};
    /// gpsd 服务器端口
    int port{2947};
    /// gpsd watch 命令，默认为启用 JSON 输出
    std::string watch{"?WATCH={\"enable\":true,\"json\":true};"};
};

class IGpsSource {
   public:
    using GpsData = protocol::GpsData;
    using DataCallback = std::function<void(const GpsData&)>;
    using ParseErrorCallback = std::function<void()>;
    using MessageCallback = std::function<void()>;

    /// @brief GPS 数据源抽象接口。
    ///
    /// 具体实现负责打开/关闭数据通道，控制输出速率，并支持重启。
    virtual ~IGpsSource() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual DeviceError set_output_rate(int hz) = 0;
    virtual DeviceError hot_restart() = 0;
    virtual DeviceError cold_restart() = 0;
};

/// @brief 串口 GPS 源实现，从串口读取 NMEA 并回调解析结果。
class SerialGpsSource final : public IGpsSource {
   public:
    SerialGpsSource(SerialGpsSourceConfig cfg,
                    DataCallback on_data,
                    ParseErrorCallback on_parse_error,
                    MessageCallback on_message);
    ~SerialGpsSource() override;

    bool open() override;
    void close() override;
    DeviceError set_output_rate(int hz) override;
    DeviceError hot_restart() override;
    DeviceError cold_restart() override;

   private:
    /// @brief 串口读取线程主循环，按行解析 NMEA 数据。
    void read_loop();

    SerialGpsSourceConfig cfg_;
    DataCallback on_data_;
    ParseErrorCallback on_parse_error_;
    MessageCallback on_message_;
    protocol::NmeaParser parser_;
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    std::string line_buf_;
};

/// @brief gpsd 源实现，通过 TCP 连接 gpsd 获取 JSON GPS 数据。
class GpsdGpsSource final : public IGpsSource {
   public:
    GpsdGpsSource(GpsdSourceConfig cfg,
                  DataCallback on_data,
                  ParseErrorCallback on_parse_error,
                  MessageCallback on_message);
    ~GpsdGpsSource() override;

    bool open() override;
    void close() override;
    DeviceError set_output_rate(int hz) override;
    DeviceError hot_restart() override;
    DeviceError cold_restart() override;

   private:
    static constexpr size_t kRxBufCap = 2048;

    bool connect_socket();
    bool send_watch();
    void read_loop();
    void handle_json_line(std::string_view line);

    GpsdSourceConfig cfg_;
    DataCallback on_data_;
    ParseErrorCallback on_parse_error_;
    MessageCallback on_message_;
    int sock_fd_{-1};
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    protocol::GpsData data_{};
    std::array<char, kRxBufCap> rx_buf_{};
    size_t rx_len_{0};
};

}  // namespace robot::device
