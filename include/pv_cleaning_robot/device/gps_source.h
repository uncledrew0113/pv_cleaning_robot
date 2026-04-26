#pragma once

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/protocol/gpsd_json_parser.h"
#include "pv_cleaning_robot/protocol/nmea_parser.h"

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace robot::hal {
class ISerialPort;
}

namespace robot::device {

struct SerialGpsSourceConfig {
    std::shared_ptr<hal::ISerialPort> serial;
};

struct GpsdSourceConfig {
    std::string host{"127.0.0.1"};
    int         port{2947};
    std::string watch{"?WATCH={\"enable\":true,\"json\":true};"};
};

class IGpsSource {
public:
    using GpsData = protocol::GpsData;
    using DataCallback = std::function<void(const GpsData&)>;
    using ParseErrorCallback = std::function<void()>;
    using MessageCallback = std::function<void()>;

    virtual ~IGpsSource() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual DeviceError set_output_rate(int hz) = 0;
    virtual DeviceError hot_restart() = 0;
    virtual DeviceError cold_restart() = 0;
};

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
    void read_loop();

    SerialGpsSourceConfig cfg_;
    DataCallback          on_data_;
    ParseErrorCallback    on_parse_error_;
    MessageCallback       on_message_;
    protocol::NmeaParser  parser_;
    std::thread           read_thread_;
    std::atomic<bool>     running_{false};
    std::string           line_buf_;
};

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

    void ingest_json_line_for_test(std::string_view line);

private:
    static constexpr size_t kRxBufCap = 2048;

    bool connect_socket();
    bool send_watch();
    void read_loop();
    void handle_json_line(std::string_view line);

    GpsdSourceConfig   cfg_;
    DataCallback       on_data_;
    ParseErrorCallback on_parse_error_;
    MessageCallback    on_message_;
    int                sock_fd_{-1};
    std::thread        read_thread_;
    std::atomic<bool>  running_{false};
    protocol::GpsData  data_{};
    std::array<char, kRxBufCap> rx_buf_{};
    size_t             rx_len_{0};
};

}  // namespace robot::device
