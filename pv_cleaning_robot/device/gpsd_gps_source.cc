// @file gpsd_gps_source.cc
// @brief GpsdGpsSource 实现：通过 TCP 连接 gpsd 并按行解析 JSON GPS 消息。

#include <chrono>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "pv_cleaning_robot/device/gps_source.h"

namespace robot::device {

namespace {

/// 关闭套接字并将描述符置为 -1。
void close_socket(int& fd) {
    if (fd >= 0) {
        ::shutdown(fd, SHUT_RDWR);
        ::close(fd);
        fd = -1;
    }
}

}  // namespace

GpsdGpsSource::GpsdGpsSource(GpsdSourceConfig cfg,
                             DataCallback on_data,
                             ParseErrorCallback on_parse_error,
                             MessageCallback on_message)
    : cfg_(std::move(cfg))
    , on_data_(std::move(on_data))
    , on_parse_error_(std::move(on_parse_error))
    , on_message_(std::move(on_message)) {}

GpsdGpsSource::~GpsdGpsSource() {
    close();
}

/// @brief 连接 gpsd 并启动后台读取线程。
/// @brief 连接 gpsd 并启动后台数据读取线程。
bool GpsdGpsSource::open() {
    if (running_.load())
        return true;
    if (!connect_socket())
        return false;
    if (!send_watch()) {
        close_socket(sock_fd_);
        return false;
    }

    running_.store(true);
    read_thread_ = std::thread(&GpsdGpsSource::read_loop, this);
    return true;
}

/// @brief 关闭 gpsd 连接并停止后台线程。
/// @brief 关闭 gpsd 连接并停止读取线程。
void GpsdGpsSource::close() {
    running_.store(false);
    close_socket(sock_fd_);
    if (read_thread_.joinable())
        read_thread_.join();
}

/// @brief gpsd 源不支持通过此接口调整输出频率。
/// @brief gpsd 源不支持设置输出速率。
DeviceError GpsdGpsSource::set_output_rate(int) {
    return DeviceError::NOT_SUPPORTED;
}

/// @brief gpsd 源不支持热重启。
/// @brief gpsd 源不支持热重启。
DeviceError GpsdGpsSource::hot_restart() {
    return DeviceError::NOT_SUPPORTED;
}

/// @brief gpsd 源不支持冷重启。
/// @brief gpsd 源不支持冷重启。
DeviceError GpsdGpsSource::cold_restart() {
    return DeviceError::NOT_SUPPORTED;
}

/// @brief 建立到 gpsd 的 TCP 连接。
/// @brief 建立到 gpsd 的 TCP 连接。
bool GpsdGpsSource::connect_socket() {
    close_socket(sock_fd_);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* results = nullptr;
    const std::string port = std::to_string(cfg_.port);
    if (::getaddrinfo(cfg_.host.c_str(), port.c_str(), &hints, &results) != 0)
        return false;

    for (addrinfo* it = results; it != nullptr; it = it->ai_next) {
        const int fd = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (fd < 0)
            continue;

        if (::connect(fd, it->ai_addr, it->ai_addrlen) == 0) {
            sock_fd_ = fd;
            ::freeaddrinfo(results);
            return true;
        }
        ::close(fd);
    }

    ::freeaddrinfo(results);
    return false;
}

/// @brief 发送 gpsd watch 命令，订阅 JSON 数据流。
/// @brief 发送 gpsd watch 命令以订阅 JSON 输出。
bool GpsdGpsSource::send_watch() {
    if (sock_fd_ < 0)
        return false;
    const auto size = cfg_.watch.size();
    const auto sent = ::send(sock_fd_, cfg_.watch.data(), size, MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(size);
}

/// @brief 从 gpsd 套接字循环读取数据并逐行解析。
/// @brief 循环读取 gpsd 输出并按行分发。
void GpsdGpsSource::read_loop() {
    char buf[512];

    while (running_.load()) {
        if (sock_fd_ < 0) {
            if (!connect_socket() || !send_watch()) {
                close_socket(sock_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        const int n = ::recv(sock_fd_, buf, sizeof(buf), 0);
        if (n <= 0) {
            close_socket(sock_fd_);
            if (!running_.load())
                break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        for (int i = 0; i < n; ++i) {
            const char c = buf[i];
            if (c == '\n') {
                std::string_view line(rx_buf_.data(), rx_len_);
                if (!line.empty() && line.back() == '\r') {
                    line.remove_suffix(1);
                }
                if (!line.empty()) {
                    if (on_message_)
                        on_message_();
                    handle_json_line(line);
                }
                rx_len_ = 0;
                continue;
            }
            if (rx_len_ + 1 < rx_buf_.size()) {
                rx_buf_[rx_len_++] = c;
            } else {
                rx_len_ = 0;
                if (on_parse_error_)
                    on_parse_error_();
            }
        }
    }
}

/// @brief 处理一行 gpsd JSON 数据并触发回调。
/// @brief 处理一行 JSON 数据，并触发相应回调。
void GpsdGpsSource::handle_json_line(std::string_view line) {
    protocol::GpsdJsonParseResult result{};
    if (!protocol::GpsdJsonParser::parse_line(line, data_, result)) {
        if (on_parse_error_)
            on_parse_error_();
        return;
    }

    if ((result.message_class == protocol::GpsdMessageClass::TPV ||
         result.message_class == protocol::GpsdMessageClass::SKY) &&
        on_data_) {
        on_data_(data_);
    }
    if (result.time_parse_error && on_parse_error_) {
        on_parse_error_();
    }
}

}  // namespace robot::device
