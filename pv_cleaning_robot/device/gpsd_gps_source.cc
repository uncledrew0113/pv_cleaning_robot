#include "pv_cleaning_robot/device/gps_source.h"

#include <chrono>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace robot::device {

namespace {

void close_socket(int& fd)
{
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
    , on_message_(std::move(on_message))
{
}

GpsdGpsSource::~GpsdGpsSource()
{
    close();
}

bool GpsdGpsSource::open()
{
    if (running_.load()) return true;
    if (!connect_socket()) return false;
    if (!send_watch()) {
        close_socket(sock_fd_);
        return false;
    }

    running_.store(true);
    read_thread_ = std::thread(&GpsdGpsSource::read_loop, this);
    return true;
}

void GpsdGpsSource::close()
{
    running_.store(false);
    close_socket(sock_fd_);
    if (read_thread_.joinable()) read_thread_.join();
}

DeviceError GpsdGpsSource::set_output_rate(int)
{
    return DeviceError::NOT_SUPPORTED;
}

DeviceError GpsdGpsSource::hot_restart()
{
    return DeviceError::NOT_SUPPORTED;
}

DeviceError GpsdGpsSource::cold_restart()
{
    return DeviceError::NOT_SUPPORTED;
}

void GpsdGpsSource::ingest_json_line_for_test(std::string_view line)
{
    handle_json_line(line);
}

bool GpsdGpsSource::connect_socket()
{
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
        if (fd < 0) continue;

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

bool GpsdGpsSource::send_watch()
{
    if (sock_fd_ < 0) return false;
    const auto size = cfg_.watch.size();
    const auto sent = ::send(sock_fd_, cfg_.watch.data(), size, MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(size);
}

void GpsdGpsSource::read_loop()
{
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
            if (!running_.load()) break;
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
                    if (on_message_) on_message_();
                    handle_json_line(line);
                }
                rx_len_ = 0;
                continue;
            }
            if (rx_len_ + 1 < rx_buf_.size()) {
                rx_buf_[rx_len_++] = c;
            } else {
                rx_len_ = 0;
                if (on_parse_error_) on_parse_error_();
            }
        }
    }
}

void GpsdGpsSource::handle_json_line(std::string_view line)
{
    protocol::GpsdJsonParseResult result{};
    if (!protocol::GpsdJsonParser::parse_line(line, data_, result)) {
        if (on_parse_error_) on_parse_error_();
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
