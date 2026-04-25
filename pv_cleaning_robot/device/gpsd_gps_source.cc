#include "pv_cleaning_robot/device/gps_source.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstring>
#include <ctime>
#include <netdb.h>
#include <string_view>
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

uint8_t clamp_sat_count(std::size_t count)
{
    return static_cast<uint8_t>(count > 255 ? 255 : count);
}

bool parse_utc_timestamp_ms(const std::string& iso8601, uint64_t& out_ms)
{
    std::tm tm{};
    const char* pos = ::strptime(iso8601.c_str(), "%Y-%m-%dT%H:%M:%S", &tm);
    if (!pos) return false;

    int millis = 0;
    if (*pos == '.') {
        ++pos;
        int digits = 0;
        while (*pos >= '0' && *pos <= '9') {
            if (digits < 3) millis = millis * 10 + (*pos - '0');
            ++digits;
            ++pos;
        }
        while (digits < 3) {
            millis *= 10;
            ++digits;
        }
    }

    int offset_seconds = 0;
    if (*pos == 'Z') {
        ++pos;
    } else if (*pos == '+' || *pos == '-') {
        const int sign = (*pos == '+') ? 1 : -1;
        ++pos;

        if (!std::isdigit(pos[0]) || !std::isdigit(pos[1])) return false;
        const int hours = (pos[0] - '0') * 10 + (pos[1] - '0');
        pos += 2;

        int minutes = 0;
        if (*pos == ':') ++pos;
        if (std::isdigit(pos[0]) && std::isdigit(pos[1])) {
            minutes = (pos[0] - '0') * 10 + (pos[1] - '0');
            pos += 2;
        }
        offset_seconds = sign * (hours * 3600 + minutes * 60);
    } else {
        return false;
    }

    if (*pos != '\0') return false;

    const std::time_t epoch = ::timegm(&tm);
    if (epoch == static_cast<std::time_t>(-1)) return false;

    const long long total_ms =
        (static_cast<long long>(epoch) - offset_seconds) * 1000LL + millis;
    if (total_ms < 0) return false;

    out_ms = static_cast<uint64_t>(total_ms);
    return true;
}

void apply_tpv(protocol::GpsData& data,
               const nlohmann::json& j,
               bool& time_parse_error)
{
    if (j.contains("lat")) data.latitude = j.at("lat").get<double>();
    if (j.contains("lon")) data.longitude = j.at("lon").get<double>();
    if (j.contains("speed")) data.speed_m_s = j.at("speed").get<float>();
    if (j.contains("track")) data.course_deg = j.at("track").get<float>();

    if (j.contains("altMSL")) data.altitude_m = j.at("altMSL").get<float>();
    else if (j.contains("altHAE")) data.altitude_m = j.at("altHAE").get<float>();

    if (j.contains("mode")) {
        const int mode = j.at("mode").get<int>();
        data.valid = mode >= 2;
        data.fix_quality = mode >= 3 ? 2 : (mode >= 2 ? 1 : 0);
    }

    if (j.contains("time")) {
        uint64_t timestamp_ms = 0;
        if (parse_utc_timestamp_ms(j.at("time").get<std::string>(), timestamp_ms)) {
            data.utc_timestamp_ms = timestamp_ms;
        } else {
            time_parse_error = true;
        }
    }
}

void apply_sky(protocol::GpsData& data, const nlohmann::json& j)
{
    if (j.contains("hdop")) data.hdop = j.at("hdop").get<float>();
    if (j.contains("pdop")) data.pdop = j.at("pdop").get<float>();
    if (j.contains("vdop")) data.vdop = j.at("vdop").get<float>();
    if (j.contains("nSat")) data.satellites_in_view = clamp_sat_count(j.at("nSat").get<int>());

    if (j.contains("satellites")) {
        const auto& satellites = j.at("satellites");
        uint8_t used = 0;
        for (const auto& sat : satellites) {
            if (sat.value("used", false)) ++used;
        }
        data.satellites_used = used;
        if (!j.contains("nSat")) data.satellites_in_view = clamp_sat_count(satellites.size());
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

void GpsdGpsSource::ingest_json_line_for_test(const std::string& line)
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

        rx_buf_.append(buf, static_cast<std::size_t>(n));
        std::size_t pos = 0;
        while ((pos = rx_buf_.find('\n')) != std::string::npos) {
            std::string line = rx_buf_.substr(0, pos);
            rx_buf_.erase(0, pos + 1);

            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            if (on_message_) on_message_();
            handle_json_line(line);
        }
    }
}

void GpsdGpsSource::handle_json_line(const std::string& line)
{
    try {
        const auto j = nlohmann::json::parse(line);
        const std::string cls = j.value("class", "");

        if (cls == "TPV") {
            bool time_parse_error = false;
            apply_tpv(data_, j, time_parse_error);
            if (time_parse_error && on_parse_error_) on_parse_error_();
            if (on_data_) on_data_(data_);
            return;
        }

        if (cls == "SKY") {
            apply_sky(data_, j);
            if (on_data_) on_data_(data_);
            return;
        }

        if (cls == "VERSION" || cls == "WATCH") return;

        if (on_parse_error_) on_parse_error_();
    } catch (...) {
        if (on_parse_error_) on_parse_error_();
    }
}

}  // namespace robot::device
