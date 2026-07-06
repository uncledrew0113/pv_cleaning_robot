/**
 * @file gpsd_json_parser.cc
 * @brief gpsd JSON 定位报文解析实现。
 *
 * 本文件解析 gpsd TPV/SKY 等 JSON 片段并转换为统一 GPS 数据结构。解析函数不访问网络，
 * 只处理单条 JSON 文本。
 */
#include "pv_cleaning_robot/protocol/gpsd_json_parser.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace robot::protocol {

namespace {

constexpr size_t kScalarBufCap = 128;

std::string_view trim_left(std::string_view v) noexcept {
    while (!v.empty() && std::isspace(static_cast<unsigned char>(v.front()))) {
        v.remove_prefix(1);
    }
    return v;
}

bool find_value_span(std::string_view json,
                     std::string_view key,
                     size_t& value_pos,
                     size_t& value_len) noexcept {
    const size_t key_pos = json.find(key);
    if (key_pos == std::string_view::npos) {
        return false;
    }
    size_t colon = json.find(':', key_pos + key.size());
    if (colon == std::string_view::npos) {
        return false;
    }
    ++colon;
    while (colon < json.size() &&
           std::isspace(static_cast<unsigned char>(json[colon]))) {
        ++colon;
    }
    if (colon >= json.size()) {
        return false;
    }

    size_t end = colon;
    if (json[colon] == '"') {
        ++end;
        bool escaped = false;
        while (end < json.size()) {
            const char c = json[end];
            if (c == '"' && !escaped) {
                ++end;
                break;
            }
            escaped = (c == '\\' && !escaped);
            if (c != '\\') {
                escaped = false;
            }
            ++end;
        }
    } else if (json[colon] == '[') {
        int depth = 1;
        ++end;
        while (end < json.size() && depth > 0) {
            if (json[end] == '[') {
                ++depth;
            } else if (json[end] == ']') {
                --depth;
            }
            ++end;
        }
    } else if (json[colon] == '{') {
        int depth = 1;
        ++end;
        while (end < json.size() && depth > 0) {
            if (json[end] == '{') {
                ++depth;
            } else if (json[end] == '}') {
                --depth;
            }
            ++end;
        }
    } else {
        while (end < json.size() && json[end] != ',' && json[end] != '}') {
            ++end;
        }
    }

    value_pos = colon;
    value_len = end - colon;
    return true;
}

bool extract_scalar(std::string_view json,
                    std::string_view key,
                    char* out,
                    size_t out_cap) noexcept {
    size_t pos = 0;
    size_t len = 0;
    if (!find_value_span(json, key, pos, len)) {
        return false;
    }
    auto value = trim_left(json.substr(pos, len));
    if (value.size() >= out_cap) {
        return false;
    }
    std::memcpy(out, value.data(), value.size());
    out[value.size()] = '\0';
    return true;
}

bool find_string(std::string_view json,
                 std::string_view key,
                 std::string_view& out) noexcept {
    size_t pos = 0;
    size_t len = 0;
    if (!find_value_span(json, key, pos, len)) {
        return false;
    }
    auto value = trim_left(json.substr(pos, len));
    if (value.size() < 2 || value.front() != '"' || value.back() != '"') {
        return false;
    }
    out = value.substr(1, value.size() - 2);
    return true;
}

bool find_double(std::string_view json, std::string_view key, double& out) noexcept {
    char buf[kScalarBufCap];
    if (!extract_scalar(json, key, buf, sizeof(buf))) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(buf, &end);
    return end && end != buf;
}

bool find_float(std::string_view json, std::string_view key, float& out) noexcept {
    double tmp = 0.0;
    if (!find_double(json, key, tmp)) {
        return false;
    }
    out = static_cast<float>(tmp);
    return true;
}

bool find_int(std::string_view json, std::string_view key, int& out) noexcept {
    char buf[kScalarBufCap];
    if (!extract_scalar(json, key, buf, sizeof(buf))) {
        return false;
    }
    char* end = nullptr;
    const long value = std::strtol(buf, &end, 10);
    if (!end || end == buf) {
        return false;
    }
    out = static_cast<int>(value);
    return true;
}

bool find_bool(std::string_view json, std::string_view key, bool& out) noexcept {
    size_t pos = 0;
    size_t len = 0;
    if (!find_value_span(json, key, pos, len)) {
        return false;
    }
    auto value = trim_left(json.substr(pos, len));
    if (value.substr(0, 4) == "true") {
        out = true;
        return true;
    }
    if (value.substr(0, 5) == "false") {
        out = false;
        return true;
    }
    return false;
}

bool find_array(std::string_view json,
                std::string_view key,
                std::string_view& out) noexcept {
    size_t pos = 0;
    size_t len = 0;
    if (!find_value_span(json, key, pos, len)) {
        return false;
    }
    auto value = trim_left(json.substr(pos, len));
    if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
        return false;
    }
    out = value.substr(1, value.size() - 2);
    return true;
}

bool parse_utc_timestamp_ms(std::string_view iso8601, uint64_t& out_ms) noexcept {
    char buf[kScalarBufCap];
    if (iso8601.size() >= sizeof(buf)) {
        return false;
    }
    std::memcpy(buf, iso8601.data(), iso8601.size());
    buf[iso8601.size()] = '\0';

    std::tm tm{};
    const char* pos = ::strptime(buf, "%Y-%m-%dT%H:%M:%S", &tm);
    if (!pos) {
        return false;
    }

    int millis = 0;
    if (*pos == '.') {
        ++pos;
        int digits = 0;
        while (*pos >= '0' && *pos <= '9') {
            if (digits < 3) {
                millis = millis * 10 + (*pos - '0');
            }
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
        if (!std::isdigit(static_cast<unsigned char>(pos[0])) ||
            !std::isdigit(static_cast<unsigned char>(pos[1]))) {
            return false;
        }
        const int hours = (pos[0] - '0') * 10 + (pos[1] - '0');
        pos += 2;

        int minutes = 0;
        if (*pos == ':') {
            ++pos;
        }
        if (std::isdigit(static_cast<unsigned char>(pos[0])) &&
            std::isdigit(static_cast<unsigned char>(pos[1]))) {
            minutes = (pos[0] - '0') * 10 + (pos[1] - '0');
            pos += 2;
        }
        offset_seconds = sign * (hours * 3600 + minutes * 60);
    } else {
        return false;
    }

    if (*pos != '\0') {
        return false;
    }

    const std::time_t epoch = ::timegm(&tm);
    if (epoch == static_cast<std::time_t>(-1)) {
        return false;
    }

    const long long total_ms =
        (static_cast<long long>(epoch) - offset_seconds) * 1000LL + millis;
    if (total_ms < 0) {
        return false;
    }
    out_ms = static_cast<uint64_t>(total_ms);
    return true;
}

uint8_t clamp_sat_count(size_t count) noexcept {
    return static_cast<uint8_t>(count > 255 ? 255 : count);
}

bool parse_tpv(std::string_view json, GpsData& data, GpsdJsonParseResult& result) noexcept {
    double lat = 0.0;
    double lon = 0.0;
    float scalar = 0.0f;
    int mode = 0;
    std::string_view time_str;

    if (find_double(json, "\"lat\"", lat)) {
        data.latitude = lat;
    }
    if (find_double(json, "\"lon\"", lon)) {
        data.longitude = lon;
    }
    if (find_float(json, "\"speed\"", scalar)) {
        data.speed_m_s = scalar;
    }
    if (find_float(json, "\"track\"", scalar)) {
        data.course_deg = scalar;
    }
    if (find_float(json, "\"altMSL\"", scalar)) {
        data.altitude_m = scalar;
    } else if (find_float(json, "\"altHAE\"", scalar)) {
        data.altitude_m = scalar;
    }
    if (find_int(json, "\"mode\"", mode)) {
        data.valid = mode >= 2;
        data.fix_quality = mode >= 3 ? 2 : (mode >= 2 ? 1 : 0);
        result.updated_fix = true;
    }
    if (find_string(json, "\"time\"", time_str)) {
        uint64_t timestamp_ms = 0;
        if (parse_utc_timestamp_ms(time_str, timestamp_ms)) {
            data.utc_timestamp_ms = timestamp_ms;
        } else {
            result.time_parse_error = true;
        }
    }
    return true;
}

bool parse_sky(std::string_view json, GpsData& data, GpsdJsonParseResult& result) noexcept {
    float scalar = 0.0f;
    int n_sat = 0;
    if (find_float(json, "\"hdop\"", scalar)) {
        data.hdop = scalar;
    }
    if (find_float(json, "\"pdop\"", scalar)) {
        data.pdop = scalar;
    }
    if (find_float(json, "\"vdop\"", scalar)) {
        data.vdop = scalar;
    }
    if (find_int(json, "\"nSat\"", n_sat)) {
        data.satellites_in_view = clamp_sat_count(static_cast<size_t>(n_sat));
    }

    std::string_view satellites;
    if (find_array(json, "\"satellites\"", satellites)) {
        size_t in_view = 0;
        uint8_t used = 0;
        size_t pos = 0;
        while ((pos = satellites.find('{', pos)) != std::string_view::npos) {
            size_t end = satellites.find('}', pos);
            if (end == std::string_view::npos) {
                result.parse_error = true;
                return false;
            }
            const auto item = satellites.substr(pos, end - pos + 1);
            bool is_used = false;
            if (find_bool(item, "\"used\"", is_used) && is_used) {
                ++used;
            }
            ++in_view;
            pos = end + 1;
        }
        data.satellites_used = used;
        if (!find_int(json, "\"nSat\"", n_sat)) {
            data.satellites_in_view = clamp_sat_count(in_view);
        }
    }
    result.updated_satellites = true;
    return true;
}

}  // namespace

bool GpsdJsonParser::parse_line(std::string_view line,
                                GpsData& data,
                                GpsdJsonParseResult& result) noexcept {
    result = {};

    std::string_view cls;
    if (!find_string(line, "\"class\"", cls)) {
        result.parse_error = true;
        return false;
    }

    if (cls == "TPV") {
        result.message_class = GpsdMessageClass::TPV;
        return parse_tpv(line, data, result);
    }
    if (cls == "SKY") {
        result.message_class = GpsdMessageClass::SKY;
        return parse_sky(line, data, result);
    }
    if (cls == "VERSION") {
        result.message_class = GpsdMessageClass::VERSION;
        return true;
    }
    if (cls == "WATCH") {
        result.message_class = GpsdMessageClass::WATCH;
        return true;
    }
    if (cls == "ERROR") {
        result.message_class = GpsdMessageClass::ERROR_MESSAGE;
        result.parse_error = true;
        return false;
    }

    result.message_class = GpsdMessageClass::UNKNOWN;
    result.parse_error = true;
    return false;
}

}  // namespace robot::protocol
