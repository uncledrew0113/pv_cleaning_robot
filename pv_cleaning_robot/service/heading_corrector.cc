#include "pv_cleaning_robot/service/heading_corrector.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <rapidjson/document.h>

namespace robot::service {

namespace {

constexpr float kWheelRpmLimit = 210.0f;
constexpr std::chrono::milliseconds kIoSleepDisabled(50);
constexpr std::chrono::milliseconds kIoPollInterval(20);

}  // namespace

HeadingCorrector::HeadingCorrector() {
    start_io_thread_if_needed();
}

HeadingCorrector::HeadingCorrector(const Params& p) : params_(p) {
    start_io_thread_if_needed();
}

HeadingCorrector::~HeadingCorrector() {
    stop_io_thread();
}

void HeadingCorrector::set_params(const Params& p) {
    std::lock_guard<std::mutex> lock(mu_);
    const bool uds_path_changed = params_.uds_path != p.uds_path;
    params_ = p;
    if (uds_path_changed) {
        close_socket_locked();
        connected_path_.clear();
        connected_ = false;
        mode_ = Mode::DISCONNECTED;
    }
}

void HeadingCorrector::enable(bool en) {
    const bool previous = enabled_.exchange(en, std::memory_order_relaxed);
    if (previous == en) {
        return;
    }

    std::lock_guard<std::mutex> lock(mu_);
    reset_control_state_locked();
    if (!en) {
        mode_ = Mode::UNINITIALIZED;
    }
}

void HeadingCorrector::reset() {
    std::lock_guard<std::mutex> lock(mu_);
    reset_control_state_locked();
}

HeadingCorrector::Output HeadingCorrector::compute(const Input& input) {
    Output output{};
    if (!enabled_.load(std::memory_order_relaxed) || !input.has_base_command) {
        return output;
    }

    output.has_speed_command = true;
    output.speed_command = input.base_command;

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mu_);

    const int64_t result_age_ms = latest_result_age_ms_locked(now);
    const bool sample_ready =
        latest_result_.available && latest_result_.valid &&
        latest_result_.confidence >= params_.min_confidence && result_age_ms >= 0 &&
        result_age_ms <= params_.result_timeout_ms;

    if (!sample_ready) {
        reset_control_state_locked();
        if (!connected_) {
            mode_ = Mode::DISCONNECTED;
        } else {
            mode_ = Mode::STALE;
        }
        return output;
    }

    // apply_correction() 将同一个有符号修正量加到四个轮子的“带符号基速”上。
    // 由于 MotionService 已经在 base_command 中编码了停车位左右和前进/返回的方向，
    // slope>0 需要的物理效果（上轮朝“减速/加速”、下轮朝相反方向变化）在所有工况下
    // 都对应同一个 correction 符号：负值。需要现场整体翻向时再用 output_sign 兜底。
    constexpr float kSlopeToCorrectionSign = -1.0f;
    float error = latest_result_.slope * params_.output_sign * kSlopeToCorrectionSign;

    if (!filter_initialized_) {
        filtered_slope_ = error;
        filter_initialized_ = true;
    } else {
        filtered_slope_ = low_pass(filtered_slope_, error, params_.slope_alpha);
    }

    error = filtered_slope_;
    if (std::abs(error) < params_.deadband_slope) {
        error = 0.0f;
    }

    if (input.dt_s <= 0.0f) {
        last_error_ = error;
        mode_ = Mode::TRACK;
        return output;
    }

    integral_term_ += error * input.dt_s;
    integral_term_ = clamp(integral_term_, -params_.integral_limit, params_.integral_limit);
    const float derivative = (error - last_error_) / input.dt_s;
    last_error_ = error;

    float correction =
        params_.kp * error + params_.ki * integral_term_ + params_.kd * derivative;
    correction = clamp(correction, -params_.max_output, params_.max_output);

    if (std::abs(correction) < params_.min_effective_output) {
        correction = 0.0f;
    }

    output.feedback_correction_rpm = correction;
    output.correction_rpm = correction;
    output.speed_command = apply_correction(input.base_command, correction);
    last_correction_ = correction;
    mode_ = Mode::TRACK;
    return output;
}

HeadingCorrector::DebugState HeadingCorrector::debug_state() const {
    std::lock_guard<std::mutex> lock(mu_);
    const auto now = std::chrono::steady_clock::now();
    return {mode_,
            connected_,
            latest_result_.valid,
            latest_result_.slope,
            latest_result_.confidence,
            filtered_slope_,
            integral_term_,
            last_correction_,
            latest_result_age_ms_locked(now)};
}

float HeadingCorrector::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float HeadingCorrector::clamp_alpha(float alpha) {
    return clamp(alpha, 0.0f, 1.0f);
}

float HeadingCorrector::low_pass(float previous, float sample, float alpha) {
    return previous + clamp_alpha(alpha) * (sample - previous);
}

HeadingCorrector::SpeedCommand HeadingCorrector::apply_correction(const SpeedCommand& base,
                                                                  float correction_rpm) {
    SpeedCommand corrected = base;
    corrected.lt_rpm = clamp(base.lt_rpm + correction_rpm, -kWheelRpmLimit, kWheelRpmLimit);
    corrected.rt_rpm = clamp(base.rt_rpm + correction_rpm, -kWheelRpmLimit, kWheelRpmLimit);
    corrected.lb_rpm = clamp(base.lb_rpm + correction_rpm, -kWheelRpmLimit, kWheelRpmLimit);
    corrected.rb_rpm = clamp(base.rb_rpm + correction_rpm, -kWheelRpmLimit, kWheelRpmLimit);
    return corrected;
}

void HeadingCorrector::start_io_thread_if_needed() {
    std::lock_guard<std::mutex> lock(mu_);
    if (io_thread_started_) {
        return;
    }
    stop_requested_.store(false, std::memory_order_relaxed);
    io_thread_ = std::thread([this]() { io_loop(); });
    io_thread_started_ = true;
}

void HeadingCorrector::stop_io_thread() {
    stop_requested_.store(true, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(mu_);
        close_socket_locked();
    }
    if (io_thread_started_ && io_thread_.joinable()) {
        io_thread_.join();
    }
    io_thread_started_ = false;
}

void HeadingCorrector::io_loop() {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        Params params;
        bool enabled = false;
        bool connected_socket = false;
        {
            std::lock_guard<std::mutex> lock(mu_);
            params = params_;
            enabled = enabled_.load(std::memory_order_relaxed);
        }

        if (!enabled || params.uds_path.empty()) {
            std::this_thread::sleep_for(kIoSleepDisabled);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mu_);
            if (socket_fd_ < 0 && !connect_locked(params)) {
                mode_ = Mode::DISCONNECTED;
            } else if (socket_fd_ >= 0) {
                consume_socket_locked();
            }
            connected_socket = socket_fd_ >= 0;
        }

        std::this_thread::sleep_for(connected_socket
                                        ? kIoPollInterval
                                        : std::chrono::milliseconds(
                                              std::max(1, params.reconnect_interval_ms)));
    }
}

void HeadingCorrector::close_socket_locked() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    rx_buffer_.clear();
    connected_ = false;
}

bool HeadingCorrector::connect_locked(const Params& params) {
    if (params.uds_path.empty()) {
        return false;
    }

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (params.uds_path.size() >= sizeof(addr.sun_path)) {
        close(fd);
        return false;
    }
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", params.uds_path.c_str());

    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    socket_fd_ = fd;
    connected_path_ = params.uds_path;
    connected_ = true;
    mode_ = Mode::STALE;
    return true;
}

void HeadingCorrector::consume_socket_locked() {
    if (socket_fd_ < 0) {
        return;
    }

    char buf[1024];
    for (;;) {
        const ssize_t n = recv(socket_fd_, buf, sizeof(buf), MSG_DONTWAIT);
        if (n > 0) {
            rx_buffer_.append(buf, static_cast<size_t>(n));
            std::string::size_type pos = std::string::npos;
            while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
                std::string line = rx_buffer_.substr(0, pos);
                rx_buffer_.erase(0, pos + 1);
                if (!line.empty()) {
                    ingest_json_line_locked(line);
                }
            }
            continue;
        }

        if (n == 0) {
            close_socket_locked();
            mode_ = Mode::DISCONNECTED;
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }

        close_socket_locked();
        mode_ = Mode::DISCONNECTED;
        return;
    }
}

void HeadingCorrector::ingest_json_line_locked(const std::string& line) {
    rapidjson::Document doc;
    doc.Parse(line.c_str(), line.size());
    if (doc.HasParseError() || !doc.IsObject()) {
        return;
    }

    VisionResult result;
    result.available = true;
    result.received_at = std::chrono::steady_clock::now();

    const auto valid_it = doc.FindMember("valid");
    if (valid_it == doc.MemberEnd() || !valid_it->value.IsBool()) {
        return;
    }
    result.valid = valid_it->value.GetBool();

    const auto slope_it = doc.FindMember("slope");
    if (slope_it == doc.MemberEnd() || !slope_it->value.IsNumber()) {
        return;
    }
    result.slope = slope_it->value.GetFloat();

    const auto confidence_it = doc.FindMember("confidence");
    if (confidence_it != doc.MemberEnd() && confidence_it->value.IsNumber()) {
        result.confidence = confidence_it->value.GetFloat();
    }

    latest_result_ = std::move(result);
}

void HeadingCorrector::reset_control_state_locked() {
    filter_initialized_ = false;
    filtered_slope_ = 0.0f;
    integral_term_ = 0.0f;
    last_error_ = 0.0f;
    last_correction_ = 0.0f;
}

int64_t HeadingCorrector::latest_result_age_ms_locked(
    std::chrono::steady_clock::time_point now) const {
    if (!latest_result_.available) {
        return -1;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - latest_result_.received_at)
        .count();
}

}  // namespace robot::service
