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

constexpr float kWheelRpmLimit = 50.0f;
constexpr std::chrono::milliseconds kIoSleepDisabled(50);
constexpr std::chrono::milliseconds kIoPollInterval(20);

float normalize_yaw_to_control_error(robot::domain::Endpoint /*primary_dock*/,
                                     robot::domain::TravelDirection travel_direction,
                                     float raw_yaw_deg) {
    // UDS yaw 的纠偏含义只与当前运动方向有关，不与停机端配置有关：
    // - yaw>0 时下轨道轮向 A 方向加速；
    // - yaw<0 时下轨道轮向 B 方向加速。
    // 当前下轮 base_command 已按目标方向设置正负号，因此两种方向都需要
    // 将 yaw>0 转换成负 correction，只作用到下轨道轮。
    switch (travel_direction) {
    case robot::domain::TravelDirection::AToB:
    case robot::domain::TravelDirection::BToA:
        return -raw_yaw_deg;
    }
    return -raw_yaw_deg;
}

HeadingCorrector::SpeedCommand apply_base_abs_rpm(const HeadingCorrector::SpeedCommand& base,
                                                  float abs_rpm) {
    HeadingCorrector::SpeedCommand out{};
    auto with_abs = [abs_rpm](float current) {
        if (current > 0.0f) {
            return abs_rpm;
        }
        if (current < 0.0f) {
            return -abs_rpm;
        }
        return 0.0f;
    };
    out.lt_rpm = with_abs(base.lt_rpm);
    out.rt_rpm = with_abs(base.rt_rpm);
    out.lb_rpm = with_abs(base.lb_rpm);
    out.rb_rpm = with_abs(base.rb_rpm);
    return out;
}

}  // namespace

HeadingCorrector::HeadingCorrector() {
    yaw_fusion_.set_params(params_.fusion);
    start_io_thread_if_needed();
}

HeadingCorrector::HeadingCorrector(const Params& p) : params_(p) {
    yaw_fusion_.set_params(params_.fusion);
    start_io_thread_if_needed();
}

HeadingCorrector::~HeadingCorrector() {
    stop_io_thread();
}

void HeadingCorrector::set_params(const Params& p) {
    std::lock_guard<std::mutex> lock(mu_);
    const bool uds_path_changed = params_.uds_path != p.uds_path;
    params_ = p;
    yaw_fusion_.set_params(params_.fusion);
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

    const bool use_fusion = params_.angle_source == AngleSource::FUSED_UDS_GYRO;
    if (!sample_ready && !use_fusion) {
        reset_control_state_locked();
        if (!connected_) {
            mode_ = Mode::DISCONNECTED;
        } else {
            mode_ = Mode::STALE;
        }
        return output;
    }

    float yaw_for_control = latest_result_.yaw_deg;
    if (use_fusion) {
        last_fusion_output_ = yaw_fusion_.update({input.dt_s,
                                                  latest_result_.yaw_deg,
                                                  sample_ready,
                                                  latest_result_.confidence,
                                                  result_age_ms,
                                                  input.gyro_z_rad_s,
                                                  input.imu_valid});
        if (!last_fusion_output_.valid) {
            reset_control_state_locked();
            mode_ = connected_ ? Mode::STALE : Mode::DISCONNECTED;
            return output;
        }
        yaw_for_control = last_fusion_output_.fused_yaw_deg;
    } else {
        last_fusion_output_ = {};
    }

    // 注意：filtered_yaw_deg / correction 表示“控制误差”的符号，不是原始视觉 yaw 的符号。
    float error = normalize_yaw_to_control_error(
        input.primary_dock, input.travel_direction, yaw_for_control);
    error *= params_.output_sign;

    if (!filter_initialized_) {
        filtered_yaw_deg_ = error;
        filter_initialized_ = true;
    } else {
        filtered_yaw_deg_ = low_pass(filtered_yaw_deg_, error, params_.yaw_alpha);
    }

    error = filtered_yaw_deg_;
    if (std::abs(error) < params_.deadband_yaw_deg) {
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
    SpeedCommand base_for_correction = input.base_command;
    if (params_.slow_on_error && std::abs(error) >= params_.yaw_slow_threshold_deg) {
        base_for_correction =
            apply_base_abs_rpm(input.base_command, std::max(0.0f, params_.slow_base_rpm));
    }
    output.speed_command =
        apply_correction(base_for_correction, correction, params_.wheel_strategy);
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
            latest_result_.yaw_deg,
            latest_result_.confidence,
            filtered_yaw_deg_,
            integral_term_,
            last_correction_,
            latest_result_age_ms_locked(now),
            last_fusion_output_.fused_yaw_deg,
            last_fusion_output_.valid,
            last_fusion_output_.gyro_z_dps,
            last_fusion_output_.innovation_deg,
            last_fusion_output_.kalman_gain_angle};
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
                                                                  float correction_rpm,
                                                                  WheelStrategy strategy) {
    SpeedCommand corrected = base;
    auto adjust_wheel = [](float base_rpm, float correction) {
        if (base_rpm > 0.0f) {
            return clamp(base_rpm + correction, 0.0f, kWheelRpmLimit);
        }
        if (base_rpm < 0.0f) {
            return clamp(base_rpm + correction, -kWheelRpmLimit, 0.0f);
        }
        return 0.0f;
    };
    auto decel_only = [](float base_rpm, float candidate) {
        if (base_rpm > 0.0f) {
            return clamp(candidate, 0.0f, base_rpm);
        }
        if (base_rpm < 0.0f) {
            return clamp(candidate, base_rpm, 0.0f);
        }
        return 0.0f;
    };

    switch (strategy) {
        case WheelStrategy::ALL_WHEELS:
            // 上下轨道共同参与纠偏：由于两条轨道基础转向相反，同一 correction 会形成轨道差速。
            corrected.lt_rpm = adjust_wheel(base.lt_rpm, correction_rpm);
            corrected.rt_rpm = adjust_wheel(base.rt_rpm, correction_rpm);
            corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
            corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
            break;
        case WheelStrategy::LOWER_ONLY:
            corrected.lt_rpm = base.lt_rpm;
            corrected.rt_rpm = base.rt_rpm;
            corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
            corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
            break;
        case WheelStrategy::TOP_DECEL_ONLY:
            corrected.lt_rpm =
                decel_only(base.lt_rpm, adjust_wheel(base.lt_rpm, correction_rpm));
            corrected.rt_rpm =
                decel_only(base.rt_rpm, adjust_wheel(base.rt_rpm, correction_rpm));
            corrected.lb_rpm = adjust_wheel(base.lb_rpm, correction_rpm);
            corrected.rb_rpm = adjust_wheel(base.rb_rpm, correction_rpm);
            break;
    }
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

    const auto yaw_it = doc.FindMember("yaw_deg");
    if (yaw_it != doc.MemberEnd() && yaw_it->value.IsNumber()) {
        result.yaw_deg = yaw_it->value.GetFloat();
    } else {
        return;
    }

    const auto confidence_it = doc.FindMember("confidence");
    if (confidence_it != doc.MemberEnd() && confidence_it->value.IsNumber()) {
        result.confidence = confidence_it->value.GetFloat();
    }

    latest_result_ = std::move(result);
}

void HeadingCorrector::reset_control_state_locked() {
    filter_initialized_ = false;
    filtered_yaw_deg_ = 0.0f;
    integral_term_ = 0.0f;
    last_error_ = 0.0f;
    last_correction_ = 0.0f;
    yaw_fusion_.reset();
    last_fusion_output_ = {};
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
