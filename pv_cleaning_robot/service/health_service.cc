/*
 * 健康与诊断 payload 序列化实现。
 *
 * HealthService 优先消费 DiagnosticsCollector 的统一快照，保证健康上报和错误管理使用
 * 同一份设备诊断事实；保留直接读取设备 getter 的构造路径仅用于兼容测试和旧调用点。
 */
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <spdlog/logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/service/health_service.h"

namespace robot::service {

namespace {

class FixedJsonWriter {
   public:
    FixedJsonWriter(char* out, size_t cap) noexcept : out_(out), cap_(cap) {
        if (out_ && cap_ > 0) {
            out_[0] = '\0';
        }
    }

    size_t size() const noexcept {
        return overflow_ ? 0u : len_;
    }

    void append_raw(const char* text) noexcept {
        append_format("%s", text ? text : "");
    }

    void append_char(char ch) noexcept {
        if (!ensure_space(1u)) {
            return;
        }
        out_[len_++] = ch;
        out_[len_] = '\0';
    }

    void append_quoted(const char* text) noexcept {
        append_char('"');
        if (!text) {
            text = "";
        }
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p != '\0';
             ++p) {
            switch (*p) {
                case '"':
                    append_raw("\\\"");
                    break;
                case '\\':
                    append_raw("\\\\");
                    break;
                case '\b':
                    append_raw("\\b");
                    break;
                case '\f':
                    append_raw("\\f");
                    break;
                case '\n':
                    append_raw("\\n");
                    break;
                case '\r':
                    append_raw("\\r");
                    break;
                case '\t':
                    append_raw("\\t");
                    break;
                default:
                    if (*p < 0x20u) {
                        append_format("\\u%04x", static_cast<unsigned>(*p));
                    } else {
                        append_char(static_cast<char>(*p));
                    }
                    break;
            }
            if (overflow_) {
                return;
            }
        }
        append_char('"');
    }

    void append_bool(bool v) noexcept {
        append_raw(v ? "true" : "false");
    }
    void append_int(int v) noexcept {
        append_format("%d", v);
    }
    void append_uint(unsigned v) noexcept {
        append_format("%u", v);
    }
    void append_uint64(uint64_t v) noexcept {
        append_format("%llu", static_cast<unsigned long long>(v));
    }
    void append_float(float v) noexcept {
        append_format("%.6f", static_cast<double>(v));
    }
    void append_double(double v) noexcept {
        append_format("%.6f", v);
    }

   private:
    bool ensure_space(size_t extra) noexcept {
        if (!out_ || cap_ == 0 || overflow_ || len_ + extra >= cap_) {
            overflow_ = true;
            if (out_ && cap_ > 0) {
                out_[cap_ - 1] = '\0';
            }
            len_ = 0;
            return false;
        }
        return true;
    }

    void append_format(const char* fmt, ...) noexcept {
        if (!out_ || cap_ == 0 || overflow_) {
            return;
        }
        va_list args;
        va_start(args, fmt);
        const int written = std::vsnprintf(out_ + len_, cap_ - len_, fmt, args);
        va_end(args);
        if (written < 0 || static_cast<size_t>(written) >= cap_ - len_) {
            overflow_ = true;
            if (out_ && cap_ > 0) {
                out_[cap_ - 1] = '\0';
            }
            len_ = 0;
            return;
        }
        len_ += static_cast<size_t>(written);
    }

    char* out_{nullptr};
    size_t cap_{0};
    size_t len_{0};
    bool overflow_{false};
};

const char* wheel_name(int idx) noexcept {
    static constexpr const char* kWheelNames[device::WalkMotorGroup::kWheelCount] = {
        "lt", "rt", "lb", "rb"};
    return kWheelNames[idx];
}

}  // namespace

size_t HealthPayloadBuilder::build_health(const HealthView& view, char* out, size_t cap) noexcept {
    FixedJsonWriter w(out, cap);

    w.append_raw("{\"ts\":");
    w.append_uint64(view.ts_ms);
    w.append_raw(",\"values\":{");
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        if (i != 0) {
            w.append_char(',');
        }
        const auto& wheel = view.walk.wheel[i];
        w.append_raw("\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_rpm\":");
        w.append_float(wheel.speed_rpm);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_cur\":");
        w.append_float(wheel.torque_a);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_err\":");
        w.append_bool(wheel.fault != protocol::WalkMotorFault::NONE);
    }
    w.append_raw(",\"br_rpm\":");
    w.append_int(view.brush.actual_rpm);
    w.append_raw(",\"br_run\":");
    w.append_bool(view.brush.running);
    w.append_raw(",\"br_err\":");
    w.append_bool(view.brush.fault);
    w.append_raw(",\"bat_soc\":");
    w.append_float(view.bms.soc_pct);
    w.append_raw(",\"bat_vol\":");
    w.append_float(view.bms.voltage_v);
    w.append_raw(",\"bat_chg\":");
    w.append_bool(view.bms.charging);
    w.append_raw(",\"bat_alm\":");
    w.append_bool(view.bms.alarm_flags != 0u);
    w.append_raw(",\"imu_p\":");
    w.append_float(view.imu.pitch_deg);
    w.append_raw(",\"imu_r\":");
    w.append_float(view.imu.roll_deg);
    w.append_raw(",\"imu_y\":");
    w.append_float(view.imu.yaw_deg);
    w.append_raw(",\"gps_lat\":");
    w.append_double(view.gps.latitude);
    w.append_raw(",\"gps_lon\":");
    w.append_double(view.gps.longitude);
    w.append_raw(",\"gps_fix\":");
    w.append_int(view.gps.fix_quality);
    w.append_char('}');
    w.append_char('}');
    return w.size();
}

size_t HealthPayloadBuilder::build_health(const DiagnosticsCollector::Snapshot& snapshot,
                                          char* out,
                                          size_t cap) noexcept {
    HealthView view{};
    view.ts_ms = snapshot.epoch_ms;
    view.walk = snapshot.walk_status;
    view.brush = snapshot.brush_status;
    view.bms = snapshot.bms_data;
    view.imu = snapshot.imu_data;
    view.gps = snapshot.gps_data;
    return build_health(view, out, cap);
}

size_t HealthPayloadBuilder::build_diagnostics(const DiagnosticsView& view,
                                               char* out,
                                               size_t cap) noexcept {
    FixedJsonWriter w(out, cap);

    w.append_raw("{\"ts\":");
    w.append_uint64(view.ts_ms);
    w.append_raw(",\"values\":{");
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        if (i != 0) {
            w.append_char(',');
        }
        const auto& wheel = view.walk.wheel[i];
        w.append_raw("\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_rpm\":");
        w.append_float(wheel.speed_rpm);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_tgt\":");
        w.append_float(wheel.target_value);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_cur\":");
        w.append_float(wheel.torque_a);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_err\":");
        w.append_bool(wheel.fault != protocol::WalkMotorFault::NONE);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_ec\":");
        w.append_int(static_cast<int>(wheel.fault));
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_on\":");
        w.append_bool(wheel.online);
        w.append_raw(",\"");
        w.append_raw(wheel_name(i));
        w.append_raw("_ce\":");
        w.append_uint(wheel.can_err_count);
    }
    w.append_raw(",\"walk_cf\":");
    w.append_uint(view.walk.ctrl_frame_count);
    w.append_raw(",\"walk_ce\":");
    w.append_uint(view.walk.ctrl_err_count);
    w.append_raw(",\"br_rpm\":");
    w.append_int(view.brush.actual_rpm);
    w.append_raw(",\"br_tgt\":");
    w.append_int(view.brush.target_rpm);
    w.append_raw(",\"br_cur\":");
    w.append_float(view.brush.current_a);
    w.append_raw(",\"br_vol\":");
    w.append_float(view.brush.bus_voltage_v);
    w.append_raw(",\"br_tmp\":");
    w.append_float(view.brush.temperature_c);
    w.append_raw(",\"br_stl\":");
    w.append_uint(view.brush.stall_count);
    w.append_raw(",\"br_ce\":");
    w.append_uint(view.brush.comm_error_count);
    w.append_raw(",\"bat_soc\":");
    w.append_float(view.bms.soc_pct);
    w.append_raw(",\"bat_vol\":");
    w.append_float(view.bms.voltage_v);
    w.append_raw(",\"bat_cur\":");
    w.append_float(view.bms.current_a);
    w.append_raw(",\"bat_tmp\":");
    w.append_float(view.bms.temperature_c);
    w.append_raw(",\"bat_cmax\":");
    w.append_float(view.bms.cell_voltage_max_v);
    w.append_raw(",\"bat_cmin\":");
    w.append_float(view.bms.cell_voltage_min_v);
    w.append_raw(",\"bat_rah\":");
    w.append_float(view.bms.remaining_capacity_ah);
    w.append_raw(",\"bat_cyc\":");
    w.append_uint(view.bms.cycle_count);
    w.append_raw(",\"bat_alm\":");
    w.append_uint(view.bms.alarm_flags);
    w.append_raw(",\"imu_ax\":");
    w.append_float(view.imu.accel[0]);
    w.append_raw(",\"imu_ay\":");
    w.append_float(view.imu.accel[1]);
    w.append_raw(",\"imu_az\":");
    w.append_float(view.imu.accel[2]);
    w.append_raw(",\"imu_gx\":");
    w.append_float(view.imu.gyro[0]);
    w.append_raw(",\"imu_gy\":");
    w.append_float(view.imu.gyro[1]);
    w.append_raw(",\"imu_gz\":");
    w.append_float(view.imu.gyro[2]);
    w.append_raw(",\"imu_p\":");
    w.append_float(view.imu.pitch_deg);
    w.append_raw(",\"imu_r\":");
    w.append_float(view.imu.roll_deg);
    w.append_raw(",\"imu_y\":");
    w.append_float(view.imu.yaw_deg);
    w.append_raw(",\"imu_fr\":");
    w.append_float(view.imu.frame_rate_hz);
    w.append_raw(",\"imu_pe\":");
    w.append_uint(view.imu.parse_error_count);
    w.append_raw(",\"gps_lat\":");
    w.append_double(view.gps.latitude);
    w.append_raw(",\"gps_lon\":");
    w.append_double(view.gps.longitude);
    w.append_raw(",\"gps_alt\":");
    w.append_float(view.gps.altitude_m);
    w.append_raw(",\"gps_spd\":");
    w.append_float(view.gps.speed_m_s);
    w.append_raw(",\"gps_sat\":");
    w.append_uint(view.gps.satellites_used);
    w.append_raw(",\"gps_hdp\":");
    w.append_float(view.gps.hdop);
    w.append_raw(",\"gps_fix\":");
    w.append_int(view.gps.fix_quality);
    w.append_raw(",\"gps_sent\":");
    w.append_uint(view.gps.sentence_count);
    w.append_char('}');
    w.append_char('}');
    return w.size();
}

size_t HealthPayloadBuilder::build_diagnostics(const DiagnosticsCollector::Snapshot& snapshot,
                                               char* out,
                                               size_t cap) noexcept {
    DiagnosticsView view{};
    view.ts_ms = snapshot.epoch_ms;
    view.walk = snapshot.walk_diagnostics;
    view.brush = snapshot.brush_diagnostics;
    view.bms = snapshot.bms_diagnostics;
    view.imu = snapshot.imu_diagnostics;
    view.gps = snapshot.gps_diagnostics;
    return build_diagnostics(view, out, cap);
}

HealthService::HealthService(std::shared_ptr<device::WalkMotorGroup> walk,
                             std::shared_ptr<device::BrushMotor> brush,
                             std::shared_ptr<device::BMS> bms,
                             std::shared_ptr<device::ImuDevice> imu,
                             std::shared_ptr<device::GpsDevice> gps,
                             std::shared_ptr<CloudService> cloud,
                             Mode mode,
                             std::string local_log_path,
                             size_t local_log_max_bytes,
                             size_t local_log_max_files)
    : walk_(std::move(walk))
    , brush_(std::move(brush))
    , bms_(std::move(bms))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , cloud_(std::move(cloud))
    , mode_(mode) {
    payload_cache_.reserve(kPayloadBufferBytes);
    // 本地 JSONL 轮转日志（仅 local_log_path 非空时开启，独立于 MQTT/LoRaWAN）
    if (!local_log_path.empty()) {
        try {
            const auto log_path = std::filesystem::path(local_log_path);
            const auto parent = log_path.parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent);
            }

            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                local_log_path,
                std::max<size_t>(1u, local_log_max_bytes),
                std::max<size_t>(1u, local_log_max_files),
                false);
            local_log_ = std::make_shared<spdlog::logger>("", std::move(sink));
            local_log_->set_pattern("%v");
            local_log_->flush_on(spdlog::level::info);
        } catch (const std::exception& ex) {
            spdlog::error("[HealthService] failed to initialize local rotating log: {}", ex.what());
            local_log_.reset();
        }
    }
}

HealthService::HealthService(std::shared_ptr<DiagnosticsCollector> diagnostics,
                             std::shared_ptr<CloudService> cloud,
                             Mode mode,
                             std::string local_log_path,
                             size_t local_log_max_bytes,
                             size_t local_log_max_files)
    : cloud_(std::move(cloud))
    , diagnostics_(std::move(diagnostics))
    , mode_(mode) {
    payload_cache_.reserve(kPayloadBufferBytes);
    // 本地 JSONL 轮转日志（仅 local_log_path 非空时开启，独立于 MQTT/LoRaWAN）
    if (!local_log_path.empty()) {
        try {
            const auto log_path = std::filesystem::path(local_log_path);
            const auto parent = log_path.parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent);
            }

            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                local_log_path,
                std::max<size_t>(1u, local_log_max_bytes),
                std::max<size_t>(1u, local_log_max_files),
                false);
            local_log_ = std::make_shared<spdlog::logger>("", std::move(sink));
            local_log_->set_pattern("%v");
            local_log_->flush_on(spdlog::level::info);
        } catch (const std::exception& ex) {
            spdlog::error("[HealthService] failed to initialize local rotating log: {}", ex.what());
            local_log_.reset();
        }
    }
}

void HealthService::update() {
    const size_t payload_len = build_payload(payload_buf_.data(), payload_buf_.size());
    if (payload_len == 0u) {
        spdlog::error("[HealthService] failed to build telemetry payload");
        return;
    }

    payload_cache_.assign(payload_buf_.data(), payload_len);
    if (cloud_)
        cloud_->publish_telemetry(payload_cache_);  // cloud_ 为 nullptr 时（单元测试场景）跳过。
    // 本地 JSONL 落盘：每条记录一行，独立于网络链路，便于现场离线排查。
    if (local_log_) {
        try {
            local_log_->log(spdlog::level::info,
                            spdlog::string_view_t(payload_cache_.data(), payload_cache_.size()));
        } catch (const std::exception& ex) {
            spdlog::error(
                "[HealthService] local rotating log write failed, disabling local log: {}",
                ex.what());
            local_log_.reset();
        }
    }
}

size_t HealthService::build_payload(char* out, size_t cap) const {
    if (diagnostics_) {
        const auto snapshot = diagnostics_->snapshot();
        if (mode_ == Mode::DIAGNOSTICS) {
            return HealthPayloadBuilder::build_diagnostics(snapshot, out, cap);
        }
        return HealthPayloadBuilder::build_health(snapshot, out, cap);
    }

    const auto now = std::chrono::system_clock::now();
    const auto ts_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());

    if (mode_ == Mode::DIAGNOSTICS) {
        HealthPayloadBuilder::DiagnosticsView view{};
        view.ts_ms = ts_ms;
        view.walk = walk_->get_group_diagnostics();
        view.brush = brush_->get_diagnostics();
        view.bms = bms_->get_diagnostics();
        view.imu = imu_->get_diagnostics();
        view.gps = gps_->get_diagnostics();
        return HealthPayloadBuilder::build_diagnostics(view, out, cap);
    }

    HealthPayloadBuilder::HealthView view{};
    view.ts_ms = ts_ms;
    view.walk = walk_->get_group_status();
    view.brush = brush_->get_status();
    view.bms = bms_->get_data();
    view.imu = imu_->get_latest();
    view.gps = gps_->get_latest();
    return HealthPayloadBuilder::build_health(view, out, cap);
}

}  // namespace robot::service
