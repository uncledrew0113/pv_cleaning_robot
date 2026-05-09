/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-23 00:06:17
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/health_service.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include <chrono>
#include <ctime>
#include <cstdarg>
#include <cstdio>
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

    size_t size() const noexcept { return overflow_ ? 0u : len_; }

    void append_raw(const char* text) noexcept { append_format("%s", text ? text : ""); }

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

    void append_bool(bool v) noexcept { append_raw(v ? "true" : "false"); }
    void append_int(int v) noexcept { append_format("%d", v); }
    void append_uint(unsigned v) noexcept { append_format("%u", v); }
    void append_float(float v) noexcept { append_format("%.6f", static_cast<double>(v)); }
    void append_double(double v) noexcept { append_format("%.6f", v); }

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

const char* wheel_name(int idx) noexcept
{
    static constexpr const char* kWheelNames[device::WalkMotorGroup::kWheelCount] = {
        "lt", "rt", "lb", "rb"};
    return kWheelNames[idx];
}

void append_health_dist(const device::DistSensorData* dist, FixedJsonWriter& w) noexcept
{
    if (!dist) {
        return;
    }

    bool any_valid = false;
    for (uint8_t i = 0; i < dist->channel_count; ++i) {
        any_valid |= dist->channels[i].valid;
    }

    w.append_raw(",\"dist\":{\"valid\":");
    w.append_bool(any_valid);
    w.append_raw(",\"ch\":[");
    for (uint8_t i = 0; i < dist->channel_count; ++i) {
        if (i != 0u) {
            w.append_char(',');
        }
        w.append_float(dist->channels[i].value_v);
    }
    w.append_raw("]}");
}

void append_diag_dist(const device::DistSensorData* dist, FixedJsonWriter& w) noexcept
{
    if (!dist) {
        return;
    }

    bool any_valid = false;
    for (uint8_t i = 0; i < dist->channel_count; ++i) {
        any_valid |= dist->channels[i].valid;
    }

    w.append_raw(",\"dist\":{\"valid\":");
    w.append_bool(any_valid);
    w.append_raw(",\"ch\":[");
    for (uint8_t i = 0; i < dist->channel_count; ++i) {
        if (i != 0u) {
            w.append_char(',');
        }
        w.append_raw("{\"v\":");
        w.append_float(dist->channels[i].value_v);
        w.append_raw(",\"ma\":");
        w.append_float(dist->channels[i].value_ma);
        w.append_raw(",\"ok\":");
        w.append_bool(dist->channels[i].valid);
        w.append_char('}');
    }
    w.append_raw("],\"comm_errors\":");
    w.append_uint(dist->error_count);
    w.append_char('}');
}

}  // namespace

size_t HealthPayloadBuilder::build_health(const HealthView& view, char* out, size_t cap) noexcept
{
    FixedJsonWriter w(out, cap);

    float avg_rpm = 0.0f;
    float avg_torque = 0.0f;
    bool any_fault = false;
    for (const auto& wheel : view.walk.wheel) {
        avg_rpm += wheel.speed_rpm;
        avg_torque += wheel.torque_a;
        any_fault |= (wheel.fault != protocol::WalkMotorFault::NONE);
    }
    avg_rpm /= static_cast<float>(device::WalkMotorGroup::kWheelCount);
    avg_torque /= static_cast<float>(device::WalkMotorGroup::kWheelCount);

    w.append_raw("{\"ts\":");
    w.append_quoted(view.ts_iso8601);
    w.append_raw(",\"walk\":{\"rpm\":");
    w.append_float(avg_rpm);
    w.append_raw(",\"torque_a\":");
    w.append_float(avg_torque);
    w.append_raw(",\"fault\":");
    w.append_bool(any_fault);
    w.append_raw(",\"temp\":0.000000},\"brush\":{\"running\":");
    w.append_bool(view.brush.running);
    w.append_raw(",\"fault\":");
    w.append_bool(view.brush.fault);
    w.append_raw("},\"battery\":{\"soc\":");
    w.append_float(view.bms.soc_pct);
    w.append_raw(",\"voltage\":");
    w.append_float(view.bms.voltage_v);
    w.append_raw(",\"charging\":");
    w.append_bool(view.bms.charging);
    w.append_raw(",\"alarm\":");
    w.append_bool(view.bms.alarm_flags != 0u);
    w.append_raw("},\"imu\":{\"pitch\":");
    w.append_float(view.imu.pitch_deg);
    w.append_raw(",\"roll\":");
    w.append_float(view.imu.roll_deg);
    w.append_raw(",\"valid\":");
    w.append_bool(view.imu.valid);
    w.append_raw("},\"gps\":{\"lat\":");
    w.append_double(view.gps.latitude);
    w.append_raw(",\"lon\":");
    w.append_double(view.gps.longitude);
    w.append_raw(",\"fix\":");
    w.append_int(view.gps.fix_quality);
    w.append_raw(",\"valid\":");
    w.append_bool(view.gps.valid);
    w.append_char('}');
    append_health_dist(view.dist, w);
    w.append_char('}');
    return w.size();
}

size_t HealthPayloadBuilder::build_diagnostics(const DiagnosticsView& view,
                                               char* out,
                                               size_t cap) noexcept
{
    FixedJsonWriter w(out, cap);

    w.append_raw("{\"ts\":");
    w.append_quoted(view.ts_iso8601);
    w.append_raw(",\"walk\":{");
    for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
        if (i != 0) {
            w.append_char(',');
        }
        const auto& wheel = view.walk.wheel[i];
        w.append_quoted(wheel_name(i));
        w.append_raw(":{\"rpm\":");
        w.append_float(wheel.speed_rpm);
        w.append_raw(",\"target\":");
        w.append_float(wheel.target_value);
        w.append_raw(",\"torque_a\":");
        w.append_float(wheel.torque_a);
        w.append_raw(",\"can_err\":");
        w.append_uint(wheel.can_err_count);
        w.append_raw(",\"fault\":");
        w.append_bool(wheel.fault != protocol::WalkMotorFault::NONE);
        w.append_raw(",\"fault_code\":");
        w.append_int(static_cast<int>(wheel.fault));
        w.append_raw(",\"online\":");
        w.append_bool(wheel.online);
        w.append_char('}');
    }
    w.append_raw(",\"temp\":0.000000,\"ctrl_frames\":");
    w.append_uint(view.walk.ctrl_frame_count);
    w.append_raw(",\"ctrl_err\":");
    w.append_uint(view.walk.ctrl_err_count);
    w.append_raw("},\"brush\":{\"rpm\":");
    w.append_int(view.brush.actual_rpm);
    w.append_raw(",\"target\":");
    w.append_int(view.brush.target_rpm);
    w.append_raw(",\"current\":");
    w.append_float(view.brush.current_a);
    w.append_raw(",\"voltage\":");
    w.append_float(view.brush.bus_voltage_v);
    w.append_raw(",\"temp\":");
    w.append_float(view.brush.temperature_c);
    w.append_raw(",\"stalls\":");
    w.append_uint(view.brush.stall_count);
    w.append_raw(",\"comm_err\":");
    w.append_uint(view.brush.comm_error_count);
    w.append_raw("},\"bms\":{\"soc\":");
    w.append_float(view.bms.soc_pct);
    w.append_raw(",\"voltage\":");
    w.append_float(view.bms.voltage_v);
    w.append_raw(",\"current\":");
    w.append_float(view.bms.current_a);
    w.append_raw(",\"temp\":");
    w.append_float(view.bms.temperature_c);
    w.append_raw(",\"cell_max\":");
    w.append_float(view.bms.cell_voltage_max_v);
    w.append_raw(",\"cell_min\":");
    w.append_float(view.bms.cell_voltage_min_v);
    w.append_raw(",\"remain_ah\":");
    w.append_float(view.bms.remaining_capacity_ah);
    w.append_raw(",\"cycles\":");
    w.append_uint(view.bms.cycle_count);
    w.append_raw(",\"alarm\":");
    w.append_uint(view.bms.alarm_flags);
    w.append_raw("},\"imu\":{\"accel\":[");
    for (int i = 0; i < 3; ++i) {
        if (i != 0) {
            w.append_char(',');
        }
        w.append_float(view.imu.accel[i]);
    }
    w.append_raw("],\"gyro\":[");
    for (int i = 0; i < 3; ++i) {
        if (i != 0) {
            w.append_char(',');
        }
        w.append_float(view.imu.gyro[i]);
    }
    w.append_raw("],\"pitch\":");
    w.append_float(view.imu.pitch_deg);
    w.append_raw(",\"roll\":");
    w.append_float(view.imu.roll_deg);
    w.append_raw(",\"yaw\":");
    w.append_float(view.imu.yaw_deg);
    w.append_raw(",\"frame_rate\":");
    w.append_float(view.imu.frame_rate_hz);
    w.append_raw(",\"parse_errors\":");
    w.append_uint(view.imu.parse_error_count);
    w.append_raw("},\"gps\":{\"lat\":");
    w.append_double(view.gps.latitude);
    w.append_raw(",\"lon\":");
    w.append_double(view.gps.longitude);
    w.append_raw(",\"alt\":");
    w.append_float(view.gps.altitude_m);
    w.append_raw(",\"speed\":");
    w.append_float(view.gps.speed_m_s);
    w.append_raw(",\"sats\":");
    w.append_uint(view.gps.satellites_used);
    w.append_raw(",\"hdop\":");
    w.append_float(view.gps.hdop);
    w.append_raw(",\"fix\":");
    w.append_int(view.gps.fix_quality);
    w.append_raw(",\"sentences\":");
    w.append_uint(view.gps.sentence_count);
    w.append_char('}');
    append_diag_dist(view.dist, w);
    w.append_char('}');
    return w.size();
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
                             size_t local_log_max_files,
                             std::shared_ptr<device::DistanceSensor> dist)
    : walk_(std::move(walk))
    , brush_(std::move(brush))
    , bms_(std::move(bms))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , dist_(std::move(dist))
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

void HealthService::update() {
    const size_t payload_len = build_payload(payload_buf_.data(), payload_buf_.size());
    if (payload_len == 0u) {
        spdlog::error("[HealthService] failed to build telemetry payload");
        return;
    }

    payload_cache_.assign(payload_buf_.data(), payload_len);
    if (cloud_)
        cloud_->publish_telemetry(payload_cache_);  // cloud_ 为 nullptr 时（单元测试场景）跳过
    // 本地 JSONL 落盘：每条记录一行，独立于网络，离线测试直接 cat 查看
    if (local_log_) {
        try {
            local_log_->log(
                spdlog::level::info,
                spdlog::string_view_t(payload_cache_.data(), payload_cache_.size()));
        } catch (const std::exception& ex) {
            spdlog::error("[HealthService] local rotating log write failed, disabling local log: {}",
                          ex.what());
            local_log_.reset();
        }
    }
}

size_t HealthService::build_payload(char* out, size_t cap) const {
    // ISO 8601 UTC 时间戳（build_payload 单线程调用，gmtime 无竞争）
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char ts_buf[24];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));
    device::DistSensorData dist_data{};
    auto attach_distance = [&](auto& view) {
        if (!dist_) {
            return;
        }
        dist_data = dist_->get_data();
        view.dist = &dist_data;
    };

    if (mode_ == Mode::DIAGNOSTICS) {
        HealthPayloadBuilder::DiagnosticsView view{};
        view.ts_iso8601 = ts_buf;
        view.walk = walk_->get_group_diagnostics();
        view.brush = brush_->get_diagnostics();
        view.bms = bms_->get_diagnostics();
        view.imu = imu_->get_diagnostics();
        view.gps = gps_->get_diagnostics();
        attach_distance(view);
        return HealthPayloadBuilder::build_diagnostics(view, out, cap);
    }

    HealthPayloadBuilder::HealthView view{};
    view.ts_iso8601 = ts_buf;
    view.walk = walk_->get_group_status();
    view.brush = brush_->get_status();
    view.bms = bms_->get_data();
    view.imu = imu_->get_latest();
    view.gps = gps_->get_latest();
    attach_distance(view);
    return HealthPayloadBuilder::build_health(view, out, cap);
}

}  // namespace robot::service
