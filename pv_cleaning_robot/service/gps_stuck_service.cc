/**
 * @file gps_stuck_service.cc
 * @brief GPS 卡滞检测服务实现。
 *
 * 本文件根据 GPS 位置变化、启停监控状态和时间窗口生成卡滞诊断事实。服务不直接停机，
 * 由 DiagnosticsCollector 与 ErrorManager 完成后续升级。
 */
#include "pv_cleaning_robot/service/gps_stuck_service.h"

#include <cmath>
#include <utility>

namespace robot::service {

GpsStuckService::GpsStuckService(std::shared_ptr<device::GpsDevice> gps)
    : GpsStuckService(std::move(gps), Config{}) {}

GpsStuckService::GpsStuckService(std::shared_ptr<device::GpsDevice> gps, Config config)
    : GpsStuckService(
          [gps = std::move(gps)]() -> GpsData { return gps ? gps->get_latest() : GpsData{}; },
          std::move(config)) {}

GpsStuckService::GpsStuckService(GpsReader gps_reader)
    : GpsStuckService(std::move(gps_reader), Config{}) {}

GpsStuckService::GpsStuckService(GpsReader gps_reader, Config config)
    : GpsStuckService(std::move(gps_reader),
                      []() { return std::chrono::steady_clock::now(); },
                      std::move(config)) {}

GpsStuckService::GpsStuckService(GpsReader gps_reader, ClockFn clock)
    : GpsStuckService(std::move(gps_reader), std::move(clock), Config{}) {}

GpsStuckService::GpsStuckService(GpsReader gps_reader, ClockFn clock, Config config)
    : gps_reader_(std::move(gps_reader)), clock_(std::move(clock)), config_(std::move(config)) {}

void GpsStuckService::update() {
    const auto now = clock_ ? clock_() : std::chrono::steady_clock::now();
    const GpsData gps = gps_reader_ ? gps_reader_() : GpsData{};

    std::lock_guard<std::mutex> lk(mtx_);

    if (!enabled_) {
        status_.state = State::kDisabled;
        status_.robot_stuck_detected = false;
        status_.reason = "disabled";
        return;
    }

    status_.latest_speed_m_s = gps.speed_m_s;
    status_.latest_hdop = gps.hdop;
    status_.latest_pdop = gps.pdop;
    status_.latest_satellites_used = gps.satellites_used;
    status_.latest_fix_quality = gps.fix_quality;

    if (!has_basic_fix(gps)) {
        reset_tracking_locked();
        status_.state = State::kWaitingForGps;
        status_.robot_stuck_detected = false;
        status_.reason = "invalid_gps";
        return;
    }

    if (has_last_sample_ && gps.utc_timestamp_ms == last_sample_utc_ms_) {
        if (now - last_sample_at_ > config_.sample_stale_timeout) {
            reset_tracking_locked();
            status_.state = State::kWaitingForGps;
            status_.robot_stuck_detected = false;
            status_.reason = "stale_gps";
        }
        return;
    }

    if (!std::isfinite(gps.speed_m_s) || gps.speed_m_s < 0.0f) {
        reset_tracking_locked();
        status_.state = State::kWaitingForGps;
        status_.robot_stuck_detected = false;
        status_.reason = "invalid_gps_speed";
        return;
    }

    has_last_sample_ = true;
    last_sample_utc_ms_ = gps.utc_timestamp_ms;
    last_sample_at_ = now;
    status_.last_utc_timestamp_ms = gps.utc_timestamp_ms;

    if (!has_acceptable_quality(gps)) {
        reset_tracking_locked();
        status_.state = State::kGpsQualityPoor;
        status_.robot_stuck_detected = false;
        status_.reason = "gps_quality_poor";
        return;
    }

    if (!tracking_started_) {
        tracking_started_ = true;
        last_motion_at_ = now;
        consecutive_speed_samples_ = gps.speed_m_s >= config_.moving_speed_mps ? 1 : 0;
        status_.state = State::kMonitoring;
        status_.robot_stuck_detected = false;
        status_.reason = "monitoring";
        return;
    }

    if (gps.speed_m_s >= config_.moving_speed_mps) {
        ++consecutive_speed_samples_;
    } else {
        consecutive_speed_samples_ = 0;
    }

    const bool speed_movement =
        consecutive_speed_samples_ >= config_.moving_speed_confirm_samples;
    if (speed_movement) {
        last_motion_at_ = now;
        status_.state = State::kMonitoring;
        status_.robot_stuck_detected = false;
        status_.reason = "gps_speed_movement";
        return;
    }

    if (now - last_motion_at_ >= config_.stuck_timeout) {
        status_.state = State::kStuck;
        status_.robot_stuck_detected = true;
        status_.reason = "stationary_timeout";
        return;
    }

    status_.state = State::kMonitoring;
    status_.robot_stuck_detected = false;
    status_.reason = "monitoring";
}

void GpsStuckService::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    status_ = Status{};
    has_last_sample_ = false;
    last_sample_utc_ms_ = 0;
    reset_tracking_locked();
}

void GpsStuckService::clear_stuck_detection() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (status_.state == State::kStuck) {
        last_motion_at_ = clock_ ? clock_() : std::chrono::steady_clock::now();
        status_.state = State::kMonitoring;
        status_.reason = "cleared";
    }
    status_.robot_stuck_detected = false;
}

void GpsStuckService::set_monitoring_enabled(bool enabled) {
    std::lock_guard<std::mutex> lk(mtx_);
    enabled_ = enabled;
    if (!enabled_) {
        status_.state = State::kDisabled;
        status_.robot_stuck_detected = false;
        status_.reason = "disabled";
        reset_tracking_locked();
    }
}

GpsStuckService::Status GpsStuckService::get_status() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return status_;
}

bool GpsStuckService::has_basic_fix(const GpsData& gps) const {
    return gps.valid && gps.utc_timestamp_ms != 0 && std::isfinite(gps.latitude) &&
           std::isfinite(gps.longitude) && gps.latitude != 0.0 && gps.longitude != 0.0 &&
           gps.fix_quality >= config_.min_fix_quality;
}

bool GpsStuckService::has_acceptable_quality(const GpsData& gps) const {
    if (gps.satellites_used < config_.min_satellites_used) {
        return false;
    }
    if (!(gps.hdop > 0.0f) || gps.hdop > config_.max_hdop) {
        return false;
    }
    if (gps.pdop > 0.0f && gps.pdop > config_.max_pdop) {
        return false;
    }
    return true;
}

void GpsStuckService::reset_tracking_locked() {
    tracking_started_ = false;
    consecutive_speed_samples_ = 0;
    last_motion_at_ = {};
}

}  // namespace robot::service
