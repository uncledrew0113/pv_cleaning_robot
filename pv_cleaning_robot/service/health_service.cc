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
#include <filesystem>
#include <spdlog/logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/service/health_service.h"

namespace robot::service {

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

    if (mode_ == Mode::DIAGNOSTICS) {
        HealthPayloadBuilder::DiagnosticsView view{};
        view.ts_iso8601 = ts_buf;
        view.walk = walk_->get_group_diagnostics();
        view.brush = brush_->get_diagnostics();
        view.bms = bms_->get_diagnostics();
        view.imu = imu_->get_diagnostics();
        view.gps = gps_->get_diagnostics();
        device::DistSensorData dist_data{};
        if (dist_) {
            dist_data = dist_->get_data();
            view.dist = &dist_data;
        }
        return HealthPayloadBuilder::build_diagnostics(view, out, cap);
    }

    HealthPayloadBuilder::HealthView view{};
    view.ts_iso8601 = ts_buf;
    view.walk = walk_->get_group_status();
    view.brush = brush_->get_status();
    view.bms = bms_->get_data();
    view.imu = imu_->get_latest();
    view.gps = gps_->get_latest();
    device::DistSensorData dist_data{};
    if (dist_) {
        dist_data = dist_->get_data();
        view.dist = &dist_data;
    }
    return HealthPayloadBuilder::build_health(view, out, cap);
}

}  // namespace robot::service
