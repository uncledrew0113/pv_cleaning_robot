#include "pv_cleaning_robot/service/thingsboard_telemetry_publisher.h"

#include <utility>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/service/business_payload_builder.h"
#include "pv_cleaning_robot/service/business_telemetry_snapshot.h"
#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"

namespace robot::service {

ThingsBoardTelemetryPublisher::ThingsBoardTelemetryPublisher(
    ConfigService& config,
    std::shared_ptr<CloudService> cloud,
    std::shared_ptr<app::RobotSupervisor> supervisor)
    : config_(config)
    , cloud_(std::move(cloud))
    , supervisor_(std::move(supervisor)) {
    business_payload_cache_.reserve(kBusinessPayloadBufferBytes);
    event_payload_cache_.reserve(kEventPayloadBufferBytes);
}

void ThingsBoardTelemetryPublisher::publish_startup_attributes() const {
    cloud_->publish_attributes(nlohmann::json{
        {"software_version", config_.get<std::string>(
                                 "device.software_version",
                                 config_.get<std::string>("device.fw_version", "1.0.0"))},
        {"hardware_version", config_.get<std::string>(
                                 "device.hardware_version",
                                 config_.get<std::string>("device.hw_version", "1.0"))},
        {"device_model", config_.get<std::string>("device.model", "pv_cleaning_robot")},
        {"device_id", config_.get<std::string>("network.mqtt.client_id", "pv_robot_001")},
        {"supported_rpc_methods",
         nlohmann::json::array({"start", "stop", "return", "terminate", "reset"})},
        {"config_schema_version", "thingsboard-v1"}}
                                 .dump());
}

void ThingsBoardTelemetryPublisher::publish_backup_fallback_event() const {
    const size_t len = ThingsBoardEventPayloadBuilder::build_status_event(
        {"config_backup_fallback", true, "loaded_from_backup"},
        event_payload_buf_.data(),
        event_payload_buf_.size());
    if (len == 0u) {
        spdlog::error(
            "[ThingsBoardTelemetryPublisher] failed to build config_backup_fallback event payload");
        return;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_telemetry(event_payload_cache_);
    spdlog::warn("[ThingsBoardTelemetryPublisher] 主配置加载失败，已从 backup 配置回退启动");
}

void ThingsBoardTelemetryPublisher::publish_status_event(const char* event_name,
                                                         bool accepted,
                                                         const char* reason) const {
    const size_t len = ThingsBoardEventPayloadBuilder::build_status_event(
        {event_name, accepted, reason}, event_payload_buf_.data(), event_payload_buf_.size());
    if (len == 0u) {
        spdlog::error("[ThingsBoardTelemetryPublisher] failed to build status event payload");
        return;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_telemetry(event_payload_cache_);
}

void ThingsBoardTelemetryPublisher::publish_command_event(
    const char* event_name, const CommandSnapshot& snapshot) const {
    const size_t len = ThingsBoardEventPayloadBuilder::build_command_event(
        {event_name, &snapshot}, event_payload_buf_.data(), event_payload_buf_.size());
    if (len == 0u) {
        spdlog::error("[ThingsBoardTelemetryPublisher] failed to build command event payload");
        return;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_telemetry(event_payload_cache_);
}

void ThingsBoardTelemetryPublisher::publish_business_telemetry() const {
    const auto runtime_snap = supervisor_->snapshot();
    BusinessTelemetrySnapshot snap;
    snap.device_state = runtime_snap.device_state;
    snap.task_state = runtime_snap.task_state;
    snap.target_half_passes = runtime_snap.target_half_passes;
    snap.completed_half_passes = runtime_snap.completed_half_passes;
    snap.clean_count = runtime_snap.clean_count;
    snap.active_config_version = runtime_snap.active_config_version;
    snap.active_config = runtime_snap.active_config;
    snap.pending_config = runtime_snap.pending_config;
    snap.active_command = runtime_snap.active_command;
    snap.last_command = runtime_snap.last_command;
    const size_t len =
        BusinessPayloadBuilder::build(snap, business_payload_buf_.data(), business_payload_buf_.size());
    if (len == 0u) {
        spdlog::error(
            "[ThingsBoardTelemetryPublisher] failed to build periodic business telemetry payload");
        return;
    }
    business_payload_cache_.assign(business_payload_buf_.data(), len);
    cloud_->publish_telemetry(business_payload_cache_);
}

}  // namespace robot::service
