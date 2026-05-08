#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <utility>

#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::service {

ThingsBoardControlPlane::ThingsBoardControlPlane(
    ConfigService& config,
    std::shared_ptr<CloudService> cloud,
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
    std::shared_ptr<CommandTracker> command_tracker,
    std::shared_ptr<app::RobotSupervisor> supervisor)
    : config_(config)
    , cloud_(std::move(cloud))
    , tb_cfg_(std::move(tb_cfg))
    , command_tracker_(std::move(command_tracker))
    , supervisor_(std::move(supervisor)) {
    business_payload_cache_.reserve(kBusinessPayloadBufferBytes);
    event_payload_cache_.reserve(kEventPayloadBufferBytes);
}

void ThingsBoardControlPlane::subscribe_shared_attributes() {
    cloud_->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
        // ControlPlane 只负责把协议入口接到配置管理器，再把结果转成 telemetry event。
        const auto result = tb_cfg_->apply_shared_attributes(attrs);
        const auto reason = result.reason.empty() ? "ok" : result.reason;
        publish_status_event("shared_attr_update", result.accepted, reason.c_str());
        if (!result.accepted) {
            spdlog::warn("[ThingsBoardControlPlane] 共享属性更新被拒绝: {}", result.reason);
        }
    });
}

void ThingsBoardControlPlane::request_shared_attributes_snapshot() const {
    static const std::vector<std::string> kReleaseSharedKeys{
        "passes",
        "clean_speed_rpm",
        "return_speed_rpm",
        "brush_rpm",
        "parking_side",
        "schedules",
    };
    if (!cloud_->request_shared_attributes_snapshot(kReleaseSharedKeys)) {
        spdlog::warn("[ThingsBoardControlPlane] 请求 shared attributes 快照失败");
    }
}

void ThingsBoardControlPlane::register_rpc_handlers(
    const std::function<bool()>& is_at_start_parking_side,
    const std::function<bool()>& is_at_start_far_end,
    const std::function<bool()>& is_at_active_parking_side) {
    // 当前 release 只暴露 5 个 RPC。每个 handler 内部都遵循同一套模式：
    // 1. 读取当前状态 / 现场条件
    // 2. 交给 RobotSupervisor 判定是否允许
    // 3. 发布 command event
    // 4. 回 RPC response
    cloud_->register_rpc("start",
                         [this, is_at_start_parking_side, is_at_start_far_end](const std::string& request_id,
                                                         const std::string& /*params*/) {
        const auto state = supervisor_->current_state();
        const bool at_parking_side = is_at_start_parking_side();
        const bool at_far_end = is_at_start_far_end();
        spdlog::info("[ThingsBoardControlPlane] RPC start received: state='{}' at_parking_side={} at_far_end={}",
                     state,
                     at_parking_side,
                     at_far_end);

        if (state == "Paused") {
            if (!supervisor_->resume_paused_task()) {
                spdlog::warn("[ThingsBoardControlPlane] RPC start rejected: resume_not_allowed_in_current_state");
                return reject_rpc_command(
                    "start", request_id, "resume_not_allowed_in_current_state");
            }
            spdlog::info("[ThingsBoardControlPlane] RPC start completed: resumed_paused_task");
            return complete_rpc_command("start", request_id, "resumed_paused_task");
        }

        if (!supervisor_->start_task(at_parking_side)) {
            const std::string reason = (state != "Idle" && state != "Charging")
                                           ? "start_not_allowed_in_current_state"
                                       : !at_parking_side ? "robot_not_at_parking_side"
                                                  : "promote_pending_config_failed";
            spdlog::warn("[ThingsBoardControlPlane] RPC start rejected: {}", reason);
            return reject_rpc_command("start", request_id, reason.c_str());
        }

        spdlog::info("[ThingsBoardControlPlane] RPC start completed: started_new_task");
        return complete_rpc_command("start", request_id, "started_new_task");
    });

    cloud_->register_rpc("stop", [this](const std::string& request_id, const std::string& /*params*/) {
        spdlog::info("[ThingsBoardControlPlane] RPC stop received: state='{}'",
                     supervisor_->current_state());
        if (!supervisor_->pause_task()) {
            spdlog::warn("[ThingsBoardControlPlane] RPC stop rejected: stop_not_allowed_in_current_state");
            return reject_rpc_command("stop", request_id, "stop_not_allowed_in_current_state");
        }

        spdlog::info("[ThingsBoardControlPlane] RPC stop completed: paused_task");
        return complete_rpc_command("stop", request_id, "paused_task");
    });

    cloud_->register_rpc("return",
                         [this](const std::string& request_id, const std::string& /*params*/) {
        spdlog::info("[ThingsBoardControlPlane] RPC return received: state='{}'",
                     supervisor_->current_state());
        if (!supervisor_->return_task()) {
            spdlog::warn("[ThingsBoardControlPlane] RPC return rejected: return_not_allowed_in_current_state");
            return reject_rpc_command("return", request_id, "return_not_allowed_in_current_state");
        }

        spdlog::info("[ThingsBoardControlPlane] RPC return completed: returning_to_home");
        return complete_rpc_command("return", request_id, "returning_to_home");
    });

    cloud_->register_rpc("terminate",
                         [this](const std::string& request_id, const std::string& /*params*/) {
        spdlog::info("[ThingsBoardControlPlane] RPC terminate received: state='{}'",
                     supervisor_->current_state());
        if (!supervisor_->terminate_task()) {
            spdlog::warn("[ThingsBoardControlPlane] RPC terminate rejected: terminate_not_allowed_in_current_state");
            return reject_rpc_command(
                "terminate", request_id, "terminate_not_allowed_in_current_state");
        }

        spdlog::info("[ThingsBoardControlPlane] RPC terminate completed: terminated_task");
        return complete_rpc_command("terminate", request_id, "terminated_task");
    });

    cloud_->register_rpc("reset",
                         [this, is_at_active_parking_side](const std::string& request_id,
                                            const std::string& /*params*/) {
        const bool at_parking_side = is_at_active_parking_side();
        spdlog::info("[ThingsBoardControlPlane] RPC reset received: state='{}' at_parking_side={}",
                     supervisor_->current_state(),
                     at_parking_side);
        if (!supervisor_->reset_task(at_parking_side)) {
            const std::string reason = !at_parking_side ? "robot_not_at_parking_side"
                                                : "reset_not_allowed_in_current_state";
            spdlog::warn("[ThingsBoardControlPlane] RPC reset rejected: {}", reason);
            return reject_rpc_command("reset", request_id, reason.c_str());
        }

        spdlog::info("[ThingsBoardControlPlane] RPC reset completed: reset_to_idle");
        return complete_rpc_command("reset", request_id, "reset_to_idle");
    });
}

void ThingsBoardControlPlane::publish_backup_fallback_event() const {
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_status_event(
        {"config_backup_fallback", true, "loaded_from_backup"},
        event_payload_buf_.data(),
        event_payload_buf_.size());
    if (!publish_event_payload(
            len,
            "[ThingsBoardControlPlane] failed to build config_backup_fallback event payload")) {
        return;
    }
    spdlog::warn("[ThingsBoardControlPlane] 主配置加载失败，已从 backup 配置回退启动");
}

void ThingsBoardControlPlane::publish_startup_attributes() const {
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_startup_attributes(
        {config_.get<std::string>(
             "device.software_version", config_.get<std::string>("device.fw_version", "1.0.0"))
             .c_str(),
         config_.get<std::string>(
             "device.hardware_version", config_.get<std::string>("device.hw_version", "1.0"))
             .c_str(),
         config_.get<std::string>("device.model", "pv_cleaning_robot").c_str(),
         config_.get<std::string>("network.mqtt.client_id", "pv_robot_001").c_str()},
        event_payload_buf_.data(),
        event_payload_buf_.size());
    publish_attributes_payload(len,
                               "[ThingsBoardControlPlane] failed to build startup attributes "
                               "payload");
}

void ThingsBoardControlPlane::publish_status_event(const char* event_name,
                                                   bool accepted,
                                                   const char* reason) const {
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_status_event(
        {event_name, accepted, reason}, event_payload_buf_.data(), event_payload_buf_.size());
    publish_event_payload(len, "[ThingsBoardControlPlane] failed to build status event payload");
}

void ThingsBoardControlPlane::publish_command_event(const char* event_name,
                                                    const CommandSnapshot& snapshot) const {
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_command_event(
        {event_name, &snapshot}, event_payload_buf_.data(), event_payload_buf_.size());
    publish_event_payload(len, "[ThingsBoardControlPlane] failed to build command event payload");
}

void ThingsBoardControlPlane::publish_business_telemetry() const {
    std::lock_guard<std::mutex> lk(publish_mtx_);
    // business telemetry 的业务真相完全来自 supervisor snapshot。
    // ControlPlane 不再派生第二份平行状态，避免云端真相和本地真相分叉。
    const auto runtime_snap = supervisor_->snapshot();
    const size_t len = ThingsBoardJsonCodec::build_business_telemetry(
        runtime_snap, business_payload_buf_.data(), business_payload_buf_.size());
    publish_business_payload(
        len, "[ThingsBoardControlPlane] failed to build periodic business telemetry payload");
}

std::string ThingsBoardControlPlane::rpc_reply(bool accepted, const std::string& reason) {
    rapidjson::StringBuffer buffer;
    buffer.Reserve(static_cast<rapidjson::SizeType>(96 + reason.size()));
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("accepted");
    writer.Bool(accepted);
    writer.Key("result");
    writer.String(accepted ? "ok" : "rejected");
    if (!reason.empty()) {
        writer.Key("reason");
        writer.String(reason.c_str(), static_cast<rapidjson::SizeType>(reason.size()));
    }
    writer.EndObject();
    return buffer.GetString();
}

bool ThingsBoardControlPlane::publish_attributes_payload(size_t len,
                                                         const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_attributes(event_payload_cache_);
    return true;
}

bool ThingsBoardControlPlane::publish_event_payload(size_t len,
                                                    const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_telemetry(event_payload_cache_);
    return true;
}

bool ThingsBoardControlPlane::publish_business_payload(size_t len,
                                                       const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    business_payload_cache_.assign(business_payload_buf_.data(), len);
    cloud_->publish_telemetry(business_payload_cache_);
    return true;
}

std::string ThingsBoardControlPlane::reject_rpc_command(const char* command_name,
                                                        const std::string& request_id,
                                                        const char* reason) {
    command_tracker_->reject(command_name, request_id, reason);
    publish_command_event("command_rejected", *command_tracker_->last_completed());
    return rpc_reply(false, reason);
}

std::string ThingsBoardControlPlane::complete_rpc_command(const char* command_name,
                                                          const std::string& request_id,
                                                          const char* completion_reason) {
    spdlog::info("[ThingsBoardControlPlane] complete_rpc_command begin: command='{}' reason='{}'",
                 command_name,
                 completion_reason);
    const auto cmd_id = command_tracker_->accept(command_name, request_id);
    spdlog::info("[ThingsBoardControlPlane] command accepted: command='{}' cmd_id={}",
                 command_name,
                 cmd_id);
    publish_command_event("command_accepted", *command_tracker_->active());
    spdlog::info("[ThingsBoardControlPlane] command_accepted event published: command='{}'",
                 command_name);
    command_tracker_->mark_running(cmd_id);
    spdlog::info("[ThingsBoardControlPlane] command marked running: command='{}' cmd_id={}",
                 command_name,
                 cmd_id);
    command_tracker_->finish_success(cmd_id, completion_reason);
    spdlog::info("[ThingsBoardControlPlane] command finished success: command='{}' cmd_id={} reason='{}'",
                 command_name,
                 cmd_id,
                 completion_reason);
    publish_command_event("command_completed", *command_tracker_->last_completed());
    spdlog::info("[ThingsBoardControlPlane] command_completed event published: command='{}'",
                 command_name);
    return rpc_reply(true);
}

}  // namespace robot::service
