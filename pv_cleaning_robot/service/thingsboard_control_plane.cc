#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

#include <chrono>
#include <thread>
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
    // snapshot keys 必须与 ConfigManager 能识别的 runtime 字段保持一致。
    // 其中 scheduler.windows 是唯一立即生效项，其余字段即使拉到，也只会先落 pending。
    static const std::vector<std::string> kReleaseSharedKeys{
        "passes",
        "clean_speed_rpm",
        "return_speed_rpm",
        "brush_rpm",
        "parking_side",
        "start_battery_soc",
        "charge_start_soc",
        "charge_stop_soc",
        "schedules",
    };
    if (!cloud_->request_shared_attributes_snapshot(kReleaseSharedKeys)) {
        spdlog::warn("[ThingsBoardControlPlane] 请求 shared attributes 快照失败");
    }
}

void ThingsBoardControlPlane::register_rpc_handlers(
    const std::function<bool()>& is_start_position_valid,
    const std::function<bool()>& is_at_start_parking_side,
    const std::function<bool()>& is_at_active_parking_side,
    const std::function<float()>& current_battery_soc,
    std::function<void()> reboot_device) {
    // 当前 release 只暴露 4 个 RPC。每个 handler 内部都遵循同一套模式：
    // 1. 读取当前状态 / 现场条件
    // 2. 交给 RobotSupervisor 判定是否允许
    // 3. 发布 command event
    // 4. 回 RPC response
    cloud_->register_rpc("start",
                         [this,
                          is_start_position_valid,
                          is_at_start_parking_side,
                          current_battery_soc](const std::string& request_id,
                                               const std::string& /*params*/) {
        const auto state = supervisor_->current_state();
        const bool position_valid = is_start_position_valid();
        const bool at_parking_side = is_at_start_parking_side();
        const float battery_soc = current_battery_soc();
        spdlog::info("[ThingsBoardControlPlane] RPC start received: state='{}' position_valid={} at_parking_side={} battery_soc={:.1f}",
                     state,
                     position_valid,
                     at_parking_side,
                     battery_soc);

        if (!supervisor_->start_task(at_parking_side, position_valid, battery_soc)) {
            // start 被允许时，Supervisor 内部会先尝试 promote pending -> active。
            // 因此这里在构造 reject reason 时，优先读取“若此刻成功启动会采用的配置视图”。
            const auto runtime_cfg = tb_cfg_->has_pending_config() ? *tb_cfg_->pending_config()
                                                                   : tb_cfg_->active_config();
            const std::string reason =
                (state != "Idle" && state != "Charging" && state != "Stopped")
                    ? "start_not_allowed_in_current_state"
                : !position_valid ? "robot_position_invalid"
                : !at_parking_side ? "robot_not_at_parking_side"
                : battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)
                    ? "battery_below_start_threshold"
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
        if (!supervisor_->stop_task()) {
            spdlog::warn("[ThingsBoardControlPlane] RPC stop rejected: stop_not_allowed_in_current_state");
            return reject_rpc_command("stop", request_id, "stop_not_allowed_in_current_state");
        }

        spdlog::info("[ThingsBoardControlPlane] RPC stop completed: stopped_task");
        return complete_rpc_command("stop", request_id, "stopped_task");
    });

    cloud_->register_rpc("return",
                         [this, is_at_active_parking_side](const std::string& request_id,
                                                           const std::string& /*params*/) {
        const bool at_parking_side = is_at_active_parking_side();
        spdlog::info("[ThingsBoardControlPlane] RPC return received: state='{}' at_parking_side={}",
                     supervisor_->current_state(),
                     at_parking_side);
        if (!supervisor_->return_task(at_parking_side)) {
            spdlog::warn("[ThingsBoardControlPlane] RPC return rejected: return_not_allowed_in_current_state");
            return reject_rpc_command("return", request_id, "return_not_allowed_in_current_state");
        }

        spdlog::info("[ThingsBoardControlPlane] RPC return completed: returning_to_parking_side");
        return complete_rpc_command("return", request_id, "returning_to_parking_side");
    });

    cloud_->register_rpc("reset",
                         [this, reboot_device = std::move(reboot_device)](
                             const std::string& request_id,
                             const std::string& /*params*/) {
        spdlog::info("[ThingsBoardControlPlane] RPC reset received: state='{}'",
                     supervisor_->current_state());
        auto reply = complete_rpc_command("reset", request_id, "rebooting_device");
        std::thread([reboot_device]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (reboot_device) {
                reboot_device();
            }
        }).detach();
        return reply;
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
