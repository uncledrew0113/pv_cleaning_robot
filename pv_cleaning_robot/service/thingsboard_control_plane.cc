#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

#include <utility>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::service {

ThingsBoardControlPlane::ThingsBoardControlPlane(
    std::shared_ptr<CloudService> cloud,
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
    std::shared_ptr<CommandTracker> command_tracker,
    std::shared_ptr<app::RobotSupervisor> supervisor,
    PublishStatusEventFn publish_status_event,
    PublishCommandEventFn publish_command_event)
    : cloud_(std::move(cloud))
    , tb_cfg_(std::move(tb_cfg))
    , command_tracker_(std::move(command_tracker))
    , supervisor_(std::move(supervisor))
    , publish_status_event_(std::move(publish_status_event))
    , publish_command_event_(std::move(publish_command_event)) {}

void ThingsBoardControlPlane::subscribe_shared_attributes() {
    cloud_->subscribe_shared_attributes([this](const nlohmann::json& attrs) {
        const auto result = tb_cfg_->apply_shared_attributes(attrs);
        const auto reason = result.reason.empty() ? "ok" : result.reason;
        publish_status_event_("shared_attr_update", result.accepted, reason.c_str());
        if (!result.accepted) {
            spdlog::warn("[ThingsBoardControlPlane] 共享属性更新被拒绝: {}", result.reason);
        }
    });
}

void ThingsBoardControlPlane::register_rpc_handlers(const std::function<bool()>& is_at_home,
                                                    const std::function<bool()>& is_at_front) {
    cloud_->register_rpc("start", [this, is_at_home, is_at_front](const std::string& /*params*/) {
        const auto state = supervisor_->current_state();
        const bool at_home = is_at_home();
        const bool at_front = is_at_front();

        if (state == "Paused") {
            if (!supervisor_->resume_paused_task()) {
                command_tracker_->reject("start", "", "resume_not_allowed_in_current_state");
                publish_command_event_("command_rejected", *command_tracker_->last_completed());
                return rpc_reply(false, "resume_not_allowed_in_current_state");
            }
            const auto cmd_id = command_tracker_->accept("start", "");
            publish_command_event_("command_accepted", *command_tracker_->active());
            command_tracker_->mark_running(cmd_id);
            command_tracker_->finish_success(cmd_id, "resumed_paused_task");
            publish_command_event_("command_completed", *command_tracker_->last_completed());
            return rpc_reply(true);
        }

        if (!supervisor_->start_manual_task(at_home, at_front)) {
            const std::string reason = (state != "Idle" && state != "Charging")
                                           ? "start_not_allowed_in_current_state"
                                       : !at_home ? "robot_not_at_home"
                                                  : "promote_pending_config_failed";
            command_tracker_->reject("start", "", reason);
            publish_command_event_("command_rejected", *command_tracker_->last_completed());
            return rpc_reply(false, reason);
        }

        const auto cmd_id = command_tracker_->accept("start", "");
        publish_command_event_("command_accepted", *command_tracker_->active());
        command_tracker_->mark_running(cmd_id);
        command_tracker_->finish_success(cmd_id, "started_new_task");
        publish_command_event_("command_completed", *command_tracker_->last_completed());
        return rpc_reply(true);
    });

    cloud_->register_rpc("stop", [this](const std::string& /*params*/) {
        if (!supervisor_->pause_task()) {
            command_tracker_->reject("stop", "", "stop_not_allowed_in_current_state");
            publish_command_event_("command_rejected", *command_tracker_->last_completed());
            return rpc_reply(false, "stop_not_allowed_in_current_state");
        }

        const auto cmd_id = command_tracker_->accept("stop", "");
        publish_command_event_("command_accepted", *command_tracker_->active());
        command_tracker_->mark_running(cmd_id);
        command_tracker_->finish_success(cmd_id, "paused_task");
        publish_command_event_("command_completed", *command_tracker_->last_completed());
        return rpc_reply(true);
    });

    cloud_->register_rpc("return", [this](const std::string& /*params*/) {
        if (!supervisor_->return_task()) {
            command_tracker_->reject("return", "", "return_not_allowed_in_current_state");
            publish_command_event_("command_rejected", *command_tracker_->last_completed());
            return rpc_reply(false, "return_not_allowed_in_current_state");
        }

        const auto cmd_id = command_tracker_->accept("return", "");
        publish_command_event_("command_accepted", *command_tracker_->active());
        command_tracker_->mark_running(cmd_id);
        command_tracker_->finish_success(cmd_id, "returning_to_home");
        publish_command_event_("command_completed", *command_tracker_->last_completed());
        return rpc_reply(true);
    });

    cloud_->register_rpc("terminate", [this](const std::string& /*params*/) {
        if (!supervisor_->terminate_task()) {
            command_tracker_->reject("terminate", "", "terminate_not_allowed_in_current_state");
            publish_command_event_("command_rejected", *command_tracker_->last_completed());
            return rpc_reply(false, "terminate_not_allowed_in_current_state");
        }

        const auto cmd_id = command_tracker_->accept("terminate", "");
        publish_command_event_("command_accepted", *command_tracker_->active());
        command_tracker_->mark_running(cmd_id);
        command_tracker_->finish_success(cmd_id, "terminated_task");
        publish_command_event_("command_completed", *command_tracker_->last_completed());
        return rpc_reply(true);
    });

    cloud_->register_rpc("reset", [this, is_at_home](const std::string& /*params*/) {
        const bool at_home = is_at_home();
        if (!supervisor_->reset_task(at_home)) {
            const std::string reason = !at_home ? "robot_not_at_home"
                                                : "reset_not_allowed_in_current_state";
            command_tracker_->reject("reset", "", reason);
            publish_command_event_("command_rejected", *command_tracker_->last_completed());
            return rpc_reply(false, reason);
        }

        const auto cmd_id = command_tracker_->accept("reset", "");
        publish_command_event_("command_accepted", *command_tracker_->active());
        command_tracker_->mark_running(cmd_id);
        command_tracker_->finish_success(cmd_id, "reset_to_idle");
        publish_command_event_("command_completed", *command_tracker_->last_completed());
        return rpc_reply(true);
    });
}

std::string ThingsBoardControlPlane::rpc_reply(bool accepted, const std::string& reason) {
    nlohmann::json j{{"accepted", accepted}};
    j["result"] = accepted ? "ok" : "rejected";
    if (!reason.empty()) {
        j["reason"] = reason;
    }
    return j.dump();
}

}  // namespace robot::service
