#pragma once

#include <functional>
#include <memory>
#include <string>

#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"

namespace robot::app {
class RobotSupervisor;
}

namespace robot::service {

class ThingsBoardConfigManager;

class ThingsBoardControlPlane {
public:
    using PublishStatusEventFn =
        std::function<void(const char* event_name, bool accepted, const char* reason)>;
    using PublishCommandEventFn =
        std::function<void(const char* event_name, const CommandSnapshot& snapshot)>;

    ThingsBoardControlPlane(std::shared_ptr<CloudService> cloud,
                            std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
                            std::shared_ptr<CommandTracker> command_tracker,
                            std::shared_ptr<app::RobotSupervisor> supervisor,
                            PublishStatusEventFn publish_status_event,
                            PublishCommandEventFn publish_command_event);

    void subscribe_shared_attributes();
    void register_rpc_handlers(const std::function<bool()>& is_at_home,
                               const std::function<bool()>& is_at_front);

private:
    static std::string rpc_reply(bool accepted, const std::string& reason = {});

    std::shared_ptr<CloudService> cloud_;
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg_;
    std::shared_ptr<CommandTracker> command_tracker_;
    std::shared_ptr<app::RobotSupervisor> supervisor_;
    PublishStatusEventFn publish_status_event_;
    PublishCommandEventFn publish_command_event_;
};

}  // namespace robot::service
