#pragma once

#include <cstddef>

#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"
#include "pv_cleaning_robot/service/command_tracker.h"

namespace robot::service {

class ThingsBoardJsonCodec {
public:
    struct StartupAttributesView {
        const char* software_version{""};
        const char* hardware_version{""};
        const char* device_model{""};
        const char* device_id{""};
    };

    struct StatusEventView {
        const char* event_name{""};
        bool accepted{false};
        const char* reason{""};
    };

    struct CommandEventView {
        const char* event_name{""};
        const CommandSnapshot* command{nullptr};
    };

    static size_t build_startup_attributes(const StartupAttributesView& view,
                                           char* out,
                                           size_t cap) noexcept;
    static size_t build_status_event(const StatusEventView& view, char* out, size_t cap) noexcept;
    static size_t build_command_event(const CommandEventView& view, char* out, size_t cap) noexcept;
    static size_t build_business_telemetry(const app::RobotRuntimeSnapshot& view,
                                           char* out,
                                           size_t cap) noexcept;
};

}  // namespace robot::service
