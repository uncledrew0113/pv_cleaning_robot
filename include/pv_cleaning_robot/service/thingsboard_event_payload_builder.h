#pragma once

#include <cstddef>

#include "pv_cleaning_robot/service/command_tracker.h"

namespace robot::service {

class ThingsBoardEventPayloadBuilder {
public:
    struct StatusEventView {
        const char* event_name{""};
        bool accepted{false};
        const char* reason{""};
    };

    struct CommandEventView {
        const char* event_name{""};
        const CommandSnapshot* command{nullptr};
    };

    static size_t build_status_event(const StatusEventView& view, char* out, size_t cap) noexcept;
    static size_t build_command_event(const CommandEventView& view, char* out, size_t cap) noexcept;
};

}  // namespace robot::service
