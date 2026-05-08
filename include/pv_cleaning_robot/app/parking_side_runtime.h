#pragma once

#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::app {

struct ParkingSideFacts {
    bool at_parking_side{false};
    bool at_far_end{false};
};

struct ParkingSideRuntime {
    static ParkingSideFacts from_physical_limits(service::ParkingSide parking_side,
                                                 bool left_limit_active,
                                                 bool right_limit_active) noexcept {
        if (parking_side == service::ParkingSide::Left) {
            return ParkingSideFacts{left_limit_active, right_limit_active};
        }
        return ParkingSideFacts{right_limit_active, left_limit_active};
    }
};

}  // namespace robot::app
