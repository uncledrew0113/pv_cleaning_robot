#pragma once

#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::app {

struct ParkingSideFacts {
    bool left_limit_active{false};
    bool right_limit_active{false};
    bool at_parking_side{false};
    bool at_far_end{false};
    bool dual_endpoint_active{false};
    bool no_endpoint_active{false};

    bool is_valid_start_position() const noexcept {
        return at_parking_side && !dual_endpoint_active && !no_endpoint_active;
    }

    bool is_invalid_position() const noexcept {
        return dual_endpoint_active || no_endpoint_active;
    }
};

struct ParkingSideRuntime {
    static ParkingSideFacts from_physical_limits(service::ParkingSide parking_side,
                                                 bool left_limit_active,
                                                 bool right_limit_active) noexcept {
        ParkingSideFacts facts;
        facts.left_limit_active = left_limit_active;
        facts.right_limit_active = right_limit_active;
        facts.dual_endpoint_active = left_limit_active && right_limit_active;
        facts.no_endpoint_active = !left_limit_active && !right_limit_active;
        if (parking_side == service::ParkingSide::Left) {
            facts.at_parking_side = left_limit_active;
            facts.at_far_end = right_limit_active;
            return facts;
        }
        facts.at_parking_side = right_limit_active;
        facts.at_far_end = left_limit_active;
        return facts;
    }
};

}  // namespace robot::app
