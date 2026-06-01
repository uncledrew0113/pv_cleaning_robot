#pragma once

#include <vector>

#include "pv_cleaning_robot/app/fault_policy.h"

namespace robot::app {

class FaultDetector {
public:
    struct Input {
        bool executing_mission{false};
        bool left_limit_active{false};
        bool right_limit_active{false};
        bool spin_free_detected{false};
        bool imu_fresh{true};
        bool attitude_out_of_range{false};
        bool bms_critical_alarm{false};
        bool brush_critical_fault{false};
        bool motor_driver_fault{false};
    };

    std::vector<FaultFact> detect(const Input& input) const;
};

}  // namespace robot::app
