#pragma once

/// @file parking_side_runtime.h
/// @brief 停机侧位置判定与运行时停车侧状态转换。

#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::app {

struct ParkingSideFacts {
    /// 左限位开关是否触发
    bool left_limit_active{false};
    /// 右限位开关是否触发
    bool right_limit_active{false};
    /// 是否处于停机侧位置
    bool at_parking_side{false};
    /// 是否处于远端侧位置
    bool at_far_end{false};
    /// 两端限位同时触发表示异常双端点
    bool dual_endpoint_active{false};
    /// 无限位触发表示位置不可判定
    bool no_endpoint_active{false};

    /// @brief 返回是否为合法的任务启动位置
    bool is_valid_start_position() const noexcept {
        return at_parking_side && !dual_endpoint_active && !no_endpoint_active;
    }

    /// @brief 返回是否存在异常位置状态
    bool is_invalid_position() const noexcept {
        return dual_endpoint_active || no_endpoint_active;
    }
};

struct ParkingSideRuntime {
    /// @brief 从实际限位开关状态生成停车侧运行事实。
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
