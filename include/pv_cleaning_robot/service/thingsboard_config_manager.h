#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::service {

enum class TerminalSide {
    Unknown,
    A,
    B,
};

enum class ParkingPolicy {
    TerminalAOnly,
    TerminalBOnly,
    Both,
};

enum class ChargingSide {
    TerminalA,
    TerminalB,
    Both,
};

inline bool allows_half_pass(ParkingPolicy policy) noexcept {
    return policy == ParkingPolicy::Both;
}

inline bool can_start_from_terminal(ParkingPolicy policy, TerminalSide side) noexcept {
    switch (policy) {
    case ParkingPolicy::TerminalAOnly:
        return side == TerminalSide::A;
    case ParkingPolicy::TerminalBOnly:
        return side == TerminalSide::B;
    case ParkingPolicy::Both:
        return side == TerminalSide::A || side == TerminalSide::B;
    }
    return false;
}

inline bool supports_charging_at(ChargingSide charging_side,
                                 TerminalSide terminal_side) noexcept {
    switch (charging_side) {
    case ChargingSide::TerminalA:
        return terminal_side == TerminalSide::A;
    case ChargingSide::TerminalB:
        return terminal_side == TerminalSide::B;
    case ChargingSide::Both:
        return terminal_side == TerminalSide::A || terminal_side == TerminalSide::B;
    }
    return false;
}

struct TbScheduleEntry {
    int hour{0};
    int minute{0};

    bool operator==(const TbScheduleEntry& other) const {
        return hour == other.hour && minute == other.minute;
    }
};

struct TbRuntimeConfig {
    double passes{1.0};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    std::vector<TbScheduleEntry> schedules;

    bool operator==(const TbRuntimeConfig& other) const {
        return passes == other.passes &&
               clean_speed_rpm == other.clean_speed_rpm &&
               return_speed_rpm == other.return_speed_rpm &&
               brush_rpm == other.brush_rpm &&
               schedules == other.schedules;
    }
};

struct SharedAttrApplyResult {
    bool accepted{false};
    std::string reason;
};

class ThingsBoardConfigManager {
public:
    ThingsBoardConfigManager(ConfigService& config, SchedulerService& scheduler);

    SharedAttrApplyResult apply_shared_attributes(const nlohmann::json& attrs);
    bool promote_pending_to_active();

    TbRuntimeConfig active_config() const;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const;

private:
    static TbRuntimeConfig parse_runtime_config(const nlohmann::json& root);
    static void apply_schedule_json(nlohmann::json& root, const nlohmann::json& schedules_json);
    static std::vector<TbScheduleEntry> parse_schedule_entries(const nlohmann::json& schedules_json);
    void apply_scheduler_windows(const std::vector<TbScheduleEntry>& schedules);

    ConfigService& config_;
    SchedulerService& scheduler_;

    mutable std::mutex mtx_;
    TbRuntimeConfig active_;
    std::optional<TbRuntimeConfig> pending_;
};

}  // namespace robot::service
