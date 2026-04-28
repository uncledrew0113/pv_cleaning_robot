#include "pv_cleaning_robot/service/business_telemetry_snapshot.h"

namespace robot::service {
namespace {

const char* command_phase_name(CommandPhase phase) {
    switch (phase) {
    case CommandPhase::Accepted:
        return "accepted";
    case CommandPhase::Running:
        return "running";
    case CommandPhase::Succeeded:
        return "succeeded";
    case CommandPhase::Failed:
        return "failed";
    case CommandPhase::Rejected:
        return "rejected";
    }
    return "unknown";
}

nlohmann::json schedule_entry_to_json(const TbScheduleEntry& entry) {
    return nlohmann::json{{"hour", entry.hour}, {"minute", entry.minute}};
}

nlohmann::json runtime_config_to_json(const TbRuntimeConfig& config) {
    nlohmann::json schedules = nlohmann::json::array();
    for (const auto& schedule : config.schedules) {
        schedules.push_back(schedule_entry_to_json(schedule));
    }

    return nlohmann::json{
        {"passes", config.passes},
        {"clean_speed_rpm", config.clean_speed_rpm},
        {"return_speed_rpm", config.return_speed_rpm},
        {"brush_rpm", config.brush_rpm},
        {"schedules", std::move(schedules)},
    };
}

nlohmann::json command_snapshot_to_json(const CommandSnapshot& command) {
    return nlohmann::json{
        {"id", command.id},
        {"name", command.name},
        {"request_id", command.request_id},
        {"phase", command_phase_name(command.phase)},
        {"reason", command.reason},
        {"accepted_at_ms", command.accepted_at_ms},
        {"finished_at_ms", command.finished_at_ms},
    };
}

}  // namespace

nlohmann::json BusinessTelemetrySnapshot::to_json() const {
    nlohmann::json j{
        {"device_state", device_state},
        {"task_state", task_state},
        {"target_half_passes", target_half_passes},
        {"completed_half_passes", completed_half_passes},
        {"clean_count", clean_count},
        {"active_config_version", active_config_version},
    };

    if (active_config) j["active_config"] = runtime_config_to_json(*active_config);
    if (pending_config) j["pending_config"] = runtime_config_to_json(*pending_config);
    if (active_command) j["active_command"] = command_snapshot_to_json(*active_command);
    if (last_command) j["last_command"] = command_snapshot_to_json(*last_command);

    return j;
}

}  // namespace robot::service
