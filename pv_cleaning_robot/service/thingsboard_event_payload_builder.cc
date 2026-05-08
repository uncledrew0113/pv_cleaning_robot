#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"

#include <rapidjson/writer.h>

#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

namespace robot::service {
namespace {

class RapidJsonFixedBufferStream {
public:
    using Ch = char;

    RapidJsonFixedBufferStream(char* out, size_t cap) noexcept : out_(out), cap_(cap) {
        if (out_ && cap_ > 0) out_[0] = '\0';
    }

    void Put(char c) noexcept {
        if (!out_ || cap_ == 0 || overflow_) return;
        if (len_ + 1u >= cap_) {
            overflow_ = true;
            out_[cap_ - 1] = '\0';
            return;
        }
        out_[len_++] = c;
        out_[len_] = '\0';
    }
    void Flush() noexcept {}
    char Peek() const noexcept { return '\0'; }
    char Take() noexcept { return '\0'; }
    size_t Tell() const noexcept { return len_; }
    char* PutBegin() noexcept { return nullptr; }
    size_t PutEnd(char*) noexcept { return 0; }

    size_t size() const noexcept { return overflow_ ? 0u : len_; }
    bool overflow() const noexcept { return overflow_; }

    char* out_{nullptr};
    size_t cap_{0};
    size_t len_{0};
    bool overflow_{false};
};

const char* command_phase_name(CommandPhase phase) noexcept {
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

template <typename WriterT>
void write_command_fields(WriterT& writer, const CommandSnapshot& command) {
    writer.Key("command_id");
    writer.String(command.id.c_str());
    writer.Key("command_name");
    writer.String(command.name.c_str());
    writer.Key("request_id");
    writer.String(command.request_id.c_str());
    writer.Key("phase");
    writer.String(command_phase_name(command.phase));
    writer.Key("reason");
    writer.String(command.reason.c_str());
    writer.Key("accepted_at_ms");
    writer.Uint64(command.accepted_at_ms);
    writer.Key("finished_at_ms");
    writer.Uint64(command.finished_at_ms);
}

template <typename WriterT>
void write_schedule_entries(const std::vector<TbScheduleEntry>& schedules, WriterT& writer) {
    writer.StartArray();
    for (const auto& schedule : schedules) {
        writer.StartObject();
        writer.Key("hour");
        writer.Int(schedule.hour);
        writer.Key("minute");
        writer.Int(schedule.minute);
        writer.EndObject();
    }
    writer.EndArray();
}

template <typename WriterT>
void write_runtime_config(const char* key, const TbRuntimeConfig& config, WriterT& writer) {
    writer.Key(key);
    writer.StartObject();
    writer.Key("passes");
    writer.Double(config.passes);
    writer.Key("clean_speed_rpm");
    writer.Double(config.clean_speed_rpm);
    writer.Key("return_speed_rpm");
    writer.Double(config.return_speed_rpm);
    writer.Key("brush_rpm");
    writer.Int(config.brush_rpm);
    writer.Key("parking_side");
    writer.String(parking_side_config_string(config.parking_side));
    writer.Key("start_battery_soc");
    writer.Double(config.start_battery_soc);
    writer.Key("charge_start_soc");
    writer.Double(config.charge_start_soc);
    writer.Key("charge_stop_soc");
    writer.Double(config.charge_stop_soc);
    writer.Key("schedules");
    write_schedule_entries(config.schedules, writer);
    writer.EndObject();
}

template <typename WriterT>
void write_command_snapshot(const char* key, const CommandSnapshot& command, WriterT& writer) {
    writer.Key(key);
    writer.StartObject();
    writer.Key("id");
    writer.String(command.id.c_str());
    writer.Key("name");
    writer.String(command.name.c_str());
    writer.Key("request_id");
    writer.String(command.request_id.c_str());
    writer.Key("phase");
    writer.String(command_phase_name(command.phase));
    writer.Key("reason");
    writer.String(command.reason.c_str());
    writer.Key("accepted_at_ms");
    writer.Uint64(command.accepted_at_ms);
    writer.Key("finished_at_ms");
    writer.Uint64(command.finished_at_ms);
    writer.EndObject();
}

}  // namespace

size_t ThingsBoardJsonCodec::build_startup_attributes(const StartupAttributesView& view,
                                                      char* out,
                                                      size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("software_version");
    writer.String(view.software_version ? view.software_version : "");
    writer.Key("hardware_version");
    writer.String(view.hardware_version ? view.hardware_version : "");
    writer.Key("device_model");
    writer.String(view.device_model ? view.device_model : "");
    writer.Key("device_id");
    writer.String(view.device_id ? view.device_id : "");
    writer.Key("supported_rpc_methods");
    writer.StartArray();
    writer.String("start");
    writer.String("stop");
    writer.String("return");
    writer.String("reset");
    writer.EndArray();
    writer.Key("config_schema_version");
    writer.String("thingsboard-v1");
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_status_event(const StatusEventView& view,
                                                char* out,
                                                size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("event");
    writer.String(view.event_name ? view.event_name : "");
    writer.Key("accepted");
    writer.Bool(view.accepted);
    writer.Key("reason");
    writer.String(view.reason ? view.reason : "");
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_command_event(const CommandEventView& view,
                                                 char* out,
                                                 size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("event");
    writer.String(view.event_name ? view.event_name : "");
    if (!view.command) {
        writer.EndObject();
        return stream.overflow() ? 0u : stream.size();
    }
    write_command_fields(writer, *view.command);
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_business_telemetry(const app::RobotRuntimeSnapshot& view,
                                                      char* out,
                                                      size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("device_state");
    writer.String(view.device_state.c_str());
    writer.Key("task_state");
    writer.String(view.task_state.c_str());
    writer.Key("target_passes");
    writer.Int(view.target_passes);
    writer.Key("completed_passes");
    writer.Int(view.completed_passes);
    writer.Key("clean_count");
    writer.Int(view.clean_count);
    writer.Key("active_config_version");
    writer.Uint64(view.active_config_version);

    if (view.active_config) write_runtime_config("active_config", *view.active_config, writer);
    if (view.pending_config) write_runtime_config("pending_config", *view.pending_config, writer);
    if (view.active_command) write_command_snapshot("active_command", *view.active_command, writer);
    if (view.last_command) write_command_snapshot("last_command", *view.last_command, writer);

    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

}  // namespace robot::service
