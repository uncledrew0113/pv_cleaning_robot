#include "pv_cleaning_robot/service/business_payload_builder.h"

#include <cstdarg>
#include <cstdio>

namespace robot::service {
namespace {

class FixedJsonWriter {
public:
    FixedJsonWriter(char* out, size_t cap) noexcept : out_(out), cap_(cap) {
        if (out_ && cap_ > 0) out_[0] = '\0';
    }

    size_t size() const noexcept { return overflow_ ? 0u : len_; }

    void append_raw(const char* text) noexcept { append_format("%s", text ? text : ""); }

    void append_char(char ch) noexcept {
        if (!ensure_space(1u)) return;
        out_[len_++] = ch;
        out_[len_] = '\0';
    }

    void append_quoted(const char* text) noexcept {
        append_char('"');
        if (!text) text = "";
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p != '\0'; ++p) {
            switch (*p) {
            case '"':
                append_raw("\\\"");
                break;
            case '\\':
                append_raw("\\\\");
                break;
            case '\b':
                append_raw("\\b");
                break;
            case '\f':
                append_raw("\\f");
                break;
            case '\n':
                append_raw("\\n");
                break;
            case '\r':
                append_raw("\\r");
                break;
            case '\t':
                append_raw("\\t");
                break;
            default:
                if (*p < 0x20u) {
                    append_format("\\u%04x", static_cast<unsigned>(*p));
                } else {
                    append_char(static_cast<char>(*p));
                }
                break;
            }
            if (overflow_) return;
        }
        append_char('"');
    }

    void append_bool(bool v) noexcept { append_raw(v ? "true" : "false"); }
    void append_int(int v) noexcept { append_format("%d", v); }
    void append_u64(uint64_t v) noexcept { append_format("%llu", static_cast<unsigned long long>(v)); }
    void append_double(double v) noexcept { append_format("%.6f", v); }

private:
    bool ensure_space(size_t extra) noexcept {
        if (!out_ || cap_ == 0 || overflow_ || len_ + extra >= cap_) {
            overflow_ = true;
            if (out_ && cap_ > 0) out_[cap_ - 1] = '\0';
            len_ = 0;
            return false;
        }
        return true;
    }

    void append_format(const char* fmt, ...) noexcept {
        if (!out_ || cap_ == 0 || overflow_) return;
        va_list args;
        va_start(args, fmt);
        const int written = std::vsnprintf(out_ + len_, cap_ - len_, fmt, args);
        va_end(args);
        if (written < 0 || static_cast<size_t>(written) >= cap_ - len_) {
            overflow_ = true;
            if (out_ && cap_ > 0) out_[cap_ - 1] = '\0';
            len_ = 0;
            return;
        }
        len_ += static_cast<size_t>(written);
    }

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

void append_schedule_entries(const std::vector<TbScheduleEntry>& schedules, FixedJsonWriter& w) noexcept {
    w.append_char('[');
    for (size_t i = 0; i < schedules.size(); ++i) {
        if (i != 0u) w.append_char(',');
        w.append_raw("{\"hour\":");
        w.append_int(schedules[i].hour);
        w.append_raw(",\"minute\":");
        w.append_int(schedules[i].minute);
        w.append_char('}');
    }
    w.append_char(']');
}

void append_runtime_config(const char* key,
                           const TbRuntimeConfig& config,
                           FixedJsonWriter& w) noexcept {
    w.append_raw(",\"");
    w.append_raw(key);
    w.append_raw("\":{\"passes\":");
    w.append_double(config.passes);
    w.append_raw(",\"clean_speed_rpm\":");
    w.append_double(config.clean_speed_rpm);
    w.append_raw(",\"return_speed_rpm\":");
    w.append_double(config.return_speed_rpm);
    w.append_raw(",\"brush_rpm\":");
    w.append_int(config.brush_rpm);
    w.append_raw(",\"schedules\":");
    append_schedule_entries(config.schedules, w);
    w.append_char('}');
}

void append_command_snapshot(const char* key,
                             const CommandSnapshot& command,
                             FixedJsonWriter& w) noexcept {
    w.append_raw(",\"");
    w.append_raw(key);
    w.append_raw("\":{\"id\":");
    w.append_quoted(command.id.c_str());
    w.append_raw(",\"name\":");
    w.append_quoted(command.name.c_str());
    w.append_raw(",\"request_id\":");
    w.append_quoted(command.request_id.c_str());
    w.append_raw(",\"phase\":");
    w.append_quoted(command_phase_name(command.phase));
    w.append_raw(",\"reason\":");
    w.append_quoted(command.reason.c_str());
    w.append_raw(",\"accepted_at_ms\":");
    w.append_u64(command.accepted_at_ms);
    w.append_raw(",\"finished_at_ms\":");
    w.append_u64(command.finished_at_ms);
    w.append_char('}');
}

}  // namespace

size_t BusinessPayloadBuilder::build(const BusinessTelemetrySnapshot& view,
                                     char* out,
                                     size_t cap) noexcept {
    FixedJsonWriter w(out, cap);

    w.append_raw("{\"device_state\":");
    w.append_quoted(view.device_state.c_str());
    w.append_raw(",\"task_state\":");
    w.append_quoted(view.task_state.c_str());
    w.append_raw(",\"target_half_passes\":");
    w.append_int(view.target_half_passes);
    w.append_raw(",\"completed_half_passes\":");
    w.append_int(view.completed_half_passes);
    w.append_raw(",\"clean_count\":");
    w.append_int(view.clean_count);
    w.append_raw(",\"active_config_version\":");
    w.append_u64(view.active_config_version);

    if (view.active_config) append_runtime_config("active_config", *view.active_config, w);
    if (view.pending_config) append_runtime_config("pending_config", *view.pending_config, w);
    if (view.active_command) append_command_snapshot("active_command", *view.active_command, w);
    if (view.last_command) append_command_snapshot("last_command", *view.last_command, w);

    w.append_char('}');
    return w.size();
}

}  // namespace robot::service
