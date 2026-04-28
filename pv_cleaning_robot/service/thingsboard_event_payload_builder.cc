#include "pv_cleaning_robot/service/thingsboard_event_payload_builder.h"

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
    void append_u64(uint64_t v) noexcept { append_format("%llu", static_cast<unsigned long long>(v)); }

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

}  // namespace

size_t ThingsBoardEventPayloadBuilder::build_status_event(const StatusEventView& view,
                                                          char* out,
                                                          size_t cap) noexcept {
    FixedJsonWriter w(out, cap);
    w.append_raw("{\"event\":");
    w.append_quoted(view.event_name);
    w.append_raw(",\"accepted\":");
    w.append_bool(view.accepted);
    w.append_raw(",\"reason\":");
    w.append_quoted(view.reason ? view.reason : "");
    w.append_char('}');
    return w.size();
}

size_t ThingsBoardEventPayloadBuilder::build_command_event(const CommandEventView& view,
                                                           char* out,
                                                           size_t cap) noexcept {
    FixedJsonWriter w(out, cap);
    w.append_raw("{\"event\":");
    w.append_quoted(view.event_name);
    if (!view.command) {
        w.append_char('}');
        return w.size();
    }

    w.append_raw(",\"command_id\":");
    w.append_quoted(view.command->id.c_str());
    w.append_raw(",\"command_name\":");
    w.append_quoted(view.command->name.c_str());
    w.append_raw(",\"request_id\":");
    w.append_quoted(view.command->request_id.c_str());
    w.append_raw(",\"phase\":");
    w.append_quoted(command_phase_name(view.command->phase));
    w.append_raw(",\"reason\":");
    w.append_quoted(view.command->reason.c_str());
    w.append_raw(",\"accepted_at_ms\":");
    w.append_u64(view.command->accepted_at_ms);
    w.append_raw(",\"finished_at_ms\":");
    w.append_u64(view.command->finished_at_ms);
    w.append_char('}');
    return w.size();
}

}  // namespace robot::service
