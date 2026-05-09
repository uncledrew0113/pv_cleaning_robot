#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace robot::service {

enum class CommandPhase {
    Accepted,
    Running,
    Succeeded,
    Failed,
    Rejected,
};

struct CommandSnapshot {
    std::string id;
    std::string name;
    std::string request_id;
    CommandPhase phase{CommandPhase::Accepted};
    std::string reason;
    uint64_t accepted_at_ms{0};
    uint64_t finished_at_ms{0};

    bool operator==(const CommandSnapshot& other) const {
        return id == other.id &&
               name == other.name &&
               request_id == other.request_id &&
               phase == other.phase &&
               reason == other.reason &&
               accepted_at_ms == other.accepted_at_ms &&
               finished_at_ms == other.finished_at_ms;
    }
};

class CommandTracker {
public:
    std::string accept(const std::string& name, const std::string& request_id);
    void mark_running(const std::string& id);
    void finish_success(const std::string& id, const std::string& reason = {});
    void finish_failure(const std::string& id, const std::string& reason);
    void reject(const std::string& name, const std::string& request_id, const std::string& reason);

    std::optional<CommandSnapshot> active() const;
    std::optional<CommandSnapshot> last_completed() const;

private:
    static uint64_t now_ms();
    CommandSnapshot make_snapshot(const std::string& name,
                                  const std::string& request_id,
                                  CommandPhase phase,
                                  const std::string& reason = {}) const;
    void finish_active(const std::string& id, CommandPhase phase, const std::string& reason);

    mutable std::mutex mtx_;
    uint64_t next_id_{1};
    std::optional<CommandSnapshot> active_;
    std::optional<CommandSnapshot> last_completed_;
};

}  // namespace robot::service
