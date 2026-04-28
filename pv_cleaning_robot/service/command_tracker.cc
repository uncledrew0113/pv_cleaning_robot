#include "pv_cleaning_robot/service/command_tracker.h"

#include <chrono>
#include <utility>

namespace robot::service {

std::string CommandTracker::accept(const std::string& name, const std::string& request_id) {
    std::lock_guard<std::mutex> lk(mtx_);

    CommandSnapshot snap;
    snap.id = "cmd-" + std::to_string(next_id_++);
    snap.name = name;
    snap.request_id = request_id;
    snap.phase = CommandPhase::Accepted;
    snap.accepted_at_ms = now_ms();
    snap.finished_at_ms = 0;
    snap.reason.clear();

    active_ = snap;
    return snap.id;
}

void CommandTracker::mark_running(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || active_->id != id)
        return;
    active_->phase = CommandPhase::Running;
}

void CommandTracker::finish_success(const std::string& id, const std::string& reason) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || active_->id != id)
        return;

    active_->phase = CommandPhase::Succeeded;
    active_->reason = reason;
    active_->finished_at_ms = now_ms();
    last_completed_ = active_;
    active_.reset();
}

void CommandTracker::finish_failure(const std::string& id, const std::string& reason) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || active_->id != id)
        return;

    active_->phase = CommandPhase::Failed;
    active_->reason = reason;
    active_->finished_at_ms = now_ms();
    last_completed_ = active_;
    active_.reset();
}

void CommandTracker::reject(const std::string& name,
                            const std::string& request_id,
                            const std::string& reason) {
    std::lock_guard<std::mutex> lk(mtx_);

    CommandSnapshot snap;
    snap.id = "cmd-" + std::to_string(next_id_++);
    snap.name = name;
    snap.request_id = request_id;
    snap.phase = CommandPhase::Rejected;
    snap.reason = reason;
    snap.accepted_at_ms = now_ms();
    snap.finished_at_ms = snap.accepted_at_ms;

    last_completed_ = std::move(snap);
}

std::optional<CommandSnapshot> CommandTracker::active() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return active_;
}

std::optional<CommandSnapshot> CommandTracker::last_completed() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_completed_;
}

uint64_t CommandTracker::now_ms() {
    const auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now());
    return static_cast<uint64_t>(now.time_since_epoch().count());
}

}  // namespace robot::service
