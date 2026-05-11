// @file command_tracker.cc
// @brief CommandTracker 实现：内部命令生命周期管理与状态查询。

#include <chrono>
#include <utility>

#include "pv_cleaning_robot/service/command_tracker.h"

namespace robot::service {

CommandSnapshot CommandTracker::make_snapshot(const std::string& name,
                                              const std::string& request_id,
                                              CommandPhase phase,
                                              const std::string& reason) const {
    // 生成命令快照时使用当前命令 ID 和时间戳
    CommandSnapshot snap;
    snap.id = "cmd-" + std::to_string(next_id_);
    snap.name = name;
    snap.request_id = request_id;
    snap.phase = phase;
    snap.reason = reason;
    snap.accepted_at_ms = now_ms();
    snap.finished_at_ms = phase == CommandPhase::Rejected ? snap.accepted_at_ms : 0;
    return snap;
}

/// @brief 接收新命令并设置为活动命令。
std::string CommandTracker::accept(const std::string& name, const std::string& request_id) {
    std::lock_guard<std::mutex> lk(mtx_);
    CommandSnapshot snap = make_snapshot(name, request_id, CommandPhase::Accepted);
    ++next_id_;
    active_ = snap;
    return snap.id;
}

/// @brief 将指定命令标记为运行中，如果它仍然是当前活动命令。
void CommandTracker::mark_running(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || active_->id != id)
        return;
    active_->phase = CommandPhase::Running;
}

/// @brief 标记当前活动命令为成功完成。
void CommandTracker::finish_success(const std::string& id, const std::string& reason) {
    finish_active(id, CommandPhase::Succeeded, reason);
}

/// @brief 标记当前活动命令为失败完成。
void CommandTracker::finish_failure(const std::string& id, const std::string& reason) {
    finish_active(id, CommandPhase::Failed, reason);
}

/// @brief 内部完成活动命令，并将其转移到 last_completed_。
void CommandTracker::finish_active(const std::string& id,
                                   CommandPhase phase,
                                   const std::string& reason) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || active_->id != id) {
        return;
    }
    active_->phase = phase;
    active_->reason = reason;
    active_->finished_at_ms = now_ms();
    last_completed_ = active_;
    active_.reset();
}

/// @brief 记录被拒绝的命令作为最近完成命令。
void CommandTracker::reject(const std::string& name,
                            const std::string& request_id,
                            const std::string& reason) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_completed_ = make_snapshot(name, request_id, CommandPhase::Rejected, reason);
    ++next_id_;
}

/// @brief 获取当前正在执行的命令快照。
std::optional<CommandSnapshot> CommandTracker::active() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return active_;
}

/// @brief 获取最近完成的命令快照。
std::optional<CommandSnapshot> CommandTracker::last_completed() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_completed_;
}

uint64_t CommandTracker::now_ms() {
    const auto now =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    return static_cast<uint64_t>(now.time_since_epoch().count());
}

}  // namespace robot::service
