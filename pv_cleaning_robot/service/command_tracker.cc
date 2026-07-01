// @file command_tracker.cc
// @brief CommandTracker 实现：RPC 命令本地 ID 生成和当前活动命令记录。

#include "pv_cleaning_robot/service/command_tracker.h"

namespace robot::service {

std::string CommandTracker::make_id(uint64_t sequence) {
    return "cmd-" + std::to_string(sequence);
}

std::string CommandTracker::accept(const std::string& name, const std::string& request_id) {
    (void)name;
    (void)request_id;
    std::lock_guard<std::mutex> lk(mtx_);
    const auto id = make_id(next_id_);
    ++next_id_;
    active_id_ = id;
    return id;
}

void CommandTracker::mark_running(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (active_id_ != id) {
        return;
    }
}

void CommandTracker::reject(const std::string& name,
                            const std::string& request_id,
                            const std::string& reason) {
    (void)name;
    (void)request_id;
    (void)reason;
    std::lock_guard<std::mutex> lk(mtx_);
    ++next_id_;
}

}  // namespace robot::service
