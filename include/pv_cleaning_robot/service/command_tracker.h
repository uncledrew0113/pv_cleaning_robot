#pragma once

/// @file command_tracker.h
/// @brief 云端命令 ID 追踪器，用于为 RPC 命令生成本地唯一 ID 并记录当前活动命令。

#include <cstdint>
#include <mutex>
#include <string>

namespace robot::service {

class CommandTracker {
   public:
    /// @brief 接收一个新命令并生成内部命令 ID。
    /// @return 本地内部命令 ID，用于日志、上报和 RPC 回复关联。
    std::string accept(const std::string& name, const std::string& request_id);

    /// @brief 将当前活动命令标记为运行中；当前 MVP 只保留本地 ID 追踪，不再暴露完成态查询。
    void mark_running(const std::string& id);

    /// @brief 拒绝一个命令请求，并消耗一个本地命令 ID，保证日志中的命令序号单调递增。
    void reject(const std::string& name, const std::string& request_id, const std::string& reason);

   private:
    static std::string make_id(uint64_t sequence);

    mutable std::mutex mtx_;
    uint64_t next_id_{1};
    std::string active_id_;
};

}  // namespace robot::service
