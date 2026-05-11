#pragma once

/// @file command_tracker.h
/// @brief 云端命令状态追踪器，用于记录命令的接收、执行和完成结果。

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
    /// 命令内部唯一 ID
    std::string id;
    /// 命令名称
    std::string name;
    /// 上层请求标识
    std::string request_id;
    /// 当前命令阶段
    CommandPhase phase{CommandPhase::Accepted};
    /// 失败或拒绝原因描述
    std::string reason;
    /// 接受命令时间戳（毫秒）
    uint64_t accepted_at_ms{0};
    /// 命令完成时间戳（毫秒），仅成功/失败/拒绝时填充
    uint64_t finished_at_ms{0};

    bool operator==(const CommandSnapshot& other) const {
        return id == other.id && name == other.name && request_id == other.request_id &&
               phase == other.phase && reason == other.reason &&
               accepted_at_ms == other.accepted_at_ms && finished_at_ms == other.finished_at_ms;
    }
};

class CommandTracker {
   public:
    /// @brief 接收一个新命令并生成内部命令 ID。
    /// @return Internal command id.
    std::string accept(const std::string& name, const std::string& request_id);

    /// @brief 将当前活动命令标记为运行中。
    void mark_running(const std::string& id);

    /// @brief 将当前活动命令标记为成功完成。
    void finish_success(const std::string& id, const std::string& reason = {});

    /// @brief 将当前活动命令标记为失败完成。
    void finish_failure(const std::string& id, const std::string& reason);

    /// @brief 拒绝一个命令请求，并记录拒绝原因。
    void reject(const std::string& name, const std::string& request_id, const std::string& reason);

    /// @brief 获取当前正在执行的命令快照。
    std::optional<CommandSnapshot> active() const;

    /// @brief 获取最近完成（成功/失败/拒绝）的命令快照。
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
