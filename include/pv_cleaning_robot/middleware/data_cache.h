#pragma once
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace robot::middleware {

/// @brief 遥测本地缓存（JSONL journal 持久化）
///
/// 关注点：
///   - push()/confirm_sent() 采用追加 journal，避免每次操作都全量重写文件
///   - open() 回放 push/ack 操作，恢复仍未确认的离线遥测
///   - journal 膨胀到阈值后做一次快照 compact，收敛文件体积
///   - 文件兼容旧格式快照行：{"id":1,"topic":"...","payload":"...","ts_ms":...}
///
/// 线程安全：所有公开方法均通过 mtx_ 保护，可从多个线程并发调用。
class DataCache {
public:
    static constexpr size_t kDefaultMaxRows = 500;  ///< 超出则丢弃最旧记录

    explicit DataCache(std::string file_path,
                       size_t max_rows = kDefaultMaxRows);
    ~DataCache();

    /// 创建父目录；从已有 JSONL 文件加载未确认记录
    bool open();
    /// 无操作（所有写入已在 push() 内完成）
    void close() {}

    /// 存入一条遥测记录，同步刷盘
    bool push(const std::string& topic, const std::string& payload,
              uint64_t ts_ms = 0);

    struct Record {
        int64_t     id;
        std::string topic;
        std::string payload;
        uint64_t    ts_ms;
    };

    /// 取出最多 max_count 条最旧记录（不删除，等 confirm_sent 后才删除）
    std::vector<Record> pop_batch(int max_count = 50);

    /// 删除已成功上传的记录（按 id），并重写文件
    void confirm_sent(const std::vector<int64_t>& ids);

    /// 当前缓存条数
    size_t size();

private:
    struct JournalStats {
        size_t append_count{0};
        size_t ack_count{0};
    };

    bool append_push_record_locked(const Record& record);
    bool append_ack_record_locked(int64_t id);
    void maybe_compact_locked();
    bool compact_to_snapshot_locked();

    std::string        file_path_;
    size_t             max_rows_;
    int64_t            next_id_{1};
    std::deque<Record> queue_;
    JournalStats       journal_stats_{};
    mutable std::mutex mtx_;
};

} // namespace robot::middleware
