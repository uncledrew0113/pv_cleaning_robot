#pragma once
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace robot::middleware {

/// @brief 遥测本地缓存（JSONL 文件持久化）
///
/// 关注点：
///   - 每次 push() 将队列全量原子重写到磁盘（SCHED_OTHER 云端线程 1Hz，重写 150KB ≈ 1ms）
///   - confirm_sent() 删除已发记录后同样重写；open() 从文件恢复未发数据
///   - 断电重启后能自动加载积压遥测并在网络恢复后补发
///   - 文件格式：每行一个 JSON：{"id":1,"topic":"...","payload":"...","ts_ms":...}
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
    /// 必须在 mtx_ 持有状态下调用：将 queue_ 原子重写到文件（.tmp → rename）
    void flush_to_file() const;

    std::string        file_path_;
    size_t             max_rows_;
    int64_t            next_id_{1};
    std::deque<Record> queue_;
    mutable std::mutex mtx_;
};

} // namespace robot::middleware
