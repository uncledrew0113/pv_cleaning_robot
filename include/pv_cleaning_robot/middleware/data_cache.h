#pragma once
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

// 前向声明，避免引入 sqlite3.h 到公共头文件
struct sqlite3;

namespace robot::middleware {

/// @brief 遥测本地缓存（SQLite3 WAL 模式）
///
/// 在网络不可用时，将遥测 JSON 缓存到本地 SQLite 数据库，
/// 网络恢复后批量上传，防止数据丢失。
class DataCache {
public:
    static constexpr size_t kDefaultMaxRows = 10000;  ///< 超过则自动清理最旧记录

    explicit DataCache(std::string db_path,
                       size_t max_rows = kDefaultMaxRows);
    ~DataCache();

    /// 打开/创建数据库并建表
    bool open();
    void close();

    /// 存入一条遥测记录
    /// @param topic    MQTT topic（用于区分遥测类型）
    /// @param payload  JSON 字符串
    /// @param ts_ms    UTC 毫秒时间戳（0=自动填当前时间）
    bool push(const std::string& topic, const std::string& payload,
              uint64_t ts_ms = 0);

    /// 取出最多 max_count 条最旧的未删除记录
    struct Record {
        int64_t     id;
        std::string topic;
        std::string payload;
        uint64_t    ts_ms;
    };
    std::vector<Record> pop_batch(int max_count = 50);

    /// 删除已成功上传的记录（按 id）
    void confirm_sent(const std::vector<int64_t>& ids);

    /// 当前缓存条数
    size_t size();

private:
    void prune_if_needed();

    std::string  db_path_;
    size_t       max_rows_;
    sqlite3*     db_{nullptr};
    mutable std::mutex mtx_;
};

} // namespace robot::middleware
