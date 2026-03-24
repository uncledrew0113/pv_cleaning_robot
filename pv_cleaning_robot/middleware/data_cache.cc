#include "pv_cleaning_robot/middleware/data_cache.h"
#include <sqlite3.h>
#include <vector>
#include <chrono>
#include <stdexcept>

namespace robot::middleware {

DataCache::DataCache(std::string db_path, size_t max_rows)
    : db_path_(std::move(db_path)), max_rows_(max_rows)
{
}

DataCache::~DataCache()
{
    close();
}

bool DataCache::open()
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (db_) return true;

    int rc = sqlite3_open(db_path_.c_str(), &db_);
    if (rc != SQLITE_OK) {
        db_ = nullptr;
        return false;
    }

    // 启用 WAL 模式、外键约束以及合理的超时
    sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);
    sqlite3_busy_timeout(db_, 3000);

    const char* create_sql =
        "CREATE TABLE IF NOT EXISTS telemetry_cache ("
        "  id      INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  topic   TEXT    NOT NULL,"
        "  payload TEXT    NOT NULL,"
        "  ts_ms   INTEGER NOT NULL"
        ");";
    rc = sqlite3_exec(db_, create_sql, nullptr, nullptr, nullptr);
    return (rc == SQLITE_OK);
}

void DataCache::close()
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool DataCache::push(const std::string& topic, const std::string& payload,
                     uint64_t ts_ms)
{
    if (ts_ms == 0) {
        ts_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if (!db_) return false;

    const char* sql = "INSERT INTO telemetry_cache (topic, payload, ts_ms) VALUES (?,?,?);";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;

    sqlite3_bind_text (stmt, 1, topic.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_text (stmt, 2, payload.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(ts_ms));

    int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc == SQLITE_DONE) {
        prune_if_needed();
        return true;
    }
    return false;
}

std::vector<DataCache::Record> DataCache::pop_batch(int max_count)
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<Record> result;
    result.reserve(static_cast<size_t>(max_count)); // 避免 SQLite 行循环中重复扩容
    if (!db_) return result;

    const char* sql =
        "SELECT id, topic, payload, ts_ms FROM telemetry_cache "
        "ORDER BY id ASC LIMIT ?;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return result;

    sqlite3_bind_int(stmt, 1, max_count);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Record r;
        r.id      = sqlite3_column_int64(stmt, 0);
        r.topic   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        r.payload = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        r.ts_ms   = static_cast<uint64_t>(sqlite3_column_int64(stmt, 3));
        result.push_back(std::move(r));
    }
    sqlite3_finalize(stmt);
    return result;
}

void DataCache::confirm_sent(const std::vector<int64_t>& ids)
{
    if (ids.empty()) return;
    std::lock_guard<std::mutex> lk(mtx_);
    if (!db_) return;

    sqlite3_exec(db_, "BEGIN;", nullptr, nullptr, nullptr);
    sqlite3_stmt* stmt = nullptr;
    const char* sql = "DELETE FROM telemetry_cache WHERE id = ?;";
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK) {
        for (auto id : ids) {
            sqlite3_bind_int64(stmt, 1, id);
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        sqlite3_finalize(stmt);
    }
    sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);
}

size_t DataCache::size()
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!db_) return 0;
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, "SELECT COUNT(*) FROM telemetry_cache;", -1, &stmt, nullptr);
    size_t cnt = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        cnt = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
    sqlite3_finalize(stmt);
    return cnt;
}

// 必须在 mtx_ 持有状态下调用
void DataCache::prune_if_needed()
{
    if (!db_) return;
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, "SELECT COUNT(*) FROM telemetry_cache;", -1, &stmt, nullptr);
    size_t cnt = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        cnt = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
    sqlite3_finalize(stmt);

    if (cnt > max_rows_) {
        // 删除最旧的 1000 条
        sqlite3_exec(db_,
            "DELETE FROM telemetry_cache WHERE id IN "
            "(SELECT id FROM telemetry_cache ORDER BY id ASC LIMIT 1000);",
            nullptr, nullptr, nullptr);
    }
}

} // namespace robot::middleware
