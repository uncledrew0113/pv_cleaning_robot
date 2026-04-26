#include "pv_cleaning_robot/middleware/data_cache.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace robot::middleware {
namespace {

constexpr const char* kPushOp = "push";
constexpr const char* kAckOp = "ack";
constexpr size_t kCompactOpThreshold = 256;

bool parse_snapshot_record(const nlohmann::json& j, DataCache::Record* out)
{
    if (!out) return false;
    out->id = j.at("id").get<int64_t>();
    out->topic = j.at("topic").get<std::string>();
    out->payload = j.at("payload").get<std::string>();
    out->ts_ms = j.at("ts_ms").get<uint64_t>();
    return true;
}

bool erase_record_by_id(std::deque<DataCache::Record>& queue, int64_t id)
{
    auto it = std::find_if(queue.begin(), queue.end(),
                           [id](const DataCache::Record& r) { return r.id == id; });
    if (it == queue.end()) return false;
    queue.erase(it);
    return true;
}

}  // namespace

DataCache::DataCache(std::string file_path, size_t max_rows)
    : file_path_(std::move(file_path)), max_rows_(max_rows) {}

DataCache::~DataCache() {}

bool DataCache::open()
{
    std::lock_guard<std::mutex> lk(mtx_);

    std::error_code ec;
    auto parent = std::filesystem::path(file_path_).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent, ec);

    queue_.clear();
    next_id_ = 1;
    journal_stats_ = {};

    std::ifstream in(file_path_);
    if (!in.is_open()) return true;

    std::string line;
    int64_t max_id = 0;
    size_t replay_push_count = 0;
    size_t replay_ack_count = 0;
    bool saw_journal_ops = false;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        try {
            auto j = nlohmann::json::parse(line);
            if (j.contains("op")) {
                saw_journal_ops = true;
                const auto op = j.at("op").get<std::string>();
                if (op == kPushOp) {
                    Record record{};
                    parse_snapshot_record(j, &record);
                    max_id = std::max(max_id, record.id);
                    erase_record_by_id(queue_, record.id);
                    queue_.push_back(std::move(record));
                    ++replay_push_count;
                } else if (op == kAckOp) {
                    const auto id = j.at("id").get<int64_t>();
                    max_id = std::max(max_id, id);
                    erase_record_by_id(queue_, id);
                    ++replay_ack_count;
                }
            } else {
                Record record{};
                parse_snapshot_record(j, &record);
                max_id = std::max(max_id, record.id);
                queue_.push_back(std::move(record));
                ++replay_push_count;
            }
        } catch (...) {
            spdlog::warn("[DataCache] 跳过损坏行: {}", line.substr(0, 80));
        }
    }

    while (queue_.size() > max_rows_) {
        queue_.pop_front();
    }

    if (max_id >= next_id_) next_id_ = max_id + 1;
    journal_stats_.append_count = queue_.size();
    journal_stats_.ack_count = 0;

    spdlog::info("[DataCache] 从文件恢复 {} 条待发送记录 (push={}, ack={})",
                 queue_.size(), replay_push_count, replay_ack_count);

    if (saw_journal_ops &&
        (replay_ack_count > queue_.size() || replay_push_count + replay_ack_count >= kCompactOpThreshold)) {
        compact_to_snapshot_locked();
    }
    return true;
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

    if (queue_.size() >= max_rows_ && !queue_.empty()) {
        const auto dropped_id = queue_.front().id;
        queue_.pop_front();
        if (!append_ack_record_locked(dropped_id)) {
            spdlog::warn("[DataCache] 记录淘汰 ack 追加失败: {}", dropped_id);
        }
    }

    Record record{next_id_++, topic, payload, ts_ms};
    queue_.push_back(record);
    if (!append_push_record_locked(record)) {
        queue_.pop_back();
        return false;
    }

    maybe_compact_locked();
    return true;
}

std::vector<DataCache::Record> DataCache::pop_batch(int max_count)
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<Record> result;
    const int n = std::min(static_cast<int>(queue_.size()), max_count);
    result.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        result.push_back(queue_[static_cast<size_t>(i)]);
    }
    return result;
}

void DataCache::confirm_sent(const std::vector<int64_t>& ids)
{
    if (ids.empty()) return;

    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto id : ids) {
        if (!erase_record_by_id(queue_, id)) continue;
        if (!append_ack_record_locked(id)) {
            spdlog::warn("[DataCache] 确认发送 ack 追加失败: {}", id);
        }
    }
    maybe_compact_locked();
}

size_t DataCache::size()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return queue_.size();
}

bool DataCache::append_push_record_locked(const Record& record)
{
    std::ofstream out(file_path_, std::ios::app);
    if (!out.is_open()) return false;

    out << nlohmann::json{{"op", kPushOp},
                          {"id", record.id},
                          {"topic", record.topic},
                          {"payload", record.payload},
                          {"ts_ms", record.ts_ms}}
               .dump()
        << '\n';
    if (!out.good()) return false;

    ++journal_stats_.append_count;
    return true;
}

bool DataCache::append_ack_record_locked(int64_t id)
{
    std::ofstream out(file_path_, std::ios::app);
    if (!out.is_open()) return false;

    out << nlohmann::json{{"op", kAckOp}, {"id", id}}.dump() << '\n';
    if (!out.good()) return false;

    ++journal_stats_.ack_count;
    return true;
}

void DataCache::maybe_compact_locked()
{
    const size_t total_ops = journal_stats_.append_count + journal_stats_.ack_count;
    if (total_ops < kCompactOpThreshold && journal_stats_.ack_count <= queue_.size()) {
        return;
    }

    compact_to_snapshot_locked();
}

bool DataCache::compact_to_snapshot_locked()
{
    const std::string tmp = file_path_ + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out.is_open()) {
            spdlog::warn("[DataCache] compact 打开临时文件失败: {}", tmp);
            return false;
        }

        for (const auto& record : queue_) {
            out << nlohmann::json{{"id", record.id},
                                  {"topic", record.topic},
                                  {"payload", record.payload},
                                  {"ts_ms", record.ts_ms}}
                       .dump()
                << '\n';
        }

        if (!out.good()) {
            spdlog::warn("[DataCache] compact 写入临时文件失败: {}", tmp);
            return false;
        }
    }

    std::error_code ec;
    std::filesystem::rename(tmp, file_path_, ec);
    if (ec) {
        std::filesystem::remove(tmp);
        spdlog::warn("[DataCache] compact 替换快照失败: {}", ec.message());
        return false;
    }

    journal_stats_.append_count = queue_.size();
    journal_stats_.ack_count = 0;
    return true;
}

}  // namespace robot::middleware
