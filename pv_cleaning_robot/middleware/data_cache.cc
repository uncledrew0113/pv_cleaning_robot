/**
 * @file data_cache.cc
 * @brief 离线遥测数据缓存实现。
 *
 * 本文件使用 JSONL 文件追加缓存待发送数据，并通过确认 ID 删除已发送记录。容量压缩时保留最新
 * 记录，防止网络长期不可用导致文件无限增长。
 */
#include "pv_cleaning_robot/middleware/data_cache.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>

namespace robot::middleware {
namespace {

constexpr const char* kPushOp = "push";
constexpr const char* kAckOp = "ack";
constexpr size_t kCompactOpThreshold = 256;
constexpr size_t kJournalParsePoolBytes = 2048;

bool parse_snapshot_record(const rapidjson::Value& value, DataCache::Record* out)
{
    if (!out) return false;
    if (!value.IsObject()) return false;

    const auto id_it = value.FindMember("id");
    const auto topic_it = value.FindMember("topic");
    const auto payload_it = value.FindMember("payload");
    const auto ts_it = value.FindMember("ts_ms");
    if (id_it == value.MemberEnd() || topic_it == value.MemberEnd() ||
        payload_it == value.MemberEnd() || ts_it == value.MemberEnd()) {
        return false;
    }
    if (!id_it->value.IsInt64() || !topic_it->value.IsString() ||
        !payload_it->value.IsString() || !ts_it->value.IsUint64()) {
        return false;
    }

    out->id = id_it->value.GetInt64();
    out->topic.assign(topic_it->value.GetString(), topic_it->value.GetStringLength());
    out->payload.assign(payload_it->value.GetString(), payload_it->value.GetStringLength());
    out->ts_ms = ts_it->value.GetUint64();
    return true;
}

std::string build_push_line(const DataCache::Record& record)
{
    rapidjson::StringBuffer buffer;
    buffer.Reserve(static_cast<rapidjson::SizeType>(
        64 + record.topic.size() + record.payload.size()));
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("op");
    writer.String(kPushOp);
    writer.Key("id");
    writer.Int64(record.id);
    writer.Key("topic");
    writer.String(record.topic.c_str(), static_cast<rapidjson::SizeType>(record.topic.size()));
    writer.Key("payload");
    writer.String(record.payload.c_str(), static_cast<rapidjson::SizeType>(record.payload.size()));
    writer.Key("ts_ms");
    writer.Uint64(record.ts_ms);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_ack_line(int64_t id)
{
    rapidjson::StringBuffer buffer;
    buffer.Reserve(32);
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("op");
    writer.String(kAckOp);
    writer.Key("id");
    writer.Int64(id);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_snapshot_line(const DataCache::Record& record)
{
    rapidjson::StringBuffer buffer;
    buffer.Reserve(static_cast<rapidjson::SizeType>(
        48 + record.topic.size() + record.payload.size()));
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("id");
    writer.Int64(record.id);
    writer.Key("topic");
    writer.String(record.topic.c_str(), static_cast<rapidjson::SizeType>(record.topic.size()));
    writer.Key("payload");
    writer.String(record.payload.c_str(), static_cast<rapidjson::SizeType>(record.payload.size()));
    writer.Key("ts_ms");
    writer.Uint64(record.ts_ms);
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

bool erase_record_by_id(std::deque<DataCache::Record>& queue, int64_t id)
{
    auto it = std::find_if(queue.begin(), queue.end(),
                           [id](const DataCache::Record& r) { return r.id == id; });
    if (it == queue.end()) return false;
    queue.erase(it);
    return true;
}

bool parse_json_object_line(const std::string& line, rapidjson::Document* out)
{
    if (!out) {
        return false;
    }
    out->Parse(line.c_str(), line.size());
    return !out->HasParseError() && out->IsObject();
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
            alignas(std::max_align_t) unsigned char pool_buffer[kJournalParsePoolBytes];
            rapidjson::MemoryPoolAllocator<rapidjson::CrtAllocator> allocator(
                pool_buffer, sizeof(pool_buffer));
            rapidjson::Document document(&allocator);
            document.Parse(line.c_str(), line.size());
            if (document.HasParseError() || !document.IsObject()) {
                throw std::runtime_error("invalid JSON line");
            }
            const auto op_it = document.FindMember("op");
            if (op_it != document.MemberEnd()) {
                saw_journal_ops = true;
                if (!op_it->value.IsString()) {
                    throw std::runtime_error("invalid journal op");
                }
                const std::string op(op_it->value.GetString(), op_it->value.GetStringLength());
                if (op == kPushOp) {
                    Record record{};
                    if (!parse_snapshot_record(document, &record)) {
                        throw std::runtime_error("invalid push record");
                    }
                    max_id = std::max(max_id, record.id);
                    erase_record_by_id(queue_, record.id);
                    queue_.push_back(std::move(record));
                    ++replay_push_count;
                } else if (op == kAckOp) {
                    const auto id_it = document.FindMember("id");
                    if (id_it == document.MemberEnd() || !id_it->value.IsInt64()) {
                        throw std::runtime_error("invalid ack record");
                    }
                    const auto id = id_it->value.GetInt64();
                    max_id = std::max(max_id, id);
                    erase_record_by_id(queue_, id);
                    ++replay_ack_count;
                }
            } else {
                Record record{};
                if (!parse_snapshot_record(document, &record)) {
                    throw std::runtime_error("invalid snapshot record");
                }
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
        if (!append_ack_record_locked(dropped_id)) {
            spdlog::warn("[DataCache] 记录淘汰 ack 追加失败: {}", dropped_id);
            return false;
        }
        queue_.pop_front();
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
        if (!append_ack_record_locked(id)) {
            spdlog::warn("[DataCache] 确认发送 ack 追加失败: {}", id);
            continue;
        }
        erase_record_by_id(queue_, id);
    }
    maybe_compact_locked();
}

size_t DataCache::size()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return queue_.size();
}

void DataCache::set_test_append_hook(AppendHook hook)
{
    std::lock_guard<std::mutex> lk(mtx_);
    test_append_hook_ = std::move(hook);
}

bool DataCache::append_push_record_locked(const Record& record)
{
    const auto line = build_push_line(record);
    if (!append_journal_line_locked(line)) {
        return false;
    }
    ++journal_stats_.append_count;
    return true;
}

bool DataCache::append_ack_record_locked(int64_t id)
{
    const auto line = build_ack_line(id);
    if (!append_journal_line_locked(line)) {
        return false;
    }
    ++journal_stats_.ack_count;
    return true;
}

bool DataCache::append_journal_line_locked(const std::string& line)
{
    rapidjson::Document document;
    if (!parse_json_object_line(line, &document)) {
        return false;
    }
    if (test_append_hook_ && !test_append_hook_(document)) {
        return false;
    }

    std::ofstream out(file_path_, std::ios::app);
    if (!out.is_open()) {
        return false;
    }
    out << line << '\n';
    return out.good();
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
            out << build_snapshot_line(record) << '\n';
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
