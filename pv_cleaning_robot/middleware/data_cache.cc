#include "pv_cleaning_robot/middleware/data_cache.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace robot::middleware {

DataCache::DataCache(std::string file_path, size_t max_rows)
    : file_path_(std::move(file_path)), max_rows_(max_rows) {}

DataCache::~DataCache() {}

bool DataCache::open() {
    std::lock_guard<std::mutex> lk(mtx_);

    // 创建父目录（/data/pv_robot/ 等目录首次启动可能不存在）
    std::error_code ec;
    auto parent = std::filesystem::path(file_path_).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent, ec);

    // 加载已有文件
    std::ifstream in(file_path_);
    if (!in.is_open()) return true;  // 文件不存在是正常情况（首次运行）

    std::string line;
    int64_t max_id = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        try {
            auto j   = nlohmann::json::parse(line);
            Record r;
            r.id      = j.at("id").get<int64_t>();
            r.topic   = j.at("topic").get<std::string>();
            r.payload = j.at("payload").get<std::string>();
            r.ts_ms   = j.at("ts_ms").get<uint64_t>();
            if (r.id > max_id) max_id = r.id;
            queue_.push_back(std::move(r));
        } catch (...) {
            spdlog::warn("[DataCache] 跳过损坏行: {}", line.substr(0, 80));
        }
    }
    if (max_id >= next_id_) next_id_ = max_id + 1;
    spdlog::info("[DataCache] 从文件加载 {} 条待发送记录", queue_.size());
    return true;
}

bool DataCache::push(const std::string& topic, const std::string& payload,
                     uint64_t ts_ms) {
    if (ts_ms == 0) {
        ts_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if (queue_.size() >= max_rows_) {
        queue_.pop_front();  // 超出容量丢弃最旧记录
    }
    queue_.push_back({next_id_++, topic, payload, ts_ms});
    flush_to_file();
    return true;
}

std::vector<DataCache::Record> DataCache::pop_batch(int max_count) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<Record> result;
    int n = std::min(static_cast<int>(queue_.size()), max_count);
    result.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        result.push_back(queue_[static_cast<size_t>(i)]);
    return result;
}

void DataCache::confirm_sent(const std::vector<int64_t>& ids) {
    if (ids.empty()) return;
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto id : ids) {
        auto it = std::find_if(queue_.begin(), queue_.end(),
                               [id](const Record& r) { return r.id == id; });
        if (it != queue_.end()) queue_.erase(it);
    }
    flush_to_file();
}

size_t DataCache::size() {
    std::lock_guard<std::mutex> lk(mtx_);
    return queue_.size();
}

void DataCache::flush_to_file() const {
    // 原子写：先写 .tmp，再 rename（Linux rename 是原子操作）
    std::string tmp = file_path_ + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        for (const auto& r : queue_) {
            out << nlohmann::json{{"id",      r.id},
                                  {"topic",   r.topic},
                                  {"payload", r.payload},
                                  {"ts_ms",   r.ts_ms}}.dump()
                << '\n';
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, file_path_, ec);
    if (ec)
        spdlog::warn("[DataCache] 文件重写失败: {}", ec.message());
}

} // namespace robot::middleware
