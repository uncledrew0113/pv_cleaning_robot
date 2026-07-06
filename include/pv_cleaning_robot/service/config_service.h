/**
 * @file config_service.h
 * @brief 运行配置和固定配置加载接口。
 *
 * ConfigService 负责加载 runtime/fixed 两份 JSON 配置，提供类型化读取、运行配置提升和
 * 持久化写回。固定配置用于现场硬件参数，运行配置用于任务参数和云端更新。
 */
#pragma once

#include <mutex>
#include <optional>
#include <rapidjson/document.h>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::service {

class SchedulerService;

using robot::domain::Endpoint;
using robot::domain::RuntimeConfig;
using robot::domain::RuntimeScheduleEntry;
using robot::domain::endpoint_config_string;

struct SharedAttrApplyResult {
    bool accepted{false};
    std::string reason;
};

/// @brief 全局配置服务（RapidJSON，多文件配置驱动）
///
/// 提供类型安全的嵌套路径访问：
///   cfg.get<std::string>("network.mqtt.broker_uri")
///
/// 约定：
/// - `config_path` 指向当前生效的 runtime 配置文件
/// - `fixed_path` 指向固定硬件/系统配置文件；缺省时按 runtime 路径自动推导
/// - `pending_path` 由 runtime 路径自动推导，仅用于下次任务生效配置
///
/// 读取规则：
/// - `get()` 先读 runtime，再回退到 fixed
/// - `get_fixed()` 只读 fixed
class ConfigService {
public:
    explicit ConfigService(std::string config_path, std::string fixed_path = {});

    /// 加载/重新加载配置文件
    bool load();

    /// 加载固定配置文件
    bool load_fixed();

    /// 获取配置项（path 以 '.' 分隔，例如 "network.mqtt.port"）
    template <typename T>
    T get(const std::string& path, const T& default_val = T{}) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (const auto value = get_optional_from_document<T>(root_, path)) {
            return *value;
        }
        if (const auto value = get_optional_from_document<T>(fixed_root_, path)) {
            return *value;
        }
        return default_val;
    }

    /// 获取固定配置项（path 以 '.' 分隔）
    template <typename T>
    T get_fixed(const std::string& path, const T& default_val = T{}) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (const auto value = get_optional_from_document<T>(fixed_root_, path)) {
            return *value;
        }
        return default_val;
    }

    /// 设置配置项（path 以 '.' 分隔）—仅更新内存，需调用 save() 持久化
    template <typename T>
    void set(const std::string& path, T value)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!root_.IsObject()) {
            root_.SetObject();
        }

        auto parts = split_path(path);
        rapidjson::Value* node = &root_;
        auto& alloc = root_.GetAllocator();
        for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
            if (!node->IsObject()) {
                node->SetObject();
            }
            auto it = node->FindMember(parts[i].c_str());
            if (it == node->MemberEnd()) {
                rapidjson::Value key(parts[i].c_str(), alloc);
                rapidjson::Value child(rapidjson::kObjectType);
                node->AddMember(key, child, alloc);
                it = node->FindMember(parts[i].c_str());
            } else if (!it->value.IsObject()) {
                it->value.SetObject();
            }
            node = &it->value;
        }

        rapidjson::Value stored;
        if constexpr (std::is_same_v<std::decay_t<T>, std::string>) {
            stored.SetString(value.c_str(), static_cast<rapidjson::SizeType>(value.size()), alloc);
        } else if constexpr (std::is_same_v<std::decay_t<T>, const char*>) {
            stored.SetString(value, alloc);
        } else if constexpr (std::is_same_v<std::decay_t<T>, bool>) {
            stored.SetBool(value);
        } else if constexpr (std::is_integral_v<std::decay_t<T>>) {
            stored.SetInt64(static_cast<int64_t>(value));
        } else if constexpr (std::is_floating_point_v<std::decay_t<T>>) {
            stored.SetDouble(static_cast<double>(value));
        } else {
            static_assert(!sizeof(T*), "Unsupported ConfigService::set type");
        }

        rapidjson::Value key(parts.back().c_str(), alloc);
        auto it = node->FindMember(parts.back().c_str());
        if (it == node->MemberEnd()) {
            node->AddMember(key, stored, alloc);
        } else {
            it->value = std::move(stored);
        }
    }

    /// 将当前内存配置写回 config_path_（原子临时文件 rename）
    bool save() const;

    /// 返回当前完整配置副本
    rapidjson::Document snapshot() const;

    /// 用完整配置副本替换当前配置并持久化；若持久化失败则回滚旧值
    bool replace_and_save(const rapidjson::Value& new_root);

    /// 持久化下一任务待生效配置
    bool save_pending(const rapidjson::Value& pending_root) const;

    /// 读取待生效配置；不存在或解析失败时返回 nullopt
    std::optional<rapidjson::Document> load_pending() const;

    /// 解析当前 active runtime 配置
    RuntimeConfig active_runtime_config() const;

    /// 解析当前 pending runtime 配置（基于 active + pending patch 合并视图）
    std::optional<RuntimeConfig> pending_runtime_config() const;

    bool has_pending_runtime_config() const;

    /// 将云端 shared attributes patch 应用到 runtime(active/pending) 配置语义
    SharedAttrApplyResult apply_runtime_patch(const rapidjson::Value& attrs,
                                              SchedulerService* scheduler = nullptr);

    /// 按 RuntimeConfig 结构体形式写入 pending 配置
    bool save_pending_runtime_config(const RuntimeConfig& pending) const;

    /// 将 pending runtime 提升为 active runtime 并清除 pending 文件
    bool promote_pending_runtime_to_active();

    /// 计算 runtime 配置版本号，用于业务 telemetry 最小真相
    uint64_t runtime_config_version(const RuntimeConfig& config) const;

    /// 将当前 active runtime 的调度窗口同步到调度器。
    void apply_active_runtime_schedules(SchedulerService& scheduler) const;

    /// 清除待生效配置
    bool clear_pending() const;

    /// 最近一次 load() 是否是从 backup 回退成功
    bool last_load_used_backup() const;

    /// 获取 JSON 子树
    rapidjson::Document get_subtree(const std::string& path) const;

    bool is_loaded() const;

    const std::string& runtime_path() const noexcept { return config_path_; }
    const std::string& fixed_path() const noexcept { return fixed_path_; }
    const std::string& pending_path() const noexcept { return pending_path_; }

private:
    template <typename T>
    static std::optional<T> get_optional_from_document(const rapidjson::Value& root,
                                                       const std::string& path)
    {
        try {
            auto parts = split_path(path);
            const rapidjson::Value* node = &root;
            for (auto& p : parts) {
                if (!node->IsObject()) {
                    return std::nullopt;
                }
                auto it = node->FindMember(p.c_str());
                if (it == node->MemberEnd()) {
                    return std::nullopt;
                }
                node = &it->value;
            }

            if constexpr (std::is_same_v<T, std::string>) {
                if (node->IsString()) {
                    return std::string(node->GetString());
                }
            } else if constexpr (std::is_same_v<T, bool>) {
                if (node->IsBool()) {
                    return node->GetBool();
                }
            } else if constexpr (std::is_integral_v<T>) {
                if (node->IsInt64()) {
                    return static_cast<T>(node->GetInt64());
                }
                if (node->IsUint64()) {
                    return static_cast<T>(node->GetUint64());
                }
                if (node->IsNumber()) {
                    return static_cast<T>(node->GetDouble());
                }
            } else if constexpr (std::is_floating_point_v<T>) {
                if (node->IsNumber()) {
                    return static_cast<T>(node->GetDouble());
                }
            }
        } catch (...) {
        }
        return std::nullopt;
    }

    static std::vector<std::string> split_path(const std::string& path);
    static Endpoint parse_endpoint_string(const std::string& value);
    static RuntimeConfig parse_runtime_config(const rapidjson::Value& root);
    static std::vector<RuntimeScheduleEntry> parse_schedule_entries(
        const rapidjson::Value& schedules_json);
    static void validate_runtime_config(const RuntimeConfig& cfg);
    static void apply_schedule_json(rapidjson::Document& root,
                                    const rapidjson::Value& schedules_json);
    static rapidjson::Document runtime_config_to_pending_root(const RuntimeConfig& config);
    static std::string derive_companion_path(const std::string& active_path, const char* suffix);
    static std::string derive_fixed_path(const std::string& runtime_path);
    std::string backup_path() const;
    static bool load_json_file_into(const std::string& path, rapidjson::Document* out);
    static bool write_json_file(const std::string& path, const rapidjson::Value& root);
    static std::optional<rapidjson::Document> read_json_file(const std::string& path);
    static rapidjson::Document clone_document(const rapidjson::Value& root);
    bool save_locked() const;

    std::string       config_path_;
    std::string       fixed_path_;
    std::string       pending_path_;
    rapidjson::Document root_;
    rapidjson::Document fixed_root_;
    bool              loaded_{false};
    bool              fixed_loaded_{false};
    bool              last_load_used_backup_{false};
    mutable std::mutex mtx_;
};

} // namespace robot::service
