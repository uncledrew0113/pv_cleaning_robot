#pragma once

#include <mutex>
#include <optional>
#include <rapidjson/document.h>
#include <string>
#include <type_traits>
#include <vector>

namespace robot::service {

/// @brief 全局配置服务（RapidJSON，config.json 驱动）
///
/// 提供类型安全的嵌套路径访问：
///   cfg.get<std::string>("network.mqtt.broker_uri")
class ConfigService {
public:
    explicit ConfigService(std::string config_path);

    /// 加载/重新加载配置文件
    bool load();

    /// 获取配置项（path 以 '.' 分隔，例如 "network.mqtt.port"）
    template <typename T>
    T get(const std::string& path, const T& default_val = T{}) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        try {
            auto parts = split_path(path);
            const rapidjson::Value* node = &root_;
            for (auto& p : parts) {
                if (!node->IsObject()) {
                    return default_val;
                }
                auto it = node->FindMember(p.c_str());
                if (it == node->MemberEnd()) {
                    return default_val;
                }
                node = &it->value;
            }

            if constexpr (std::is_same_v<T, std::string>) {
                if (node->IsString()) {
                    return node->GetString();
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

    /// 清除待生效配置
    bool clear_pending() const;

    /// 最近一次 load() 是否是从 backup 回退成功
    bool last_load_used_backup() const;

    /// 获取 JSON 子树
    rapidjson::Document get_subtree(const std::string& path) const;

    bool is_loaded() const;

private:
    static std::vector<std::string> split_path(const std::string& path);
    static std::string derive_companion_path(const std::string& active_path, const char* suffix);
    static bool write_json_file(const std::string& path, const rapidjson::Value& root);
    static std::optional<rapidjson::Document> read_json_file(const std::string& path);
    static rapidjson::Document clone_document(const rapidjson::Value& root);
    bool save_locked() const;

    std::string       config_path_;
    rapidjson::Document root_;
    bool              loaded_{false};
    bool              last_load_used_backup_{false};
    mutable std::mutex mtx_;
};

} // namespace robot::service
