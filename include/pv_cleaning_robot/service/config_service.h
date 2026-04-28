#pragma once
#include <nlohmann/json.hpp>
#include <mutex>
#include <optional>
#include <string>

namespace robot::service {

/// @brief 全局配置服务（nlohmann/json，config.json 驱动）
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
            const nlohmann::json* node = &root_;
            for (auto& p : parts) {
                node = &node->at(p);
            }
            return node->get<T>();
        } catch (...) {
            return default_val;
        }
    }

    /// 设置配置项（path 以 '.' 分隔）—仅更新内存，需调用 save() 持久化
    template <typename T>
    void set(const std::string& path, T value)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto parts = split_path(path);
        nlohmann::json* node = &root_;
        for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
            node = &(*node)[parts[i]];
        }
        (*node)[parts.back()] = std::move(value);
    }

    /// 将当前内存配置写回 config_path_（原子临时文件 rename）
    bool save() const;

    /// 返回当前完整配置副本
    nlohmann::json snapshot() const;

    /// 用完整配置副本替换当前配置并持久化；若持久化失败则回滚旧值
    bool replace_and_save(nlohmann::json new_root);

    /// 持久化下一任务待生效配置
    bool save_pending(nlohmann::json pending_root) const;

    /// 读取待生效配置；不存在或解析失败时返回 nullopt
    std::optional<nlohmann::json> load_pending() const;

    /// 清除待生效配置
    bool clear_pending() const;

    /// 最近一次 load() 是否是从 backup 回退成功
    bool last_load_used_backup() const;

    /// 获取 JSON 子树
    nlohmann::json get_subtree(const std::string& path) const;

    bool is_loaded() const;

private:
    static std::vector<std::string> split_path(const std::string& path);
    static std::string derive_companion_path(const std::string& active_path, const char* suffix);
    static bool write_json_file(const std::string& path, const nlohmann::json& root);
    static std::optional<nlohmann::json> read_json_file(const std::string& path);
    bool save_locked() const;

    std::string       config_path_;
    nlohmann::json    root_;
    bool              loaded_{false};
    bool              last_load_used_backup_{false};
    mutable std::mutex mtx_;
};

} // namespace robot::service
