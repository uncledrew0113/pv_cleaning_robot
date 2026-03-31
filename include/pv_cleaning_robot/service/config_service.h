#pragma once
#include <nlohmann/json.hpp>
#include <mutex>
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

    /// 获取 JSON 子树
    nlohmann::json get_subtree(const std::string& path) const;

    bool is_loaded() const;

private:
    static std::vector<std::string> split_path(const std::string& path);

    std::string       config_path_;
    nlohmann::json    root_;
    bool              loaded_{false};
    mutable std::mutex mtx_;
};

} // namespace robot::service
