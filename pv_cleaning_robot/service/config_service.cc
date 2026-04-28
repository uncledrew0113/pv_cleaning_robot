#include "pv_cleaning_robot/service/config_service.h"
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>

namespace robot::service {

ConfigService::ConfigService(std::string config_path)
    : config_path_(std::move(config_path))
{
}

bool ConfigService::load()
{
    std::lock_guard<std::mutex> lk(mtx_);
    last_load_used_backup_ = false;

    if (auto main_root = read_json_file(config_path_)) {
        root_ = std::move(*main_root);
        loaded_ = true;
        return true;
    }

    const auto backup_path = derive_companion_path(config_path_, "backup");
    if (auto backup_root = read_json_file(backup_path)) {
        root_ = std::move(*backup_root);
        loaded_ = true;
        last_load_used_backup_ = true;
        return true;
    }

    loaded_ = false;
    return false;
}

nlohmann::json ConfigService::get_subtree(const std::string& path) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    try {
        auto parts = split_path(path);
        const nlohmann::json* node = &root_;
        for (auto& p : parts) {
            node = &node->at(p);
        }
        return *node;
    } catch (...) {
        return nlohmann::json{};
    }
}

nlohmann::json ConfigService::snapshot() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return root_;
}

bool ConfigService::is_loaded() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return loaded_;
}

std::vector<std::string> ConfigService::split_path(const std::string& path)
{
    std::vector<std::string> parts;
    std::istringstream ss(path);
    std::string token;
    while (std::getline(ss, token, '.')) {
        if (!token.empty()) parts.push_back(token);
    }
    return parts;
}

bool ConfigService::save() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return save_locked();
}

bool ConfigService::replace_and_save(nlohmann::json new_root)
{
    std::lock_guard<std::mutex> lk(mtx_);
    auto old_root = root_;
    const bool old_loaded = loaded_;
    const auto backup_path = derive_companion_path(config_path_, "backup");

    if (old_loaded && !write_json_file(backup_path, old_root))
        return false;

    root_ = std::move(new_root);
    loaded_ = true;

    if (save_locked())
        return true;

    root_ = std::move(old_root);
    loaded_ = old_loaded;
    return false;
}

bool ConfigService::save_pending(nlohmann::json pending_root) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return write_json_file(derive_companion_path(config_path_, "pending"), pending_root);
}

std::optional<nlohmann::json> ConfigService::load_pending() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return read_json_file(derive_companion_path(config_path_, "pending"));
}

bool ConfigService::clear_pending() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::error_code ec;
    std::filesystem::remove(derive_companion_path(config_path_, "pending"), ec);
    return !ec;
}

bool ConfigService::last_load_used_backup() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return last_load_used_backup_;
}

std::string ConfigService::derive_companion_path(const std::string& active_path, const char* suffix)
{
    const auto dot = active_path.rfind('.');
    if (dot == std::string::npos)
        return active_path + "." + suffix;
    return active_path.substr(0, dot) + "." + suffix + active_path.substr(dot);
}

bool ConfigService::write_json_file(const std::string& path, const nlohmann::json& root)
{
    const std::string tmp_path = path + ".tmp";
    try {
        std::ofstream ofs(tmp_path);
        if (!ofs.is_open()) return false;
        ofs << root.dump(2);
        ofs.close();
        std::filesystem::rename(tmp_path, path);
        return true;
    } catch (...) {
        std::filesystem::remove(tmp_path);
        return false;
    }
}

std::optional<nlohmann::json> ConfigService::read_json_file(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
        return std::nullopt;

    try {
        nlohmann::json root;
        ifs >> root;
        return root;
    } catch (...) {
        return std::nullopt;
    }
}

bool ConfigService::save_locked() const
{
    return write_json_file(config_path_, root_);
}

} // namespace robot::service
