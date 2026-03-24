#include "pv_cleaning_robot/service/config_service.h"
#include <fstream>
#include <sstream>

namespace robot::service {

ConfigService::ConfigService(std::string config_path)
    : config_path_(std::move(config_path))
{
}

bool ConfigService::load()
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::ifstream ifs(config_path_);
    if (!ifs.is_open()) return false;
    try {
        ifs >> root_;
        loaded_ = true;
        return true;
    } catch (...) {
        loaded_ = false;
        return false;
    }
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

} // namespace robot::service
