#include "pv_cleaning_robot/service/config_service.h"
#include <filesystem>
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

bool ConfigService::save() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    // 原子写：先写临时文件再 rename，防止写入中途断电损坏原文件
    const std::string tmp_path = config_path_ + ".tmp";
    try {
        std::ofstream ofs(tmp_path);
        if (!ofs.is_open()) return false;
        ofs << root_.dump(2);
        ofs.close();
        std::filesystem::rename(tmp_path, config_path_);
        return true;
    } catch (...) {
        std::filesystem::remove(tmp_path);
        return false;
    }
}

} // namespace robot::service
