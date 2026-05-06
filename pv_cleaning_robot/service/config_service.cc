#include "pv_cleaning_robot/service/config_service.h"

#include <filesystem>
#include <fstream>
#include <optional>
#include <rapidjson/error/en.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <sstream>
#include <vector>

namespace robot::service {

namespace {

std::optional<rapidjson::Document> parse_json_text(const std::string& text)
{
    rapidjson::Document doc;
    doc.Parse(text.c_str());
    if (doc.HasParseError()) {
        return std::nullopt;
    }
    return doc;
}

const rapidjson::Value* find_path(const rapidjson::Value& root,
                                  const std::vector<std::string>& parts)
{
    const rapidjson::Value* node = &root;
    for (const auto& part : parts) {
        if (!node->IsObject()) {
            return nullptr;
        }
        auto it = node->FindMember(part.c_str());
        if (it == node->MemberEnd()) {
            return nullptr;
        }
        node = &it->value;
    }
    return node;
}

}  // namespace

ConfigService::ConfigService(std::string config_path)
    : config_path_(std::move(config_path))
{
    root_.SetObject();
}

bool ConfigService::load()
{
    std::lock_guard<std::mutex> lk(mtx_);
    last_load_used_backup_ = false;

    if (auto main_root = read_json_file(config_path_)) {
        root_.Swap(*main_root);
        loaded_ = true;
        return true;
    }

    const auto backup_path = derive_companion_path(config_path_, "backup");
    if (auto backup_root = read_json_file(backup_path)) {
        root_.Swap(*backup_root);
        loaded_ = true;
        last_load_used_backup_ = true;
        return true;
    }

    root_.SetObject();
    loaded_ = false;
    return false;
}

rapidjson::Document ConfigService::get_subtree(const std::string& path) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    auto out = rapidjson::Document();
    if (const auto* node = find_path(root_, split_path(path))) {
        out.CopyFrom(*node, out.GetAllocator());
    } else {
        out.SetObject();
    }
    return out;
}

rapidjson::Document ConfigService::snapshot() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return clone_document(root_);
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
        if (!token.empty()) {
            parts.push_back(token);
        }
    }
    return parts;
}

bool ConfigService::save() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return save_locked();
}

bool ConfigService::replace_and_save(const rapidjson::Value& new_root)
{
    std::lock_guard<std::mutex> lk(mtx_);
    auto old_root = clone_document(root_);
    const bool old_loaded = loaded_;
    const auto backup_path = derive_companion_path(config_path_, "backup");

    if (old_loaded && !write_json_file(backup_path, old_root)) {
        return false;
    }

    root_.CopyFrom(new_root, root_.GetAllocator());
    loaded_ = true;

    if (save_locked()) {
        return true;
    }

    root_.Swap(old_root);
    loaded_ = old_loaded;
    return false;
}

bool ConfigService::save_pending(const rapidjson::Value& pending_root) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return write_json_file(derive_companion_path(config_path_, "pending"), pending_root);
}

std::optional<rapidjson::Document> ConfigService::load_pending() const
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
    if (dot == std::string::npos) {
        return active_path + "." + suffix;
    }
    return active_path.substr(0, dot) + "." + suffix + active_path.substr(dot);
}

bool ConfigService::write_json_file(const std::string& path, const rapidjson::Value& root)
{
    const std::string tmp_path = path + ".tmp";
    try {
        rapidjson::StringBuffer buffer;
        std::error_code ec;
        const auto existing_size = std::filesystem::file_size(path, ec);
        if (!ec) {
            buffer.Reserve(static_cast<rapidjson::SizeType>(existing_size));
        } else {
            buffer.Reserve(4096);
        }
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        root.Accept(writer);

        std::ofstream ofs(tmp_path);
        if (!ofs.is_open()) {
            return false;
        }
        ofs << buffer.GetString();
        ofs.close();
        std::filesystem::rename(tmp_path, path);
        return true;
    } catch (...) {
        std::filesystem::remove(tmp_path);
        return false;
    }
}

std::optional<rapidjson::Document> ConfigService::read_json_file(const std::string& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        return std::nullopt;
    }

    std::string text;
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    if (!ec) {
        text.reserve(static_cast<size_t>(size));
    }

    text.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    return parse_json_text(text);
}

rapidjson::Document ConfigService::clone_document(const rapidjson::Value& root)
{
    rapidjson::Document out;
    out.CopyFrom(root, out.GetAllocator());
    return out;
}

bool ConfigService::save_locked() const
{
    return write_json_file(config_path_, root_);
}

}  // namespace robot::service
