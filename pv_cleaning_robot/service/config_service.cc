#include "pv_cleaning_robot/service/config_service.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <rapidjson/error/en.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "pv_cleaning_robot/service/scheduler_service.h"

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

bool is_supported_runtime_patch_field(const std::string& key)
{
    return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
           key == "brush_rpm" || key == "return_brush_rpm" || key == "parking_side" ||
           key == "start_battery_soc" || key == "charge_start_soc" || key == "charge_stop_soc" ||
           key == "schedules";
}

bool is_integer_passes(double value)
{
    return std::isfinite(value) && value > 0.0 && std::floor(value) == value;
}

rapidjson::Value* ensure_object_member(rapidjson::Value& parent,
                                       const char* key,
                                       rapidjson::Document::AllocatorType& alloc)
{
    if (!parent.IsObject()) {
        parent.SetObject();
    }
    auto it = parent.FindMember(key);
    if (it == parent.MemberEnd()) {
        rapidjson::Value name(key, alloc);
        rapidjson::Value child(rapidjson::kObjectType);
        parent.AddMember(name, child, alloc);
        it = parent.FindMember(key);
    } else if (!it->value.IsObject()) {
        it->value.SetObject();
    }
    return &it->value;
}

void set_double_member(rapidjson::Value& parent,
                       const char* key,
                       double value,
                       rapidjson::Document::AllocatorType& alloc)
{
    auto it = parent.FindMember(key);
    if (it == parent.MemberEnd()) {
        rapidjson::Value name(key, alloc);
        parent.AddMember(name, rapidjson::Value(value), alloc);
    } else {
        it->value.SetDouble(value);
    }
}

void set_int_member(rapidjson::Value& parent,
                    const char* key,
                    int value,
                    rapidjson::Document::AllocatorType& alloc)
{
    auto it = parent.FindMember(key);
    if (it == parent.MemberEnd()) {
        rapidjson::Value name(key, alloc);
        parent.AddMember(name, rapidjson::Value(value), alloc);
    } else {
        it->value.SetInt(value);
    }
}

void set_string_member(rapidjson::Value& parent,
                       const char* key,
                       const char* value,
                       rapidjson::Document::AllocatorType& alloc)
{
    auto it = parent.FindMember(key);
    if (it == parent.MemberEnd()) {
        rapidjson::Value name(key, alloc);
        rapidjson::Value str(value, alloc);
        parent.AddMember(name, str, alloc);
    } else {
        it->value.SetString(value, alloc);
    }
}

void merge_object_members(rapidjson::Value& dst,
                          const rapidjson::Value& src,
                          rapidjson::Document::AllocatorType& alloc)
{
    if (!src.IsObject()) {
        return;
    }
    if (!dst.IsObject()) {
        dst.SetObject();
    }

    for (auto it = src.MemberBegin(); it != src.MemberEnd(); ++it) {
        auto dst_it = dst.FindMember(it->name.GetString());
        if (dst_it == dst.MemberEnd()) {
            rapidjson::Value key(it->name.GetString(), alloc);
            rapidjson::Value value;
            value.CopyFrom(it->value, alloc);
            dst.AddMember(key, value, alloc);
        } else if (dst_it->value.IsObject() && it->value.IsObject()) {
            merge_object_members(dst_it->value, it->value, alloc);
        } else {
            dst_it->value.CopyFrom(it->value, alloc);
        }
    }
}

rapidjson::Document merge_runtime_root(const rapidjson::Value& active_root,
                                       const rapidjson::Value& pending_patch)
{
    rapidjson::Document merged;
    merged.CopyFrom(active_root, merged.GetAllocator());
    merge_object_members(merged, pending_patch, merged.GetAllocator());
    return merged;
}

}  // namespace

ConfigService::ConfigService(std::string config_path, std::string fixed_path)
    : config_path_(std::move(config_path))
    , fixed_path_(fixed_path.empty() ? derive_fixed_path(config_path_) : std::move(fixed_path))
    , pending_path_(derive_companion_path(config_path_, "pending"))
{
    root_.SetObject();
    fixed_root_.SetObject();
}

bool ConfigService::load()
{
    std::lock_guard<std::mutex> lk(mtx_);
    last_load_used_backup_ = false;
    fixed_loaded_ = load_json_file_into(fixed_path_, &fixed_root_);

    if (load_json_file_into(config_path_, &root_)) {
        loaded_ = true;
        return true;
    }

    if (load_json_file_into(backup_path(), &root_)) {
        loaded_ = true;
        last_load_used_backup_ = true;
        return true;
    }

    root_.SetObject();
    loaded_ = false;
    return false;
}

bool ConfigService::load_fixed()
{
    std::lock_guard<std::mutex> lk(mtx_);
    fixed_loaded_ = load_json_file_into(fixed_path_, &fixed_root_);
    return fixed_loaded_;
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

ParkingSide ConfigService::parse_parking_side_string(const std::string& value)
{
    if (value == parking_side_config_string(ParkingSide::Left)) {
        return ParkingSide::Left;
    }
    if (value == parking_side_config_string(ParkingSide::Right)) {
        return ParkingSide::Right;
    }
    throw std::runtime_error("parking_side must be left or right");
}

RuntimeConfig ConfigService::parse_runtime_config(const rapidjson::Value& root)
{
    RuntimeConfig cfg;

    if (root.IsObject()) {
        if (const auto robot_it = root.FindMember("robot");
            robot_it != root.MemberEnd() && robot_it->value.IsObject()) {
            const auto& robot = robot_it->value;
            if (const auto it = robot.FindMember("passes");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.passes = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("clean_speed_rpm");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.clean_speed_rpm = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("return_speed_rpm");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.return_speed_rpm = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("brush_rpm");
                it != robot.MemberEnd() && it->value.IsInt()) {
                cfg.brush_rpm = it->value.GetInt();
            }
            if (const auto it = robot.FindMember("return_brush_rpm");
                it != robot.MemberEnd() && it->value.IsInt()) {
                cfg.return_brush_rpm = it->value.GetInt();
            }
            if (const auto it = robot.FindMember("parking_side");
                it != robot.MemberEnd() && it->value.IsString()) {
                cfg.parking_side = parse_parking_side_string(it->value.GetString());
            }
            if (const auto it = robot.FindMember("start_battery_soc");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.start_battery_soc = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("charge_start_soc");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.charge_start_soc = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("charge_stop_soc");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.charge_stop_soc = it->value.GetDouble();
            }
        }

        if (const auto scheduler_it = root.FindMember("scheduler");
            scheduler_it != root.MemberEnd() && scheduler_it->value.IsObject()) {
            if (const auto windows_it = scheduler_it->value.FindMember("windows");
                windows_it != scheduler_it->value.MemberEnd()) {
                cfg.schedules = parse_schedule_entries(windows_it->value);
            }
        }
    }

    validate_runtime_config(cfg);
    return cfg;
}

std::vector<RuntimeScheduleEntry> ConfigService::parse_schedule_entries(
    const rapidjson::Value& schedules_json)
{
    if (!schedules_json.IsArray()) {
        throw std::runtime_error("schedules must be an array");
    }

    std::set<std::pair<int, int>> seen;
    std::vector<RuntimeScheduleEntry> out;
    for (const auto& item : schedules_json.GetArray()) {
        if (!item.IsObject()) {
            throw std::runtime_error("schedule entry must be an object");
        }
        const auto hour_it = item.FindMember("hour");
        const auto minute_it = item.FindMember("minute");
        if (hour_it == item.MemberEnd() || minute_it == item.MemberEnd() ||
            !hour_it->value.IsInt() || !minute_it->value.IsInt()) {
            throw std::runtime_error("schedule hour/minute must be integers");
        }
        const int hour = hour_it->value.GetInt();
        const int minute = minute_it->value.GetInt();
        if (hour < 0 || hour > 23 || minute < 0 || minute > 59) {
            throw std::runtime_error("schedule hour/minute out of range");
        }
        if (!seen.insert({hour, minute}).second) {
            throw std::runtime_error("duplicate schedule entry");
        }
        out.push_back(RuntimeScheduleEntry{hour, minute});
    }
    return out;
}

void ConfigService::validate_runtime_config(const RuntimeConfig& cfg)
{
    if (!is_integer_passes(cfg.passes)) {
        throw std::runtime_error("passes must be a positive integer");
    }
    const auto valid_soc = [](double value) {
        return std::isfinite(value) && value >= 0.0 && value <= 100.0;
    };
    if (!valid_soc(cfg.start_battery_soc)) {
        throw std::runtime_error("start_battery_soc must be within [0,100]");
    }
    if (!valid_soc(cfg.charge_start_soc)) {
        throw std::runtime_error("charge_start_soc must be within [0,100]");
    }
    if (!valid_soc(cfg.charge_stop_soc)) {
        throw std::runtime_error("charge_stop_soc must be within [0,100]");
    }
    if (!(cfg.charge_start_soc < cfg.charge_stop_soc)) {
        throw std::runtime_error("charge_start_soc must be less than charge_stop_soc");
    }
}

void ConfigService::apply_schedule_json(rapidjson::Document& root,
                                        const rapidjson::Value& schedules_json)
{
    auto* scheduler = ensure_object_member(root, "scheduler", root.GetAllocator());
    rapidjson::Value windows(rapidjson::kArrayType);
    for (const auto& item : schedules_json.GetArray()) {
        rapidjson::Value entry(rapidjson::kObjectType);
        entry.AddMember("hour", item.FindMember("hour")->value.GetInt(), root.GetAllocator());
        entry.AddMember("minute", item.FindMember("minute")->value.GetInt(), root.GetAllocator());
        windows.PushBack(entry, root.GetAllocator());
    }

    auto it = scheduler->FindMember("windows");
    if (it == scheduler->MemberEnd()) {
        rapidjson::Value key("windows", root.GetAllocator());
        scheduler->AddMember(key, windows, root.GetAllocator());
    } else {
        it->value = std::move(windows);
    }
}

rapidjson::Document ConfigService::runtime_config_to_pending_root(const RuntimeConfig& config)
{
    rapidjson::Document root;
    root.SetObject();
    auto* robot = ensure_object_member(root, "robot", root.GetAllocator());
    set_double_member(*robot, "passes", config.passes, root.GetAllocator());
    set_double_member(*robot, "clean_speed_rpm", config.clean_speed_rpm, root.GetAllocator());
    set_double_member(*robot, "return_speed_rpm", config.return_speed_rpm, root.GetAllocator());
    set_int_member(*robot, "brush_rpm", config.brush_rpm, root.GetAllocator());
    set_int_member(*robot, "return_brush_rpm", config.return_brush_rpm, root.GetAllocator());
    set_string_member(*robot,
                      "parking_side",
                      parking_side_config_string(config.parking_side),
                      root.GetAllocator());
    set_double_member(*robot, "start_battery_soc", config.start_battery_soc, root.GetAllocator());
    set_double_member(*robot, "charge_start_soc", config.charge_start_soc, root.GetAllocator());
    set_double_member(*robot, "charge_stop_soc", config.charge_stop_soc, root.GetAllocator());

    rapidjson::Value windows(rapidjson::kArrayType);
    for (const auto& schedule : config.schedules) {
        rapidjson::Value entry(rapidjson::kObjectType);
        entry.AddMember("hour", schedule.hour, root.GetAllocator());
        entry.AddMember("minute", schedule.minute, root.GetAllocator());
        windows.PushBack(entry, root.GetAllocator());
    }
    auto* scheduler = ensure_object_member(root, "scheduler", root.GetAllocator());
    rapidjson::Value key("windows", root.GetAllocator());
    scheduler->AddMember(key, windows, root.GetAllocator());
    return root;
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

    if (old_loaded && !write_json_file(backup_path(), old_root)) {
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
    return write_json_file(pending_path_, pending_root);
}

std::optional<rapidjson::Document> ConfigService::load_pending() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return read_json_file(pending_path_);
}

RuntimeConfig ConfigService::active_runtime_config() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return parse_runtime_config(root_);
}

std::optional<RuntimeConfig> ConfigService::pending_runtime_config() const
{
    const auto pending_root = load_pending();
    if (!pending_root) {
        return std::nullopt;
    }
    return parse_runtime_config(merge_runtime_root(snapshot(), *pending_root));
}

bool ConfigService::has_pending_runtime_config() const
{
    return load_pending().has_value();
}

SharedAttrApplyResult ConfigService::apply_runtime_patch(const rapidjson::Value& attrs,
                                                         SchedulerService* scheduler)
{
    if (!attrs.IsObject()) {
        return {false, "attributes_must_be_object"};
    }

    const auto active_root_before = snapshot();
    const auto pending_root_before = load_pending();
    auto active_root_after = clone_document(active_root_before);
    auto pending_root_after =
        pending_root_before ? clone_document(*pending_root_before) : rapidjson::Document{};
    if (!pending_root_after.IsObject()) {
        pending_root_after.SetObject();
    }

    bool touches_active = false;
    bool touches_pending = false;
    bool touched_supported = false;

    for (auto it = attrs.MemberBegin(); it != attrs.MemberEnd(); ++it) {
        if (is_supported_runtime_patch_field(it->name.GetString())) {
            touched_supported = true;
            break;
        }
    }
    if (!touched_supported) {
        return {false, "no_supported_fields"};
    }

    try {
        if (const auto it = attrs.FindMember("schedules"); it != attrs.MemberEnd()) {
            parse_schedule_entries(it->value);
            apply_schedule_json(active_root_after, it->value);
            touches_active = true;
        }

        auto* robot =
            ensure_object_member(pending_root_after, "robot", pending_root_after.GetAllocator());

        if (const auto it = attrs.FindMember("passes"); it != attrs.MemberEnd()) {
            if (!it->value.IsNumber() || !is_integer_passes(it->value.GetDouble())) {
                throw std::runtime_error("passes must be a positive integer");
            }
            set_double_member(*robot, "passes", it->value.GetDouble(), pending_root_after.GetAllocator());
            touches_pending = true;
        }

        const auto apply_positive_number = [&](const char* key, const char* message) {
            const auto it = attrs.FindMember(key);
            if (it == attrs.MemberEnd()) {
                return;
            }
            if (!it->value.IsNumber()) {
                throw std::runtime_error(message);
            }
            const double value = it->value.GetDouble();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error(message);
            }
            set_double_member(*robot, key, value, pending_root_after.GetAllocator());
            touches_pending = true;
        };
        apply_positive_number("clean_speed_rpm", "clean_speed_rpm must be a positive finite number");
        apply_positive_number("return_speed_rpm", "return_speed_rpm must be a positive finite number");

        const auto apply_positive_int = [&](const char* key, const char* message) {
            const auto it = attrs.FindMember(key);
            if (it == attrs.MemberEnd()) {
                return;
            }
            if (!it->value.IsInt()) {
                throw std::runtime_error(message);
            }
            const int value = it->value.GetInt();
            if (value <= 0) {
                throw std::runtime_error(message);
            }
            set_int_member(*robot, key, value, pending_root_after.GetAllocator());
            touches_pending = true;
        };
        apply_positive_int("brush_rpm", "brush_rpm must be > 0");
        apply_positive_int("return_brush_rpm", "return_brush_rpm must be > 0");

        if (const auto it = attrs.FindMember("parking_side"); it != attrs.MemberEnd()) {
            if (!it->value.IsString()) {
                throw std::runtime_error("parking_side must be left or right");
            }
            const auto side = parse_parking_side_string(it->value.GetString());
            set_string_member(*robot,
                              "parking_side",
                              parking_side_config_string(side),
                              pending_root_after.GetAllocator());
            touches_pending = true;
        }

        const auto apply_soc = [&](const char* key, const char* message) {
            const auto it = attrs.FindMember(key);
            if (it == attrs.MemberEnd()) {
                return;
            }
            if (!it->value.IsNumber()) {
                throw std::runtime_error(message);
            }
            const double value = it->value.GetDouble();
            if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
                throw std::runtime_error(message);
            }
            set_double_member(*robot, key, value, pending_root_after.GetAllocator());
            touches_pending = true;
        };
        apply_soc("start_battery_soc", "start_battery_soc must be within [0,100]");
        apply_soc("charge_start_soc", "charge_start_soc must be within [0,100]");
        apply_soc("charge_stop_soc", "charge_stop_soc must be within [0,100]");

        if (touches_pending) {
            validate_runtime_config(
                parse_runtime_config(merge_runtime_root(active_root_after, pending_root_after)));
        }
        if (touches_active) {
            validate_runtime_config(parse_runtime_config(active_root_after));
        }
    } catch (const std::exception& ex) {
        return {false, ex.what()};
    }

    if (touches_pending && !save_pending(pending_root_after)) {
        return {false, "persist_pending_failed"};
    }
    if (touches_active && !replace_and_save(active_root_after)) {
        if (pending_root_before) {
            save_pending(*pending_root_before);
        } else if (touches_pending) {
            clear_pending();
        }
        return {false, "persist_active_failed"};
    }
    if (touches_active && scheduler != nullptr) {
        apply_active_runtime_schedules(*scheduler);
    }
    return {true, {}};
}

bool ConfigService::save_pending_runtime_config(const RuntimeConfig& pending) const
{
    validate_runtime_config(pending);
    const auto root = runtime_config_to_pending_root(pending);
    return save_pending(root);
}

bool ConfigService::promote_pending_runtime_to_active()
{
    const auto pending_root = load_pending();
    if (!pending_root) {
        return true;
    }

    const auto merged_root = merge_runtime_root(snapshot(), *pending_root);
    if (!clear_pending()) {
        return false;
    }
    if (!replace_and_save(merged_root)) {
        save_pending(*pending_root);
        return false;
    }
    return true;
}

uint64_t ConfigService::runtime_config_version(const RuntimeConfig& config) const
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("passes");
    writer.Double(config.passes);
    writer.Key("clean_speed_rpm");
    writer.Double(config.clean_speed_rpm);
    writer.Key("return_speed_rpm");
    writer.Double(config.return_speed_rpm);
    writer.Key("brush_rpm");
    writer.Int(config.brush_rpm);
    writer.Key("return_brush_rpm");
    writer.Int(config.return_brush_rpm);
    writer.Key("parking_side");
    writer.String(parking_side_config_string(config.parking_side));
    writer.Key("start_battery_soc");
    writer.Double(config.start_battery_soc);
    writer.Key("charge_start_soc");
    writer.Double(config.charge_start_soc);
    writer.Key("charge_stop_soc");
    writer.Double(config.charge_stop_soc);
    writer.Key("schedules");
    writer.StartArray();
    for (const auto& entry : config.schedules) {
        writer.StartObject();
        writer.Key("hour");
        writer.Int(entry.hour);
        writer.Key("minute");
        writer.Int(entry.minute);
        writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();

    const std::string text(buffer.GetString(), buffer.GetSize());
    return std::hash<std::string>{}(text);
}

void ConfigService::apply_active_runtime_schedules(SchedulerService& scheduler) const
{
    const auto runtime_cfg = active_runtime_config();
    scheduler.clear_windows();
    for (const auto& schedule : runtime_cfg.schedules) {
        scheduler.add_window(SchedulerService::TimeWindow{schedule.hour, schedule.minute});
    }
}

bool ConfigService::clear_pending() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    std::error_code ec;
    std::filesystem::remove(pending_path_, ec);
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

std::string ConfigService::derive_fixed_path(const std::string& runtime_path)
{
    const std::string runtime_marker = ".runtime.json";
    if (runtime_path.size() >= runtime_marker.size() &&
        runtime_path.compare(runtime_path.size() - runtime_marker.size(),
                             runtime_marker.size(),
                             runtime_marker) == 0) {
        return runtime_path.substr(0, runtime_path.size() - runtime_marker.size()) + ".fixed.json";
    }

    const auto dot = runtime_path.rfind('.');
    if (dot == std::string::npos) {
        return runtime_path + ".fixed";
    }
    return runtime_path.substr(0, dot) + ".fixed" + runtime_path.substr(dot);
}

std::string ConfigService::backup_path() const
{
    return derive_companion_path(config_path_, "backup");
}

bool ConfigService::load_json_file_into(const std::string& path, rapidjson::Document* out)
{
    if (!out || path.empty()) {
        if (out) {
            out->SetObject();
        }
        return false;
    }

    if (auto root = read_json_file(path)) {
        out->Swap(*root);
        return true;
    }

    out->SetObject();
    return false;
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
