#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

#include <cmath>
#include <set>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace robot::service {

namespace {

bool is_supported_field(const std::string& key)
{
    return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
           key == "brush_rpm" || key == "parking_policy" || key == "charging_side" ||
           key == "schedules";
}

bool is_integer_passes(double value)
{
    return std::isfinite(value) && value > 0.0 && std::floor(value) == value;
}

ParkingPolicy parse_parking_policy_string(const std::string& value)
{
    if (value == parking_policy_config_string(ParkingPolicy::TerminalAOnly)) {
        return ParkingPolicy::TerminalAOnly;
    }
    if (value == parking_policy_config_string(ParkingPolicy::TerminalBOnly)) {
        return ParkingPolicy::TerminalBOnly;
    }
    if (value == parking_policy_config_string(ParkingPolicy::Both)) {
        return ParkingPolicy::Both;
    }
    throw std::runtime_error("parking_policy must be terminal_a_only, terminal_b_only, or both");
}

ChargingSide parse_charging_side_string(const std::string& value)
{
    if (value == charging_side_config_string(ChargingSide::TerminalA)) {
        return ChargingSide::TerminalA;
    }
    if (value == charging_side_config_string(ChargingSide::TerminalB)) {
        return ChargingSide::TerminalB;
    }
    if (value == charging_side_config_string(ChargingSide::Both)) {
        return ChargingSide::Both;
    }
    throw std::runtime_error("charging_side must be terminal_a, terminal_b, or both");
}

void validate_release_runtime_config(const TbRuntimeConfig& cfg)
{
    // 当前 release 明确只支持整数趟和单侧停车。
    if (!is_integer_passes(cfg.passes)) {
        throw std::runtime_error("passes must be a positive integer in this release");
    }
    if (cfg.parking_policy == ParkingPolicy::Both) {
        throw std::runtime_error("parking_policy=both is not supported in this release");
    }
}

rapidjson::Document clone_document(const rapidjson::Value& value)
{
    rapidjson::Document out;
    out.CopyFrom(value, out.GetAllocator());
    return out;
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

}  // namespace

ThingsBoardConfigManager::ThingsBoardConfigManager(ConfigService& config,
                                                   SchedulerService& scheduler)
    : config_(config)
    , scheduler_(scheduler)
    , active_(parse_runtime_config(config.snapshot()))
{
    if (const auto pending_root = config_.load_pending()) {
        pending_ = parse_runtime_config(*pending_root);
    }
}

SharedAttrApplyResult ThingsBoardConfigManager::apply_shared_attributes(
    const rapidjson::Value& attrs)
{
    if (!attrs.IsObject()) {
        return {false, "attributes_must_be_object"};
    }

    const auto active_root_before = config_.snapshot();
    const auto pending_root_before = config_.load_pending();
    auto active_root_after = clone_document(active_root_before);
    auto pending_root_after = pending_root_before ? clone_document(*pending_root_before)
                                                  : clone_document(active_root_before);

    bool touches_active = false;
    bool touches_pending = false;
    bool touched_supported = false;

    for (auto it = attrs.MemberBegin(); it != attrs.MemberEnd(); ++it) {
        const std::string key = it->name.GetString();
        if (!is_supported_field(key)) {
            spdlog::warn("[TBConfig] 忽略不支持的字段: {}", key);
            continue;
        }
        touched_supported = true;
    }

    if (!touched_supported) {
        return {false, "no_supported_fields"};
    }

    try {
        // schedules 立即影响调度器，因此同时写 active 和 pending。
        if (const auto it = attrs.FindMember("schedules"); it != attrs.MemberEnd()) {
            const auto& schedules_json = it->value;
            parse_schedule_entries(schedules_json);
            apply_schedule_json(active_root_after, schedules_json);
            apply_schedule_json(pending_root_after, schedules_json);
            touches_active = true;
            touches_pending = touches_pending || pending_root_before.has_value();
        }

        // 任务相关参数只写 pending。这样不会在运行中途直接改变当前任务语义，
        // 而是等下一次任务启动前由 promote_pending_to_active() 提升。
        auto* robot = ensure_object_member(
            pending_root_after, "robot", pending_root_after.GetAllocator());

        if (const auto it = attrs.FindMember("passes"); it != attrs.MemberEnd()) {
            if (!it->value.IsNumber()) {
                throw std::runtime_error("passes must be a positive integer in this release");
            }
            const double value = it->value.GetDouble();
            if (!is_integer_passes(value)) {
                throw std::runtime_error("passes must be a positive integer in this release");
            }
            set_double_member(*robot, "passes", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("clean_speed_rpm"); it != attrs.MemberEnd()) {
            if (!it->value.IsNumber()) {
                throw std::runtime_error("clean_speed_rpm must be a positive finite number");
            }
            const double value = it->value.GetDouble();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error("clean_speed_rpm must be a positive finite number");
            }
            set_double_member(
                *robot, "clean_speed_rpm", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("return_speed_rpm"); it != attrs.MemberEnd()) {
            if (!it->value.IsNumber()) {
                throw std::runtime_error("return_speed_rpm must be a positive finite number");
            }
            const double value = it->value.GetDouble();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error("return_speed_rpm must be a positive finite number");
            }
            set_double_member(
                *robot, "return_speed_rpm", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("brush_rpm"); it != attrs.MemberEnd()) {
            if (!it->value.IsInt()) {
                throw std::runtime_error("brush_rpm must be > 0");
            }
            const int value = it->value.GetInt();
            if (value <= 0) {
                throw std::runtime_error("brush_rpm must be > 0");
            }
            set_int_member(*robot, "brush_rpm", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("parking_policy"); it != attrs.MemberEnd()) {
            if (!it->value.IsString()) {
                throw std::runtime_error(
                    "parking_policy must be terminal_a_only, terminal_b_only, or both");
            }
            const auto policy = parse_parking_policy_string(it->value.GetString());
            if (policy == ParkingPolicy::Both) {
                throw std::runtime_error("parking_policy=both is not supported in this release");
            }
            set_string_member(
                *robot,
                "parking_policy",
                parking_policy_config_string(policy),
                pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("charging_side"); it != attrs.MemberEnd()) {
            if (!it->value.IsString()) {
                throw std::runtime_error("charging_side must be terminal_a, terminal_b, or both");
            }
            const auto charging_side = parse_charging_side_string(it->value.GetString());
            set_string_member(
                *robot,
                "charging_side",
                charging_side_config_string(charging_side),
                pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (touches_pending) {
            validate_release_runtime_config(parse_runtime_config(pending_root_after));
        }
    } catch (const std::exception& ex) {
        spdlog::warn("[TBConfig] 拒绝共享属性更新: {}", ex.what());
        return {false, ex.what()};
    }

    if (touches_pending && !config_.save_pending(pending_root_after)) {
        return {false, "persist_pending_failed"};
    }

    if (touches_active && !config_.replace_and_save(active_root_after)) {
        if (pending_root_before) {
            config_.save_pending(*pending_root_before);
        } else if (touches_pending) {
            config_.clear_pending();
        }
        return {false, "persist_active_failed"};
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (touches_active) {
            active_ = parse_runtime_config(active_root_after);
        }
        if (touches_pending) {
            pending_ = parse_runtime_config(pending_root_after);
        }
    }

    if (touches_active) {
        apply_scheduler_windows(parse_runtime_config(active_root_after).schedules);
    }

    return {true, {}};
}

bool ThingsBoardConfigManager::promote_pending_to_active()
{
    const auto pending_root = config_.load_pending();
    if (!pending_root) {
        return true;
    }

    // 提升顺序固定为：
    // 1. 清 pending 文件
    // 2. 用 pending 覆盖 active
    // 3. 刷新 scheduler
    // 若中途失败，尽量把 pending 写回，避免配置真相丢失。
    if (!config_.clear_pending()) {
        return false;
    }
    if (!config_.replace_and_save(*pending_root)) {
        config_.save_pending(*pending_root);
        return false;
    }

    const auto new_active = parse_runtime_config(*pending_root);
    apply_scheduler_windows(new_active.schedules);

    {
        std::lock_guard<std::mutex> lk(mtx_);
        active_ = new_active;
        pending_.reset();
    }
    return true;
}

TbRuntimeConfig ThingsBoardConfigManager::active_config() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return active_;
}

std::optional<TbRuntimeConfig> ThingsBoardConfigManager::pending_config() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_;
}

bool ThingsBoardConfigManager::has_pending_config() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_.has_value();
}

TbRuntimeConfig ThingsBoardConfigManager::parse_runtime_config(const rapidjson::Value& root)
{
    TbRuntimeConfig cfg;

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
            if (const auto it = robot.FindMember("parking_policy");
                it != robot.MemberEnd() && it->value.IsString()) {
                cfg.parking_policy = parse_parking_policy_string(it->value.GetString());
            }
            if (const auto it = robot.FindMember("charging_side");
                it != robot.MemberEnd() && it->value.IsString()) {
                cfg.charging_side = parse_charging_side_string(it->value.GetString());
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

    validate_release_runtime_config(cfg);
    return cfg;
}

void ThingsBoardConfigManager::apply_schedule_json(rapidjson::Document& root,
                                                   const rapidjson::Value& schedules_json)
{
    auto* scheduler = ensure_object_member(root, "scheduler", root.GetAllocator());
    rapidjson::Value windows(rapidjson::kArrayType);
    for (const auto& w : schedules_json.GetArray()) {
        rapidjson::Value entry(rapidjson::kObjectType);
        entry.AddMember("hour", w.FindMember("hour")->value.GetInt(), root.GetAllocator());
        entry.AddMember("minute", w.FindMember("minute")->value.GetInt(), root.GetAllocator());
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

std::vector<TbScheduleEntry> ThingsBoardConfigManager::parse_schedule_entries(
    const rapidjson::Value& schedules_json)
{
    if (!schedules_json.IsArray()) {
        throw std::runtime_error("schedules must be an array");
    }

    std::set<std::pair<int, int>> seen;
    std::vector<TbScheduleEntry> out;
    for (const auto& w : schedules_json.GetArray()) {
        if (!w.IsObject()) {
            throw std::runtime_error("schedule entry must be an object");
        }

        const auto hour_it = w.FindMember("hour");
        const auto minute_it = w.FindMember("minute");
        if (hour_it == w.MemberEnd() || minute_it == w.MemberEnd() ||
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

        out.push_back(TbScheduleEntry{hour, minute});
    }

    return out;
}

void ThingsBoardConfigManager::apply_scheduler_windows(
    const std::vector<TbScheduleEntry>& schedules)
{
    scheduler_.clear_windows();
    for (const auto& schedule : schedules) {
        scheduler_.add_window(SchedulerService::TimeWindow{schedule.hour, schedule.minute});
    }
}

}  // namespace robot::service
