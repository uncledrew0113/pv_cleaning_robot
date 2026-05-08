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
           key == "brush_rpm" || key == "return_brush_rpm" || key == "parking_side" ||
           key == "start_battery_soc" || key == "charge_start_soc" ||
           key == "charge_stop_soc" || key == "schedules";
}

bool is_integer_passes(double value)
{
    return std::isfinite(value) && value > 0.0 && std::floor(value) == value;
}

ParkingSide parse_parking_side_string(const std::string& value)
{
    if (value == parking_side_config_string(ParkingSide::Left)) {
        return ParkingSide::Left;
    }
    if (value == parking_side_config_string(ParkingSide::Right)) {
        return ParkingSide::Right;
    }
    throw std::runtime_error("parking_side must be left or right");
}

void validate_runtime_config(const TbRuntimeConfig& cfg)
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

rapidjson::Document clone_document(const rapidjson::Value& value)
{
    rapidjson::Document out;
    out.CopyFrom(value, out.GetAllocator());
    return out;
}

rapidjson::Document make_empty_root()
{
    rapidjson::Document out;
    out.SetObject();
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
    auto merged = clone_document(active_root);
    merge_object_members(merged, pending_patch, merged.GetAllocator());
    return merged;
}

}  // namespace

ThingsBoardConfigManager::ThingsBoardConfigManager(ConfigService& config,
                                                   SchedulerService& scheduler)
    : config_(config)
    , scheduler_(scheduler)
    , active_(parse_runtime_config(config.snapshot()))
{
    apply_scheduler_windows(active_.schedules);
    // pending 文件只保存“下次任务生效”的 patch。构造时把 active runtime 与
    // pending patch 合并成一份候选视图，便于上层直接看到下一次 start 会采用的配置。
    if (const auto pending_root = config_.load_pending()) {
        pending_ = parse_runtime_config(merge_runtime_root(config.snapshot(), *pending_root));
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
                                                  : make_empty_root();

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
        // schedules 是唯一立即生效的 runtime 字段；它只更新 active 与调度器，
        // 不进入 pending，避免“下一次任务启动又把旧窗口覆盖回来”。
        if (const auto it = attrs.FindMember("schedules"); it != attrs.MemberEnd()) {
            const auto& schedules_json = it->value;
            parse_schedule_entries(schedules_json);
            apply_schedule_json(active_root_after, schedules_json);
            touches_active = true;
        }

        // 其余 runtime 字段全部只写 pending patch。这样当前任务的速度、方向、
        // 趟数和充电阈值都不会在任务执行中途变化。
        auto* robot = ensure_object_member(
            pending_root_after, "robot", pending_root_after.GetAllocator());

        if (const auto it = attrs.FindMember("passes"); it != attrs.MemberEnd()) {
            if (!it->value.IsNumber()) {
                throw std::runtime_error("passes must be a positive integer");
            }
            const double value = it->value.GetDouble();
            if (!is_integer_passes(value)) {
                throw std::runtime_error("passes must be a positive integer");
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

        if (const auto it = attrs.FindMember("return_brush_rpm"); it != attrs.MemberEnd()) {
            if (!it->value.IsInt()) {
                throw std::runtime_error("return_brush_rpm must be > 0");
            }
            const int value = it->value.GetInt();
            if (value <= 0) {
                throw std::runtime_error("return_brush_rpm must be > 0");
            }
            set_int_member(
                *robot, "return_brush_rpm", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

        if (const auto it = attrs.FindMember("parking_side"); it != attrs.MemberEnd()) {
            if (!it->value.IsString()) {
                throw std::runtime_error("parking_side must be left or right");
            }
            const auto side = parse_parking_side_string(it->value.GetString());
            set_string_member(
                *robot,
                "parking_side",
                parking_side_config_string(side),
                pending_root_after.GetAllocator());
            touches_pending = true;
        }

        const auto apply_pending_soc = [&](const char* key, const char* error_message) {
            const auto it = attrs.FindMember(key);
            if (it == attrs.MemberEnd()) {
                return;
            }
            if (!it->value.IsNumber()) {
                throw std::runtime_error(error_message);
            }
            const double value = it->value.GetDouble();
            if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
                throw std::runtime_error(error_message);
            }
            set_double_member(*robot, key, value, pending_root_after.GetAllocator());
            touches_pending = true;
        };
        apply_pending_soc("start_battery_soc", "start_battery_soc must be within [0,100]");
        apply_pending_soc("charge_start_soc", "charge_start_soc must be within [0,100]");
        apply_pending_soc("charge_stop_soc", "charge_stop_soc must be within [0,100]");

        if (touches_pending) {
            validate_runtime_config(parse_runtime_config(
                merge_runtime_root(active_root_after, pending_root_after)));
        }
        if (touches_active) {
            validate_runtime_config(parse_runtime_config(active_root_after));
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
            pending_ = parse_runtime_config(merge_runtime_root(active_root_after, pending_root_after));
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

    const auto active_root = config_.snapshot();
    const auto merged_root = merge_runtime_root(active_root, *pending_root);

    // 提升顺序固定为：
    // 1. 清 pending 文件
    // 2. 用 pending patch 合并当前 active runtime
    // 3. 刷新 scheduler
    // 这个入口只应在“准备启动新任务”之前调用。
    // 若中途失败，尽量把 pending 写回，避免配置真相丢失。
    if (!config_.clear_pending()) {
        return false;
    }
    if (!config_.replace_and_save(merged_root)) {
        config_.save_pending(*pending_root);
        return false;
    }

    const auto new_active = parse_runtime_config(merged_root);
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
            } else if (const auto it = robot.FindMember("battery_low_soc");
                       it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.charge_start_soc = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("charge_stop_soc");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.charge_stop_soc = it->value.GetDouble();
            } else if (const auto it = robot.FindMember("battery_full_soc");
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
