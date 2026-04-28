#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

#include <cmath>
#include <optional>
#include <set>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace robot::service {

namespace {

bool is_supported_field(const std::string& key) {
    return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
           key == "brush_rpm" || key == "schedules";
}

}  // namespace

ThingsBoardConfigManager::ThingsBoardConfigManager(ConfigService& config, SchedulerService& scheduler)
    : config_(config)
    , scheduler_(scheduler)
    , active_(parse_runtime_config(config.snapshot())) {
    if (const auto pending_root = config_.load_pending()) {
        pending_ = parse_runtime_config(*pending_root);
    }
}

SharedAttrApplyResult ThingsBoardConfigManager::apply_shared_attributes(const nlohmann::json& attrs) {
    if (!attrs.is_object()) {
        return {false, "attributes_must_be_object"};
    }

    const auto active_root_before = config_.snapshot();
    const auto pending_root_before = config_.load_pending();
    auto active_root_after = active_root_before;
    auto pending_root_after = pending_root_before.value_or(active_root_before);

    bool touches_active = false;
    bool touches_pending = false;
    bool touched_supported = false;

    for (const auto& item : attrs.items()) {
        if (!is_supported_field(item.key())) {
            spdlog::warn("[TBConfig] 忽略不支持的字段: {}", item.key());
            continue;
        }
        touched_supported = true;
    }

    if (!touched_supported) {
        return {false, "no_supported_fields"};
    }

    try {
        if (attrs.contains("schedules")) {
            const auto& schedules_json = attrs.at("schedules");
            parse_schedule_entries(schedules_json);
            apply_schedule_json(active_root_after, schedules_json);
            apply_schedule_json(pending_root_after, schedules_json);
            touches_active = true;
            touches_pending = touches_pending || pending_root_before.has_value();
        }

        if (attrs.contains("passes")) {
            const double value = attrs.at("passes").get<double>();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error("passes must be a positive finite number");
            }
            pending_root_after["robot"]["passes"] = value;
            touches_pending = true;
        }

        if (attrs.contains("clean_speed_rpm")) {
            const double value = attrs.at("clean_speed_rpm").get<double>();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error("clean_speed_rpm must be a positive finite number");
            }
            pending_root_after["robot"]["clean_speed_rpm"] = value;
            touches_pending = true;
        }

        if (attrs.contains("return_speed_rpm")) {
            const double value = attrs.at("return_speed_rpm").get<double>();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error("return_speed_rpm must be a positive finite number");
            }
            pending_root_after["robot"]["return_speed_rpm"] = value;
            touches_pending = true;
        }

        if (attrs.contains("brush_rpm")) {
            const int value = attrs.at("brush_rpm").get<int>();
            if (value <= 0) {
                throw std::runtime_error("brush_rpm must be > 0");
            }
            pending_root_after["robot"]["brush_rpm"] = value;
            touches_pending = true;
        }
    } catch (const std::exception& ex) {
        spdlog::warn("[TBConfig] 拒绝共享属性更新: {}", ex.what());
        return {false, ex.what()};
    }

    if (touches_pending) {
        if (!config_.save_pending(pending_root_after)) {
            return {false, "persist_pending_failed"};
        }
    }

    if (touches_active) {
        if (!config_.replace_and_save(active_root_after)) {
            if (pending_root_before) {
                config_.save_pending(*pending_root_before);
            } else if (touches_pending) {
                config_.clear_pending();
            }
            return {false, "persist_active_failed"};
        }
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

bool ThingsBoardConfigManager::promote_pending_to_active() {
    const auto pending_root = config_.load_pending();
    if (!pending_root) {
        return true;
    }

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

TbRuntimeConfig ThingsBoardConfigManager::active_config() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return active_;
}

std::optional<TbRuntimeConfig> ThingsBoardConfigManager::pending_config() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_;
}

bool ThingsBoardConfigManager::has_pending_config() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_.has_value();
}

TbRuntimeConfig ThingsBoardConfigManager::parse_runtime_config(const nlohmann::json& root) {
    TbRuntimeConfig cfg;

    const auto robot_it = root.find("robot");
    if (robot_it != root.end() && robot_it->is_object()) {
        cfg.passes = robot_it->value("passes", cfg.passes);
        cfg.clean_speed_rpm = robot_it->value("clean_speed_rpm", cfg.clean_speed_rpm);
        cfg.return_speed_rpm = robot_it->value("return_speed_rpm", cfg.return_speed_rpm);
        cfg.brush_rpm = robot_it->value("brush_rpm", cfg.brush_rpm);
    }

    const auto scheduler_it = root.find("scheduler");
    if (scheduler_it != root.end() && scheduler_it->is_object()) {
        const auto windows_it = scheduler_it->find("windows");
        if (windows_it != scheduler_it->end()) {
            cfg.schedules = parse_schedule_entries(*windows_it);
        }
    }

    return cfg;
}

void ThingsBoardConfigManager::apply_schedule_json(nlohmann::json& root,
                                                   const nlohmann::json& schedules_json) {
    root["scheduler"]["windows"] = nlohmann::json::array();
    for (const auto& w : schedules_json) {
        root["scheduler"]["windows"].push_back(
            nlohmann::json{{"hour", w.at("hour").get<int>()}, {"minute", w.at("minute").get<int>()}});
    }
}

std::vector<TbScheduleEntry> ThingsBoardConfigManager::parse_schedule_entries(
    const nlohmann::json& schedules_json) {
    if (!schedules_json.is_array()) {
        throw std::runtime_error("schedules must be an array");
    }

    std::set<std::pair<int, int>> seen;
    std::vector<TbScheduleEntry> out;
    for (const auto& w : schedules_json) {
        if (!w.is_object()) {
            throw std::runtime_error("schedule entry must be an object");
        }

        const int hour = w.at("hour").get<int>();
        const int minute = w.at("minute").get<int>();
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

void ThingsBoardConfigManager::apply_scheduler_windows(const std::vector<TbScheduleEntry>& schedules) {
    scheduler_.clear_windows();
    for (const auto& schedule : schedules) {
        scheduler_.add_window(SchedulerService::TimeWindow{schedule.hour, schedule.minute});
    }
}

}  // namespace robot::service
