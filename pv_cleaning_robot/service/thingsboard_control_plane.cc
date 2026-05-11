#include <chrono>
#include <cmath>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <set>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "pv_cleaning_robot/app/robot_runtime_snapshot.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

namespace robot::service {
namespace {

// ThingsBoard 控制平面实现文件。
// 负责处理共享属性更新、运行时配置合并、RPC 命令注册、事件/遥测生成和发布。
// 其中包含：
// - 将 ThingsBoard shared attributes 映射到 robot.runtime 配置
// - 验证配置合法性并持久化 active/pending 配置
// - 将调度窗口同步到 SchedulerService
// - 生成 JSON 负载并通过 CloudService 发布

bool is_supported_field(const std::string& key) {
    // 仅允许 ThingsBoard shared attribute 更新这些字段。
    return key == "passes" || key == "clean_speed_rpm" || key == "return_speed_rpm" ||
           key == "brush_rpm" || key == "return_brush_rpm" || key == "parking_side" ||
           key == "start_battery_soc" || key == "charge_start_soc" || key == "charge_stop_soc" ||
           key == "schedules";
}

bool is_integer_passes(double value) {
    // passes 必须是正整数。
    return std::isfinite(value) && value > 0.0 && std::floor(value) == value;
}

ParkingSide parse_parking_side_string(const std::string& value) {
    // 将字符串映射到 ParkingSide 枚举，支持 left/right。
    if (value == parking_side_config_string(ParkingSide::Left)) {
        return ParkingSide::Left;
    }
    if (value == parking_side_config_string(ParkingSide::Right)) {
        return ParkingSide::Right;
    }
    throw std::runtime_error("parking_side must be left or right");
}

void validate_runtime_config(const TbRuntimeConfig& cfg) {
    // 验证运行时配置的基础约束。
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

rapidjson::Document clone_document(const rapidjson::Value& value) {
    // 复制一个 JSON 片段到新的 Document 中，保留其结构和值。
    rapidjson::Document out;
    out.CopyFrom(value, out.GetAllocator());
    return out;
}

rapidjson::Document make_empty_root() {
    // 创建一个空的 JSON 根对象。
    rapidjson::Document out;
    out.SetObject();
    return out;
}

rapidjson::Value* ensure_object_member(rapidjson::Value& parent,
                                       const char* key,
                                       rapidjson::Document::AllocatorType& alloc) {
    // 确保 parent 对象包含 key 字段，并且该字段是一个对象。
    // 如果不存在，则创建一个空对象；如果存在但不是对象，则覆盖为对象。
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
                       rapidjson::Document::AllocatorType& alloc) {
    // 写入或更新父对象中指定的 double 值成员。
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
                    rapidjson::Document::AllocatorType& alloc) {
    // 写入或更新父对象中指定的 int 值成员。
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
                       rapidjson::Document::AllocatorType& alloc) {
    // 写入或更新父对象中指定的字符串成员。
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
                          rapidjson::Document::AllocatorType& alloc) {
    // 递归合并 src 到 dst：
    // - 不存在的字段直接拷贝
    // - 两端都是对象时递归合并
    // - 否则直接覆盖 dst 的值
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
                                       const rapidjson::Value& pending_patch) {
    // 将 active_root 与 pending_patch 合并成一个新的 Document，返回合并结果。
    // pending_patch 的值会覆盖 active_root 中对应字段。
    auto merged = clone_document(active_root);
    merge_object_members(merged, pending_patch, merged.GetAllocator());
    return merged;
}

// 用于将 rapidjson 输出写入固定长度缓冲区，避免动态分配。
// 如果输出超出 cap，则 overflow_ 置位，调用方可以通过 size()==0 判断失败。
class RapidJsonFixedBufferStream {
   public:
    using Ch = char;

    RapidJsonFixedBufferStream(char* out, size_t cap) noexcept : out_(out), cap_(cap) {
        if (out_ && cap_ > 0) {
            out_[0] = '\0';
        }
    }

    void Put(char c) noexcept {
        if (!out_ || cap_ == 0 || overflow_) {
            return;
        }
        if (len_ + 1u >= cap_) {
            overflow_ = true;
            out_[cap_ - 1] = '\0';
            return;
        }
        out_[len_++] = c;
        out_[len_] = '\0';
    }

    void Flush() noexcept {}
    char Peek() const noexcept {
        return '\0';
    }
    char Take() noexcept {
        return '\0';
    }
    size_t Tell() const noexcept {
        return len_;
    }
    char* PutBegin() noexcept {
        return nullptr;
    }
    size_t PutEnd(char*) noexcept {
        return 0;
    }

    size_t size() const noexcept {
        return overflow_ ? 0u : len_;
    }
    bool overflow() const noexcept {
        return overflow_;
    }

   private:
    char* out_{nullptr};
    size_t cap_{0};
    size_t len_{0};
    bool overflow_{false};
};

const char* command_phase_name(CommandPhase phase) noexcept {
    // 将命令执行阶段枚举转换为 ThingsBoard RPC 事件中使用的字符串。
    switch (phase) {
        case CommandPhase::Accepted:
            return "accepted";
        case CommandPhase::Running:
            return "running";
        case CommandPhase::Succeeded:
            return "succeeded";
        case CommandPhase::Failed:
            return "failed";
        case CommandPhase::Rejected:
            return "rejected";
    }
    return "unknown";
}

template <typename WriterT>
void write_command_fields(WriterT& writer, const CommandSnapshot& command) {
    // 生成用于命令事件的公共字段。
    writer.Key("command_id");
    writer.String(command.id.c_str());
    writer.Key("command_name");
    writer.String(command.name.c_str());
    writer.Key("request_id");
    writer.String(command.request_id.c_str());
    writer.Key("phase");
    writer.String(command_phase_name(command.phase));
    writer.Key("reason");
    writer.String(command.reason.c_str());
    writer.Key("accepted_at_ms");
    writer.Uint64(command.accepted_at_ms);
    writer.Key("finished_at_ms");
    writer.Uint64(command.finished_at_ms);
}

template <typename WriterT>
void write_schedule_entries(const std::vector<TbScheduleEntry>& schedules, WriterT& writer) {
    // 将调度窗口列表写为 JSON 数组。
    writer.StartArray();
    for (const auto& schedule : schedules) {
        writer.StartObject();
        writer.Key("hour");
        writer.Int(schedule.hour);
        writer.Key("minute");
        writer.Int(schedule.minute);
        writer.EndObject();
    }
    writer.EndArray();
}

template <typename WriterT>
void write_runtime_config(const char* key, const TbRuntimeConfig& config, WriterT& writer) {
    // 将 TbRuntimeConfig 序列化为 JSON 对象，键名由调用方指定。
    writer.Key(key);
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
    write_schedule_entries(config.schedules, writer);
    writer.EndObject();
}

template <typename WriterT>
void write_command_snapshot(const char* key, const CommandSnapshot& command, WriterT& writer) {
    // 将命令快照写入一个 JSON 对象，常用于 active/last command 信息上报。
    writer.Key(key);
    writer.StartObject();
    writer.Key("id");
    writer.String(command.id.c_str());
    writer.Key("name");
    writer.String(command.name.c_str());
    writer.Key("request_id");
    writer.String(command.request_id.c_str());
    writer.Key("phase");
    writer.String(command_phase_name(command.phase));
    writer.Key("reason");
    writer.String(command.reason.c_str());
    writer.Key("accepted_at_ms");
    writer.Uint64(command.accepted_at_ms);
    writer.Key("finished_at_ms");
    writer.Uint64(command.finished_at_ms);
    writer.EndObject();
}

}  // namespace

ThingsBoardConfigManager::ThingsBoardConfigManager(ConfigService& config,
                                                   SchedulerService& scheduler)
    : config_(config), scheduler_(scheduler), active_(parse_runtime_config(config.snapshot())) {
    // 构造函数读取当前 active 配置并初始化调度窗口。
    apply_scheduler_windows(active_.schedules);
    if (const auto pending_root = config_.load_pending()) {
        // 读取 pending 配置并与 active 配置合并为 pending_ 配置视图。
        pending_ = parse_runtime_config(merge_runtime_root(config.snapshot(), *pending_root));
    }
}

SharedAttrApplyResult ThingsBoardConfigManager::apply_shared_attributes(
    const rapidjson::Value& attrs) {
    // 处理来自 ThingsBoard 的共享属性更新。
    // 只允许受支持的字段，并将更新写入 pending 或 active 配置。
    if (!attrs.IsObject()) {
        return {false, "attributes_must_be_object"};
    }

    const auto active_root_before = config_.snapshot();
    const auto pending_root_before = config_.load_pending();
    auto active_root_after = clone_document(active_root_before);
    auto pending_root_after =
        pending_root_before ? clone_document(*pending_root_before) : make_empty_root();

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
        if (const auto it = attrs.FindMember("schedules"); it != attrs.MemberEnd()) {
            const auto& schedules_json = it->value;
            // schedules 生效在 active 配置中，并且同时会立即更新调度器。
            parse_schedule_entries(schedules_json);
            apply_schedule_json(active_root_after, schedules_json);
            touches_active = true;
        }

        auto* robot =
            ensure_object_member(pending_root_after, "robot", pending_root_after.GetAllocator());

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
            set_double_member(*robot, "clean_speed_rpm", value, pending_root_after.GetAllocator());
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
            set_double_member(*robot, "return_speed_rpm", value, pending_root_after.GetAllocator());
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
            set_int_member(*robot, "return_brush_rpm", value, pending_root_after.GetAllocator());
            touches_pending = true;
        }

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
            // 如果 pending 配置发生变化，则先验证 active+pending 合并后的结果。
            validate_runtime_config(
                parse_runtime_config(merge_runtime_root(active_root_after, pending_root_after)));
        }
        if (touches_active) {
            // 如果 active 配置发生变化，则验证 active 配置本身。
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
            pending_ =
                parse_runtime_config(merge_runtime_root(active_root_after, pending_root_after));
        }
    }

    if (touches_active) {
        apply_scheduler_windows(parse_runtime_config(active_root_after).schedules);
    }

    return {true, {}};
}

bool ThingsBoardConfigManager::promote_pending_to_active() {
    // 将 pending 配置提升为 active 配置。
    // 仅在存在 pending 配置时执行，执行过程为：
    // 1. 合并 active 根和 pending 根
    // 2. 清除 pending 配置
    // 3. 写入并保存新的 active 配置
    // 4. 更新内存中的 active_/pending_ 视图并同步调度窗口
    const auto pending_root = config_.load_pending();
    if (!pending_root) {
        return true;
    }

    const auto active_root = config_.snapshot();
    const auto merged_root = merge_runtime_root(active_root, *pending_root);

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

TbRuntimeConfig ThingsBoardConfigManager::active_config() const {
    // 返回当前可见 active 运行时配置快照。
    std::lock_guard<std::mutex> lk(mtx_);
    return active_;
}

std::optional<TbRuntimeConfig> ThingsBoardConfigManager::pending_config() const {
    // 返回当前 pending 配置视图，供上层查询。
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_;
}

bool ThingsBoardConfigManager::has_pending_config() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_.has_value();
}

TbRuntimeConfig ThingsBoardConfigManager::parse_runtime_config(const rapidjson::Value& root) {
    // 从 JSON 根对象解析出 TbRuntimeConfig，支持 robot 和 scheduler 两个子树。
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
                // 兼容旧命名 battery_low_soc。
                cfg.charge_start_soc = it->value.GetDouble();
            }
            if (const auto it = robot.FindMember("charge_stop_soc");
                it != robot.MemberEnd() && it->value.IsNumber()) {
                cfg.charge_stop_soc = it->value.GetDouble();
            } else if (const auto it = robot.FindMember("battery_full_soc");
                       it != robot.MemberEnd() && it->value.IsNumber()) {
                // 兼容旧命名 battery_full_soc。
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
                                                   const rapidjson::Value& schedules_json) {
    // 将 schedules_json 写入 root 的 scheduler.windows 节点。
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
    const rapidjson::Value& schedules_json) {
    // 解析 schedule 数组，并校验每一项为合法时间，且没有重复项。
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
        if (hour_it == w.MemberEnd() || minute_it == w.MemberEnd() || !hour_it->value.IsInt() ||
            !minute_it->value.IsInt()) {
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
    const std::vector<TbScheduleEntry>& schedules) {
    scheduler_.clear_windows();
    for (const auto& schedule : schedules) {
        scheduler_.add_window(SchedulerService::TimeWindow{schedule.hour, schedule.minute});
    }
}

size_t ThingsBoardJsonCodec::build_startup_attributes(const StartupAttributesView& view,
                                                      char* out,
                                                      size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("software_version");
    writer.String(view.software_version ? view.software_version : "");
    writer.Key("hardware_version");
    writer.String(view.hardware_version ? view.hardware_version : "");
    writer.Key("device_model");
    writer.String(view.device_model ? view.device_model : "");
    writer.Key("device_id");
    writer.String(view.device_id ? view.device_id : "");
    writer.Key("supported_rpc_methods");
    writer.StartArray();
    writer.String("start");
    writer.String("stop");
    writer.String("return");
    writer.String("reset");
    writer.EndArray();
    writer.Key("config_schema_version");
    writer.String("thingsboard-v1");
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_status_event(const StatusEventView& view,
                                                char* out,
                                                size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("event");
    writer.String(view.event_name ? view.event_name : "");
    writer.Key("accepted");
    writer.Bool(view.accepted);
    writer.Key("reason");
    writer.String(view.reason ? view.reason : "");
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_command_event(const CommandEventView& view,
                                                 char* out,
                                                 size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("event");
    writer.String(view.event_name ? view.event_name : "");
    if (!view.command) {
        writer.EndObject();
        return stream.overflow() ? 0u : stream.size();
    }
    write_command_fields(writer, *view.command);
    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

size_t ThingsBoardJsonCodec::build_business_telemetry(const app::RobotRuntimeSnapshot& view,
                                                      char* out,
                                                      size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("device_state");
    writer.String(view.device_state.c_str());
    writer.Key("task_state");
    writer.String(view.task_state.c_str());
    writer.Key("target_passes");
    writer.Int(view.target_passes);
    writer.Key("completed_passes");
    writer.Int(view.completed_passes);
    writer.Key("clean_count");
    writer.Int(view.clean_count);
    writer.Key("active_config_version");
    writer.Uint64(view.active_config_version);

    if (view.active_config) {
        write_runtime_config("active_config", *view.active_config, writer);
    }
    if (view.pending_config) {
        write_runtime_config("pending_config", *view.pending_config, writer);
    }
    if (view.active_command) {
        write_command_snapshot("active_command", *view.active_command, writer);
    }
    if (view.last_command) {
        write_command_snapshot("last_command", *view.last_command, writer);
    }

    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

ThingsBoardControlPlane::ThingsBoardControlPlane(ConfigService& config,
                                                 std::shared_ptr<CloudService> cloud,
                                                 std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
                                                 std::shared_ptr<CommandTracker> command_tracker,
                                                 std::shared_ptr<app::RobotSupervisor> supervisor)
    : config_(config)
    , cloud_(std::move(cloud))
    , tb_cfg_(std::move(tb_cfg))
    , command_tracker_(std::move(command_tracker))
    , supervisor_(std::move(supervisor)) {
    business_payload_cache_.reserve(kBusinessPayloadBufferBytes);
    event_payload_cache_.reserve(kEventPayloadBufferBytes);
}

SharedAttrApplyResult ThingsBoardControlPlane::apply_shared_attributes(
    const rapidjson::Value& attrs) {
    return tb_cfg_->apply_shared_attributes(attrs);
}

bool ThingsBoardControlPlane::promote_pending_to_active() {
    return tb_cfg_->promote_pending_to_active();
}

TbRuntimeConfig ThingsBoardControlPlane::active_config() const {
    return tb_cfg_->active_config();
}

std::optional<TbRuntimeConfig> ThingsBoardControlPlane::pending_config() const {
    return tb_cfg_->pending_config();
}

bool ThingsBoardControlPlane::has_pending_config() const {
    return tb_cfg_->has_pending_config();
}

void ThingsBoardControlPlane::subscribe_shared_attributes() {
    // 订阅 ThingsBoard shared attributes 更新回调。
    // 每次 cloud 收到共享属性变化后，将调用 apply_shared_attributes 进行校验和持久化。
    cloud_->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
        const auto result = apply_shared_attributes(attrs);
        const auto reason = result.reason.empty() ? "ok" : result.reason;
        publish_status_event("shared_attr_update", result.accepted, reason.c_str());
        if (!result.accepted) {
            spdlog::warn("[ThingsBoardControlPlane] 共享属性更新被拒绝: {}", result.reason);
        }
    });
}

void ThingsBoardControlPlane::request_shared_attributes_snapshot() const {
    // 向 ThingsBoard 请求当前 shared attributes 快照，避免启动后配置不一致。
    static const std::vector<std::string> kReleaseSharedKeys{
        "passes",
        "clean_speed_rpm",
        "return_speed_rpm",
        "brush_rpm",
        "return_brush_rpm",
        "parking_side",
        "start_battery_soc",
        "charge_start_soc",
        "charge_stop_soc",
        "schedules",
    };
    if (!cloud_->request_shared_attributes_snapshot(kReleaseSharedKeys)) {
        spdlog::warn("[ThingsBoardControlPlane] 请求 shared attributes 快照失败");
    }
}

void ThingsBoardControlPlane::register_rpc_handlers(
    const std::function<bool()>& is_start_position_valid,
    const std::function<bool()>& is_at_start_parking_side,
    const std::function<bool()>& is_at_active_parking_side,
    const std::function<float()>& current_battery_soc,
    std::function<void()> reboot_device) {
    // 注册 ThingsBoard RPC 处理器，包括 start、stop、return 和 reset。
    // 这些处理器使用外部回调查询当前状态/位置/电量，并通过 RobotSupervisor 执行任务控制。
    cloud_->register_rpc(
        "start",
        [this, is_start_position_valid, is_at_start_parking_side, current_battery_soc](
            const std::string& request_id, const std::string& /*params*/) {
            const auto state = supervisor_->current_state();
            const bool position_valid = is_start_position_valid();
            const bool at_parking_side = is_at_start_parking_side();
            const float battery_soc = current_battery_soc();
            spdlog::info(
                "[ThingsBoardControlPlane] RPC start received: state='{}' position_valid={} "
                "at_parking_side={} battery_soc={:.1f}",
                state,
                position_valid,
                at_parking_side,
                battery_soc);

            if (!supervisor_->start_task_from_current_position(position_valid, battery_soc)) {
                const auto runtime_cfg = has_pending_config() ? *pending_config() : active_config();
                const std::string reason =
                    (state != "Idle" && state != "Charging" && state != "Stopped")
                        ? "start_not_allowed_in_current_state"
                    : !position_valid  ? "robot_position_invalid"
                    : battery_soc < static_cast<float>(runtime_cfg.start_battery_soc)
                        ? "battery_below_start_threshold"
                        : "promote_pending_config_failed";
                spdlog::warn("[ThingsBoardControlPlane] RPC start rejected: {}", reason);
                return reject_rpc_command("start", request_id, reason.c_str());
            }

            spdlog::info("[ThingsBoardControlPlane] RPC start completed: started_new_task");
            return complete_rpc_command("start", request_id, "started_new_task");
        });

    cloud_->register_rpc(
        "stop", [this](const std::string& request_id, const std::string& /*params*/) {
            // stop RPC 仅允许在当前任务可停止时调用。
            spdlog::info("[ThingsBoardControlPlane] RPC stop received: state='{}'",
                         supervisor_->current_state());
            if (!supervisor_->stop_task()) {
                spdlog::warn(
                    "[ThingsBoardControlPlane] RPC stop rejected: "
                    "stop_not_allowed_in_current_state");
                return reject_rpc_command("stop", request_id, "stop_not_allowed_in_current_state");
            }

            spdlog::info("[ThingsBoardControlPlane] RPC stop completed: stopped_task");
            return complete_rpc_command("stop", request_id, "stopped_task");
        });

    cloud_->register_rpc(
        "return",
        [this, is_at_active_parking_side](const std::string& request_id,
                                          const std::string& /*params*/) {
            // return RPC 用于请求机器人返回当前活动停车侧。
            const bool at_parking_side = is_at_active_parking_side();
            spdlog::info(
                "[ThingsBoardControlPlane] RPC return received: state='{}' at_parking_side={}",
                supervisor_->current_state(),
                at_parking_side);
            if (!supervisor_->return_task(at_parking_side)) {
                spdlog::warn(
                    "[ThingsBoardControlPlane] RPC return rejected: "
                    "return_not_allowed_in_current_state");
                return reject_rpc_command(
                    "return", request_id, "return_not_allowed_in_current_state");
            }

            spdlog::info(
                "[ThingsBoardControlPlane] RPC return completed: returning_to_parking_side");
            return complete_rpc_command("return", request_id, "returning_to_parking_side");
        });

    cloud_->register_rpc(
        "reset",
        [this, reboot_device = std::move(reboot_device)](const std::string& request_id,
                                                         const std::string& /*params*/) {
            // reset RPC 立即响应 rebooting_device，然后异步重启设备。
            spdlog::info("[ThingsBoardControlPlane] RPC reset received: state='{}'",
                         supervisor_->current_state());
            auto reply = complete_rpc_command("reset", request_id, "rebooting_device");
            std::thread([reboot_device]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                if (reboot_device) {
                    reboot_device();
                }
            }).detach();
            return reply;
        });
}

void ThingsBoardControlPlane::publish_backup_fallback_event() const {
    // 发布 backup 回退事件，说明主配置加载失败，系统已使用备份启动。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_status_event(
        {"config_backup_fallback", true, "loaded_from_backup"},
        event_payload_buf_.data(),
        event_payload_buf_.size());
    if (!publish_event_payload(
            len,
            "[ThingsBoardControlPlane] failed to build config_backup_fallback event payload")) {
        return;
    }
    spdlog::warn("[ThingsBoardControlPlane] 主配置加载失败，已从 backup 配置回退启动");
}

void ThingsBoardControlPlane::publish_startup_attributes() const {
    // 发布设备启动属性，用于 ThingsBoard 设备连接后读取静态信息。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_startup_attributes(
        {config_
             .get<std::string>("device.software_version",
                               config_.get<std::string>("device.fw_version", "1.0.0"))
             .c_str(),
         config_
             .get<std::string>("device.hardware_version",
                               config_.get<std::string>("device.hw_version", "1.0"))
             .c_str(),
         config_.get<std::string>("device.model", "pv_cleaning_robot").c_str(),
         config_.get<std::string>("network.mqtt.client_id", "pv_robot_001").c_str()},
        event_payload_buf_.data(),
        event_payload_buf_.size());
    publish_attributes_payload(
        len, "[ThingsBoardControlPlane] failed to build startup attributes payload");
}

void ThingsBoardControlPlane::publish_status_event(const char* event_name,
                                                   bool accepted,
                                                   const char* reason) const {
    // 发布通用状态事件，通常用于 shared attribute 更新结果等。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_status_event(
        {event_name, accepted, reason}, event_payload_buf_.data(), event_payload_buf_.size());
    publish_event_payload(len, "[ThingsBoardControlPlane] failed to build status event payload");
}

void ThingsBoardControlPlane::publish_command_event(const char* event_name,
                                                    const CommandSnapshot& snapshot) const {
    // 发布命令相关事件，包括 command_accepted、command_completed、command_rejected。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const size_t len = ThingsBoardJsonCodec::build_command_event(
        {event_name, &snapshot}, event_payload_buf_.data(), event_payload_buf_.size());
    publish_event_payload(len, "[ThingsBoardControlPlane] failed to build command event payload");
}

void ThingsBoardControlPlane::publish_business_telemetry() const {
    // 定期上报业务遥测数据，包含任务状态、完成次数、配置版本等。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const auto runtime_snap = supervisor_->snapshot();
    const size_t len = ThingsBoardJsonCodec::build_business_telemetry(
        runtime_snap, business_payload_buf_.data(), business_payload_buf_.size());
    publish_business_payload(
        len, "[ThingsBoardControlPlane] failed to build periodic business telemetry payload");
}

std::string ThingsBoardControlPlane::rpc_reply(bool accepted, const std::string& reason) {
    // 生成 RPC 回复 JSON 字符串，返回 accepted/result/reason。
    rapidjson::StringBuffer buffer;
    buffer.Reserve(static_cast<rapidjson::SizeType>(96 + reason.size()));
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("accepted");
    writer.Bool(accepted);
    writer.Key("result");
    writer.String(accepted ? "ok" : "rejected");
    if (!reason.empty()) {
        writer.Key("reason");
        writer.String(reason.c_str(), static_cast<rapidjson::SizeType>(reason.size()));
    }
    writer.EndObject();
    return buffer.GetString();
}

bool ThingsBoardControlPlane::publish_attributes_payload(size_t len,
                                                         const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_attributes(event_payload_cache_);
    return true;
}

bool ThingsBoardControlPlane::publish_event_payload(size_t len, const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    event_payload_cache_.assign(event_payload_buf_.data(), len);
    cloud_->publish_telemetry(event_payload_cache_);
    return true;
}

bool ThingsBoardControlPlane::publish_business_payload(size_t len,
                                                       const char* error_message) const {
    if (len == 0u) {
        spdlog::error("{}", error_message);
        return false;
    }
    business_payload_cache_.assign(business_payload_buf_.data(), len);
    cloud_->publish_telemetry(business_payload_cache_);
    return true;
}

std::string ThingsBoardControlPlane::reject_rpc_command(const char* command_name,
                                                        const std::string& request_id,
                                                        const char* reason) {
    // 记录命令被拒绝并上报命令事件。
    command_tracker_->reject(command_name, request_id, reason);
    publish_command_event("command_rejected", *command_tracker_->last_completed());
    return rpc_reply(false, reason);
}

std::string ThingsBoardControlPlane::complete_rpc_command(const char* command_name,
                                                          const std::string& request_id,
                                                          const char* completion_reason) {
    // 处理命令成功完成的完整流程：接受、发布 accepted 事件、标记运行、完成、发布 completed 事件。
    spdlog::info("[ThingsBoardControlPlane] complete_rpc_command begin: command='{}' reason='{}'",
                 command_name,
                 completion_reason);
    const auto cmd_id = command_tracker_->accept(command_name, request_id);
    spdlog::info(
        "[ThingsBoardControlPlane] command accepted: command='{}' cmd_id={}", command_name, cmd_id);
    publish_command_event("command_accepted", *command_tracker_->active());
    spdlog::info("[ThingsBoardControlPlane] command_accepted event published: command='{}'",
                 command_name);
    command_tracker_->mark_running(cmd_id);
    spdlog::info("[ThingsBoardControlPlane] command marked running: command='{}' cmd_id={}",
                 command_name,
                 cmd_id);
    command_tracker_->finish_success(cmd_id, completion_reason);
    spdlog::info(
        "[ThingsBoardControlPlane] command finished success: command='{}' cmd_id={} reason='{}'",
        command_name,
        cmd_id,
        completion_reason);
    publish_command_event("command_completed", *command_tracker_->last_completed());
    spdlog::info("[ThingsBoardControlPlane] command_completed event published: command='{}'",
                 command_name);
    return rpc_reply(true);
}

}  // namespace robot::service
