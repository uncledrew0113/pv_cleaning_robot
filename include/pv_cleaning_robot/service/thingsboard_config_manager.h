#pragma once

#include <mutex>
#include <optional>
#include <rapidjson/document.h>
#include <string>
#include <vector>

#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::service {

enum class ParkingSide {
    Left,
    Right,
};

inline const char* parking_side_config_string(ParkingSide value) noexcept {
    switch (value) {
    case ParkingSide::Left:
        return "left";
    case ParkingSide::Right:
        return "right";
    }
    return "left";
}

struct TbScheduleEntry {
    int hour{0};
    int minute{0};

    bool operator==(const TbScheduleEntry& other) const {
        return hour == other.hour && minute == other.minute;
    }
};

struct TbRuntimeConfig {
    double passes{1.0};
    double clean_speed_rpm{300.0};
    double return_speed_rpm{300.0};
    int brush_rpm{1000};
    ParkingSide parking_side{ParkingSide::Left};
    std::vector<TbScheduleEntry> schedules;

    bool operator==(const TbRuntimeConfig& other) const {
        return passes == other.passes &&
               clean_speed_rpm == other.clean_speed_rpm &&
               return_speed_rpm == other.return_speed_rpm &&
               brush_rpm == other.brush_rpm &&
               parking_side == other.parking_side &&
               schedules == other.schedules;
    }
};

struct SharedAttrApplyResult {
    bool accepted{false};
    std::string reason;
};

/// @brief ThingsBoard 共享属性配置管理器
///
/// 职责边界：
/// - 解析并校验 shared attributes
/// - 维护 active / pending 两份运行配置视图
/// - 管理 config.json / config.pending.json 的持久化
/// - 将 schedule 立即同步到 SchedulerService
///
/// 当前首版限制：
/// - `passes` 只允许正整数
/// - `parking_side` 只允许 `left` / `right`
/// - 任务相关参数先进 `pending`，下一次任务启动前再提升到 `active`
/// - 当前没有 shared attributes 版本合并机制；谁最后到达并通过校验，谁覆盖当前视图
///   - 设备上线后会主动请求一次 shared attributes 快照
///   - 如果平台随后再次修改，后到达的更新会覆盖先到达的快照结果
class ThingsBoardConfigManager {
public:
    ThingsBoardConfigManager(ConfigService& config, SchedulerService& scheduler);

    /// 应用服务端 shared attributes。
    /// 返回值只表示“当前 release 规则下是否接受”，不会吞掉拒绝原因。
    SharedAttrApplyResult apply_shared_attributes(const rapidjson::Value& attrs);
    /// 将 pending 提升为 active，并清空 pending 持久化文件。
    bool promote_pending_to_active();

    TbRuntimeConfig active_config() const;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const;

private:
    static TbRuntimeConfig parse_runtime_config(const rapidjson::Value& root);
    static void apply_schedule_json(rapidjson::Document& root,
                                    const rapidjson::Value& schedules_json);
    static std::vector<TbScheduleEntry> parse_schedule_entries(const rapidjson::Value& schedules_json);
    void apply_scheduler_windows(const std::vector<TbScheduleEntry>& schedules);

    ConfigService& config_;
    SchedulerService& scheduler_;

    mutable std::mutex mtx_;
    TbRuntimeConfig active_;
    std::optional<TbRuntimeConfig> pending_;
};

}  // namespace robot::service
