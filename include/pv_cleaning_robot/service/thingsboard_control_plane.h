#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rapidjson/document.h>

#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::app {
struct RobotRuntimeSnapshot;
class RobotSupervisor;
}

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
    int return_brush_rpm{1000};
    ParkingSide parking_side{ParkingSide::Left};
    double start_battery_soc{30.0};
    double charge_start_soc{15.0};
    double charge_stop_soc{95.0};
    std::vector<TbScheduleEntry> schedules;

    bool operator==(const TbRuntimeConfig& other) const {
        return passes == other.passes &&
               clean_speed_rpm == other.clean_speed_rpm &&
               return_speed_rpm == other.return_speed_rpm &&
               brush_rpm == other.brush_rpm &&
               return_brush_rpm == other.return_brush_rpm &&
               parking_side == other.parking_side &&
               start_battery_soc == other.start_battery_soc &&
               charge_start_soc == other.charge_start_soc &&
               charge_stop_soc == other.charge_stop_soc &&
               schedules == other.schedules;
    }
};

struct SharedAttrApplyResult {
    bool accepted{false};
    std::string reason;
};

/// ThingsBoard 上行 payload 只有 4 个家族：
/// 1. startup attributes: 设备静态身份 + 支持的 RPC 能力
/// 2. status event: 单次结果，例如 shared-attribute 接受/拒绝
/// 3. command event: CommandTracker 维护的命令生命周期快照
/// 4. business telemetry: Supervisor 持有的周期性运行真相
class ThingsBoardJsonCodec {
public:
    struct StartupAttributesView {
        const char* software_version{""};
        const char* hardware_version{""};
        const char* device_model{""};
        const char* device_id{""};
    };

    struct StatusEventView {
        const char* event_name{""};
        bool accepted{false};
        const char* reason{""};
    };

    struct CommandEventView {
        const char* event_name{""};
        const CommandSnapshot* command{nullptr};
    };

    static size_t build_startup_attributes(const StartupAttributesView& view,
                                           char* out,
                                           size_t cap) noexcept;
    static size_t build_status_event(const StatusEventView& view, char* out, size_t cap) noexcept;
    static size_t build_command_event(const CommandEventView& view, char* out, size_t cap) noexcept;
    static size_t build_business_telemetry(const app::RobotRuntimeSnapshot& view,
                                           char* out,
                                           size_t cap) noexcept;
};

/// 运行时配置语义仍然保持独立对象，便于当前 app 层沿用已有持有关系；
/// 但声明和实现已经合并回主 ThingsBoard 头/源，避免再拆成平行文件。
class ThingsBoardConfigManager {
public:
    ThingsBoardConfigManager(ConfigService& config, SchedulerService& scheduler);

    /// shared attributes 分两类边界：
    /// - `schedules` 立即生效，因为调度器需要立刻知道未来窗口
    /// - 其余运行参数先写 pending，等下一个任务开始前再提升到 active，
    ///   避免当前任务执行中途突然改速度、方向或电量阈值
    SharedAttrApplyResult apply_shared_attributes(const rapidjson::Value& attrs);
    bool promote_pending_to_active();

    TbRuntimeConfig active_config() const;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const;

private:
    static TbRuntimeConfig parse_runtime_config(const rapidjson::Value& root);
    static void apply_schedule_json(rapidjson::Document& root,
                                    const rapidjson::Value& schedules_json);
    static std::vector<TbScheduleEntry> parse_schedule_entries(
        const rapidjson::Value& schedules_json);
    void apply_scheduler_windows(const std::vector<TbScheduleEntry>& schedules);

    ConfigService& config_;
    SchedulerService& scheduler_;

    mutable std::mutex mtx_;
    TbRuntimeConfig active_;
    std::optional<TbRuntimeConfig> pending_;
};

/// @brief ThingsBoard 运行时入口
///
/// 职责边界：
/// - ingress:
///   - 订阅 shared attributes
///   - 注册 RPC handlers
/// - egress:
///   - 发布 startup attributes
///   - 发布 status / command event
///   - 发布 business telemetry
///
/// 它不自己维护业务真相：
/// - shared attributes 的 active/pending 语义，交给 ThingsBoardConfigManager
/// - RPC 是否允许，交给 RobotSupervisor
/// - RPC reply 的真相，来自本地 Supervisor/CommandTracker 的裁决，不来自云端
/// - telemetry 真相，来自 RobotSupervisor::snapshot()
class ThingsBoardControlPlane {
public:
    ThingsBoardControlPlane(ConfigService& config,
                            std::shared_ptr<CloudService> cloud,
                            std::shared_ptr<ThingsBoardConfigManager> tb_cfg,
                            std::shared_ptr<CommandTracker> command_tracker,
                            std::shared_ptr<app::RobotSupervisor> supervisor);

    /// 对外暴露配置入口，便于调用方只依赖一个 ThingsBoard 公共头。
    SharedAttrApplyResult apply_shared_attributes(const rapidjson::Value& attrs);
    bool promote_pending_to_active();
    TbRuntimeConfig active_config() const;
    std::optional<TbRuntimeConfig> pending_config() const;
    bool has_pending_config() const;

    /// 注册 shared attributes 下行入口。
    void subscribe_shared_attributes();
    /// 设备上线后主动请求一次当前 release 关心的 shared attributes 快照。
    void request_shared_attributes_snapshot() const;
    /// 注册当前 release 支持的 RPC: start / stop / return / reset
    void register_rpc_handlers(const std::function<bool()>& is_start_position_valid,
                               const std::function<bool()>& is_at_start_parking_side,
                               const std::function<bool()>& is_at_active_parking_side,
                               const std::function<float()>& current_battery_soc,
                               std::function<void()> reboot_device);
    void publish_backup_fallback_event() const;
    void publish_startup_attributes() const;
    void publish_status_event(const char* event_name, bool accepted, const char* reason) const;
    void publish_command_event(const char* event_name, const CommandSnapshot& snapshot) const;
    void publish_business_telemetry() const;

private:
    static constexpr size_t kBusinessPayloadBufferBytes = 4096;
    static constexpr size_t kEventPayloadBufferBytes = 1024;

    static std::string rpc_reply(bool accepted, const std::string& reason = {});
    bool publish_attributes_payload(size_t len, const char* error_message) const;
    bool publish_event_payload(size_t len, const char* error_message) const;
    bool publish_business_payload(size_t len, const char* error_message) const;
    std::string reject_rpc_command(const char* command_name,
                                   const std::string& request_id,
                                   const char* reason);
    std::string complete_rpc_command(const char* command_name,
                                     const std::string& request_id,
                                     const char* completion_reason);

    ConfigService& config_;
    std::shared_ptr<CloudService> cloud_;
    std::shared_ptr<ThingsBoardConfigManager> tb_cfg_;
    std::shared_ptr<CommandTracker> command_tracker_;
    std::shared_ptr<app::RobotSupervisor> supervisor_;
    mutable std::mutex publish_mtx_;
    mutable std::array<char, kBusinessPayloadBufferBytes> business_payload_buf_{};
    mutable std::string business_payload_cache_;
    mutable std::array<char, kEventPayloadBufferBytes> event_payload_buf_{};
    mutable std::string event_payload_cache_;
};

}  // namespace robot::service
