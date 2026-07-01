#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"

namespace robot::service {

struct RobotCommandResult {
    bool accepted{false};
    std::string reason;
};

struct RobotCommandPort {
    std::function<RobotCommandResult(const domain::RobotCommand&)> submit_command;
    std::function<domain::RobotRuntimeSnapshot()> snapshot;
};

/// @brief ThingsBoard 上行 JSON 编码器。
///
/// 上行 payload 只有两个主家族：
/// 1. startup attributes：设备静态身份；
/// 2. business telemetry：RobotController 持有的周期性运行事实。
class ThingsBoardJsonCodec {
public:
    struct StartupAttributesView {
        const char* software_version{""};
        const char* hardware_version{""};
        const char* device_model{""};
        const char* device_id{""};
    };

    static size_t build_startup_attributes(const StartupAttributesView& view,
                                           char* out,
                                           size_t cap) noexcept;
    static size_t build_business_telemetry(const domain::RobotRuntimeSnapshot& view,
                                           char* out,
                                           size_t cap) noexcept;
};

/// @brief ThingsBoard 运行时入口。
///
/// 职责边界：
/// - ingress：订阅 shared attributes，注册 RPC handlers；
/// - egress：发布 startup attributes 和 business telemetry；
///
/// 它不自己维护业务事实：
/// - shared attributes 的 active/pending 语义交给 ConfigService；
/// - RPC 是否允许交给 RobotCommandPort 背后的 app 层状态机；
/// - RPC reply 来自本地 RobotController/CommandTracker 的裁决，不来自云端推断；
/// - telemetry 来自 RobotCommandPort::snapshot()。
class ThingsBoardControlPlane {
public:
    ThingsBoardControlPlane(ConfigService& config,
                            SchedulerService* scheduler,
                            std::shared_ptr<CloudService> cloud,
                            std::shared_ptr<CommandTracker> command_tracker,
                            RobotCommandPort robot);

    /// 注册 shared attributes 下行入口。
    void subscribe_shared_attributes();
    /// 设备上线后主动请求一次当前版本关心的 shared attributes 快照。
    void request_shared_attributes_snapshot() const;
    /// 注册当前版本支持的机器人 RPC: clean_to_return / clean_to_parking /
    /// start_configured / stop / fault_reset。
    void register_rpc_handlers();
    void publish_startup_attributes() const;
    void publish_business_telemetry() const;

private:
    static constexpr size_t kBusinessPayloadBufferBytes = 4096;
    static constexpr size_t kEventPayloadBufferBytes = 1024;

    static std::string rpc_reply(const std::string& code);
    bool publish_attributes_payload(size_t len, const char* error_message) const;
    bool publish_business_payload(size_t len, const char* error_message) const;
    void register_command_rpc(const char* method, domain::RobotCommandKind kind);
    std::string reject_rpc_command(const char* command_name,
                                   const std::string& request_id,
                                   const char* reason);
    std::string accept_rpc_command(const char* command_name, const std::string& request_id);

    ConfigService& config_;
    SchedulerService* scheduler_{nullptr};
    std::shared_ptr<CloudService> cloud_;
    std::shared_ptr<CommandTracker> command_tracker_;
    RobotCommandPort robot_;
    mutable std::mutex publish_mtx_;
    mutable std::array<char, kBusinessPayloadBufferBytes> business_payload_buf_{};
    mutable std::string business_payload_cache_;
    mutable std::array<char, kEventPayloadBufferBytes> event_payload_buf_{};
    mutable std::string event_payload_cache_;
};

}  // namespace robot::service
