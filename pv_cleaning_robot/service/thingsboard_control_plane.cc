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
void write_schedule_entries(const std::vector<RuntimeScheduleEntry>& schedules, WriterT& writer) {
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
void write_runtime_config(const char* key, const RuntimeConfig& config, WriterT& writer) {
    // 将 RuntimeConfig 序列化为 JSON 对象，键名由调用方指定。
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
    writer.Key("active_config_version");
    writer.Uint64(view.active_config_version);

    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

ThingsBoardControlPlane::ThingsBoardControlPlane(ConfigService& config,
                                                 SchedulerService* scheduler,
                                                 std::shared_ptr<CloudService> cloud,
                                                 std::shared_ptr<CommandTracker> command_tracker,
                                                 std::shared_ptr<app::RobotSupervisor> supervisor)
    : config_(config)
    , scheduler_(scheduler)
    , cloud_(std::move(cloud))
    , command_tracker_(std::move(command_tracker))
    , supervisor_(std::move(supervisor)) {
    business_payload_cache_.reserve(kBusinessPayloadBufferBytes);
    event_payload_cache_.reserve(kEventPayloadBufferBytes);
}

void ThingsBoardControlPlane::subscribe_shared_attributes() {
    // 订阅 ThingsBoard shared attributes 更新回调。
    // 每次 cloud 收到共享属性变化后，将调用 apply_shared_attributes 进行校验和持久化。
    cloud_->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
        const auto result = config_.apply_runtime_patch(attrs, scheduler_);
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
                const auto runtime_cfg = config_.has_pending_runtime_config()
                                             ? *config_.pending_runtime_config()
                                             : config_.active_runtime_config();
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
    // 保留本地命令真相，但不再发布高频 command event。
    command_tracker_->reject(command_name, request_id, reason);
    return rpc_reply(false, reason);
}

std::string ThingsBoardControlPlane::complete_rpc_command(const char* command_name,
                                                          const std::string& request_id,
                                                          const char* completion_reason) {
    // 保留本地命令生命周期真相，但不再发布高频 command event。
    spdlog::info("[ThingsBoardControlPlane] complete_rpc_command begin: command='{}' reason='{}'",
                 command_name,
                 completion_reason);
    const auto cmd_id = command_tracker_->accept(command_name, request_id);
    spdlog::info(
        "[ThingsBoardControlPlane] command accepted: command='{}' cmd_id={}", command_name, cmd_id);
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
    return rpc_reply(true);
}

}  // namespace robot::service
