#include <chrono>
#include <cmath>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <set>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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
    writer.Key("repeat_count");
    writer.Uint(config.repeat_count);
    writer.Key("clean_speed_rpm");
    writer.Double(config.clean_speed_rpm);
    writer.Key("return_speed_rpm");
    writer.Double(config.return_speed_rpm);
    writer.Key("brush_rpm");
    writer.Int(config.brush_rpm);
    writer.Key("primary_dock");
    writer.String(endpoint_config_string(config.primary_dock));
    writer.Key("min_battery_soc");
    writer.Double(config.min_battery_soc);
    writer.Key("charge_stop_soc");
    writer.Double(config.charge_stop_soc);
    writer.Key("schedules");
    write_schedule_entries(config.schedules, writer);
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

size_t ThingsBoardJsonCodec::build_business_telemetry(const domain::RobotRuntimeSnapshot& view,
                                                      char* out,
                                                      size_t cap) noexcept {
    RapidJsonFixedBufferStream stream(out, cap);
    rapidjson::Writer<RapidJsonFixedBufferStream> writer(stream);
    writer.StartObject();
    writer.Key("state");
    writer.String(view.state.c_str());
    writer.Key("fault");
    writer.Uint(view.fault);
    writer.Key("cfg_ver");
    writer.Uint64(view.cfg_ver);
    writer.Key("repeat_count");
    writer.Uint(view.repeat_count);
    writer.Key("completed_cycles");
    writer.Uint(view.completed_cycles);

    writer.EndObject();
    return stream.overflow() ? 0u : stream.size();
}

ThingsBoardControlPlane::ThingsBoardControlPlane(ConfigService& config,
                                                 SchedulerService* scheduler,
                                                 std::shared_ptr<CloudService> cloud,
                                                 std::shared_ptr<CommandTracker> command_tracker,
                                                 RobotCommandPort robot)
    : config_(config)
    , scheduler_(scheduler)
    , cloud_(std::move(cloud))
    , command_tracker_(std::move(command_tracker))
    , robot_(std::move(robot)) {
    business_payload_cache_.reserve(kBusinessPayloadBufferBytes);
    event_payload_cache_.reserve(kEventPayloadBufferBytes);
}

void ThingsBoardControlPlane::subscribe_shared_attributes() {
    // 订阅 ThingsBoard shared attributes 更新回调。
    // 每次 cloud 收到共享属性变化后，将调用 apply_shared_attributes 进行校验和持久化。
    cloud_->subscribe_shared_attributes([this](const rapidjson::Document& attrs) {
        const auto result = config_.apply_runtime_patch(attrs, scheduler_);
        if (!result.accepted) {
            spdlog::warn("[ThingsBoardControlPlane] 共享属性更新被拒绝: {}", result.reason);
        }
    });
}

void ThingsBoardControlPlane::request_shared_attributes_snapshot() const {
    // 向 ThingsBoard 请求当前 shared attributes 快照，避免启动后配置不一致。
    static const std::vector<std::string> kReleaseSharedKeys{
        "repeat_count",
        "clean_speed_rpm",
        "return_speed_rpm",
        "brush_rpm",
        "primary_dock",
        "min_battery_soc",
        "charge_stop_soc",
        "schedules",
    };
    if (!cloud_->request_shared_attributes_snapshot(kReleaseSharedKeys)) {
        spdlog::warn("[ThingsBoardControlPlane] 请求 shared attributes 快照失败");
    }
}

void ThingsBoardControlPlane::register_rpc_handlers() {
    register_command_rpc("clean_to_return", domain::RobotCommandKind::CleanTowardOppositeEndpoint);
    register_command_rpc("clean_to_parking",
                         domain::RobotCommandKind::CleanTowardPrimaryDock);
    register_command_rpc("start_configured", domain::RobotCommandKind::StartConfiguredMission);
    register_command_rpc("stop", domain::RobotCommandKind::Stop);
    register_command_rpc("fault_reset", domain::RobotCommandKind::FaultReset);
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

void ThingsBoardControlPlane::publish_business_telemetry() const {
    // 定期上报业务遥测数据，包含任务状态、完成次数、配置版本等。
    std::lock_guard<std::mutex> lk(publish_mtx_);
    const auto runtime_snap = robot_.snapshot();
    const size_t len = ThingsBoardJsonCodec::build_business_telemetry(
        runtime_snap, business_payload_buf_.data(), business_payload_buf_.size());
    publish_business_payload(
        len, "[ThingsBoardControlPlane] failed to build periodic business telemetry payload");
}

std::string ThingsBoardControlPlane::rpc_reply(const std::string& code) {
    // 生成 RPC 回复 JSON 字符串，只返回统一 code。
    rapidjson::StringBuffer buffer;
    buffer.Reserve(static_cast<rapidjson::SizeType>(64 + code.size()));
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("code");
    writer.String(code.c_str(), static_cast<rapidjson::SizeType>(code.size()));
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

void ThingsBoardControlPlane::register_command_rpc(const char* method,
                                                   domain::RobotCommandKind kind) {
    cloud_->register_rpc(method,
                         [this, method, kind](const std::string& request_id,
                                              const std::string& /*params*/) {
        spdlog::info("[ThingsBoardControlPlane] RPC {} received", method);
        if (!robot_.submit_command) {
            return reject_rpc_command(method, request_id, "robot_command_port_missing");
        }

        const auto result = robot_.submit_command(
            domain::RobotCommand{kind, domain::CommandSource::Rpc, request_id});
        if (!result.accepted) {
            const auto reason = result.reason.empty() ? "rejected" : result.reason;
            spdlog::warn("[ThingsBoardControlPlane] RPC {} rejected: {}", method, reason);
            return reject_rpc_command(method, request_id, reason.c_str());
        }

        spdlog::info("[ThingsBoardControlPlane] RPC {} accepted", method);
        return accept_rpc_command(method, request_id);
    });
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
    return rpc_reply(reason);
}

std::string ThingsBoardControlPlane::accept_rpc_command(const char* command_name,
                                                        const std::string& request_id) {
    // RPC 回复只表示本地业务层已接受命令，最终完成/失败由 app 状态上报闭环。
    const auto cmd_id = command_tracker_->accept(command_name, request_id);
    spdlog::info(
        "[ThingsBoardControlPlane] command accepted: command='{}' cmd_id={}", command_name, cmd_id);
    command_tracker_->mark_running(cmd_id);
    spdlog::info("[ThingsBoardControlPlane] command marked running: command='{}' cmd_id={}",
                 command_name,
                 cmd_id);
    return rpc_reply("accepted");
}

}  // namespace robot::service
