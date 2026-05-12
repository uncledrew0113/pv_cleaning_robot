#include "pv_cleaning_robot/service/cloud_service.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <spdlog/spdlog.h>

namespace robot::service {
namespace {

constexpr size_t kRpcPoolBytes = 4096;
std::atomic<uint64_t> g_attr_request_id{1};

std::string stringify_json_value(const rapidjson::Value& value)
{
    rapidjson::StringBuffer buffer;
    if (value.IsString()) {
        buffer.Reserve(static_cast<rapidjson::SizeType>(value.GetStringLength() + 8));
    } else {
        buffer.Reserve(256);
    }
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);
    return {buffer.GetString(), buffer.GetSize()};
}

std::string build_rpc_response(bool accepted, const char* reason)
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("accepted");
    writer.Bool(accepted);
    writer.Key("result");
    writer.String(accepted ? "ok" : "rejected");
    if (reason && reason[0] != '\0') {
        writer.Key("reason");
        writer.String(reason);
    }
    writer.EndObject();
    return {buffer.GetString(), buffer.GetSize()};
}

rapidjson::Document parse_small_json_object(const std::string& payload,
                                            const char* parse_error_message)
{
    rapidjson::Document document;
    document.Parse(payload.c_str(), payload.size());
    if (document.HasParseError() || !document.IsObject()) {
        throw std::runtime_error(parse_error_message);
    }
    return document;
}

}  // namespace

CloudService::CloudService(std::shared_ptr<middleware::NetworkManager> network,
                           std::shared_ptr<middleware::DataCache>      cache)
    : CloudService(std::move(network), std::move(cache), Topics{})
{}

CloudService::CloudService(std::shared_ptr<middleware::NetworkManager> network,
                           std::shared_ptr<middleware::DataCache>      cache,
                           Topics                                      topics)
    : network_(std::move(network))
    , cache_(std::move(cache))
    , topics_(std::move(topics))
{
    // ThingsBoard RPC 下行:
    //   v1/devices/me/rpc/request/{request_id}
    // CloudService 只负责 topic 路由、JSON 解析和 response topic 回写。
    network_->subscribe(topics_.rpc_request,
        [this](const std::string& t, const std::string& p) {
            on_rpc_message(t, p);
        });
    // ThingsBoard shared attributes 下行:
    //   v1/devices/me/attributes
    // 服务端推送以及设备初始接入时的全量属性都会走这里。
    network_->subscribe(topics_.attributes,
        [this](const std::string& /*t*/, const std::string& p) {
            on_shared_attributes_message(p);
        });
    network_->subscribe(topics_.attributes_response,
        [this](const std::string& /*t*/, const std::string& p) {
            on_shared_attributes_response_message(p);
        });
}

bool CloudService::publish_telemetry(const std::string& json_payload)
{
    if (!network_->is_connected()) {
        // 离线：缓存到 SQLite
        if (cache_) cache_->push(topics_.telemetry, json_payload);
        return false;
    }
    return network_->publish(topics_.telemetry, json_payload);
}

bool CloudService::publish_attributes(const std::string& json_payload)
{
    return network_->publish(topics_.attributes, json_payload);
}

void CloudService::register_rpc(const std::string& method, RpcHandler handler)
{
    std::lock_guard<std::mutex> lk(rpc_mtx_);
    rpc_handlers_[method] = std::move(handler);
}

void CloudService::subscribe_shared_attributes(AttrCallback cb)
{
    std::lock_guard<std::mutex> lk(attr_cb_mtx_);
    attr_cb_ = std::move(cb);
}

bool CloudService::request_shared_attributes_snapshot(const std::vector<std::string>& shared_keys)
{
    if (shared_keys.empty()) {
        return false;
    }

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("sharedKeys");
    std::string joined_keys;
    for (size_t i = 0; i < shared_keys.size(); ++i) {
        if (shared_keys[i].empty()) {
            continue;
        }
        if (!joined_keys.empty()) {
            joined_keys.push_back(',');
        }
        joined_keys += shared_keys[i];
    }
    if (joined_keys.empty()) {
        return false;
    }
    writer.String(joined_keys.c_str(), static_cast<rapidjson::SizeType>(joined_keys.size()));
    writer.EndObject();

    const auto request_id = g_attr_request_id.fetch_add(1, std::memory_order_relaxed);
    return network_->publish(
        topics_.attributes_request_prefix + std::to_string(request_id),
        std::string(buffer.GetString(), buffer.GetSize()));
}

void CloudService::flush_cache()
{
    if (!cache_ || !network_->is_connected()) return;

    auto batch = cache_->pop_batch(50);
    if (batch.empty()) return;

    std::vector<int64_t> sent_ids;
    for (auto& rec : batch) {
        if (network_->publish(rec.topic, rec.payload)) {
            sent_ids.push_back(rec.id);
        }
    }
    cache_->confirm_sent(sent_ids);
}

void CloudService::update()
{
    // 每次 update() 尝试回填缓存数据
    flush_cache();
}

void CloudService::on_shared_attributes_response_message(const std::string& payload)
{
    AttrCallback cb;
    {
        std::lock_guard<std::mutex> lk(attr_cb_mtx_);
        cb = attr_cb_;
    }
    if (!cb) return;

    try {
        auto document = parse_small_json_object(
            payload, "invalid shared attributes response JSON");

        const auto shared_it = document.FindMember("shared");
        if (shared_it == document.MemberEnd() || !shared_it->value.IsObject()) {
            spdlog::warn("[CloudService] Shared attributes response missing object field 'shared'");
            return;
        }

        rapidjson::Document shared_attrs;
        shared_attrs.SetObject();
        shared_attrs.CopyFrom(shared_it->value, shared_attrs.GetAllocator());
        cb(shared_attrs);
    } catch (const std::exception& ex) {
        spdlog::warn("[CloudService] Failed to process shared attributes response payload: {}",
                     ex.what());
    }
}

void CloudService::on_shared_attributes_message(const std::string& payload)
{
    AttrCallback cb;
    {
        std::lock_guard<std::mutex> lk(attr_cb_mtx_);
        cb = attr_cb_;
    }
    if (!cb) return;

    try {
        auto document = parse_small_json_object(payload, "invalid JSON");
        cb(document);
    } catch (const std::exception& ex) {
        spdlog::warn("[CloudService] Failed to process shared attributes payload: {}", ex.what());
    }
}

void CloudService::on_rpc_message(const std::string& topic,
                                  const std::string& payload)
{
    spdlog::info("[CloudService] RPC message arrived: topic='{}' payload={}", topic, payload);
    // ThingsBoard RPC topic: v1/devices/me/rpc/request/{request_id}
    // 从 topic 末尾提取 request_id
    std::string request_id;
    auto pos = topic.rfind('/');
    if (pos != std::string::npos)
        request_id = topic.substr(pos + 1);
    const auto publish_reject = [this, &request_id](const char* reason) {
        if (request_id.empty()) {
            return;
        }
        const auto response_topic = topics_.rpc_response_prefix + request_id;
        const auto response = build_rpc_response(false, reason);
        spdlog::warn("[CloudService] rejecting RPC request: request_id='{}' reason='{}'",
                     request_id,
                     reason);
        const bool ok = network_->publish(response_topic, response);
        spdlog::info("[CloudService] published rejected RPC response: topic='{}' ok={}",
                     response_topic,
                     ok);
    };

    // params 大小防御：超大 payload 拒绝，防止栈耗尽
    if (payload.size() > kMaxRpcParamsBytes) {
        spdlog::warn("[CloudService] RPC payload too large: {} bytes (max {})",
                     payload.size(), kMaxRpcParamsBytes);
        publish_reject("payload_too_large");
        return;
    }

    std::string method;
    std::string params;
    // RPC 请求和 shared attributes 一样，优先走局部 pool，减少 parse 热路径上的 heap churn。
    alignas(std::max_align_t) unsigned char pool_buffer[kRpcPoolBytes];
    rapidjson::MemoryPoolAllocator<rapidjson::CrtAllocator> allocator(
        pool_buffer, sizeof(pool_buffer));
    rapidjson::Document document(&allocator);
    document.Parse(payload.c_str(), payload.size());
    if (document.HasParseError() || !document.IsObject()) {
        publish_reject("invalid_request");
        return;
    }
    const auto method_it = document.FindMember("method");
    if (method_it != document.MemberEnd()) {
        if (!method_it->value.IsString()) {
            publish_reject("invalid_request");
            return;
        }
        method.assign(method_it->value.GetString(), method_it->value.GetStringLength());
    } else {
        publish_reject("invalid_request");
        return;
    }
    const auto params_it = document.FindMember("params");
    params = params_it != document.MemberEnd() ? stringify_json_value(params_it->value) : "{}";

    std::string response{"false"};
    {
        std::lock_guard<std::mutex> lk(rpc_mtx_);
        // method 白名单：只允许已注册的 handler（防止未知方法名路由）
        auto it = rpc_handlers_.find(method);
        if (it == rpc_handlers_.end()) {
            spdlog::warn("[CloudService] Rejected unknown RPC method: {}", method);
            publish_reject("method_not_supported");
            return;
        }
        spdlog::info("[CloudService] invoking RPC handler: method='{}' params={}", method, params);
        try {
            response = it->second(request_id, params);
            spdlog::info("[CloudService] RPC handler returned: method='{}' response={}",
                         method,
                         response);
        } catch (...) {
            spdlog::warn("[CloudService] RPC handler threw exception: method='{}'", method);
        }
    }

    // 回复
    if (!request_id.empty()) {
        const auto response_topic = topics_.rpc_response_prefix + request_id;
        spdlog::info("[CloudService] publishing RPC response: topic='{}' payload={}",
                     response_topic,
                     response);
        const bool ok = network_->publish(response_topic, response);
        spdlog::info("[CloudService] published RPC response: topic='{}' ok={}",
                     response_topic,
                     ok);
    }
}

} // namespace robot::service
