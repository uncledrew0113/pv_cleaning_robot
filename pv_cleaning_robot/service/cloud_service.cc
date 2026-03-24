#include "pv_cleaning_robot/service/cloud_service.h"
#include <nlohmann/json.hpp>

namespace robot::service {

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
    // 订阅 RPC 下行
    network_->subscribe(topics_.rpc_request,
        [this](const std::string& t, const std::string& p) {
            on_rpc_message(t, p);
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

void CloudService::on_rpc_message(const std::string& topic,
                                  const std::string& payload)
{
    // ThingsBoard RPC topic: v1/devices/me/rpc/request/{request_id}
    // 从 topic 末尾提取 request_id
    std::string request_id;
    auto pos = topic.rfind('/');
    if (pos != std::string::npos)
        request_id = topic.substr(pos + 1);

    std::string method;
    std::string params;
    try {
        auto j = nlohmann::json::parse(payload);
        method = j.value("method", "");
        params = j.contains("params") ? j["params"].dump() : "{}";
    } catch (...) {
        return;
    }

    std::string response{"false"};
    {
        std::lock_guard<std::mutex> lk(rpc_mtx_);
        auto it = rpc_handlers_.find(method);
        if (it != rpc_handlers_.end()) {
            try { response = it->second(params); } catch (...) {}
        }
    }

    // 回复
    if (!request_id.empty()) {
        network_->publish(topics_.rpc_response_prefix + request_id, response);
    }
}

} // namespace robot::service
