#pragma once
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/network_manager.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::service {

/// @brief 云端通信服务——遥测上报 + RPC 下行处理
///
/// 负责：
///   1. 定期从各 Service 收集 Status/Diagnostics 并通过 NetworkManager 上报
///   2. 订阅 ThingsBoard RPC request topic，分发到注册的 RPC 处理器
///   3. 网络离线时将遥测写入 DataCache；网络恢复后回填上报
class CloudService : public middleware::IRunnable {
   public:
    using RpcHandler = std::function<std::string(const std::string& params)>;
    /// 共享属性下行回调（ThingsBoard 服务端推送，参数为解析后的 JSON 对象）
    using AttrCallback = std::function<void(const nlohmann::json& attrs)>;

    struct Topics {
        std::string telemetry{"v1/devices/me/telemetry"};
        std::string attributes{"v1/devices/me/attributes"};
        std::string rpc_request{"v1/devices/me/rpc/request/+"};
        std::string rpc_response_prefix{"v1/devices/me/rpc/response/"};
    };

    CloudService(std::shared_ptr<middleware::NetworkManager> network,
                 std::shared_ptr<middleware::DataCache> cache,
                 Topics topics);
    /// 使用默认 Topics（ThingsBoard 标准路径）
    CloudService(std::shared_ptr<middleware::NetworkManager> network,
                 std::shared_ptr<middleware::DataCache> cache);

    /// 上报遥测数据（自动路由到 NetworkManager 或 DataCache）
    bool publish_telemetry(const std::string& json_payload);

    /// 上报设备属性
    bool publish_attributes(const std::string& json_payload);

    /// 注册 RPC 方法处理器
    /// @param method  RPC 方法名（如 "setCleanMode"）
    /// @param handler 返回 JSON 字符串作为响应
    void register_rpc(const std::string& method, RpcHandler handler);

    /// 注册服务端共享属性下行回调
    /// 订阅 ThingsBoard v1/devices/me/attributes topic，服务端推送时调用 cb
    /// @note 须在网络连接后调用（NetworkManager::connect() 之后）
    void subscribe_shared_attributes(AttrCallback cb);

    /// 尝试将 DataCache 中的积压数据上传（网络恢复时调用）
    void flush_cache();

    void update() override;  ///< 由 ThreadExecutor 调用

   private:
    void on_rpc_message(const std::string& topic, const std::string& payload);

    std::shared_ptr<middleware::NetworkManager> network_;
    std::shared_ptr<middleware::DataCache> cache_;
    Topics topics_;
    std::unordered_map<std::string, RpcHandler> rpc_handlers_;
    AttrCallback attr_cb_;  ///< 共享属性回调（为空则不分发）
    std::mutex rpc_mtx_;
};

}  // namespace robot::service
