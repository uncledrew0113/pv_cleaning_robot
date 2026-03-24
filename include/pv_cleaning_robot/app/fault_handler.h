#pragma once
#include <functional>

#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::app {

/// @brief 故障处理器——根据故障等级决定处理策略
///
/// P0 → 立即原地停机，置 FSM Fault 状态
/// P1 → 停止清扫，启动返回停机位流程，置 FSM Returning 状态
/// P2 → 降速继续，告警上报（由 FaultService 发布 EventBus）
/// P3 → 仅记录日志
class FaultHandler {
   public:
    using FsmDispatchFn = std::function<void(service::FaultService::FaultEvent)>;

    FaultHandler(std::shared_ptr<service::MotionService> motion,
                 middleware::EventBus& bus,
                 FsmDispatchFn dispatch_fn);

    ~FaultHandler();  ///< 析构时自动取消 EventBus 订阅，防止回调悬空指针/// 注册 EventBus 监听（在
                      ///< EventBus 上订阅 FaultEvent）
    void start_listening();

   private:
    void on_fault(const service::FaultService::FaultEvent& evt);

    std::shared_ptr<service::MotionService> motion_;
    middleware::EventBus& bus_;
    FsmDispatchFn dispatch_fn_;
    int subscription_id_{-1};
};

}  // namespace robot::app
