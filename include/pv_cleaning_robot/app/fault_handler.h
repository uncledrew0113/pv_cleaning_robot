#pragma once
#include <functional>
#include <memory>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/service/fault_service.h"

namespace robot::app {

/// @brief 故障处理器——根据故障等级决定处理策略
///
/// P0 → 立即原地停机，置 FSM FaultStopped 状态
/// P1 → 停刷并切换到无刷返航段，由 FSM 执行返航
/// P2 → 仅告警上报（由 FaultService 发布 EventBus）
/// P3 → 仅记录日志
class FaultHandler {
   public:
    using FsmDispatchFn = std::function<void(service::FaultEvent)>;

    FaultHandler(std::shared_ptr<domain::EmergencyStopPort> emergency_stop,
                 std::shared_ptr<service::FaultReporter> fault,
                 middleware::EventBus& bus,
                 FsmDispatchFn dispatch_fn);

    /// 析构时自动取消 EventBus 订阅，防止回调访问悬空指针
    ~FaultHandler();

    /// 注册 EventBus 监听（订阅 FaultEvent），必须在急停端口就绪后调用
    void start_listening();

   private:
    static bool should_promote_p2_to_p1(uint32_t code) noexcept;
    void on_fault(const service::FaultEvent& evt);

    std::shared_ptr<domain::EmergencyStopPort> emergency_stop_;
    std::shared_ptr<service::FaultReporter> fault_;
    middleware::EventBus& bus_;
    FsmDispatchFn dispatch_fn_;
    int subscription_id_{-1};
};

}  // namespace robot::app
