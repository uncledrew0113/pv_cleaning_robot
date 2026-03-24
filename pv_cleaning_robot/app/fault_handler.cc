#include "pv_cleaning_robot/app/fault_handler.h"

namespace robot::app {

FaultHandler::FaultHandler(std::shared_ptr<service::MotionService> motion,
                           middleware::EventBus&                   bus,
                           FsmDispatchFn                           dispatch_fn)
    : motion_(std::move(motion))
    , bus_(bus)
    , dispatch_fn_(std::move(dispatch_fn))
{
}

FaultHandler::~FaultHandler()
{
    if (subscription_id_ >= 0) {
        bus_.unsubscribe(subscription_id_);
        subscription_id_ = -1;
    }
}

void FaultHandler::start_listening()
{
    subscription_id_ = bus_.subscribe<service::FaultService::FaultEvent>(
        [this](const service::FaultService::FaultEvent& evt) {
            on_fault(evt);
        });
}

void FaultHandler::on_fault(const service::FaultService::FaultEvent& evt)
{
    using Level = service::FaultService::FaultEvent::Level;
    switch (evt.level) {
    case Level::P0:
        // 立即急停，然后通知 FSM
        motion_->emergency_stop();
        dispatch_fn_(evt);
        break;
    case Level::P1:
        // 停止清扫，启动返回
        motion_->stop_cleaning();
        motion_->start_returning();
        dispatch_fn_(evt);
        break;
    case Level::P2:
        // 仅上报，不停机（降速由 MotionService 外部策略决定）
        break;
    case Level::P3:
        // 仅日志（已由 FaultService 发布）
        break;
    }
}

} // namespace robot::app
