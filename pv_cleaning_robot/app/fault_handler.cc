#include "pv_cleaning_robot/app/fault_handler.h"

#include <array>

namespace robot::app {

bool FaultHandler::should_promote_p2_to_p1(uint32_t code) noexcept {
    static constexpr std::array<uint32_t, 1> kPromoteToP1Codes{
        service::FaultCode::kGpsLostRequiresReturn,
    };
    for (const auto promote_code : kPromoteToP1Codes) {
        if (promote_code == code) {
            return true;
        }
    }
    return false;
}

FaultHandler::FaultHandler(std::shared_ptr<domain::EmergencyStopPort> emergency_stop,
                           std::shared_ptr<service::FaultReporter> fault,
                           middleware::EventBus& bus,
                           FsmDispatchFn dispatch_fn)
    : emergency_stop_(std::move(emergency_stop))
    , fault_(std::move(fault))
    , bus_(bus)
    , dispatch_fn_(std::move(dispatch_fn)) {}

FaultHandler::~FaultHandler() {
    if (subscription_id_ >= 0) {
        bus_.unsubscribe(subscription_id_);
        subscription_id_ = -1;
    }
}

void FaultHandler::start_listening() {
    subscription_id_ =
        bus_.subscribe<service::FaultEvent>([this](const service::FaultEvent& evt) { on_fault(evt); });
}

void FaultHandler::on_fault(const service::FaultEvent& evt) {
    using Level = service::FaultEvent::Level;
    switch (evt.level) {
        case Level::P0:
            // 立即急停，然后通知 FSM
            emergency_stop_->emergency_stop();
            dispatch_fn_(evt);
            break;
        case Level::P1:
            // P1：停止滚刷并进入无刷返航段，由 FSM 维护返航状态与完成条件。
            dispatch_fn_(evt);
            break;
        case Level::P2:
            if (evt.code == service::FaultCode::kTransientAttitudeError) {
                dispatch_fn_(evt);
                break;
            }
            if (should_promote_p2_to_p1(evt.code)) {
                // 统一通过 FaultService::report() 走“记录 active fault → EventBus → FaultHandler(P1) →
                // FSM”的单一真相链，避免重复 dispatch 和外部 fault 失真。
                fault_->report(Level::P1, evt.code, evt.description);
            }
            break;
        case Level::P3:
            // 仅日志（已由 FaultService 发布）
            break;
    }
}

}  // namespace robot::app
