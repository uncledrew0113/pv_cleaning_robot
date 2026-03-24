#include "pv_cleaning_robot/service/fault_service.h"
#include <chrono>

namespace robot::service {

FaultService::FaultService(middleware::EventBus& bus)
    : bus_(bus)
{
}

void FaultService::report(FaultEvent::Level level, uint32_t code,
                          const std::string& description)
{
    FaultEvent evt;
    evt.level       = level;
    evt.code        = code;
    evt.description = description;
    evt.timestamp_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_fault_ = evt;
        has_fault_  = true;
    }

    bus_.publish(evt);
}

bool FaultService::has_active_fault(FaultEvent::Level min_level) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_fault_) return false;
    return static_cast<int>(last_fault_.level) <= static_cast<int>(min_level);
}

const FaultService::FaultEvent& FaultService::last_fault() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return last_fault_;
}

} // namespace robot::service
