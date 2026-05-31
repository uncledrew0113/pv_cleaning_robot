// @file fault_service.cc
// @brief FaultService 实现：记录故障事件并将它们发布到事件总线。

#include <chrono>

#include "pv_cleaning_robot/service/fault_service.h"

namespace robot::service {

FaultService::FaultService(middleware::EventBus& bus) : bus_(bus) {}

void FaultService::report(FaultEvent::Level level, uint32_t code, const std::string& description) {
    // 将故障信息封装并保存到本地状态，然后发布到事件总线
    FaultEvent evt;
    evt.level = level;
    evt.code = code;
    evt.description = description;
    evt.timestamp_ms =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count());

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_fault_ = evt;
        has_fault_ = level == FaultEvent::Level::P0 || level == FaultEvent::Level::P1 ||
                     level == FaultEvent::Level::P2;
    }

    bus_.publish(evt);
}

void FaultService::clear_active_fault() {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    has_fault_ = false;
}

bool FaultService::has_active_fault(FaultEvent::Level min_level) const {
    // 线程安全检查是否存在不低于指定等级的活动故障
    std::lock_guard<hal::PiMutex> lk(mtx_);
    if (!has_fault_)
        return false;
    return static_cast<int>(last_fault_.level) <= static_cast<int>(min_level);
}

FaultService::FaultEvent FaultService::last_fault() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return last_fault_;
}

}  // namespace robot::service
