#pragma once
#include "pv_cleaning_robot/middleware/event_bus.h"
#include <cstdint>
#include <functional>
#include <string>

namespace robot::service {

/// @brief 故障服务——四级故障分类与处理策略
///
/// 故障等级：
///   P0 — 立即停机（限位触发、通信失联）
///   P1 — 安全返回停机位
///   P2 — 降速继续清扫，告警上报
///   P3 — 仅记录日志
class FaultService {
public:
    struct FaultEvent {
        enum class Level { P0, P1, P2, P3 };
        Level       level{Level::P3};
        uint32_t    code{0};
        std::string description;
        uint64_t    timestamp_ms{0};
    };

    explicit FaultService(middleware::EventBus& bus);

    /// 报告一个故障（发布到 EventBus，由 RobotFsm 和 FaultHandler 响应）
    void report(FaultEvent::Level level, uint32_t code,
                const std::string& description);

    bool has_active_fault(FaultEvent::Level min_level = FaultEvent::Level::P2) const;

    const FaultEvent& last_fault() const;

private:
    middleware::EventBus& bus_;
    FaultEvent            last_fault_{};
    bool                  has_fault_{false};
    mutable std::mutex    mtx_;
};

} // namespace robot::service
