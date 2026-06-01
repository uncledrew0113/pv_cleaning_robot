#pragma once
#include <cstdint>
#include <functional>
#include <string>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/middleware/event_bus.h"


namespace robot::service {

struct FaultEvent {
    enum class Level {
        P0,  ///< 严重故障：立即急停，进入故障锁定
        P1,  ///< 一般故障：锁存上报，具体动作由 app 故障策略决定
        P2,  ///< 告警：仅上报（EventBus），不转换 FSM 状态
        P3   ///< 提示：仅记录日志，不影响运行
    };
    Level level{Level::P3};
    uint32_t code{0};
    std::string description;
    uint64_t timestamp_ms{0};
};

namespace FaultCode = robot::domain::FaultCode;

class FaultReporter {
   public:
    virtual ~FaultReporter() = default;
    virtual void report(FaultEvent::Level level, uint32_t code, const std::string& description) = 0;
    virtual void clear_active_fault() = 0;
    virtual bool has_active_fault(FaultEvent::Level min_level = FaultEvent::Level::P2) const = 0;
    virtual FaultEvent last_fault() const = 0;
};

/// @brief 故障服务：记录故障、发布事件，并缓存当前活跃故障。
///
/// 故障等级：
///   P0 — 立即停机（限位触发、通信失联）
///   P1 — 一般故障，锁存上报
///   P2 — 告警上报，不改变当前任务
///   P3 — 仅记录日志
///
/// 该服务不再决定故障动作；业务动作由 app::FaultPolicy 和 RobotController
/// 统一处理，避免 service 层和 app 层各自维护一套策略表。
class FaultService : public FaultReporter {
   public:
    using FaultEvent = service::FaultEvent;

    explicit FaultService(middleware::EventBus& bus);

    /// 报告一个故障；发布到 EventBus，具体动作由 app 层故障桥接统一裁决。
    void report(FaultEvent::Level level, uint32_t code, const std::string& description) override;

    /// 清除当前活跃故障；保留最后一次故障记录用于排障。
    void clear_active_fault() override;

    bool has_active_fault(FaultEvent::Level min_level = FaultEvent::Level::P2) const override;

    FaultEvent last_fault() const override;

   private:
    middleware::EventBus& bus_;
    FaultEvent last_fault_{};
    bool has_fault_{false};
    mutable hal::PiMutex mtx_;
};

}  // namespace robot::service
