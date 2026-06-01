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
        P1,  ///< 一般故障：停滚刷并请求无刷返航
        P2,  ///< 告警：仅上报（EventBus），不转换 FSM 状态
        P3   ///< 提示：仅记录日志，不影响运行
    };
    Level level{Level::P3};
    uint32_t code{0};
    std::string description;
    uint64_t timestamp_ms{0};
};

namespace FaultCode = robot::domain::FaultCode;

enum class FaultAction {
    WarnOnly,
    RejectStart,
    ProtectiveStop,
    StartRecovery,
    BrushOffReturnHome,
    ImmediateEmergencyStop,
};

struct FaultDecision {
    FaultAction action{FaultAction::WarnOnly};
    bool latch{false};
    bool report{true};
};

class FaultReporter {
   public:
    virtual ~FaultReporter() = default;
    virtual void report(FaultEvent::Level level, uint32_t code, const std::string& description) = 0;
    virtual void clear_active_fault() = 0;
    virtual bool has_active_fault(FaultEvent::Level min_level = FaultEvent::Level::P2) const = 0;
    virtual FaultEvent last_fault() const = 0;
};

/// @brief 故障服务——四级故障分类与处理策略
///
/// 故障等级：
///   P0 — 立即停机（限位触发、通信失联）
///   P1 — 安全返回停机位
///   P2 — 告警上报，不改变当前任务
///   P3 — 仅记录日志
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

    /// 根据故障码返回处理策略；不修改内部状态，不发布事件。
    FaultDecision decide(const FaultEvent& event) const;

   private:
    middleware::EventBus& bus_;
    FaultEvent last_fault_{};
    bool has_fault_{false};
    mutable hal::PiMutex mtx_;
};

}  // namespace robot::service
