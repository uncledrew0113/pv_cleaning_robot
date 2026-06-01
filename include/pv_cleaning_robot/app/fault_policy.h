#pragma once

#include <cstdint>
#include <string>

namespace robot::app {

enum class FaultSource {
    SafetyMonitor,
    Watchdog,
    FaultDetector,
    SelfCheck,
    Motion,
    Recovery,
    Controller,
};

struct FaultFact {
    FaultSource source{FaultSource::Controller};
    uint32_t code{0};
    std::string detail;
};

enum class FaultAction {
    WarnOnly,
    RejectStart,
    StartRecovery,
    EmergencyStopAndLatch,
};

struct FaultDecision {
    FaultAction action{FaultAction::WarnOnly};
    bool latch{false};
};

class FaultPolicy {
public:
    FaultDecision decide(const FaultFact& fact) const noexcept;
};

}  // namespace robot::app
