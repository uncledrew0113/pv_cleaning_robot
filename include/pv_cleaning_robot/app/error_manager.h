#pragma once

#include <cstdint>
#include <array>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "pv_cleaning_robot/domain/diagnostics_snapshot.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::app {

enum class ErrorCode {
    DriverCommError,
    WalkMotorStall,
    BrushMotorFault,
    GpsStuck,
    AttitudeLimit,
    AttitudeLimitBoth,
    RecoveryFailed,
};

enum class ComponentKind {
    WalkMotorGroup,
    BrushMotor,
    Bms,
    Gps,
    Imu,
    GpsStuckService,
    AttitudeLimitSwitch,
};

struct ComponentId {
    ComponentKind kind{ComponentKind::WalkMotorGroup};
    int index{0};
};

/// @brief 外部模块上报给 ErrorManager 的标准错误事实。
///
/// 该结构只描述“已经发生了什么”，不携带处理策略。ErrorManager 只依据这些
/// 确定性字段仲裁错误、选择恢复流程，不直接访问硬件或服务对象。
struct ErrorFact {
    ErrorCode code{ErrorCode::DriverCommError};
    ComponentId component{};
    std::string detail;
    uint64_t timestamp_ms{0};
};

enum class ErrorAction {
    Ignore,
    WarnOnly,
    StartRecovery,
    FaultStopped,
};

enum class RecoveryPlanId {
    None,
    RecoverWalkMotorGroup,
    RecoverBms,
    RecoverBrushMotor,
    RecoverGps,
    RecoverImu,
    RecoverWalkStall,
    RecoverGpsStuckReverse,
    RecoverAttitudeCenter,
};

struct ErrorDecision {
    ErrorAction action{ErrorAction::Ignore};
    RecoveryPlanId plan{RecoveryPlanId::None};
    ComponentId component{};
    bool requires_robot_recovering{false};
    bool latch_fault{false};
    ErrorFact root_error{};
};

struct RecoveryResultFact {
    ErrorDecision decision{};
    bool ok{false};
    std::string reason;
    uint64_t timestamp_ms{0};
};

class ErrorManager {
public:
    /// @brief 主动提交一个已发现的错误事实，并立即返回确定性处理决策。
    ErrorDecision submit_error(const ErrorFact& fact);

    /// @brief 将 WatchdogMgr 的线程超时事件转换为标准错误事实。
    ///
    /// 线程名在这里映射到对应组件，避免各个调用点重复维护“线程 -> 设备根因”的规则。
    ErrorDecision submit_watchdog_timeout(const std::string& thread_name, uint64_t now_ms);

    /// @brief 按周期更新诊断快照，内部根据计数停滞和时间窗口产生错误决策。
    ///
    /// 诊断快照来自 DiagnosticsCollector；这里不主动读取设备，保证错误仲裁与硬件 IO 解耦。
    void update(const domain::DiagnosticsSnapshot& snapshot, uint64_t now_ms);
    void mark_recovery_started(const ErrorDecision& decision);
    ErrorDecision mark_recovery_finished(const RecoveryResultFact& result);

    /// @brief 取出并清空待处理决策；调用方负责按优先级交给状态机和恢复执行器。
    std::vector<ErrorDecision> drain_decisions();

private:
    // 连续错误计数状态：只要 count 变化就视为一次错误递增，兼容 uint32_t 回绕。
    struct ErrorCounterState {
        bool initialized{false};
        uint32_t previous{0};
        uint32_t consecutive_increments{0};
    };

    // 持续布尔错误状态：用于“连续保持一段时间才算错误”的规则。
    struct DurationFlagState {
        bool active{false};
        uint64_t active_since_ms{0};
        bool reported{false};
    };

    // 数据流超时状态：同一次停滞只上报一次，直到数据流重新更新后重新武装。
    struct StreamTimeoutState {
        bool timed_out{false};
        uint64_t last_seen_update_ms{0};
    };

    // 恢复失败计数只按当前恢复目标连续累计；目标变化或恢复成功都会清零。
    struct RecoveryFailureState {
        bool active{false};
        RecoveryPlanId plan{RecoveryPlanId::None};
        ComponentId component{};
        uint32_t consecutive_failures{0};
    };

    static RecoveryPlanId plan_for_component(ComponentKind kind) noexcept;
    static bool requires_robot_recovering(RecoveryPlanId plan) noexcept;
    static bool same_component(ComponentId lhs, ComponentId rhs) noexcept;
    static bool same_error_key(const ErrorFact& lhs, const ErrorFact& rhs) noexcept;
    static bool is_motion_recovery_plan(RecoveryPlanId plan) noexcept;
    bool should_suppress_locked(const ErrorFact& fact) const;
    ErrorDecision decide(const ErrorFact& fact);
    ErrorDecision submit_error_locked(const ErrorFact& fact);
    void update_error_counter(ErrorCounterState& state,
                              uint32_t count,
                              ComponentKind component,
                              uint64_t now_ms);
    void update_stream_timeout(const domain::StreamHealth& health,
                               StreamTimeoutState& state,
                               ComponentKind component,
                               uint64_t timeout_ms,
                               uint64_t now_ms);

    mutable std::mutex mtx_;
    std::vector<ErrorDecision> pending_decisions_;
    ErrorCounterState brush_error_state_;
    DurationFlagState walk_stall_state_;
    StreamTimeoutState bms_update_timeout_state_;
    StreamTimeoutState gps_timeout_state_;
    StreamTimeoutState imu_timeout_state_;
    std::array<StreamTimeoutState, 4> walk_feedback_timeout_states_{};
    DurationFlagState brush_fault_state_;
    std::vector<uint64_t> walk_stall_event_times_;
    bool gps_stuck_snapshot_active_{false};
    std::vector<uint64_t> gps_stuck_event_times_;
    RecoveryFailureState recovery_failure_state_;
    std::optional<ErrorDecision> active_recovery_;
};

/// @brief 错误处理调度服务：封装“采集错误、仲裁、执行恢复、通知状态机”的周期循环。
///
/// 该类和 ErrorManager 放在同一文件中，避免把 v1 错误处理拆成过多模块。它只通过 Ports
/// 访问外部对象，不直接持有设备、服务或控制器指针，便于组合根组装和单元测试。
class ErrorHandlingService : public middleware::IRunnable {
public:
    struct Ports {
        std::function<std::string()> current_state;
        std::function<std::optional<ErrorFact>()> consume_error_event;
        std::function<domain::DiagnosticsSnapshot()> diagnostics_snapshot;
        std::function<void(bool)> set_gps_stuck_monitoring;
        std::function<void(const ErrorDecision&)> apply_error_decision;
        std::function<RecoveryResultFact(const ErrorDecision&)> execute_recovery;
        std::function<void(bool)> post_recovery_finished;
        std::function<uint64_t()> now_ms;
    };

    ErrorHandlingService(ErrorManager& manager, Ports ports);

    void update() override;

private:
    static bool is_task_running_state(const std::string& state);
    static bool is_motion_recovery_plan(RecoveryPlanId plan);
    static bool recovery_plan_allowed(const std::string& state, RecoveryPlanId plan);
    static int decision_priority(const ErrorDecision& decision) noexcept;
    uint64_t now_ms() const;

    ErrorManager& manager_;
    Ports ports_;
    bool gps_stuck_monitoring_enabled_{false};
};

}  // namespace robot::app
