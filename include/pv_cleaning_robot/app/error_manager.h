/**
 * @file error_manager.h
 * @brief 应用层错误仲裁和恢复调度接口。
 *
 * 本文件定义标准错误事实、错误决策、恢复计划和错误处理周期服务。ErrorManager
 * 只基于诊断快照和外部上报事件做确定性仲裁，不直接访问硬件；ErrorHandlingService
 * 负责把仲裁结果交给状态机和恢复执行器，避免恢复动作与错误判定耦合。
 */
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

/// @brief 应用层统一错误码，用于屏蔽设备层不同错误来源。
enum class ErrorCode {
    DriverCommError,
    WalkMotorStall,
    BrushMotorFault,
    GpsStuck,
    AttitudeLimit,
    AttitudeLimitBoth,
    RecoveryFailed,
};

/// @brief 错误所属组件类别，用于映射故障码和恢复策略。
enum class ComponentKind {
    WalkMotorGroup,
    BrushMotor,
    Bms,
    Gps,
    Imu,
    GpsStuckService,
    AttitudeLimitSwitch,
};

/// @brief 组件实例标识；index 用于区分同类多实例设备。
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

/// @brief ErrorManager 对单个错误事实给出的处理动作。
enum class ErrorAction {
    Ignore,
    WarnOnly,
    StartRecovery,
    FaultStopped,
};

/// @brief 可由恢复执行器执行的恢复策略标识。
enum class RecoveryPlanId {
    None,
    RecoverAttitudeCenter,
    RecoverAttitudeCenterThenReverse,
};

/// @brief 错误仲裁结果，供状态机和恢复执行器消费。
struct ErrorDecision {
    ErrorAction action{ErrorAction::Ignore};
    RecoveryPlanId plan{RecoveryPlanId::None};
    ComponentId component{};
    bool requires_robot_recovering{false};
    bool latch_fault{false};
    ErrorFact root_error{};
};

/// @brief 恢复执行器返回给 ErrorManager 的执行结果事实。
struct RecoveryResultFact {
    ErrorDecision decision{};
    bool ok{false};
    std::string reason;
    uint64_t timestamp_ms{0};
};

/**
 * @brief 错误仲裁核心。
 *
 * 该类维护连续错误计数、数据流超时状态和姿态限位恢复重复计数。所有状态受内部互斥锁保护；
 * 调用方可以从不同周期任务提交事件，但硬件 IO 和状态机切换必须在类外完成。
 */
class ErrorManager {
public:
    struct Config {
        uint32_t consecutive_error_limit{10};
        uint64_t stream_timeout_ms{3000};
        uint64_t walk_stall_duration_ms{2500};
        uint64_t attitude_repeat_gap_ms{8000};
        uint32_t attitude_reverse_attempt_count{3};
        uint32_t attitude_fault_count{4};
    };

    ErrorManager();
    explicit ErrorManager(Config config);

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

    // 姿态恢复重复计数只关注“同一任务段 key 恢复后是否很快再次触发”。
    // key 由 ErrorFact.detail 传入；不使用 GPS 距离，避免低精度位置影响恢复判定。
    struct AttitudeRecoveryState {
        bool has_last_key{false};
        std::string last_key;
        uint64_t last_recovery_finished_ms{0};
        uint32_t repeat_count{0};
    };

    static bool same_component(ComponentId lhs, ComponentId rhs) noexcept;
    static bool same_error_key(const ErrorFact& lhs, const ErrorFact& rhs) noexcept;
    static bool is_motion_recovery_plan(RecoveryPlanId plan) noexcept;
    static std::string attitude_limit_key(const ErrorFact& fact);
    bool should_suppress_locked(const ErrorFact& fact) const;
    ErrorDecision decide(const ErrorFact& fact);
    ErrorDecision decide_attitude_limit(const ErrorFact& fact);
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
    bool gps_stuck_snapshot_active_{false};
    AttitudeRecoveryState attitude_recovery_state_;
    std::optional<ErrorDecision> active_recovery_;
    Config config_{};
};

/// @brief 错误处理调度服务：封装“采集错误、仲裁、执行恢复、通知状态机”的周期循环。
///
/// 该类只通过 Ports 访问外部对象，不直接持有设备、服务或控制器指针，便于组合根组装
/// 和单元测试。恢复执行发生在周期服务内，状态切换仍由 RobotController 决定。
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
