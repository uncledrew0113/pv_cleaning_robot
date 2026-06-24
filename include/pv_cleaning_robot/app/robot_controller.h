#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "pv_cleaning_robot/app/fault_policy.h"
#include "pv_cleaning_robot/domain/robot_domain.h"

namespace robot::app {

enum class RobotState {
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    Recovering,
    Charging,
    FaultStopped,
};

struct CommandResult {
    bool accepted{false};
    std::string reason;
};

struct RobotControllerSnapshot {
    std::string state{"Idle"};
    std::optional<uint32_t> fault;
    uint32_t repeat_count{0};
    int completed_cycles{0};
    uint64_t cfg_ver{0};
    std::optional<domain::RuntimeConfig> active_config;
    std::optional<domain::RuntimeConfig> pending_config;
};

class RobotController {
public:
    struct ActionPorts {
        std::function<bool(const domain::MissionSegment&)> start_segment;
        std::function<void()> stop_motion;
        std::function<void()> emergency_stop;
        std::function<void()> start_recovery;
        std::function<void()> clear_fault;
    };

    struct ConfigPorts {
        std::function<domain::RuntimeConfig()> active_runtime_config;
        std::function<std::optional<domain::RuntimeConfig>()> pending_runtime_config;
        std::function<uint64_t(const domain::RuntimeConfig&)> runtime_config_version;
        std::function<bool()> promote_pending_runtime_config;
        std::function<domain::LaneConfig()> lane_config;
    };

    RobotController() = default;
    explicit RobotController(ActionPorts ports);
    ~RobotController();

    void start();
    void stop();
    void set_config_ports(ConfigPorts ports);
    void set_position_state_query(std::function<domain::PositionState()> query);
    void set_battery_soc_query(std::function<float()> query);

    CommandResult submit_command(const domain::RobotCommand& command);
    RobotControllerSnapshot snapshot() const;

    void post_limit_settled(domain::Endpoint endpoint);
    void post_limit_unstable(domain::Endpoint endpoint);
    void post_watchdog_timeout(std::string thread_name);
    void post_recovery_finished(bool ok);
    void post_schedule_window_hit();
    void post_fault(FaultFact fact);
    void post_tick();

    void complete_self_check(bool ok);
    void complete_self_check_for_test(bool ok);
    void handle_limit_settled_for_test(domain::Endpoint endpoint);
    void handle_fault_for_test(const FaultFact& fact);
    void post_for_test(std::function<void()> fn);
    void drain_for_test();

private:
    static const char* state_name(RobotState state) noexcept;
    void loop();
    void post(std::function<void()> fn);
    bool mission_active() const noexcept;
    CommandResult submit_command_locked(const domain::RobotCommand& command);
    CommandResult start_command_locked(const domain::RobotCommand& command);
    CommandResult validate_start_command_locked(const domain::RobotCommand& command,
                                                const domain::LaneConfig& lane,
                                                domain::PositionState position_state) const;
    domain::MissionContext build_start_mission_locked(const domain::RobotCommand& command,
                                                      const domain::LaneConfig& lane,
                                                      domain::PositionState position_state) const;
    CommandResult stop_locked();
    bool start_current_segment_locked();
    void reset_source_limit_repeat_locked() noexcept;
    void complete_self_check_locked(bool ok);
    void handle_limit_settled_locked(domain::Endpoint endpoint);
    void handle_source_limit_repeat_locked(domain::Endpoint endpoint);
    void handle_limit_unstable_locked(domain::Endpoint endpoint);
    void handle_watchdog_timeout_locked(const std::string& thread_name);
    void handle_recovery_finished_locked(bool ok);
    void handle_fault_locked(const FaultFact& fact);

    mutable std::mutex mtx_;
    RobotState state_{RobotState::Idle};
    std::optional<domain::MissionContext> mission_;
    std::optional<uint32_t> active_fault_;
    FaultPolicy fault_policy_;
    ActionPorts actions_{};
    ConfigPorts config_{};
    std::function<domain::PositionState()> position_state_query_;
    std::function<float()> battery_soc_query_;
    int source_limit_repeat_count_{0};

    mutable std::mutex queue_mtx_;
    std::condition_variable queue_cv_;
    std::condition_variable idle_cv_;
    std::deque<std::function<void()>> queue_;
    bool running_{false};
    bool stop_requested_{false};
    bool handling_event_{false};
    std::thread worker_;
};

}  // namespace robot::app
