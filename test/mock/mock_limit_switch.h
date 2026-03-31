#pragma once
#include <functional>

#include "pv_cleaning_robot/device/limit_switch.h"

/// @brief LimitSwitch 手工 mock（不依赖 IGpioPin 实际硬件）
struct MockLimitSwitch {
    // ── 可配置返回值 ──────────────────────────────────────────
    bool open_result{true};
    bool current_level_result{false};
    bool triggered_result{false};
    robot::device::LimitSide side_val{robot::device::LimitSide::FRONT};

    // ── 注册的回调（simulate_trigger 触发时调用）──────────────
    robot::device::LimitSwitch::TriggerCallback registered_cb;

    // ── 调用记录 ──────────────────────────────────────────────
    bool opened{false};
    bool closed{false};
    int start_monitoring_count{0};
    int stop_monitoring_count{0};
    int clear_trigger_count{0};

    bool open(int = 95, int = 2, int = 0) {
        opened = open_result;
        return open_result;
    }
    void close() {
        opened = false;
        closed = true;
    }

    void set_trigger_callback(robot::device::LimitSwitch::TriggerCallback cb) {
        registered_cb = std::move(cb);
    }
    void start_monitoring() {
        ++start_monitoring_count;
    }
    void stop_monitoring() {
        ++stop_monitoring_count;
    }

    bool is_triggered() const {
        return triggered_result;
    }
    void clear_trigger() {
        triggered_result = false;
        ++clear_trigger_count;
    }
    bool read_current_level() const {
        return current_level_result;
    }
    robot::device::LimitSide side() const {
        return side_val;
    }

    /// 模拟边沿触发：置位标志并同步调用已注册回调
    void simulate_trigger() {
        triggered_result = true;
        if (registered_cb)
            registered_cb(side_val);
    }
};
