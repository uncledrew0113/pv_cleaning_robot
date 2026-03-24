#pragma once
#include "pv_cleaning_robot/hal/i_gpio_pin.h"
#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace robot::device {

enum class LimitSide { FRONT, REAR };

/// @brief 感应式限位开关（GPIO边沿触发）
///
/// 触发后通过回调通知 SafetyMonitor（安全优先路径），
/// 同时置位原子标志供轮询查询。由 SafetyMonitor 完成
/// 紧急停车指令发送（目标：端到端 ≤50 ms）。
class LimitSwitch {
public:
    using TriggerCallback = std::function<void(LimitSide)>;

    explicit LimitSwitch(std::shared_ptr<hal::IGpioPin> pin, LimitSide side);
    ~LimitSwitch();

    // ── 生命周期 ─────────────────────────────────────────────
    bool open();
    void close();

    // ── 监控 ─────────────────────────────────────────────────
    /// 启动边沿监控线程（FALLING 触发 = 感应到障碍物/轨道端头）
    void start_monitoring();
    void stop_monitoring();

    // ── 回调（由 SafetyMonitor 注册，在监控线程内调用）──────
    void set_trigger_callback(TriggerCallback cb);

    // ── 状态（原子，可供任意线程轮询）───────────────────────
    bool is_triggered() const;
    void clear_trigger();

    LimitSide side() const { return side_; }

private:
    void on_edge();

    std::shared_ptr<hal::IGpioPin> pin_;
    LimitSide                      side_;
    TriggerCallback                callback_;

    std::atomic<bool>              triggered_{false};
};

}  // namespace robot::device
