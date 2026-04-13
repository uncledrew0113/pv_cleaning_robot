/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:19:39
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 16:38:30
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/device/limit_switch.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

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
    /// @param rt_priority 实时调度优先级（默认95，确保极限响应）
    /// @param debounce_ms 软件消抖时间
    /// @param cpu_affinity CPU 亲和性掩码（0=不绑定，1<<N=绑定到核心N）
    bool open(int rt_priority = 95, int debounce_ms = 2, int cpu_affinity = 0,
              bool use_irq = true);
    void close();

    // ── 监控 ─────────────────────────────────────────────────
    /// 启动边沿监控线程（FALLING 触发 = 感应到障碍物/轨道端头）
    void start_monitoring();
    void stop_monitoring();

    // ── 回调（由 SafetyMonitor 注册，在监控线程内调用）──────
    /// @warning 请在 start_monitoring() 前调用以保证最佳性能
    void set_trigger_callback(TriggerCallback cb);

    // ── 状态（原子，可供任意线程轮询）───────────────────────
    bool is_triggered() const;
    void clear_trigger();

    /// @brief 直接读取 GPIO 当前电平（true=高/1，false=低/0）
    /// @note 用于上电自检：尾部期望 false（低=在停机位），前部期望 true（高=无遮挡）
    /// @pre  必须在 open() 之后调用
    bool read_current_level() const;

    LimitSide side() const {
        return side_;
    }

   private:
    void on_edge();

    std::shared_ptr<hal::IGpioPin> pin_;
    LimitSide side_;

    // 保护 callback_ 免受跨线程并发读写造成的 UB
    hal::PiMutex cb_mtx_;
    TriggerCallback callback_;

    std::atomic<bool> triggered_{false};
};

}  // namespace robot::device
