/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:19:23
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 11:37:17
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/driver/libgpiod_pin.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <shared_mutex>
#include <string>
#include <thread>

#include "pv_cleaning_robot/driver/pi_mutex.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

// 前置声明，避免暴露 libgpiod C 类型
struct gpiod_chip;
struct gpiod_line;

namespace robot::driver {

/// @brief libgpiod v1.6 GPIO 引脚实现（支持输入/输出两种模式）
class LibGpiodPin final : public hal::IGpioPin {
   public:
    /// @param chip_name GPIO 控制器名，如 "gpiochip0"
    /// @param line_num  GPIO 引脚编号
    /// @param consumer  申请标识字符串（调试用）
    LibGpiodPin(std::string chip_name, unsigned int line_num, std::string consumer = "pv_robot");
    ~LibGpiodPin() override;

    bool open(const hal::GpioConfig& config) override;
    void close() override;
    bool is_open() const override;

    bool read_value() override;
    bool write_value(bool high) override;

    void set_edge_callback(hal::GpioEdge edge, std::function<void()> cb) override;
    void start_monitoring() override;
    void stop_monitoring() override;

   private:
    void monitor_loop();     ///< 中断驱动循环（硬件 IRQ 可用时）
    void poll_loop();        ///< 轮询循环（IRQ 不可用时自动回退，1ms 检测周期）
    void setup_thread_rt_(); ///< RT 提权 + CPU 绑定（两个循环共用）
    bool request_line_as_input();
    void stop_monitoring_locked();

    std::string chip_name_;
    unsigned int line_num_;
    std::string consumer_;

    gpiod_chip* chip_{nullptr};
    gpiod_line* line_{nullptr};

    hal::GpioConfig config_;
    hal::GpioEdge edge_{hal::GpioEdge::BOTH};
    std::function<void()> callback_;

    std::thread monitor_thread_;
    std::atomic<bool> running_{false};

    // ── 核心并发控制器 ──
    // 读写锁：保证 close() 与 read/write 并发时的绝对安全 (防 TOCTOU)
    mutable std::shared_mutex io_mutex_;
    // 用于瞬间打断 poll() 的事件通知 FD
    int cancel_fd_{-1};

    // RT 优先级继承互斥量：防止主线程（低优先级）持锁时，
    // GPIO 监控线程（高 RT 优先级）被阻塞而引发优先级反转。
    PiMutex cb_mutex_;
    // 软件消抖状态
    std::chrono::steady_clock::time_point last_event_time_;
};
}  // namespace robot::driver
