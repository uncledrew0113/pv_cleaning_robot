/**
 * @file libgpiod_pin.h
 * @brief libgpiod v1.6 GPIO 引脚驱动接口。
 *
 * 支持输入边沿监控和输出 DO 控制。边沿监控优先使用硬件 IRQ；当 GPIO 控制器不支持
 * 可靠边沿事件时，降级为固定周期软件轮询。业务层 read_value() 读取缓存值，避免
 * SafetyMonitor / 主线程直接卡入 pca953x 等慢速 GPIO 扩展芯片的 ioctl 路径。
 */
#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <shared_mutex>
#include <string>
#include <thread>

#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"

// 前置声明，避免在公共头文件暴露 libgpiod C 类型。
struct gpiod_chip;
struct gpiod_line;

namespace robot::driver {

/// @brief libgpiod v1.6 GPIO 引脚实现，支持输入和输出两种模式。
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
    void monitor_loop();     ///< IRQ 监控循环，硬件边沿事件可用时使用。
    void poll_loop();        ///< 轮询监控循环，IRQ 不可用时降级使用。
    void setup_thread_rt_(); ///< 实时优先级和 CPU 绑定设置，两个监控循环共用。
    bool request_line_as_input();
    void stop_monitoring_locked();
    bool read_hardware_value_locked(bool& value);
    void update_cached_value(bool value);

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
    std::atomic<bool> cached_value_{false};
    std::atomic<bool> cache_valid_{false};
    std::atomic<uint32_t> read_error_count_{0};

    // 读写锁：保证 close() 与 read/write 并发时不会出现文件描述符 TOCTOU。
    mutable std::shared_mutex io_mutex_;
    // 用于立即打断 poll()/等待循环的事件通知 FD。
    std::atomic<int> cancel_fd_{-1};

    // RT 优先级继承互斥量：防止主线程（低优先级）持锁时，
    // GPIO 监控线程（高 RT 优先级）被阻塞而引发优先级反转。
    hal::PiMutex cb_mutex_;
    // 软件去抖状态。
    std::chrono::steady_clock::time_point last_event_time_;
};
}  // namespace robot::driver
