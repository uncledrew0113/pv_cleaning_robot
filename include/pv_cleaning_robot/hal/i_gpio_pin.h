/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:18:49
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-16 23:08:52
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/hal/i_gpio_pin.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <functional>

namespace robot::hal {

/// @brief GPIO 边沿类型
enum class GpioEdge {
    RISING,   ///< 上升沿（低→高）
    FALLING,  ///< 下降沿（高→低）
    BOTH      ///< 双边沿
};

/// @brief GPIO 方向
enum class GpioDirection {
    INPUT,  ///< 输入（默认）
    OUTPUT  ///< 输出（继电器/电子开关控制）
};

/// @brief GPIO 上下拉偏置
enum class GpioBias {
    DISABLE,   ///< 无上下拉
    PULL_UP,   ///< 上拉
    PULL_DOWN  ///< 下拉
};

/// @brief GPIO 统一配置结构体
struct GpioConfig {
    GpioDirection direction{GpioDirection::INPUT};
    GpioBias bias{GpioBias::DISABLE};
    int debounce_ms{0};   ///< 软件消抖时间（毫秒），0 表示不开启消抖
    int rt_priority = 0; ///< SCHED_FIFO 优先级（0=不提升）
    int cpu_affinity = 0; ///< CPU 亲和性掩码（0=不绑定；1<<N=绑定到核心N）
};

/// @brief GPIO 引脚硬件抽象接口
///
/// 支持输入和输出两种方向：
///   - 输入：轮询 read_value() 或 set_edge_callback() + start_monitoring()
///   - 输出：open() 前调 set_direction(OUTPUT)，然后 write_value()
///
/// 实现：driver/libgpiod_pin（基于 libgpiod v1.6）
class IGpioPin {
   public:
    virtual ~IGpioPin() = default;

    /// @brief 申请 GPIO 线并应用配置，失败返回 false
    virtual bool open(const GpioConfig& config) = 0;

    /// @brief 释放 GPIO 线资源
    virtual void close() = 0;

    /// @brief 是否已成功打开
    virtual bool is_open() const = 0;

    /// @brief 读取当前引脚电平（true=高电平）
    virtual bool read_value() = 0;

    /// @brief 输出高/低电平（仅输出模式有效，true=高电平）
    virtual bool write_value(bool high) = 0;

    /// @brief 注册边沿回调（仅输入模式，start_monitoring() 后触发）
    /// @note 回调在专用线程中执行，严禁在回调中执行长耗时/阻塞操作！
    /// @note 禁止在回调内调用 stop_monitoring()：实现层虽有 self-join 兜底（detach
    ///       而非死锁），但 detach 后资源清理顺序不受控。建议通过 EventBus 将
    ///       停止请求转发到主线程处理。
    virtual void set_edge_callback(GpioEdge edge, std::function<void()> cb) = 0;

    /// @brief 启动边沿事件监听线程
    virtual void start_monitoring() = 0;

    /// @brief 停止边沿事件监听线程
    virtual void stop_monitoring() = 0;
};

}  // namespace robot::hal
