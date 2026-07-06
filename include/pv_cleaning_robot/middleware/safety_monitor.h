/**
 * @file safety_monitor.h
 * @brief 主限位安全监控器接口。
 *
 * SafetyMonitor 负责主限位接近传感器触发后的首响急停、去抖确认和到位事件发布。
 * 任务切段、故障锁存和恢复流程由 RobotController / ErrorManager 决定。
 */
#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "pv_cleaning_robot/domain/robot_domain.h"
#include "pv_cleaning_robot/middleware/event_bus.h"

namespace robot::device {
class LimitSwitch;
}

namespace robot::middleware {

/// @brief 主限位安全监控器。
///
/// - 安全监控主循环：SCHED_FIFO 优先级 94，绑定 CPU 4；
/// - 端到端响应路径（触发 -> 急停指令发送）目标不超过 50 ms：
///     LimitSwitch::on_edge [GPIO 监控线程，SCHED_FIFO 95]
///         -> on_limit_trigger() [在 GPIO 监控线程中同步调用]
///             -> 注入的 emergency_stop 回调 [生产环境直写行走轮急停 CAN 帧]
///
/// @note 急停函数由组合根注入。当前生产路径绑定 WalkMotorGroup::emergency_override(0.0f)，
/// 只负责最快速度停行走轮；滚刷停机由任务停止或系统级急停路径处理。
class SafetyMonitor {
public:
    struct Config {
        uint64_t limit_settle_stable_ms{30};
        uint64_t limit_release_stable_ms{30};
    };

    /// @brief 限位开关去抖完成事件，monitor_loop 确认持续触发稳定后发布。
    ///
    /// device 层左/右接近传感器在本模块边界转换为 domain 的 Endpoint::A/B，
    /// 上层状态机只处理业务端点，不关心 GPIO 左右安装细节。
    struct LimitSettledEvent {
        domain::Endpoint endpoint;
    };

    SafetyMonitor(std::function<void()> emergency_stop,
                  std::shared_ptr<device::LimitSwitch> left_switch,
                  std::shared_ptr<device::LimitSwitch> right_switch,
                  EventBus& event_bus);
    SafetyMonitor(std::function<void()> emergency_stop,
                  std::shared_ptr<device::LimitSwitch> left_switch,
                  std::shared_ptr<device::LimitSwitch> right_switch,
                  EventBus& event_bus,
                  Config config);
    ~SafetyMonitor();

    /// @brief 启动安全监控。
    ///
    /// GPIO 监控线程负责首响急停，monitor_loop 负责去抖确认和事件发布。
    /// @return 启动成功返回 true；缺少限位开关或 GPIO 未打开时返回 false。
    bool start();

    /// @brief 停止安全监控并等待后台线程退出。
    void stop();

    /// @brief 设置主限位去抖完成后的同步回调。
    void set_limit_settled_callback(std::function<void(domain::Endpoint)> cb);

private:
    /// LimitSwitch 触发回调；在 GPIO 监控线程中执行，必须保持极短路径。
    void on_limit_trigger(domain::Endpoint endpoint);

    /// 安全监控主循环：5ms 轮询兜底，负责去抖确认和 release re-arm。
    void monitor_loop();

    std::function<void()> emergency_stop_;
    std::shared_ptr<device::LimitSwitch> left_switch_;
    std::shared_ptr<device::LimitSwitch> right_switch_;
    EventBus& event_bus_;
    Config config_{};
    std::function<void(domain::Endpoint)> limit_settled_cb_;

    std::atomic<bool> running_{false};
    /// 去抖 pending 时间戳（ms）：GPIO 线程触发后记录触发时刻，0 表示未触发。
    /// monitor_loop 每 5ms 非阻塞检查，确认持续触发稳定后发布 LimitSettledEvent。
    /// 同一侧 pending 不为 0 时，GPIO 回调与备用轮询都会抑制重复触发。
    std::atomic<uint64_t> pending_left_ts_{0};
    std::atomic<uint64_t> pending_right_ts_{0};
    /// 同一侧触发一次后，必须先物理释放（电平回高）才允许重新 armed。
    std::atomic<bool> left_wait_release_{false};
    std::atomic<bool> right_wait_release_{false};
    /// release 候选时间戳（ms）：看到电平回高后开始计时，连续稳定后才清 wait_release。
    std::atomic<uint64_t> left_release_ts_{0};
    std::atomic<uint64_t> right_release_ts_{0};
    std::thread monitor_thread_;
};

}  // namespace robot::middleware
