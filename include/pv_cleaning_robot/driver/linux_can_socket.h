/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:19:23
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 09:44:57
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/driver/linux_can_socket.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "pv_cleaning_robot/hal/i_can_bus.h"

namespace robot::driver {

/// @brief Linux SocketCAN 实现（基于 linux/can.h + linux/can/raw.h）
///
/// **线程安全合约（Thread-Safety Contract）：**
///   - `is_open()` / `is_bus_off()` / `get_last_error()` 并发安全（atomic 实现）
///   - `send()` 与 `recv()` 可被不同线程并发调用（两者不共享写状态）
///   - `close()` 调用方**必须**保证所有 `send()` / `recv()` 线程已退出后再调用，
///     否则存在 FD 窜号或野指针风险
class LinuxCanSocket final : public hal::ICanBus {
   public:
    /// @param interface CAN 接口名，如 "can0"
    explicit LinuxCanSocket(std::string interface);
    ~LinuxCanSocket() override;

    bool open() override;
    void close() override;
    bool is_open() const override;

    bool send(const hal::CanFrame& frame) override;
    bool recv(hal::CanFrame& frame, int timeout_ms) override;

    bool set_filters(const hal::CanFilter* filters, size_t count) override;
    bool clear_filter() override;

    hal::CanResult get_last_error() const override {
        return last_error_.load();
    }
    bool is_bus_off() const override {
        return bus_off_.load();
    }
    bool recover() override;
    uint32_t get_tx_drop_count() const override {
        return tx_drop_count_.load(std::memory_order_relaxed);
    }

   private:
    std::string interface_;
    int cancel_fd_{-1};
    // ── 线程安全合约 ──────────────────────────────────────────────────────────
    // socket_fd_ 使用 atomic<int> 避免并发读写的 C++ UB。
    // 但 close() 与 send()/recv() 之间的 TOCTOU（FD 窜号）由调用方保证：
    //   调用方必须在所有 send()/recv() 线程 join 后再调用 close()。
    std::atomic<int> socket_fd_{-1};
    std::atomic<bool> connected_{false};
    std::atomic<bool> bus_off_{false};
    // last_error_ 仅由 recv() 写入（接收方状态）；send() 错误通过返回值传递，
    // 不再写 last_error_，彻底消除 TX/RX 线程互相覆盖错误状态的竞争。
    std::atomic<hal::CanResult> last_error_{hal::CanResult::OK};
    // TX 缓冲区满丢帧计数（RT 热路径，无锁，无日志）
    std::atomic<uint32_t> tx_drop_count_{0};

    // 保存上次 set_filters() 配置，供 recover() 重建 socket 后恢复
    // filter_mutex_ 防止 set_filters()/clear_filter()/recover() 并发修改数组导致的数据竞争
    static constexpr size_t kMaxFilters = 16u;
    hal::CanFilter saved_filters_[kMaxFilters]{};
    size_t saved_filter_count_{0u};
    mutable std::mutex filter_mutex_;
};

}  // namespace robot::driver
