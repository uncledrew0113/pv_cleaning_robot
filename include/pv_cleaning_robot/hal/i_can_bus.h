/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:18:49
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-16 22:18:12
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/hal/i_can_bus.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace robot::hal {

/// @brief CAN 总线操作的具体结果状态 (从底层实现提取到了接口层)
enum class CanResult {
    OK = 0,
    TIMEOUT,   // 接收超时
    BUS_OFF,   // 总线关闭 (致命错误)
    TX_FULL,   // 发送缓冲区满
    SYS_ERROR  // 系统级/硬件级错误
};

/// @brief CAN 原始数据帧（标准帧 + 扩展帧均支持）
struct CanFrame {
    uint32_t id;      ///< CAN ID（标准帧11位，扩展帧29位）
    uint8_t len;      ///< 数据长度 0~8
    uint8_t data[8];  ///< 数据载荷
    bool is_ext;      ///< true = 扩展帧（29-bit ID）
    bool is_rtr;      ///< true = 远程帧

    CanFrame() : id(0), len(0), data{}, is_ext(false), is_rtr(false) {}
};

/// @brief CAN 过滤器配置结构体
struct CanFilter {
    uint32_t id;
    uint32_t mask;
};

/// @brief CAN 总线硬件抽象接口（换底层只需替换此接口实现）
/// 实现：driver/linux_can_socket（基于 linux/can.h SocketCAN）
class ICanBus {
   public:
    virtual ~ICanBus() = default;

    /// 打开并绑定 CAN 接口，失败返回 false
    virtual bool open() = 0;

    /// 关闭并释放资源
    virtual void close() = 0;

    /// 是否已成功打开
    virtual bool is_open() const = 0;

    /// 发送一帧，成功返回 true
    virtual bool send(const CanFrame& frame) = 0;

    /// 接收一帧，阻塞最多 timeout_ms 毫秒
    /// @return true=成功收到一帧; false=超时或错误
    virtual bool recv(CanFrame& frame, int timeout_ms) = 0;

    /// 设置接收过滤器；id & mask == frame.id & mask 的帧才接收
    /// 纯虚接口使用指针+长度，零堆分配；不改变接口时可用下方 vector 便利重载
    virtual bool set_filters(const CanFilter* filters, size_t count) = 0;

    /// 便利重载：接受 vector，转发到纯虚接口，不增加额外堆分配路径
    bool set_filters(const std::vector<CanFilter>& v) {
        return set_filters(v.data(), v.size());
    }

    /// 清除所有过滤器（接收全部帧）
    virtual bool clear_filter() = 0;

    /// @brief 获取最后一次 send 或 recv 的详细错误码
    virtual CanResult get_last_error() const = 0;

    /// @brief 检查当前底层总线是否处于 Bus-Off (硬件掉线) 状态
    virtual bool is_bus_off() const = 0;

    /// @brief 尝试恢复底层硬件故障（如 Bus-Off 复位）
    virtual bool recover() = 0;

    /// @brief TX 缓冲区满导致的丢帧累计计数（无锁原子计数，线程安全）
    /// 默认返回 0；驱动层如有实现可覆盖。可用于上层周期性监控 CAN 总线拥塞程度。
    virtual uint32_t get_tx_drop_count() const { return 0u; }
};

}  // namespace robot::hal
