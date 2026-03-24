/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:18:49
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-16 22:44:11
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/hal/i_serial_port.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace robot::hal {

/// @brief 串口操作的具体结果状态
enum class UartResult {
    OK = 0,
    TIMEOUT,       // 读/写超时 (未收到预期数据)
    DISCONNECTED,  // 设备断开 (未打开，或运行中如 USB 转串口被拔出)
    SYS_ERROR  // 其他系统级错误 (如 libserialport 内部配置错误、底层驱动异常)
};

/// @brief 串口帧格式及底层通信配置
struct UartConfig {
    int baudrate{115200};       ///< 波特率，默认 115200
    int data_bits{8};           ///< 数据位：5/6/7/8
    char parity{'N'};           ///< 校验：'N'=无  'E'=偶  'O'=奇
    int stop_bits{1};           ///< 停止位：1 或 2
    bool flow_control{false};   ///< 是否启用 RTS/CTS 硬件流控
    int write_timeout_ms{200};  ///< 写操作的默认超时时间（毫秒），避免永远死锁
};

/// @brief 串口（UART/RS232）硬件抽象接口
/// 实现：driver/libserialport_port（基于 libserialport）
/// RS485 的物理层切换由硬件自动完成（RTS 方向控制），此接口无需感知
class ISerialPort {
   public:
    virtual ~ISerialPort() = default;

    /// @brief 打开并配置串口，失败返回 false
    virtual bool open() = 0;

    /// @brief 关闭并释放串口
    virtual void close() = 0;

    /// @brief 是否已成功打开
    virtual bool is_open() const = 0;

    /// @brief 写入数据
    /// @param timeout_ms 写入超时。若为 -1，则使用 UartConfig 中的默认配置
    /// @return 实际写入字节数；-1 表示错误
    virtual int write(const uint8_t* buf, size_t len, int timeout_ms = -1) = 0;

    /// @brief 读取数据，阻塞最多 timeout_ms 毫秒
    /// @return 实际读取字节数；0 表示超时；-1 表示错误
    virtual int read(uint8_t* buf, size_t max_len, int timeout_ms) = 0;

    /// @brief 清空输入缓冲区
    virtual bool flush_input() = 0;

    /// @brief 清空输出缓冲区
    virtual bool flush_output() = 0;

    /// @brief 获取接收缓冲区中待读取的字节数
    virtual int bytes_available() = 0;

    /// @brief 获取最后一次底层操作的详细状态/错误码
    virtual UartResult get_last_error() const = 0;
};

}  // namespace robot::hal
