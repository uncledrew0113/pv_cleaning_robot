/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:19:23
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 10:28:13
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/driver/libserialport_port.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>

#include "pv_cleaning_robot/hal/i_serial_port.h"

// 前置声明，避免在头文件暴露 libserialport 类型
struct sp_port;

namespace robot::driver {

/// @brief libserialport 串口实现
/// 支持 UART（IMU/GPS）和 RS485（由硬件自动切换方向）
class LibSerialPort final : public hal::ISerialPort {
   public:
    /// @param port_name 串口设备路径，如 "/dev/ttyS1"
    /// @param baudrate  波特率，如 921600
    /// @param cfg       帧格式（省略则使用默认 8N1）
    explicit LibSerialPort(std::string port_name, hal::UartConfig config = hal::UartConfig{});
    ~LibSerialPort() override;

    bool open() override;
    void close() override;
    bool is_open() const override;

    int write(const uint8_t* buf, size_t len, int timeout_ms = -1) override;
    int read(uint8_t* buf, size_t max_len, int timeout_ms) override;

    bool flush_input() override;
    bool flush_output() override;
    int bytes_available() override;

    hal::UartResult get_last_error() const override {
        return last_error_.load();
    }

   private:
    std::string port_name_;
    hal::UartConfig config_;
    sp_port* port_{nullptr};

    // 原子标志：is_open() 通过此标志读取，避免裸指针 port_ 的数据竞争
    std::atomic<bool> connected_{false};
    // 并发安全的错误状态标志
    std::atomic<hal::UartResult> last_error_{hal::UartResult::OK};

    // 防 TOCTOU：close() 与 write()/read() 并发时 port_ 可能已被 sp_free_port 释放。
    // io_mutex_ 保证 close() 等待所有 I/O 完成后再释放资源（close 取独占，I/O 取共享）。
    // 由于 Modbus 层已通过 bus_mutex_ 串行化，IMU 层只有读操作，此锁无额外 RT 开销。
    mutable std::shared_mutex port_rwlock_;

    // 内部辅助函数，用于检查 libserialport 的返回值并设置 last_error_
    bool check_sp_return(int ret_code, const char* operation);

    void close_locked();
};

}  // namespace robot::driver
