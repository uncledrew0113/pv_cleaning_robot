/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:18:49
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-17 17:24:41
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/hal/i_modbus_master.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <cstdint>

namespace robot::hal {

/// @brief Modbus 操作的具体结果状态
enum class ModbusResult {
    OK = 0,
    TIMEOUT,       // 从站无响应 (线缆断开、设备掉电或波特率错误)
    DISCONNECTED,  // 主站串口本身断开 (如 USB-RS485 被拔出)
    EXCEPTION,     // 通信正常，但从站返回了 Modbus 异常码 (如非法地址 0x02)
    SYS_ERROR      // 其他系统级/配置级错误
};

/// @brief Modbus 通信与物理层配置
struct ModbusConfig {
    int baudrate{9600};
    char parity{'E'};
    int data_bits{8};
    int stop_bits{1};
};

/// @brief Modbus RTU 主站硬件抽象接口
///
/// 此接口封装了完整的 Modbus RTU 事务处理流程：
///   帧构建 → CRC16 计算 → 串口发送 → 等待从站响应 → CRC 校验 → 解包
///
/// 返回值语义（Level-1 通信层）：
///   >=0 : 通信成功（返回实际操作的寄存器数）
///   -1  : 任何级别的通信失败（超时/CRC 错/无响应/Modbus 异常码）
///
/// Level-3 应用语义判定（设备返回的业务错误码）由设备层负责。
///
/// 实现：driver/libmodbus_master（基于 libmodbus）
class IModbusMaster {
   public:
    virtual ~IModbusMaster() = default;

    /// @brief 打开并配置 RS485 串口，失败返回 false
    virtual bool open() = 0;

    /// @brief 关闭并释放资源
    virtual void close() = 0;

    /// @brief 是否已成功打开
    virtual bool is_open() const = 0;

    /// @brief 读取保持寄存器（功能码 0x03）
    /// @return 读取的寄存器数；-1 表示失败
    virtual int read_registers(int slave_id, int addr, int count, uint16_t* out) = 0;

    /// @brief 写单个保持寄存器（功能码 0x06）
    /// @return 0 成功；-1 失败
    virtual int write_register(int slave_id, int addr, uint16_t val) = 0;

    /// @brief 写多个保持寄存器（功能码 0x10）
    /// @return 0 成功；-1 失败
    virtual int write_registers(int slave_id, int addr, int count, const uint16_t* vals) = 0;

    /// @brief 运行时动态修改请求超时时间
    virtual void set_timeout_ms(int ms) = 0;

    /// @brief 获取最后一次事务的详细状态/错误码
    virtual ModbusResult get_last_error() const = 0;
};

}  // namespace robot::hal
