/*
 * 串口硬件抽象接口。
 *
 * 上层设备通过该接口访问 UART/RS232/USB 转串口；具体驱动负责超时、错误码和缓冲区管理。
 * close/open 前应先停止相关 update 线程，避免读写与句柄重建并发。
 */
/**
 * @file i_serial_port.h
 * @brief 串口 HAL 抽象接口。
 *
 * 本接口统一 UART 打开、关闭、阻塞读写、缓冲区清理和错误查询，供 BMS、IMU、GPS、滚刷等设备复用。
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace robot::hal {

/// @brief 串口操作的具体结果状态。
enum class UartResult {
    OK = 0,
    TIMEOUT,       // 读/写超时 (未收到预期数据)
    DISCONNECTED,  // 设备断开 (未打开，或运行中如 USB 转串口被拔出)
    SYS_ERROR  // 其他系统级错误 (如 libserialport 内部配置错误、底层驱动异常)
};

/// @brief 串口帧格式及底层通信配置。
struct UartConfig {
    int baudrate{115200};       ///< 波特率，默认 115200
    int data_bits{8};           ///< 数据位：5/6/7/8
    char parity{'N'};           ///< 校验：'N'=无  'E'=偶  'O'=奇
    int stop_bits{1};           ///< 停止位：1 或 2
    bool flow_control{false};   ///< 是否启用 RTS/CTS 硬件流控
    int write_timeout_ms{200};  ///< 写操作默认超时（毫秒），避免串口异常时永久阻塞。
};

/// @brief 串口（UART/RS232）硬件抽象接口。
///
/// 生产实现为 driver/libserialport_port。RS485 的物理层方向切换由硬件自动完成
/// 或由底层驱动处理，本接口不暴露该差异。
class ISerialPort {
   public:
    virtual ~ISerialPort() = default;

    /// @brief 打开并配置串口，失败返回 false。
    virtual bool open() = 0;

    /// @brief 关闭并释放串口。
    virtual void close() = 0;

    /// @brief 是否已成功打开。
    virtual bool is_open() const = 0;

    /// @brief 写入数据。
    /// @param timeout_ms 写入超时；-1 表示使用 UartConfig 中的默认配置。
    /// @return 实际写入字节数；-1 表示错误。
    virtual int write(const uint8_t* buf, size_t len, int timeout_ms = -1) = 0;

    /// @brief 读取数据，阻塞最多 timeout_ms 毫秒。
    /// @return 实际读取字节数；0 表示超时；-1 表示错误。
    virtual int read(uint8_t* buf, size_t max_len, int timeout_ms) = 0;

    /// @brief 清空输入缓冲区。
    virtual bool flush_input() = 0;

    /// @brief 清空输出缓冲区。
    virtual bool flush_output() = 0;

    /// @brief 获取接收缓冲区中待读取的字节数。
    virtual int bytes_available() = 0;

    /// @brief 获取最后一次底层操作的详细状态/错误码。
    virtual UartResult get_last_error() const = 0;
};

}  // namespace robot::hal
