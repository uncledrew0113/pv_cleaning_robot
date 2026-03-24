#pragma once
#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/protocol/nmea_parser.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace robot::device {

/// @brief GPS 定位设备（UART，NMEA 0183 协议）
/// 内部启动读取线程，按行缓冲并解析 NMEA 句子
class GpsDevice {
public:
    using GpsData = protocol::GpsData;

    struct Diagnostics : GpsData {
        uint32_t sentence_count;      ///< 接收句子总数
        uint32_t parse_error_count;   ///< 解析失败句子数
        uint32_t fix_loss_count;      ///< 定位丢失次数
    };

    explicit GpsDevice(std::shared_ptr<hal::ISerialPort> serial);
    ~GpsDevice();

    // ── 生命周期 ──────────────────────────────────────────────
    bool open();
    void close();

    // ── 配置命令（向 GPS 模组发送 NMEA 配置句子）────────────
    /// 设置 NMEA 输出频率（1/5/10 Hz），部分模组支持
    DeviceError set_output_rate(int hz);

    /// 热启动（保留星历数据）
    DeviceError hot_restart();

    /// 冷启动（清除所有辅助数据）
    DeviceError cold_restart();

    // ── 数据访问（缓存，无 I/O，线程安全）────────────────────
    GpsData     get_latest()     const;
    Diagnostics get_diagnostics() const;

private:
    void read_loop();

    std::shared_ptr<hal::ISerialPort> serial_;
    protocol::NmeaParser              parser_;

    mutable std::mutex mtx_;
    Diagnostics        diag_{};

    std::thread        read_thread_;
    std::atomic<bool>  running_{false};

    std::string        line_buf_;  ///< 行缓冲（直到 '\n'）
};

}  // namespace robot::device
