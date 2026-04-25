#pragma once
#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/device/gps_source.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/protocol/nmea_parser.h"
#include <memory>
#include <mutex>

namespace robot::device {

/// @brief GPS 定位设备统一门面
/// 对上层暴露线程安全缓存访问，对下层可接串口 NMEA 或 gpsd TCP 数据源。
class GpsDevice {
public:
    using GpsData = protocol::GpsData;

    struct Diagnostics : GpsData {
        uint32_t sentence_count{0};      ///< 接收报文总数
        uint32_t parse_error_count{0};   ///< 解析失败报文数
        uint32_t fix_loss_count{0};      ///< 定位丢失次数
    };

    explicit GpsDevice(std::shared_ptr<hal::ISerialPort> serial);
    explicit GpsDevice(std::unique_ptr<IGpsSource> source);
    static std::shared_ptr<GpsDevice> create_gpsd(const GpsdSourceConfig& cfg);
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
    GpsDevice() = default;
    void on_source_message(const GpsData& data);
    void on_source_parse_error();
    void on_source_message_count();

    std::unique_ptr<IGpsSource> source_;
    mutable std::mutex          mtx_;
    Diagnostics                 diag_{};
};

}  // namespace robot::device
