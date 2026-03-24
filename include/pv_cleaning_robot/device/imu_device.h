#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/protocol/imu_protocol.h"

namespace robot::device {

/// @brief 9轴 IMU 设备（UART，WIT Motion 协议）
/// 内部启动读取线程，流式接收并解析帧
class ImuDevice {
   public:
    struct ImuData {
        float accel[3];  ///< 加速度 m/s²  [x,y,z]
        float gyro[3];   ///< 角速度 rad/s  [x,y,z]
        float mag[3];    ///< 磁场   uT      [x,y,z]
        float quat[4];   ///< 四元数 [w,x,y,z]
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        uint64_t timestamp_us;
        bool valid;
    };

    struct DeviceInfo {
        std::string model;
        std::string firmware_version;
    };

    struct Diagnostics : ImuData {
        uint32_t frame_count;        ///< 接收帧总数
        uint32_t parse_error_count;  ///< 协议解析失败帧数
        float frame_rate_hz;         ///< 实测帧率
    };

    explicit ImuDevice(std::shared_ptr<hal::ISerialPort> serial);
    ~ImuDevice();

    // ── 生命周期 ──────────────────────────────────────────────
    bool open();   ///< 打开串口，启动读取线程
    void close();  ///< 停止读取线程，关闭串口

    // ── 配置命令（发送配置帧，等待生效，验证）──────────────
    DeviceError set_output_rate(int hz);  ///< 设置输出频率
    DeviceError calibrate_gyro();         ///< 陀螺零飘校准（需静止）
    DeviceError save_config();            ///< 保存至 Flash
    DeviceError reset();                  ///< 软件复位

    // ── 数据访问（缓存，无 I/O，线程安全）────────────────────
    ImuData get_latest() const;
    Diagnostics get_diagnostics() const;

   private:
    void read_loop();

    DeviceError send_command(const protocol::ImuProtocol::Cmd& cmd, int wait_ms = 100);

    /// 发送写寄存器序列：unlock → cmd（写指令必须先解锁）
    DeviceError send_write_cmd(const protocol::ImuProtocol::Cmd& cmd, int wait_ms = 150);

    std::shared_ptr<hal::ISerialPort> serial_;
    protocol::ImuProtocol parser_;

    mutable std::mutex mtx_;
    Diagnostics diag_{};

    std::thread read_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace robot::device
