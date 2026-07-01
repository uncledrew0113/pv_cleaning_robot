/*
 * IMU 设备接口。
 *
 * 本类内部维护串口读取线程，持续解析 WIT Motion 数据帧并缓存最新姿态。
 * 配置命令期间会暂停后台读帧，避免 ACK 与流式数据互相抢占串口响应。
 */
#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/protocol/imu_protocol.h"

namespace robot::device {

/// @brief 9 轴 IMU 设备（UART，WIT Motion 协议）。
class ImuDevice {
   public:
    struct ImuData {
        float accel[3];  ///< 加速度 m/s²  [x,y,z]
        float gyro[3];   ///< 角速度 rad/s  [x,y,z]
        float mag[3];    ///< 磁场   uT      [x,y,z]
        float quat[4];   ///< 四元数 [w,x,y,z]
        float    roll_deg;       ///< 横滚角（度，-180~+180）
        float    pitch_deg;      ///< 俯仰角（度，-90~+90）
        float    yaw_deg;        ///< 航向角（度，-180~+180；顺时针为正）
        uint64_t timestamp_us;   ///< 设备本地时间戳（微秒，取自系统单调时钟）
        bool     valid;          ///< 数据有效标志：read_loop 至少解析出 1 帧后置 true；超过 1s 无帧则清除
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
    void request_stop();  ///< 请求后台读取线程尽快退出；不替代 close/open 生命周期管理
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

    mutable hal::PiMutex mtx_;
    Diagnostics diag_{};

    std::thread read_thread_;
    std::atomic<bool> running_{false};

    // 配置命令标志：下发命令时暂停后台读取线程，防止 ACK 与流式帧冲突。
    std::atomic<bool> is_configuring_{false};
};

}  // namespace robot::device
