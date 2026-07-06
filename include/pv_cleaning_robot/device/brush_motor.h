/**
 * @file brush_motor.h
 * @brief 滚刷电机设备接口。
 *
 * 本模块通过 ODrive ASCII 串口协议控制滚刷转速、停止、清错和复位。周期 update() 负责读取
 * 驱动状态；状态和诊断接口只返回缓存，不阻塞运动控制线程。
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::device {

/// @brief 滚刷电机设备封装。
class BrushMotor {
public:
    /// 运行态快照。所有字段来自最近一次成功 update() 或控制命令缓存，不触发串口 I/O。
    struct Status {
        int actual_rpm{0};
        float current_a{0.0f};
        bool running{false};
        bool fault{false};
        uint32_t fault_code{0};
    };

    /// 诊断快照。comm_error_count 由通信超时/解析失败累加，供 DiagnosticsCollector 判定串口健康。
    struct Diagnostics : Status {
        float temperature_c{0.0f};
        float bus_voltage_v{0.0f};
        int target_rpm{0};
        uint32_t stall_count{0};
        uint32_t comm_error_count{0};
    };

    BrushMotor(std::shared_ptr<hal::ISerialPort> serial, uint8_t axis);
    ~BrushMotor() noexcept;

    /// 打开 ODrive ASCII 串口连接。
    bool open();
    /// 关闭串口连接；调用方应先停止周期 update()，避免 close() 与读写并发。
    void close();
    /// 请求当前 update()/串口事务尽快返回；不关闭串口，串口生命周期仍由 close() 管理。
    void request_stop();
    /// 清除协作停止标志；open()/恢复重启后允许重新执行通信事务。
    void clear_stop_request();

    /// 设置滚刷目标转速；命令成功只表示串口写入完成，不代表驱动器已经达到目标转速。
    DeviceError set_rpm(int rpm);
    /// 发送 0 转速命令，用于任务停止、急停和恢复流程前的安全收敛。
    DeviceError stop();
    /// 清除 ODrive 当前错误码。
    DeviceError clear_fault();
    /// 发送 ODrive 软复位命令；恢复流程会在复位后等待控制器重新启动并重开串口。
    DeviceError restart();

    /// 返回最近一次状态快照，无串口 I/O。
    Status get_status() const;
    /// 返回最近一次诊断快照，无串口 I/O。
    Diagnostics get_diagnostics() const;
    /// 周期读取 ODrive 状态；应由独立 brush_exec 调度，避免阻塞运动控制线程。
    void update();

private:
    DeviceError write_ascii_locked(const char* line, size_t len);
    DeviceError request_ascii_locked(const char* line,
                                     size_t len,
                                     char* response,
                                     size_t response_cap,
                                     int timeout_ms = 200);
    DeviceError read_line_locked(char* response, size_t response_cap, int timeout_ms = 200);
    bool stop_requested() const;
    void mark_comm_error_locked();
    void update_running_locked();

    std::shared_ptr<hal::ISerialPort> serial_;
    uint8_t axis_;
    mutable hal::PiMutex mtx_;
    std::atomic<bool> stop_requested_{false};
    Diagnostics diag_{};
};

}  // namespace robot::device
