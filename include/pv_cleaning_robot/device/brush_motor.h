#pragma once
#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include <memory>
#include <mutex>

namespace robot::device {

/// @brief 辊刷电机设备（RS485 Modbus RTU，独立总线1）
/// update() 由 brush_ctrl_thread (SCHED_FIFO 60, 50ms) 周期调用
///
/// @note 寄存器地址请根据辊刷电机驱动器实际手册调整（当前为占位地址）
class BrushMotor {
public:
    // ── 寄存器地址（根据实际驱动器手册修改）─────────────────────
    static constexpr int REG_TARGET_RPM = 0x1000;  ///< 目标转速（写）
    static constexpr int REG_ENABLE     = 0x1001;  ///< 使能控制（写：1=启动，0=停止）
    static constexpr int REG_CLR_FAULT  = 0x1002;  ///< 清故障（写：1=清除）
    static constexpr int REG_ACT_RPM    = 0x2000;  ///< 实际转速（读）
    static constexpr int REG_CURRENT    = 0x2001;  ///< 相电流（读，A*100）
    static constexpr int REG_VOLTAGE    = 0x2002;  ///< 总线电压（读，V*10）
    static constexpr int REG_TEMP       = 0x2003;  ///< 温度（读，℃*10）
    static constexpr int REG_STATUS     = 0x2004;  ///< 状态字（读）
    static constexpr int REG_FAULT_CODE = 0x2005;  ///< 故障码（读）

    // 状态字位定义
    static constexpr uint16_t STATUS_RUNNING   = 0x0001;
    static constexpr uint16_t STATUS_FAULT     = 0x0002;
    static constexpr uint16_t STATUS_OVERCURR  = 0x0004;
    static constexpr uint16_t STATUS_STALL     = 0x0008;

    struct Status {
        int      actual_rpm;
        float    current_a;
        bool     running;
        bool     fault;
        uint16_t fault_code;
    };

    struct Diagnostics : Status {
        float    temperature_c;
        float    bus_voltage_v;
        int      target_rpm;
        uint32_t stall_count;
        uint32_t comm_error_count;
    };

    BrushMotor(std::shared_ptr<hal::IModbusMaster> modbus, int slave_id);

    /// 打开底层 Modbus（用于上电自检/初始化）
    bool open();

    // ── 控制接口 ─────────────────────────────────────────────────
    DeviceError start();
    DeviceError stop();
    DeviceError set_rpm(int rpm);
    DeviceError clear_fault();

    // ── 状态读取（缓存，无 I/O）────────────────────────────────
    Status      get_status() const;
    Diagnostics get_diagnostics() const;

    // ── 周期更新（50ms）─────────────────────────────────────────
    void update();

private:
    DeviceError refresh_status();  // 读取寄存器更新缓存

    std::shared_ptr<hal::IModbusMaster> modbus_;
    int                                 slave_id_;
    mutable std::mutex                  mtx_;
    Diagnostics                         diag_{};
    int                                 target_rpm_{0};
};

}  // namespace robot::device
