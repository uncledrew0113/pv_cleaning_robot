#include "pv_cleaning_robot/device/brush_motor.h"

namespace robot::device {

BrushMotor::BrushMotor(std::shared_ptr<hal::IModbusMaster> modbus, int slave_id)
    : modbus_(std::move(modbus)), slave_id_(slave_id)
{
}

bool BrushMotor::open()
{
    if (!modbus_) return false;
    if (modbus_->is_open()) return true;
    return modbus_->open();
}

DeviceError BrushMotor::start()
{
    int ret = modbus_->write_register(slave_id_, REG_ENABLE, 1);
    if (ret < 0) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.comm_error_count;
        return DeviceError::COMM_TIMEOUT;
    }
    return DeviceError::OK;
}

DeviceError BrushMotor::stop()
{
    int ret = modbus_->write_register(slave_id_, REG_ENABLE, 0);
    if (ret < 0) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.comm_error_count;
        return DeviceError::COMM_TIMEOUT;
    }
    return DeviceError::OK;
}

DeviceError BrushMotor::set_rpm(int rpm)
{
    int ret = modbus_->write_register(slave_id_, REG_TARGET_RPM, static_cast<uint16_t>(rpm));
    if (ret < 0) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.comm_error_count;
        return DeviceError::COMM_TIMEOUT;
    }
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        target_rpm_ = rpm;
        diag_.target_rpm = rpm;
    }
    return DeviceError::OK;
}

DeviceError BrushMotor::clear_fault()
{
    int ret = modbus_->write_register(slave_id_, REG_CLR_FAULT, 1);
    if (ret < 0) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.comm_error_count;
        return DeviceError::COMM_TIMEOUT;
    }
    return DeviceError::OK;
}

BrushMotor::Status BrushMotor::get_status() const
{
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return static_cast<Status>(diag_);
}

BrushMotor::Diagnostics BrushMotor::get_diagnostics() const
{
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return diag_;
}

DeviceError BrushMotor::refresh_status()
{

    // 批量读取状态/故障/实际转速/电流寄存器
    constexpr int kRegCount = 6;  // REG_ACT_RPM .. REG_FAULT_CODE
    uint16_t regs[kRegCount] = {};
    int ret = modbus_->read_registers(slave_id_, REG_ACT_RPM, kRegCount, regs);
    if (ret < 0) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.comm_error_count;
        return DeviceError::COMM_TIMEOUT;
    }

    // 偏移：0=ACT_RPM, 1=CURRENT, 2=VOLTAGE, 3=TEMP, 4=STATUS, 5=FAULT_CODE
    const uint16_t act_rpm    = regs[0];
    const float    current_a  = static_cast<float>(regs[1]) / 100.0f;
    const float    voltage_v  = static_cast<float>(regs[2]) / 10.0f;
    const float    temp_c     = static_cast<float>(regs[3]) / 10.0f;
    const uint16_t status_w   = regs[4];
    const uint16_t fault_code = regs[5];

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        diag_.actual_rpm    = static_cast<int>(act_rpm);
        diag_.current_a     = current_a;
        diag_.bus_voltage_v = voltage_v;
        diag_.temperature_c = temp_c;
        diag_.fault_code    = static_cast<uint16_t>(fault_code);
        diag_.running       = (status_w & STATUS_RUNNING)  != 0;
        diag_.fault         = (status_w & STATUS_FAULT)    != 0;
        if ((status_w & STATUS_OVERCURR) != 0) {
            ++diag_.stall_count;  // 借用 stall_count 计超流（统计用）
        }
        if ((status_w & STATUS_STALL) != 0) {
            ++diag_.stall_count;
        }
        if (diag_.running) {
        }
    }
    return DeviceError::OK;
}

void BrushMotor::update()
{
    refresh_status();
}

} // namespace robot::device
