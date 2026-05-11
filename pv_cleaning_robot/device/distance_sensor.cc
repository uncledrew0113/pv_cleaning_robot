#include "pv_cleaning_robot/device/distance_sensor.h"
#include "pv_cleaning_robot/protocol/distance_sensor_protocol.h"

#include <algorithm>
#include <array>
#include <spdlog/spdlog.h>

namespace robot::device {

using protocol::DistanceSensorProtocol;
using protocol::DistAnalogType;
using protocol::kDistSensorCh1InputAddr;
using protocol::kDistSensorMaxChannels;

DistanceSensor::DistanceSensor(std::shared_ptr<hal::IModbusMaster> modbus,
                                DistanceSensorConfig                 cfg)
    : modbus_(std::move(modbus)), cfg_(cfg)
{
    // 保护性截断：确保通道数在 [1, kDistSensorMaxChannels] 范围内
    cfg_.channel_count = std::clamp(
        cfg_.channel_count,
        static_cast<uint8_t>(1),
        static_cast<uint8_t>(kDistSensorMaxChannels));
    diag_.channel_count = cfg_.channel_count;
}

DistanceSensor::~DistanceSensor() {
    close();
}

bool DistanceSensor::open() {
    if (!modbus_) return false;
    if (modbus_->is_open()) return true;
    return modbus_->open();
}

void DistanceSensor::close() {
    if (modbus_) {
        modbus_->close();
    }
}

void DistanceSensor::update() {
    if (!modbus_ || !modbus_->is_open()) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++diag_.error_count;
        ++diag_.comm_error_count;
        ++diag_.total_update_count;
        for (uint8_t i = 0; i < cfg_.channel_count; ++i)
            diag_.channels[i].valid = false;
        return;
    }

    std::array<uint16_t, kDistSensorMaxChannels> regs{};
    const int ret = modbus_->read_input_registers(
        cfg_.slave_id,
        static_cast<int>(kDistSensorCh1InputAddr),
        static_cast<int>(cfg_.channel_count),
        regs.data());

    std::lock_guard<hal::PiMutex> lk(mtx_);
    ++diag_.total_update_count;

    if (ret < 0) {
        ++diag_.error_count;
        ++diag_.comm_error_count;
        for (uint8_t i = 0; i < cfg_.channel_count; ++i)
            diag_.channels[i].valid = false;
        spdlog::debug("[DistanceSensor] slave={} read_input_registers failed (total_err={})",
                      cfg_.slave_id, diag_.comm_error_count);
        return;
    }

    for (uint8_t i = 0; i < cfg_.channel_count; ++i) {
        const float v = DistanceSensorProtocol::decode_register(regs[i], cfg_.decimal_mode);
        diag_.channels[i].value_v  = v;
        diag_.channels[i].value_ma =
            (cfg_.analog_type == DistAnalogType::CURRENT)
                ? DistanceSensorProtocol::voltage_to_ma(v, cfg_.sampling_resistor_ohm)
                : 0.0f;
        diag_.channels[i].valid = true;
    }
}

DistSensorData DistanceSensor::get_data() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return static_cast<DistSensorData>(diag_);
}

DistSensorDiagnostics DistanceSensor::get_diagnostics() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return diag_;
}

}  // namespace robot::device
