#include "pv_cleaning_robot/protocol/distance_sensor_protocol.h"
#include <cmath>

namespace robot::protocol {

float DistanceSensorProtocol::decode_register(uint16_t raw, DistDecimalMode mode) noexcept {
    switch (mode) {
    case DistDecimalMode::VARIABLE: {
        // 万位（raw / 10000）= 小数位数；后4位（raw % 10000）= 整数数值
        // 示例：31000 → decimal=3, mantissa=1000 → 1000 × 10^-3 = 1.000 V
        const int     decimal_places = raw / 10000;
        const uint16_t mantissa      = raw % 10000;
        const float    scale         = std::pow(10.0f, -static_cast<float>(decimal_places));
        return static_cast<float>(mantissa) * scale;
    }
    case DistDecimalMode::FIXED_2:
        return static_cast<float>(raw) * 0.01f;
    case DistDecimalMode::FIXED_3:
        return static_cast<float>(raw) * 0.001f;
    case DistDecimalMode::FIXED_4:
        return static_cast<float>(raw) * 0.0001f;
    default:
        return static_cast<float>(raw);
    }
}

float DistanceSensorProtocol::voltage_to_ma(float voltage_v, float resistor_ohm) noexcept {
    if (resistor_ohm <= 0.0f) return 0.0f;
    return (voltage_v / resistor_ohm) * 1000.0f;
}

}  // namespace robot::protocol
