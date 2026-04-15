/**
 * 距离传感器协议层单元测试
 *
 * 测试分组：[distance_sensor][protocol]
 *
 * 覆盖：
 *   - decode_register() 可变小数点模式
 *   - decode_register() 固定小数点模式（FIXED_2/3/4）
 *   - voltage_to_ma() 换算
 *   - 边界与异常输入
 */
#include <catch2/catch.hpp>
#include "pv_cleaning_robot/protocol/distance_sensor_protocol.h"

using namespace robot::protocol;

// ── decode_register：可变小数点 ────────────────────────────────────────────
TEST_CASE("decode_register - variable decimal: protocol manual examples",
          "[distance_sensor][protocol]")
{
    // 协议手册例1：31000 → decimal=3, mantissa=1000 → 1.000 V
    CHECK(DistanceSensorProtocol::decode_register(31000, DistDecimalMode::VARIABLE)
          == Approx(1.0f).epsilon(0.001f));

    // decimal=2, mantissa=5000 → 50.00 V
    CHECK(DistanceSensorProtocol::decode_register(25000, DistDecimalMode::VARIABLE)
          == Approx(50.0f).epsilon(0.001f));

    // decimal=1, mantissa=300 → 30.0 V
    CHECK(DistanceSensorProtocol::decode_register(10300, DistDecimalMode::VARIABLE)
          == Approx(30.0f).epsilon(0.001f));
}

TEST_CASE("decode_register - variable decimal: zero and max mantissa",
          "[distance_sensor][protocol]")
{
    // 0 → 0 V（无论小数位如何）
    CHECK(DistanceSensorProtocol::decode_register(0, DistDecimalMode::VARIABLE)
          == Approx(0.0f));

    // decimal=0, mantissa=9999 → 9999 （无缩放）
    CHECK(DistanceSensorProtocol::decode_register(9999, DistDecimalMode::VARIABLE)
          == Approx(9999.0f).epsilon(0.001f));

    // decimal=3, mantissa=9999 → 9.999 V
    CHECK(DistanceSensorProtocol::decode_register(39999, DistDecimalMode::VARIABLE)
          == Approx(9.999f).epsilon(0.001f));

    // decimal=4, mantissa=9999 → 0.9999 V
    CHECK(DistanceSensorProtocol::decode_register(49999, DistDecimalMode::VARIABLE)
          == Approx(0.9999f).epsilon(0.0001f));
}

// ── decode_register：固定小数点 ────────────────────────────────────────────
TEST_CASE("decode_register - fixed decimal: protocol manual examples",
          "[distance_sensor][protocol]")
{
    // 协议手册页11示例：0x2F2C = 12076, fixed-3 → 12.076 V
    CHECK(DistanceSensorProtocol::decode_register(12076, DistDecimalMode::FIXED_3)
          == Approx(12.076f).epsilon(0.001f));

    // 0x2F14 = 12052, fixed-3 → 12.052 V
    CHECK(DistanceSensorProtocol::decode_register(12052, DistDecimalMode::FIXED_3)
          == Approx(12.052f).epsilon(0.001f));
}

TEST_CASE("decode_register - fixed decimal: FIXED_2 and FIXED_4",
          "[distance_sensor][protocol]")
{
    // FIXED_2: raw=1000 → 10.00 V
    CHECK(DistanceSensorProtocol::decode_register(1000, DistDecimalMode::FIXED_2)
          == Approx(10.0f).epsilon(0.001f));

    // FIXED_4: raw=50000 → 5.0000 V
    CHECK(DistanceSensorProtocol::decode_register(50000, DistDecimalMode::FIXED_4)
          == Approx(5.0f).epsilon(0.001f));

    // Zero for any fixed mode
    CHECK(DistanceSensorProtocol::decode_register(0, DistDecimalMode::FIXED_2) == Approx(0.0f));
    CHECK(DistanceSensorProtocol::decode_register(0, DistDecimalMode::FIXED_4) == Approx(0.0f));
}

// ── voltage_to_ma ─────────────────────────────────────────────────────────
TEST_CASE("voltage_to_ma: protocol manual example and edge cases",
          "[distance_sensor][protocol]")
{
    // 协议手册例：1V / 249Ω × 1000 = 4.016 mA
    CHECK(DistanceSensorProtocol::voltage_to_ma(1.0f)
          == Approx(4.0161f).epsilon(0.01f));

    // 0V → 0 mA
    CHECK(DistanceSensorProtocol::voltage_to_ma(0.0f) == Approx(0.0f));

    // 4.98V / 249Ω × 1000 ≈ 20 mA（满量程4~20mA传感器接近上限）
    CHECK(DistanceSensorProtocol::voltage_to_ma(4.98f)
          == Approx(20.0f).epsilon(0.2f));

    // 无效采样电阻 → 0
    CHECK(DistanceSensorProtocol::voltage_to_ma(5.0f, 0.0f)  == Approx(0.0f));
    CHECK(DistanceSensorProtocol::voltage_to_ma(5.0f, -1.0f) == Approx(0.0f));
}

TEST_CASE("voltage_to_ma: custom resistor",
          "[distance_sensor][protocol]")
{
    // 使用 100Ω 采样电阻：2V / 100Ω × 1000 = 20 mA
    CHECK(DistanceSensorProtocol::voltage_to_ma(2.0f, 100.0f)
          == Approx(20.0f).epsilon(0.01f));
}
