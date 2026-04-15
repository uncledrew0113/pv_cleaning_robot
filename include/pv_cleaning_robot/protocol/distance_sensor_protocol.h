#pragma once
#include <cstdint>

namespace robot::protocol {

// ── 输入寄存器（FC=0x04）通道地址 ─────────────────────────────────────────
/// 通道 N（1-based）的输入寄存器地址 = kDistSensorCh1InputAddr + (N-1)
static constexpr uint16_t kDistSensorCh1InputAddr  = 0x0000;
static constexpr uint8_t  kDistSensorMaxChannels   = 18;  ///< HK-18AI 最大通道数

// ── 保持寄存器（FC=0x03/0x06）配置地址 ────────────────────────────────────
static constexpr uint16_t kDistSensorAutoUploadReg  = 0x0031;  ///< 自动上传间隔（0.01s/cnt，0=禁用）
static constexpr uint16_t kDistSensorSlaveAddrReg   = 0x0032;  ///< RS485站号（1~255，掉电保存）
static constexpr uint16_t kDistSensorBaudRateReg    = 0x0033;  ///< 波特率（见枚举，掉电保存）
static constexpr uint16_t kDistSensorDecimalModeReg = 0x003A;  ///< 数据解析方式（掉电保存）

/// 波特率寄存器编码值（写入 kDistSensorBaudRateReg）
static constexpr uint16_t kDistSensorBaud4800   = 0;
static constexpr uint16_t kDistSensorBaud9600   = 1;  ///< 出厂默认
static constexpr uint16_t kDistSensorBaud14400  = 2;
static constexpr uint16_t kDistSensorBaud19200  = 3;
static constexpr uint16_t kDistSensorBaud38400  = 4;
static constexpr uint16_t kDistSensorBaud56000  = 5;
static constexpr uint16_t kDistSensorBaud57600  = 6;
static constexpr uint16_t kDistSensorBaud115200 = 7;

/// 4-20mA 电流型通道标准采样电阻（Ω）
static constexpr float kDistSensorSamplingResistorOhm = 249.0f;

// ── 枚举 ──────────────────────────────────────────────────────────────────

/// @brief 数据解析方式（对应保持寄存器 0x003A）
///
/// 协议规定（出厂默认为可变小数点）：
///   - VARIABLE：万位（x/10000）= 小数位数，后4位（x%10000）= 数值
///              示例：31000 → 小数位=3，数值=1000 → 1.000 V
///   - FIXED_N ：原始值 × 10^(-N) 即为物理量
enum class DistDecimalMode : uint8_t {
    VARIABLE = 0,  ///< 可变小数点（出厂默认）
    FIXED_2  = 1,  ///< 固定2位小数：物理量 = raw × 0.01
    FIXED_3  = 2,  ///< 固定3位小数：物理量 = raw × 0.001
    FIXED_4  = 3,  ///< 固定4位小数：物理量 = raw × 0.0001
};

/// @brief 模拟量输入类型
enum class DistAnalogType : uint8_t {
    VOLTAGE = 0,  ///< 电压型（0~5/10/30/60V 等），解码结果单位 V
    CURRENT = 1,  ///< 电流型（0~20mA / 4~20mA）：寄存器值为249Ω两端电压(V)，
                  ///< 需调用 voltage_to_ma() 换算为 mA
};

// ── 协议类（纯静态，无状态，无 I/O）─────────────────────────────────────────

/// @brief 汇控电子 HK-xAI 模拟量输入模块 Modbus RTU 协议解析
///
/// 协议要点（参考《模拟量输入系列使用手册(RS485版)》2023版）：
///   - 读通道：FC=0x04，输入寄存器 0x0000~0x0011，每通道1个16位无符号整数
///   - 配置：  FC=0x03/0x06，保持寄存器 0x0031~0x003A
///   - 可变小数点：万位=小数位数，其余4位为数值（例：31000 → 1.000 V）
///   - 电流型换算：采样电阻 249Ω，V / 249 × 1000 = mA
class DistanceSensorProtocol {
public:
    DistanceSensorProtocol() = delete;

    /// @brief 将16位原始寄存器值解码为物理量
    ///
    /// 电压型通道直接返回电压（V）。
    /// 电流型通道先用此函数得到采样电阻两端电压（V），再调 voltage_to_ma() 得 mA。
    ///
    /// @param raw   寄存器原始值（0~65535）
    /// @param mode  数据解析方式
    /// @return 物理量浮点数（V）
    static float decode_register(uint16_t raw, DistDecimalMode mode) noexcept;

    /// @brief 将采样电阻两端电压换算为电流
    /// @param voltage_v     电压（V）
    /// @param resistor_ohm  采样电阻阻值（Ω），默认 249Ω
    /// @return 电流（mA）；若 resistor_ohm ≤ 0 返回 0
    static float voltage_to_ma(
        float voltage_v,
        float resistor_ohm = kDistSensorSamplingResistorOhm) noexcept;
};

}  // namespace robot::protocol
