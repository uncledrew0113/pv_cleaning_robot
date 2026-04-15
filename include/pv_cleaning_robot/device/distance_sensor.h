#pragma once
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/protocol/distance_sensor_protocol.h"

#include <array>
#include <cstdint>
#include <memory>

namespace robot::device {

// ── 配置结构体 ────────────────────────────────────────────────────────────

/// @brief 距离传感器（汇控电子 HK-xAI）设备配置
struct DistanceSensorConfig {
    int     slave_id{1};       ///< Modbus 从站地址（1~255，与拨码开关一致）
    uint8_t channel_count{4};  ///< 实际使用通道数（1~18）

    /// 数据解析方式（须与传感器保持寄存器 0x003A 设置一致）
    protocol::DistDecimalMode decimal_mode{protocol::DistDecimalMode::VARIABLE};

    /// 模拟量类型（电压型 or 电流型）
    protocol::DistAnalogType analog_type{protocol::DistAnalogType::VOLTAGE};

    /// 电流型通道采样电阻（Ω）；电压型通道忽略此字段
    float sampling_resistor_ohm{protocol::kDistSensorSamplingResistorOhm};
};

// ── 数据结构 ──────────────────────────────────────────────────────────────

/// @brief 单通道采样数据
struct DistChannelReading {
    float value_v{0.0f};   ///< 解码后电压值（V）；电压型直接使用，电流型为采样电阻两端电压
    float value_ma{0.0f};  ///< 换算电流（mA），仅电流型有效
    bool  valid{false};    ///< 本帧数据是否成功读取
};

/// @brief 全部通道快照（供外部消费，线程安全拷贝）
struct DistSensorData {
    std::array<DistChannelReading, protocol::kDistSensorMaxChannels> channels{};
    uint8_t  channel_count{0};  ///< 实际通道数（由配置决定）
    uint32_t error_count{0};    ///< 累计通信错误次数
};

/// @brief 含调试统计的完整诊断数据
struct DistSensorDiagnostics : DistSensorData {
    uint32_t total_update_count{0};  ///< 累计 update() 调用次数
    uint32_t comm_error_count{0};    ///< 等同 error_count，语义明确别名
};

// ── 设备类 ────────────────────────────────────────────────────────────────

/// @brief 汇控电子 HK-xAI 模拟量输入模块设备驱动
///
/// - 通过 Modbus RTU FC=0x04 周期读取各通道输入寄存器
/// - update() 由 ThreadExecutor（SCHED_OTHER，~100ms）周期调用
/// - get_data() / get_diagnostics() 线程安全，可在任意线程读取
/// - 电流型通道：decode_register() 得到采样电阻两端电压(V)，再换算为 mA
class DistanceSensor {
public:
    explicit DistanceSensor(std::shared_ptr<hal::IModbusMaster> modbus,
                            DistanceSensorConfig                 cfg = DistanceSensorConfig{});

    /// @brief 打开底层 Modbus 连接（上电初始化时调用）
    bool open();

    /// @brief 周期轮询（~100ms）：通过 FC=0x04 读取全部通道并解码
    void update();

    /// @brief 获取最新通道数据快照（线程安全）
    DistSensorData        get_data()        const;

    /// @brief 获取含统计信息的完整诊断数据（线程安全）
    DistSensorDiagnostics get_diagnostics() const;

private:
    std::shared_ptr<hal::IModbusMaster> modbus_;
    DistanceSensorConfig                cfg_;
    // PiMutex (PTHREAD_PRIO_INHERIT)：防止高优先级任务读取时发生优先级反转
    mutable hal::PiMutex                mtx_;
    DistSensorDiagnostics               diag_{};
};

}  // namespace robot::device
