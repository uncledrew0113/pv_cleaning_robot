#pragma once
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include <array>
#include <cstdint>
#include <optional>

namespace robot::protocol {

// ── CAN ID 常量（M1502E_111，标准帧，500 Kbps）──────────────────────────────
/// 控制帧 ID（主控 → 电机）
static constexpr uint32_t kWalkMotorCtrlIdGroup1  = 0x032u;  ///< 设定值，电机 1~4
static constexpr uint32_t kWalkMotorCtrlIdGroup2  = 0x033u;  ///< 设定值，电机 5~8
static constexpr uint32_t kWalkMotorSetModeId     = 0x105u;  ///< 设置运行模式
static constexpr uint32_t kWalkMotorSetFeedbackId = 0x106u;  ///< 设置反馈方式
static constexpr uint32_t kWalkMotorQueryId       = 0x107u;  ///< 主动查询（查询方式）
static constexpr uint32_t kWalkMotorSetNodeId     = 0x108u;  ///< 设置电机 ID
static constexpr uint32_t kWalkMotorTermResId     = 0x109u;  ///< CAN 终端电阻配置
static constexpr uint32_t kWalkMotorCommTimeoutId = 0x10Au;  ///< 通信超时配置
static constexpr uint32_t kWalkMotorFwQueryId     = 0x10Bu;  ///< 固件版本查询
/// 反馈帧基础 ID（电机 → 主控）
static constexpr uint32_t kWalkMotorStatusBase          = 0x096u;  ///< +motor_id → 0x97~0x9E
static constexpr uint32_t kWalkMotorModeRespBase        = 0x200u;  ///< +motor_id
static constexpr uint32_t kWalkMotorFbModeBase          = 0x264u;  ///< +motor_id
static constexpr uint32_t kWalkMotorTermRespBase        = 0x390u;  ///< +motor_id（终端电阻应答）
static constexpr uint32_t kWalkMotorCommTimeoutRespBase = 0x2C8u;  ///< +motor_id（通信超时应答）
static constexpr uint32_t kWalkMotorFwRespBase          = 0x32Cu;  ///< +motor_id（固件版本应答）

// ── 枚举 ────────────────────────────────────────────────────────────────────
/// 电机运行模式（0x105 帧 DATA 字段值）
enum class WalkMotorMode : uint8_t {
    OPEN_LOOP = 0x00,  ///< 电压开环
    CURRENT   = 0x01,  ///< 电流环
    SPEED     = 0x02,  ///< 速度环
    POSITION  = 0x03,  ///< 位置环
    DISABLE   = 0x09,  ///< 失能电机
    ENABLE    = 0x0A,  ///< 使能电机（默认）
};

/// 电机故障码（状态帧 DATA[6]）
enum class WalkMotorFault : uint8_t {
    NONE              = 0x00,
    UNDER_VOLTAGE_2   = 0x01,  ///< 电压 < 17V
    UNDER_VOLTAGE_1   = 0x02,  ///< 17V < 电压 < 22V
    OVER_VOLTAGE      = 0x03,  ///< 电压 > 36V
    OVER_CURRENT      = 0x0A,  ///< 母线电流 > 15A
    OVER_SPEED        = 0x14,  ///< 转速 > 250 RPM
    OVER_TEMP_2       = 0x1F,  ///< 绕组温度 > 120°C
    OVER_TEMP_1       = 0x20,  ///< 绕组温度 > 80°C
    POS_SENSOR_FAULT  = 0x2A,  ///< 位置传感器故障
    POS_SENSOR_SIGNAL = 0x2B,  ///< 位置传感器信号异常
    COMM_TIMEOUT      = 0x3C,  ///< 通信超时
    STALL             = 0x62,  ///< 堵转（电流>5A 且转速<1RPM）
};

/// 通信超时帧操作类型（0x10A DATA[1]）
enum class CommTimeoutOp : uint8_t {
    WRITE = 0x10,  ///< 写入超时时间
    RESET = 0x11,  ///< 复位为默认值（0，即禁用）
};

/// 查询目标（0x107 帧 DATA[1~3]）
enum class WalkMotorQueryTarget : uint8_t {
    SPEED    = 0x01,
    TORQUE   = 0x02,
    TEMP     = 0x03,
    POSITION = 0x04,
    FAULT    = 0x05,
    MODE     = 0x06,
};

// ── 数据结构 ─────────────────────────────────────────────────────────────────
/// 电机状态反馈（来自 0x96+motor_id 帧，默认 100 Hz 主动推送）
struct WalkMotorStatus {
    float          speed_rpm;     ///< 实测转速（-210 ~ +210 RPM，分辨率 0.01 RPM）
    float          torque_a;      ///< 转矩电流（-33 ~ +33 A）
    float          position_deg;  ///< 电机当前位置（0 ~ 360°）
    WalkMotorFault fault;         ///< 故障码
    WalkMotorMode  mode;          ///< 当前运行模式
};

/// 通信超时应答（来自 0x2C8+motor_id 帧）
struct WalkMotorCommTimeoutResp {
    uint8_t       motor_id;
    CommTimeoutOp op;           ///< WRITE 或 RESET
    uint16_t      timeout_ms;   ///< 当前超时时间，0=禁用，单位 ms（range 0~65535）
};

/// 固件版本信息（来自 0x32C+motor_id 帧）
struct WalkMotorFirmwareInfo {
    uint8_t motor_id;
    uint8_t sw_major;
    uint8_t sw_minor;
    uint8_t hw_major;
    uint8_t hw_minor;
    uint8_t year;   ///< 两位数年份（如 22 表示 2022）
    uint8_t month;
    uint8_t day;
};

// ── 编解码器 ─────────────────────────────────────────────────────────────────
/// M1502E_111 行走电机 CAN 帧编解码器
///
/// 每个实例对应一台电机（motor_id 1~8）。
/// 控制帧（0x32/0x33）将 4 台电机设定值打包到 8 Byte：
///   motor 1~4 → 0x32，slot 字节偏移 = (motor_id-1)*2
///   motor 5~8 → 0x33，slot 字节偏移 = (motor_id-5)*2
/// 本实例只填写自身 slot，其余 6 Byte 置 0（不改变其他电机）。
///
/// 量化规则：
///   速度：int16 = RPM × 100  （范围 -21000 ~ +21000 → -210 ~ +210 RPM）
///   电流：int16 = A × (32767/33)  （范围 -32767 ~ +32767 → -33 ~ +33 A）
///   位置：uint16 = deg × (32767/360)（范围 0 ~ 32767 → 0 ~ 360°）
class WalkMotorCanCodec {
public:
    static constexpr uint8_t kMotorIdMin = 1;
    static constexpr uint8_t kMotorIdMax = 8;

    /// @param motor_id 电机 ID，取值 1~8
    explicit WalkMotorCanCodec(uint8_t motor_id);

    uint8_t  motor_id()      const { return motor_id_; }
    /// 控制帧 CAN ID（0x32 或 0x33，取决于 motor_id）
    uint32_t ctrl_can_id()   const { return ctrl_id_; }
    /// 状态反馈帧 CAN ID（0x96 + motor_id，即 0x97~0x9E）
    uint32_t status_can_id() const { return kWalkMotorStatusBase + motor_id_; }

    // ── 发送帧编码 ──────────────────────────────────────────────────────────

    /// 速度环给定（-210 ~ +210 RPM）
    hal::CanFrame encode_speed(float rpm) const;

    /// 电流环给定（-33 ~ +33 A）
    hal::CanFrame encode_current(float amps) const;

    /// 位置环给定（0 ~ 360°，绝对位置）
    hal::CanFrame encode_position(float deg) const;

    /// 开环电压给定（-32767 ~ +32767 raw）
    hal::CanFrame encode_open_loop(int16_t raw_value) const;

    /// 设置运行模式（0x105）
    hal::CanFrame encode_set_mode(WalkMotorMode mode) const;

    /// 设置反馈方式（0x106）
    /// @param period_ms 主动上报周期 1~127 ms；传 0 则切换为查询方式（DATA 最高位=1）
    hal::CanFrame encode_set_feedback(uint8_t period_ms) const;

    /// 查询帧（0x107），反馈方式为查询方式时手动触发
    hal::CanFrame encode_query(WalkMotorQueryTarget t1 = WalkMotorQueryTarget::SPEED,
                               WalkMotorQueryTarget t2 = WalkMotorQueryTarget::TORQUE,
                               WalkMotorQueryTarget t3 = WalkMotorQueryTarget::MODE,
                               uint8_t custom          = 0x00) const;

    /// 固件版本查询（0x10B，全零，广播，所有电机均回复）
    static hal::CanFrame encode_query_firmware();

    /// 设置电机 ID（0x108）— 每次上电仅支持设置一次，通过反馈帧判断是否成功
    hal::CanFrame encode_set_node_id() const;

    /// 写入通信超时（0x10A，op=WRITE）；timeout_ms=0 表示禁用，最大 65535 ms
    hal::CanFrame encode_set_comm_timeout(uint16_t timeout_ms) const;

    /// 复位通信超时为默认值 0（0x10A，op=RESET）
    hal::CanFrame encode_reset_comm_timeout() const;

    /// 读取通信超时（0x10A，DATA[2]=0x00 读模式）
    hal::CanFrame encode_read_comm_timeout() const;

    // ── 8电机批量编码（static）──────────────────────────────────────────────
    /// 批量设置全部8台电机运行模式（0x105）— DATA[0~7] = motor 1~8 的模式值
    static hal::CanFrame encode_set_mode_batch(const std::array<WalkMotorMode, 8>& modes);

    /// 批量设置全部8台电机反馈方式（0x106）— DATA[0~7] = motor 1~8 的反馈方式值
    ///   Bit7=1：查询方式；Bit7=0：主动上报（低7位=上报周期 ms，1~127）
    static hal::CanFrame encode_set_feedback_batch(const std::array<uint8_t, 8>& fb_modes);

    /// 批量设置全部8台电机 CAN 终端电阻（0x109）— DATA[0~7]=0/1 对应 motor 1~8
    static hal::CanFrame encode_set_termination_batch(const std::array<bool, 8>& enables);

    // ── 4电机组同步批量编码（static）────────────────────────────────────────
    /// 一帧同时设定同组4台电机的速度给定
    ///   id_base=1 → CAN ID 0x32，管控 motor 1~4
    ///   id_base=5 → CAN ID 0x33，管控 motor 5~8
    /// vN 下标 0~3 与组内电机偏移 0~3（即 motor_id = id_base + N - 1）对应
    static hal::CanFrame encode_group_speed(uint8_t id_base,
                                            float rpm0, float rpm1,
                                            float rpm2, float rpm3);

    /// 一帧同时设定4台电机的电流环给定（-33 ~ +33 A）
    static hal::CanFrame encode_group_current(uint8_t id_base,
                                              float a0, float a1,
                                              float a2, float a3);

    /// 一帧同时设定4台电机的开环电压给定（-32767 ~ +32767 raw）
    static hal::CanFrame encode_group_open_loop(uint8_t id_base,
                                                int16_t v0, int16_t v1,
                                                int16_t v2, int16_t v3);

    /// 一帧同时设定4台电机的位置环给定（0 ~ 360°，绝对位置）
    static hal::CanFrame encode_group_position(uint8_t id_base,
                                               float deg0, float deg1,
                                               float deg2, float deg3);

    // ── 接收帧解码 ──────────────────────────────────────────────────────────

    /// 解析状态反馈帧（frame.id == 0x96 + motor_id），否则返回 nullopt
    std::optional<WalkMotorStatus>          decode_status(const hal::CanFrame& frame) const;

    /// 解析模式应答帧（0x200 + motor_id），否则返回 nullopt
    std::optional<WalkMotorMode>            decode_mode_resp(const hal::CanFrame& frame) const;

    /// 解析反馈方式应答帧（0x264 + motor_id），返回反馈方式字节
    std::optional<uint8_t>                  decode_fb_mode_resp(const hal::CanFrame& frame) const;

    /// 解析终端电阻应答帧（0x390 + motor_id），返回是否使能
    std::optional<bool>                     decode_termination_resp(const hal::CanFrame& frame) const;

    /// 解析通信超时应答帧（0x2C8 + motor_id）
    std::optional<WalkMotorCommTimeoutResp> decode_comm_timeout_resp(const hal::CanFrame& frame) const;

    /// 解析固件版本帧（0x32C + motor_id），否则返回 nullopt
    std::optional<WalkMotorFirmwareInfo>    decode_firmware(const hal::CanFrame& frame) const;

private:
    uint8_t  motor_id_;  ///< 1~8
    uint32_t ctrl_id_;   ///< 0x32（motor 1~4）或 0x33（motor 5~8）
    uint8_t  slot_;      ///< 本电机在控制帧 data[] 中的起始字节偏移（0/2/4/6）

    /// 填写本电机 slot，其余置 0，构造控制帧
    hal::CanFrame make_ctrl_frame(int16_t value) const;
};

}  // namespace robot::protocol
