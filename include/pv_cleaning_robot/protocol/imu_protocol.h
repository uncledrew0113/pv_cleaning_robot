/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 13:19:14
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-22 21:37:48
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/protocol/imu_protocol.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace robot::protocol {

/// @brief IMU 测量数据（完整9轴 + 姿态解算结果）
struct ImuData {
    float accel[3];         ///< 加速度 (m/s²) [x, y, z]
    float gyro[3];          ///< 角速度 (rad/s) [x, y, z]
    float mag[3];           ///< 磁场 (uT)       [x, y, z]
    float quat[4];          ///< 四元数 [w, x, y, z]
    float roll_deg;         ///< 横滚角（度）
    float pitch_deg;        ///< 俯仰角（度，正=上仰）
    float yaw_deg;          ///< 偏航角（度）
    uint64_t timestamp_us;  ///< 设备时间戳（微秒）
    bool valid;             ///< 数据有效标志
};

/// @brief WIT Motion / 维特智能 UART 协议解析器
///
/// 帧格式（每帧11字节）：
///   [0x55][type][d0][d1][d2][d3][d4][d5][d6][d7][checksum]
///   checksum = (0x55 + type + d0~d7) & 0xFF
///
/// 数据类型：
///   0x51 = 加速度 (Ax, Ay, Az, T)
///   0x52 = 角速度 (Wx, Wy, Wz, T)
///   0x53 = 欧拉角 (Roll, Pitch, Yaw, T)
///   0x59 = 四元数 (Q0, Q1, Q2, Q3)
///   0x54 = 磁场   (Mx, My, Mz, T)
///
/// @note 此类为流式解析器，内部维护帧缓冲和同步状态；
///       push_byte() 每次输入一字节，frame_complete() 通知完整帧就绪
class ImuProtocol {
   public:
    static constexpr size_t FRAME_LEN = 11;
    static constexpr uint8_t FRAME_HEADER = 0x55;

    /// 推入一个字节，内部维护状态机
    void push_byte(uint8_t b);

    /// 是否已积累了一个完整可用帧（推入后立即查询）
    bool frame_complete() const {
        return frame_ready_;
    }

    /// 取出解析后的数据（同时清除 frame_ready_ 标志）
    const ImuData& take_frame();

    /// 清除缓冲区并重置解析状态
    void reset();

    // ── 配置命令编码（发送给 IMU 模组）──────────────────────────────────
    // 所有写寄存器命令均为固定 5 字节：0xFF 0xAA [reg] [data_lo] [data_hi]
    using Cmd = std::array<uint8_t, 5>;

    /// 编码设置输出频率命令（0x01~0x0B 对应 0.1~200Hz，常用 0x06=50Hz）
    static Cmd encode_set_rate(uint8_t rate_code);

    /// 编码设置波特率命令（0x00=4800 ~ 0x05=115200，0x06=230400，0x07=921600）
    static Cmd encode_set_baudrate(uint8_t baud_code);

    /// 编码保存当前配置命令（写入 Flash）
    static Cmd encode_save_config();

    /// 编码软件复位命令
    static Cmd encode_reset();

    /// 编码陀螺仪标定命令（设备需静止）
    static Cmd encode_calibrate_gyro();

    /// 编码解锁命令（写入任何配置寄存器前必须发送，有效期 10 秒）
    /// 解锁序列：0xFF 0xAA 0x69 0x88 0xB5
    static Cmd encode_unlock();

   private:
    bool parse_frame(const uint8_t* frame);

    uint8_t buf_[FRAME_LEN]{};
    size_t buf_pos_{0};
    bool synced_{false};
    bool frame_ready_{false};
    ImuData current_{};
};

}  // namespace robot::protocol
