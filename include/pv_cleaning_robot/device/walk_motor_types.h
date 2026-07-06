/**
 * @file walk_motor_types.h
 * @brief 行走电机状态和诊断数据类型。
 *
 * 本文件定义行走电机设备层缓存状态，供 WalkMotorGroup、DiagnosticsCollector 和测试共同使用。
 */
#pragma once
#include <cstdint>

#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

namespace robot::device {

/// @brief 单轮状态与诊断数据类型定义
///
/// 供 WalkMotorGroup、健康上报和诊断采集共享使用。
/// 不包含任何硬件操作接口，仅作单轮数据结构定义。
class WalkMotor {
public:
    /// 单轮精简状态
    struct Status {
        float speed_rpm;                 ///< 实测转速（-210 ~ +210 RPM）
        float torque_a;                  ///< 转矩电流（-33 ~ +33 A）
        float position_deg;              ///< 当前位置（0 ~ 360°）
        protocol::WalkMotorFault fault;  ///< 故障码
        protocol::WalkMotorMode  mode;   ///< 当前运行模式
        bool online;                     ///< 最近 500 ms 内是否收到过反馈帧
    };

    /// 完整诊断数据（开发阶段）
    struct Diagnostics : Status {
        float    target_value;          ///< 当前设定值（量纲取决于模式）
        uint32_t feedback_frame_count;  ///< 累计收到反馈帧数
        uint32_t feedback_lost_count;   ///< 累计在线→离线转换次数
        uint32_t can_err_count;         ///< CAN 发送失败次数
    };
};

}  // namespace robot::device
