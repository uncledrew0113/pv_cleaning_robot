#pragma once
#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace robot::device {

/// 行走电机设备（M1502E_111，CAN 总线，500 Kbps）
///
/// 每个实例管控 1 台电机（motor_id 1~8）。
/// open() 后启动后台接收线程，消费电机 100 Hz 主动推送的状态帧。
/// set_speed() / set_current() 等控制方法直接发送 CAN 帧，延迟 < 1 ms。
/// update() 建议每 10 ms 调用一次，重发当前设定值作为通信保活心跳。
class WalkMotor {
public:
    /// 精简状态（生产上报）
    struct Status {
        float          speed_rpm;     ///< 实测转速（-210 ~ +210 RPM）
        float          torque_a;      ///< 转矩电流（-33 ~ +33 A）
        float          position_deg;  ///< 当前位置（0 ~ 360°）
        protocol::WalkMotorFault fault;  ///< 故障码
        protocol::WalkMotorMode  mode;   ///< 当前运行模式
        bool           online;        ///< 最近 500 ms 内是否收到过反馈帧
    };

    /// 完整诊断数据（开发阶段）
    struct Diagnostics : Status {
        float    target_value;          ///< 当前设定值（量纲取决于模式）
        uint32_t feedback_frame_count;  ///< 累计收到反馈帧数
        uint32_t feedback_lost_count;   ///< 累计在线→离线转换次数
        uint32_t can_err_count;         ///< CAN 发送失败次数
    };

    WalkMotor(std::shared_ptr<hal::ICanBus> can, uint8_t motor_id);
    ~WalkMotor();

    // ── 生命周期 ──────────────────────────────────────────────────────────
    /// 打开 CAN 总线，设置接收过滤器，启动接收线程
    DeviceError open();
    /// 停止接收线程，关闭 CAN 总线
    void        close();

    // ── 控制接口 ──────────────────────────────────────────────────────────
    /// 切换运行模式（使能/失能/速度/电流/位置/开环）
    DeviceError set_mode(protocol::WalkMotorMode mode);
    /// 使能（等价于 set_mode(ENABLE)）
    DeviceError enable();
    /// 失能（等价于 set_mode(DISABLE)）
    DeviceError disable();

    /// 设置反馈方式（0x106），period_ms=0 切换为查询方式，1~127 为主动上报周期
    DeviceError set_feedback_mode(uint8_t period_ms);

    /// 设置电机 ID（0x108）— 每次上电仅支持设置一次
    DeviceError set_node_id();

    /// 写入通信超时（0x10A）；timeout_ms=0 禁用超时，范围 0~65535 ms
    DeviceError set_comm_timeout(uint16_t timeout_ms);
    /// 复位通信超时为默认值 0
    DeviceError reset_comm_timeout();
    /// 读取当前通信超时设置
    DeviceError read_comm_timeout();

    /// 设置本电机 CAN 终端电阻（0x109）
    DeviceError set_termination(bool enable);

    /// 固件版本查询广播（0x10B，所有电机均回复）
    DeviceError query_firmware();

    /// 速度环给定（需先切换到 SPEED 模式）；范围 -210 ~ +210 RPM
    DeviceError set_speed(float rpm);
    /// 电流环给定（需先切换到 CURRENT 模式）；范围 -33 ~ +33 A
    DeviceError set_current(float amps);
    /// 位置环给定（需先切换到 POSITION 模式）；范围 0 ~ 360°
    DeviceError set_position(float deg);
    /// 开环电压给定（需先切换到 OPEN_LOOP 模式）；范围 -32767 ~ +32767
    DeviceError set_open_loop(int16_t raw_value);

    // ── 状态读取（线程安全，无 I/O）─────────────────────────────────────
    Status      get_status()      const;
    Diagnostics get_diagnostics() const;

    // ── 周期心跳（建议由控制线程调用，10 ms）────────────────────────────
    /// 重发当前设定值；更新 online 状态；维持电机通信看门狗
    void update();

private:
    std::shared_ptr<hal::ICanBus>  can_;
    protocol::WalkMotorCanCodec    codec_;

    std::thread       recv_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mtx_;
    Diagnostics        diag_{};

    /// 当前设定值对应的 CAN 帧（由 set_xx 方法填写）
    hal::CanFrame last_ctrl_frame_{};
    bool          has_ctrl_frame_{false};
    float         target_value_{0.0f};

    /// 最近一次收到反馈帧的时刻（epoch 表示"从未收到"）
    std::chrono::steady_clock::time_point last_fb_time_{};

    static constexpr auto kOnlineTimeout = std::chrono::milliseconds(500);

    void        recv_loop();
    DeviceError send_frame(const hal::CanFrame& frame);
};

}  // namespace robot::device
