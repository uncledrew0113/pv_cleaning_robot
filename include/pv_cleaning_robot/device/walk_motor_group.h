#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

namespace robot::device {

/// 4轮行走电机组（M1502E_111，单 CAN 总线）
///
/// 核心优化：协议层一帧覆盖4台电机（0x32 或 0x33），
/// set_speeds() 仅发送 1 帧即可同步更新全部4路速度给定，
/// 彻底消除逐台发送带来的时间偏斜与总线负载（4帧→1帧，负载降低 75%）。
///
/// 物理布局（清扫机器人）：
///   Wheel::LT （左上）  motor_id = id_base + 0
///   Wheel::RT （右上）  motor_id = id_base + 1
///   Wheel::LB （左下）  motor_id = id_base + 2
///   Wheel::RB （右下）  motor_id = id_base + 3
///
/// 使用步骤：
///   1. 构造时传入共享 CAN 总线实例和 id_base（默认 1）
///   2. open() → set_mode_all(SPEED) → set_speeds(...)
///   3. 每 10 ms 调用 update() 维持心跳
class WalkMotorGroup {
   public:
    static constexpr int kWheelCount = 4;

    /// 车轮角色（下标用于数组索引，与 motor_id 偏移一一对应）
    enum class Wheel : int {
        LT = 0,  ///< Left-Top  （左上轮）
        RT = 1,  ///< Right-Top （右上轮）
        LB = 2,  ///< Left-Bottom（左下轮）
        RB = 3,  ///< Right-Bottom（右下轮）
    };

    using WheelArr = std::array<float, kWheelCount>;

    /// 4轮速度命令（RPM，正=正转，负=反转）
    struct SpeedCmd {
        float lt_rpm = 0.0f;  ///< 左上
        float rt_rpm = 0.0f;  ///< 右上
        float lb_rpm = 0.0f;  ///< 左下
        float rb_rpm = 0.0f;  ///< 右下
    };

    /// 各轮精简状态
    struct GroupStatus {
        std::array<WalkMotor::Status, kWheelCount> wheel{};
    };

    /// 完整诊断数据（含发帧统计）
    struct GroupDiagnostics {
        std::array<WalkMotor::Diagnostics, kWheelCount> wheel{};
        uint32_t ctrl_frame_count = 0;  ///< 已发出的组合控制帧总数
        uint32_t ctrl_err_count = 0;    ///< 控制帧发送失败次数
    };

    /// @param can      与4台电机共用的 CAN 总线实例
    /// @param id_base  组内首台电机的 motor_id（必须为 1 或 5）
    explicit WalkMotorGroup(std::shared_ptr<hal::ICanBus> can, uint8_t id_base = 1u);
    ~WalkMotorGroup();

    // ── 生命周期 ──────────────────────────────────────────────────────────
    /// 打开 CAN 总线，设置4路接收过滤器，启动单一后台接收线程
    DeviceError open();
    /// 停止接收线程，关闭 CAN 总线
    void close();

    // ── 模式控制 ──────────────────────────────────────────────────────────
    /// 向全部4台电机发出tsinglemode帧（0x105 批量 1 帧）
    DeviceError set_mode_all(protocol::WalkMotorMode mode);
    /// 每轮指定不同模式（0x105 批量 1 帧）
    DeviceError set_modes(protocol::WalkMotorMode lt, protocol::WalkMotorMode rt,
                          protocol::WalkMotorMode lb, protocol::WalkMotorMode rb);
    /// 使能全部
    DeviceError enable_all();
    /// 失能全部
    DeviceError disable_all();

    /// 批量设置反馈方式（0x106 批量 1 帧）
    ///   period_ms=0 切换为查询方式，1~127 为主动上报周期 ms
    DeviceError set_feedback_mode_all(uint8_t period_ms);

    /// 设置4轮终端电阻（0x109）
    DeviceError set_terminations(bool lt, bool rt, bool lb, bool rb);

    /// 固件版本查询广播（0x10B）
    DeviceError query_firmware();

    // ── 同步批量给定 ─────────────────────────────────────────────────────
    /// 速度环给定：一帧同步设定4台电机（-210 ~ +210 RPM）
    DeviceError set_speeds(float lt, float rt, float lb, float rb);
    DeviceError set_speeds(const SpeedCmd& cmd);
    /// 全部相同速度（正=前进，负=后退）
    DeviceError set_speed_uniform(float rpm);

    /// 电流环给定：一帧同步设定4台电机（-33 ~ +33 A）
    DeviceError set_currents(float lt, float rt, float lb, float rb);

    /// 开环电压给定：一帧同步设定4台电机（-32767 ~ +32767 raw）
    DeviceError set_open_loops(int16_t lt, int16_t rt, int16_t lb, int16_t rb);
    /// 位置环给定：一帧同步设定4台电机（0 ~ 360°，絶对位置）
    DeviceError set_positions(float lt_deg, float rt_deg, float lb_deg, float rb_deg);
    // ── 状态读取（线程安全，无 I/O）────────────────────────────────────
    WalkMotor::Status get_wheel_status(Wheel w) const;
    WalkMotor::Diagnostics get_wheel_diagnostics(Wheel w) const;
    GroupStatus get_group_status() const;
    GroupDiagnostics get_group_diagnostics() const;

    // ── 周期心跳（建议由控制线程调用，10 ms）─────────────────────────
    /// 重发当前设定值；轮询 online 超时状态
    void update();

   private:
    std::shared_ptr<hal::ICanBus> can_;
    uint8_t id_base_;   ///< 1 或 5
    uint32_t ctrl_id_;  ///< 0x32 或 0x33
    std::array<protocol::WalkMotorCanCodec, kWheelCount> codecs_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mtx_;
    std::array<WalkMotor::Diagnostics, kWheelCount> diag_{};
    std::array<std::chrono::steady_clock::time_point, kWheelCount> last_fb_time_{};

    hal::CanFrame last_ctrl_frame_{};
    bool has_ctrl_frame_{false};

    // 带统计的发帧计数
    uint32_t ctrl_frame_count_{0};
    uint32_t ctrl_err_count_{0};

    static constexpr auto kOnlineTimeout = std::chrono::milliseconds(500);

    DeviceError send_ctrl(const hal::CanFrame& frame);
    void recv_loop();
};

}  // namespace robot::device
