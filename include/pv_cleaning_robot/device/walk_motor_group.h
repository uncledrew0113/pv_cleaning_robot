#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/device/walk_motor_types.h"
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

namespace robot::device {

/// 4轮行走电机组（M1502E_111，单 CAN 总线）
///
/// 核心优化：协议层一帧覆盖4台电机（0x32 或 0x33），
/// set_speeds() 仅更新当前 normal 控制槽，由 update() 统一重发同步下发，
/// 彻底消除逐台发送带来的时间偏斜与总线负载（4帧→1帧，负载降低 75%）。
///
/// 物理布局（清扫机器人）：
///   Wheel::LT （左上）  motor_id = id_base + 0
///   Wheel::RT （右上）  motor_id = id_base + 1
///   Wheel::LB （左下）  motor_id = id_base + 2
///   Wheel::RB （右下）  motor_id = id_base + 3
///
/// 新增功能：
///   - 通信超时下发：open() 时自动向电机写入 comm_timeout_ms 超时时间
///   - 锁存式紧急覆盖：emergency_override() 立即发停车或反转帧，
///     并锁存屏蔽 normal 心跳，直到 clear_override() 被 update() 应用
///
/// 使用步骤：
///   1. 构造时传入共享 CAN 总线实例和 id_base（默认 1）
///   2. open() → set_mode_all(SPEED) → set_speeds(...)
///   3. 周期调用 update() 维持当前 normal 控制帧心跳
class WalkMotorGroup {
   public:
    static constexpr int kWheelCount = 4;

    enum class ControlMode : uint8_t {
        Normal = 0,
        LatchedOverride = 1,
    };

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
    /// @param comm_timeout_ms  开机时写入电机的通信超时（ms），0=禁用；
    ///                         建议设为 update() 周期的 3~5 倍，如
    ///                         建议设为 update() 周期的 5~10 倍，如
    ///                         update()=20ms 时设 200ms（10× 余量）
    /// @param termination_init_enabled open() 时是否自动发送终端电阻初始化帧
    /// @param termination_init_retry_count 终端电阻初始化发送次数，0 按 1 次处理
    /// @param termination_motor_id 需打开终端电阻的物理 motor_id
    explicit WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                            uint8_t id_base = 1u,
                            uint16_t comm_timeout_ms = 200u,
                            bool termination_init_enabled = true,
                            uint8_t termination_init_retry_count = 3u,
                            uint8_t termination_motor_id = 2u);
    ~WalkMotorGroup();

    // ── 生命周期 ──────────────────────────────────────────────────────────
    /// 打开 CAN 总线，设置4路接收过滤器，启动单一后台接收线程；
    /// 若 comm_timeout_ms > 0，向每台电机写入通信超时
    DeviceError open();
    /// 先安全停机，再停接收线程，再关闭 CAN
    void close();

    // ── 模式控制 ──────────────────────────────────────────────────────────
    /// 向全部4台电机发出单mode帧（0x105 批量 1 帧）
    DeviceError set_mode_all(protocol::WalkMotorMode mode);
    /// 每轮指定不同模式（0x105 批量 1 帧）
    DeviceError set_modes(protocol::WalkMotorMode lt,
                          protocol::WalkMotorMode rt,
                          protocol::WalkMotorMode lb,
                          protocol::WalkMotorMode rb);
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
    /// 速度环给定：更新当前 normal 速度控制槽（-210 ~ +210 RPM）
    DeviceError set_speeds(float lt, float rt, float lb, float rb);
    DeviceError set_speeds(const SpeedCmd& cmd);
    /// 全部相同速度（正=前进，负=后退）
    DeviceError set_speed_uniform(float rpm);

    /// 电流环给定：更新当前 normal 电流控制槽（-33 ~ +33 A）
    DeviceError set_currents(float lt, float rt, float lb, float rb);

    /// 开环电压给定：更新当前 normal 开环控制槽（-32767 ~ +32767 raw）
    DeviceError set_open_loops(int16_t lt, int16_t rt, int16_t lb, int16_t rb);
    /// 位置环给定：更新当前 normal 位置控制槽（0 ~ 360°，絶对位置）
    DeviceError set_positions(float lt_deg, float rt_deg, float lb_deg, float rb_deg);

    // ── 边缘紧急覆盖（优先级最高，立即生效）────────────────────────────
    /// 立即发送停止或反转帧，并暂停心跳重发直到 clear_override() + update()
    /// @param reverse_rpm  反转速度（>0 表示反转，0 表示原地停止）
    DeviceError emergency_override(float reverse_rpm = 0.0f);
    /// 请求解除紧急覆盖；真正切回 Normal 发生在下一次 update() 中。
    /// @note 调用本函数后须调用一次 update() 才能令 is_override_active() 返回 false；
    ///       这一设计保证锁存覆盖先被状态机确认解除，再恢复 normal 控制心跳。
    void clear_override();
    bool is_override_active() const;
    /// 每当 clear_override() 在 update() 中真正生效一次，generation 加 1。
    uint32_t override_clear_generation() const;

    // ── 状态读取（线程安全，无 I/O）────────────────────────────────────
    WalkMotor::Status get_wheel_status(Wheel w) const;
    WalkMotor::Diagnostics get_wheel_diagnostics(Wheel w) const;
    GroupStatus get_group_status() const;
    GroupDiagnostics get_group_diagnostics() const;

    // ── 周期心跳（建议由控制线程调用，50 ms）─────────────────────────
    /// 心跳推进：更新 online 状态；在 Normal 模式重发当前 normal 控制帧。
    void update();

   private:
    // ── Normal / Override 控制状态 ──────────────────────────────────────
    // 普通控制采用“最新槽位”语义：set_* 更新当前 normal 控制帧，update() 仅重发最后一条。
    hal::CanFrame normal_ctrl_frame_{};
    std::array<float, 4> normal_target_values_{};
    bool has_normal_ctrl_frame_{false};

    hal::CanFrame override_frame_{};
    std::array<float, 4> override_target_values_{};

    ControlMode control_mode_{ControlMode::Normal};
    uint32_t override_clear_generation_{0};

    std::shared_ptr<hal::ICanBus> can_;
    uint8_t id_base_;                           ///< 1 或 5
    uint16_t comm_timeout_ms_;                  ///< 开机写入电机的通信超时
    bool termination_init_enabled_{true};       ///< open() 时是否自动发终端电阻初始化
    uint8_t termination_init_retry_count_{3u};  ///< 终端电阻初始化发送次数
    uint8_t termination_motor_id_{2u};          ///< 目标物理 motor_id
    std::array<protocol::WalkMotorCanCodec, kWheelCount> codecs_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> closing_{false};

    mutable hal::PiMutex mtx_;
    std::array<WalkMotor::Diagnostics, kWheelCount> diag_{};
    std::array<std::chrono::steady_clock::time_point, kWheelCount> last_fb_time_{};

    // 带统计的发帧计数
    uint32_t ctrl_frame_count_{0};
    uint32_t ctrl_err_count_{0};

    // ── 边缘紧急覆盖 ──────────────────────────────────────────────────
    // clear_override() 请求标志；在下一次 update() 中真正把状态从 override 切回 normal。
    bool clear_override_pending_{false};
    /// CAN TX 串行化锁：确保 emergency_override() 的覆盖帧与 update() 的 normal 心跳
    /// 不会交错发送；在 send 锁内会二次确认当前控制模式。
    hal::PiMutex send_mtx_;

    static constexpr auto kOnlineTimeout = std::chrono::milliseconds(500);

    DeviceError send_ctrl(const hal::CanFrame& frame);
    void recv_loop();
};

}  // namespace robot::device
