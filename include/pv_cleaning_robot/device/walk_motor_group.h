#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include "pv_cleaning_robot/hal/pi_mutex.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"
#include "pv_cleaning_robot/service/heading_pid_controller.h"

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
/// 新增功能：
///   - 通信超时下发：open() 时自动向电机写入 comm_timeout_ms 超时时间
///   - 航向角 PID：set_heading_pid_params() 设置参数后，
///     update(yaw_deg) 周期计算左右差速补偿，维持直线行驶
///   - 边缘紧急覆盖：emergency_override() 立即发停车或反转帧，
///     并抑制 update() 心跳重发，直到 clear_override() 解除
///   - 温度轮询：update() 每 kTempQueryIntervalMs 发一次 0x107 查询帧
///
/// 使用步骤：
///   1. 构造时传入共享 CAN 总线实例和 id_base（默认 1）
///   2. open() → set_mode_all(SPEED) → set_speeds(...)
///   3. 每 10 ms 调用 update(yaw_deg) 维持心跳+PID
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

    /// 航向 PID 参数（等价于 service::HeadingPidController::Params，向后兼容别名）
    using HeadingPidParams = service::HeadingPidController::Params;

    /// @param can      与4台电机共用的 CAN 总线实例
    /// @param id_base  组内首台电机的 motor_id（必须为 1 或 5）
    /// @param comm_timeout_ms  开机时写入电机的通信超时（ms），0=禁用；
    ///                         建议设为 update() 周期的 3~5 倍，如
    ///                         建议设为 update() 周期的 5~10 倍，如
    ///                         update()=20ms 时设 200ms（10× 余量）
    explicit WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                            uint8_t id_base = 1u,
                            uint16_t comm_timeout_ms = 200u);
    ~WalkMotorGroup();

    // ── 生命周期 ──────────────────────────────────────────────────────────
    /// 打开 CAN 总线，设置4路接收过滤器，启动单一后台接收线程；
    /// 若 comm_timeout_ms > 0，向每台电机写入通信超时
    DeviceError open();
    /// 停止接收线程，关闭 CAN 总线
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

    // ── 航向 PID 控制 ────────────────────────────────────────────────────
    /// 设置航向 PID 参数（默认已有合理初值）
    void set_heading_pid_params(const HeadingPidParams& p);
    /// 使能/禁用航向 PID（set_speeds 设定目标速度后，update(yaw) 自动补偿差速）
    void enable_heading_control(bool en);
    /// 更新航向目标（首次调用时锁定当前航向为目标）
    void set_target_heading(float yaw_deg);

    // ── 边缘紧急覆盖（优先级最高，立即生效）────────────────────────────
    /// 立即发送停止或反转帧，并暂停心跳重发直到 clear_override()
    /// @param reverse_rpm  反转速度（>0 表示反转，0 表示原地停止）
    DeviceError emergency_override(float reverse_rpm = 0.0f);
    /// 解除紧急覆盖，恢复心跳重发和 PID
    void clear_override();
    bool is_override_active() const;

    // ── 状态读取（线程安全，无 I/O）────────────────────────────────────
    WalkMotor::Status get_wheel_status(Wheel w) const;
    WalkMotor::Diagnostics get_wheel_diagnostics(Wheel w) const;
    GroupStatus get_group_status() const;
    GroupDiagnostics get_group_diagnostics() const;

    // ── 周期心跳（建议由控制线程调用，50 ms）─────────────────────────
    /// 重发当前设定值（含 PID 差速补偿）；轮询 online 超时状态；
    /// @param yaw_deg  当前航向角（来自 IMU），仅在 PID 使能时有效
    void update(float yaw_deg = 0.0f);

   private:
    // ── 命令队列（Command Queue）─────────────────────────────────────────
    /// set_speeds/set_currents/set_open_loops/set_positions/clear_override
    /// 全部异步投递此队列，update()（walk_ctrl 线程）作为唯一消费者，
    /// 序列化所有控制帧状态变更，彻底消除多核竞态（Q7/Q8 修复）
    struct Cmd {
        enum class Type : uint8_t { SET_CTRL_FRAME } type{Type::SET_CTRL_FRAME};
        hal::CanFrame frame{};    ///< 预编码 CAN 帧（SET_CTRL_FRAME 时有效）
        float base_lt_rpm{0.0f};  ///< PID 基础速度（速度帧填，其余为 0）
        float base_rt_rpm{0.0f};
        std::array<float, 4> target_rpms{};  ///< diag[i].target_value 对应值
    };
    static constexpr int kCmdQueueSize = 8;  ///< 环形缓冲深度（正常最多 3 条/周期）
    std::array<Cmd, kCmdQueueSize> cmd_buf_{};
    int cmd_head_{0};       ///< 写指针（生产者写入，cmd_mtx_ 保护）
    int cmd_tail_{0};       ///< 读指针（update() 消费，cmd_mtx_ 保护）
    hal::PiMutex cmd_mtx_;  ///< 独立于 mtx_，保护 cmd_buf_/cmd_head_/cmd_tail_

    std::shared_ptr<hal::ICanBus> can_;
    uint8_t id_base_;           ///< 1 或 5
    uint32_t ctrl_id_;          ///< 0x32 或 0x33
    uint16_t comm_timeout_ms_;  ///< 开机写入电机的通信超时
    std::array<protocol::WalkMotorCanCodec, kWheelCount> codecs_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    mutable hal::PiMutex mtx_;
    std::array<WalkMotor::Diagnostics, kWheelCount> diag_{};
    std::array<std::chrono::steady_clock::time_point, kWheelCount> last_fb_time_{};

    hal::CanFrame last_ctrl_frame_{};
    bool has_ctrl_frame_{false};
    float base_lt_rpm_{0.0f};  ///< PID 前基础左侧速度
    float base_rt_rpm_{0.0f};  ///< PID 前基础右侧速度

    // 带统计的发帧计数
    uint32_t ctrl_frame_count_{0};
    uint32_t ctrl_err_count_{0};

    // ── 航向 PID 控制器 ─────────────────────────────────────────────────
    /// 在 mtx_ 锁保护下访问；不含硬件依赖，便于独立单元测试。
    service::HeadingPidController pid_ctrl_;

    // ── 边缘紧急覆盖 ──────────────────────────────────────────────────
    std::atomic<bool> override_active_{false};
    /// CLEAR_OVERRIDE 原子标志：clear_override() 直写，update() step2 优先检查，
    /// 不占命令队列位置，任意满载下均不可丢失。
    std::atomic<bool> pending_clear_override_{false};
    /// CAN TX 串行化锁：emergency_override() 与 update() PID 发帧路径共享。
    /// 保证 stop 帧永远是总线上最后一帧（两条防线之二，详见 CONCURRENCY.md）。
    /// 注意：仅保护 update() PID 控制帧路径；
    ///       配置帧（enable_all / set_mode_all 等）不经此锁，避免 start_returning()
    ///       在 CLEAR_OVERRIDE 入队但未消费期间错误丢帧。
    hal::PiMutex send_mtx_;
    /// update() 调用间隔计算基准（替代 static 局部变量，支持多实例和 close/open 重启重置）
    std::chrono::steady_clock::time_point last_update_time_{};

    static constexpr auto kOnlineTimeout = std::chrono::milliseconds(500);

    DeviceError send_ctrl(const hal::CanFrame& frame);
    void enqueue_cmd(const Cmd& cmd);  ///< 投入环形缓冲，满时丢弃最旧并 warn
    void recv_loop();
};

}  // namespace robot::device
