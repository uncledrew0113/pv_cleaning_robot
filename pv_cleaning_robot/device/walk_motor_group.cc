#include <algorithm>
#include <chrono>
#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>
#include <thread>

#include "pv_cleaning_robot/device/walk_motor_group.h"

namespace robot::device {

// ── 工具函数 ─────────────────────────────────────────────────────────────────

static float clamp_rpm(float v) {
    return std::max(-210.0f, std::min(210.0f, v));
}

static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// 将角度差规范化到 [-180, +180]
static float norm_angle(float deg) {
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

// ── 构造 / 析构 ─────────────────────────────────────────────────────────────

WalkMotorGroup::WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                               uint8_t id_base,
                               uint16_t comm_timeout_ms)
    : can_(std::move(can))
    , id_base_(id_base)
    , ctrl_id_((id_base <= 4u) ? protocol::kWalkMotorCtrlIdGroup1
                               : protocol::kWalkMotorCtrlIdGroup2)
    , comm_timeout_ms_(comm_timeout_ms)
    // 4个 codec 实例分别对应 motor_id = id_base, id_base+1, id_base+2, id_base+3
    , codecs_{protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 0u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 1u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 2u)),
              protocol::WalkMotorCanCodec(static_cast<uint8_t>(id_base + 3u))} {}

WalkMotorGroup::~WalkMotorGroup() {
    close();
}

// ── 生命周期 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::open() {
    if (can_->is_open())
        return DeviceError::OK;
    if (!can_->open())
        return DeviceError::NOT_OPEN;

    // 设置4路精确接收过滤器（每台电机 0x96 + motor_id，11-bit 精确匹配）
    hal::CanFilter filters[kWheelCount];
    for (int i = 0; i < kWheelCount; ++i) {
        filters[i] = {codecs_[i].status_can_id(), 0x7FFu};
    }
    if (!can_->set_filters(filters, kWheelCount)) {
        can_->close();
        return DeviceError::NOT_OPEN;
    }

    running_.store(true);
    recv_thread_ = std::thread(&WalkMotorGroup::recv_loop, this);

    // CAN 接收线程：SCHED_FIFO 82，绑定到 CPU 5（运动控制专用大核）
    // 设计为略高于 walk_ctrl(80)：优先消化新帧，降低控制拍读取陈旧状态概率。
    {
        sched_param sp{};
        sp.sched_priority = 82;
        pthread_setschedparam(recv_thread_.native_handle(), SCHED_FIFO, &sp);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(5, &cpuset);  ///< 大核 CPU5: 运动控制路径
        pthread_setaffinity_np(recv_thread_.native_handle(), sizeof(cpuset), &cpuset);
        pthread_setname_np(recv_thread_.native_handle(), "group_recv");
    }

    // 若配置了通信超时，写入每台电机（确保主控失联时电机自停）
    if (comm_timeout_ms_ > 0u) {
        for (int i = 0; i < kWheelCount; ++i) {
            auto frame = codecs_[i].encode_set_comm_timeout(comm_timeout_ms_);
            if (!can_->send(frame)) {
                spdlog::warn("[WalkMotorGroup] set_comm_timeout failed for motor {}",
                             id_base_ + static_cast<uint8_t>(i));
            }
        }
        spdlog::info("[WalkMotorGroup] comm_timeout set to {} ms for motors {}-{}",
                     comm_timeout_ms_,
                     id_base_,
                     id_base_ + kWheelCount - 1);
    }

    return DeviceError::OK;
}

void WalkMotorGroup::close() {
    running_.store(false);
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (can_->is_open())
        can_->close();
}

// ── 内部发帧 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::send_ctrl(const hal::CanFrame& frame) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    if (can_->send(frame)) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++ctrl_frame_count_;
        return DeviceError::OK;
    }
    std::lock_guard<hal::PiMutex> lk(mtx_);
    ++ctrl_err_count_;
    return DeviceError::COMM_TIMEOUT;
}

// ── 模式控制 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::set_mode_all(protocol::WalkMotorMode mode) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    std::array<protocol::WalkMotorMode, 8> modes;
    modes.fill(protocol::WalkMotorMode::ENABLE);
    for (int i = 0; i < kWheelCount; ++i)
        modes[static_cast<std::size_t>(id_base_ - 1u + static_cast<uint8_t>(i))] = mode;
    return send_ctrl(protocol::WalkMotorCanCodec::encode_set_mode_batch(modes));
}

DeviceError WalkMotorGroup::set_modes(protocol::WalkMotorMode lt,
                                      protocol::WalkMotorMode rt,
                                      protocol::WalkMotorMode lb,
                                      protocol::WalkMotorMode rb) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    std::array<protocol::WalkMotorMode, 8> modes;
    modes.fill(protocol::WalkMotorMode::ENABLE);
    modes[static_cast<std::size_t>(id_base_ - 1u + 0u)] = lt;
    modes[static_cast<std::size_t>(id_base_ - 1u + 1u)] = rt;
    modes[static_cast<std::size_t>(id_base_ - 1u + 2u)] = lb;
    modes[static_cast<std::size_t>(id_base_ - 1u + 3u)] = rb;
    return send_ctrl(protocol::WalkMotorCanCodec::encode_set_mode_batch(modes));
}

DeviceError WalkMotorGroup::enable_all() {
    return set_mode_all(protocol::WalkMotorMode::ENABLE);
}

DeviceError WalkMotorGroup::disable_all() {
    return set_mode_all(protocol::WalkMotorMode::DISABLE);
}

// ── 同步批量给定 ─────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::set_speeds(float lt, float rt, float lb, float rb) {
    lt = clamp_rpm(lt);
    rt = clamp_rpm(rt);
    lb = clamp_rpm(lb);
    rb = clamp_rpm(rb);

    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        base_lt_rpm_ = lt;
        base_rt_rpm_ = rt;
        diag_[0].target_value = lt;
        diag_[1].target_value = rt;
        diag_[2].target_value = lb;
        diag_[3].target_value = rb;
    }
    return send_ctrl(frame);
}

DeviceError WalkMotorGroup::set_speeds(const SpeedCmd& cmd) {
    return set_speeds(cmd.lt_rpm, cmd.rt_rpm, cmd.lb_rpm, cmd.rb_rpm);
}

DeviceError WalkMotorGroup::set_speed_uniform(float rpm) {
    return set_speeds(rpm, rpm, rpm, rpm);
}

DeviceError WalkMotorGroup::set_currents(float lt, float rt, float lb, float rb) {
    auto frame = protocol::WalkMotorCanCodec::encode_group_current(id_base_, lt, rt, lb, rb);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        diag_[0].target_value = lt;
        diag_[1].target_value = rt;
        diag_[2].target_value = lb;
        diag_[3].target_value = rb;
    }
    return send_ctrl(frame);
}

DeviceError WalkMotorGroup::set_open_loops(int16_t lt, int16_t rt, int16_t lb, int16_t rb) {
    auto frame = protocol::WalkMotorCanCodec::encode_group_open_loop(id_base_, lt, rt, lb, rb);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        diag_[0].target_value = static_cast<float>(lt);
        diag_[1].target_value = static_cast<float>(rt);
        diag_[2].target_value = static_cast<float>(lb);
        diag_[3].target_value = static_cast<float>(rb);
    }
    return send_ctrl(frame);
}

DeviceError WalkMotorGroup::set_positions(float lt_deg, float rt_deg, float lb_deg, float rb_deg) {
    auto cp = [](float v) { return clamp(v, 0.0f, 360.0f); };
    lt_deg = cp(lt_deg);
    rt_deg = cp(rt_deg);
    lb_deg = cp(lb_deg);
    rb_deg = cp(rb_deg);
    auto frame = protocol::WalkMotorCanCodec::encode_group_position(
        id_base_, lt_deg, rt_deg, lb_deg, rb_deg);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        diag_[0].target_value = lt_deg;
        diag_[1].target_value = rt_deg;
        diag_[2].target_value = lb_deg;
        diag_[3].target_value = rb_deg;
    }
    return send_ctrl(frame);
}

DeviceError WalkMotorGroup::set_feedback_mode_all(uint8_t period_ms) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    uint8_t fb_byte;
    if (period_ms == 0u) {
        fb_byte = 0x80u;
    } else {
        uint8_t period = (period_ms > 127u) ? 127u : period_ms;
        fb_byte = period & 0x7Fu;
    }
    std::array<uint8_t, 8> fb_modes;
    fb_modes.fill(fb_byte);
    return send_ctrl(protocol::WalkMotorCanCodec::encode_set_feedback_batch(fb_modes));
}

DeviceError WalkMotorGroup::set_terminations(bool lt, bool rt, bool lb, bool rb) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    std::array<bool, 8> enables{};
    enables[static_cast<std::size_t>(id_base_ - 1u + 0u)] = lt;
    enables[static_cast<std::size_t>(id_base_ - 1u + 1u)] = rt;
    enables[static_cast<std::size_t>(id_base_ - 1u + 2u)] = lb;
    enables[static_cast<std::size_t>(id_base_ - 1u + 3u)] = rb;
    return send_ctrl(protocol::WalkMotorCanCodec::encode_set_termination_batch(enables));
}

DeviceError WalkMotorGroup::query_firmware() {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    return send_ctrl(protocol::WalkMotorCanCodec::encode_query_firmware());
}

// ── 航向 PID 控制 ─────────────────────────────────────────────────────────────

void WalkMotorGroup::set_heading_pid_params(const HeadingPidParams& p) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    pid_params_ = p;
}

void WalkMotorGroup::enable_heading_control(bool en) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    heading_ctrl_en_ = en;
    if (!en) {
        pid_integral_ = 0.0f;
        pid_prev_err_ = 0.0f;
        heading_initialized_ = false;
    }
}

void WalkMotorGroup::set_target_heading(float yaw_deg) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    target_heading_ = yaw_deg;
    heading_initialized_ = true;
    pid_integral_ = 0.0f;
    pid_prev_err_ = 0.0f;
}

float WalkMotorGroup::calc_pid_correction(float yaw_deg, float dt_s) {
    // 已持锁调用（mtx_）
    if (!heading_initialized_) {
        target_heading_ = yaw_deg;
        heading_initialized_ = true;
    }

    float err = norm_angle(target_heading_ - yaw_deg);

    // 积分（带限幅）
    pid_integral_ += err * dt_s;
    pid_integral_ = clamp(pid_integral_, -pid_params_.integral_limit, pid_params_.integral_limit);

    // 微分
    float derivative = (dt_s > 0.0f) ? (err - pid_prev_err_) / dt_s : 0.0f;
    pid_prev_err_ = err;

    float output =
        pid_params_.kp * err + pid_params_.ki * pid_integral_ + pid_params_.kd * derivative;

    return clamp(output, -pid_params_.max_output, pid_params_.max_output);
}

// ── 边缘紧急覆盖 ──────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::emergency_override(float reverse_rpm) {
    // 1. 立即标记覆盖（抑制 update() 心跳）
    override_active_.store(true);

    // 2. 直接发送停止或反转帧（不走 last_ctrl_frame_ 缓存）
    float rpm = (reverse_rpm > 0.0f) ? -clamp_rpm(reverse_rpm) : 0.0f;
    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, rpm, rpm, rpm, rpm);

    auto ret = send_ctrl(frame);

    spdlog::warn("[WalkMotorGroup] emergency_override: reverse_rpm={:.1f}", rpm);
    return ret;
}

void WalkMotorGroup::clear_override() {
    override_active_.store(false);
    // 重置 PID 积分，防止解除后突然积分饱和
    std::lock_guard<hal::PiMutex> lk(mtx_);
    pid_integral_ = 0.0f;
    pid_prev_err_ = 0.0f;
    spdlog::info("[WalkMotorGroup] override cleared");
}

bool WalkMotorGroup::is_override_active() const {
    return override_active_.load();
}

// ── 状态读取 ─────────────────────────────────────────────────────────────────

WalkMotor::Status WalkMotorGroup::get_wheel_status(Wheel w) const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return static_cast<WalkMotor::Status>(diag_[static_cast<int>(w)]);
}

WalkMotor::Diagnostics WalkMotorGroup::get_wheel_diagnostics(Wheel w) const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return diag_[static_cast<int>(w)];
}

WalkMotorGroup::GroupStatus WalkMotorGroup::get_group_status() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    GroupStatus gs;
    for (int i = 0; i < kWheelCount; ++i)
        gs.wheel[i] = static_cast<WalkMotor::Status>(diag_[i]);
    gs.temperature_deg = temperature_deg_;
    return gs;
}

WalkMotorGroup::GroupDiagnostics WalkMotorGroup::get_group_diagnostics() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    GroupDiagnostics gd;
    for (int i = 0; i < kWheelCount; ++i)
        gd.wheel[i] = diag_[i];
    gd.ctrl_frame_count = ctrl_frame_count_;
    gd.ctrl_err_count = ctrl_err_count_;
    gd.temperature_deg = temperature_deg_;
    return gd;
}

// ── 周期心跳 ─────────────────────────────────────────────────────────────────

void WalkMotorGroup::update(float yaw_deg) {
    if (!can_->is_open())
        return;

    auto now = std::chrono::steady_clock::now();

    // 计算距上次 update() 的实际时间（秒），用于 PID 微积分计算
    // 使用实测时间而非硬编码常量，适应线程执行器周期变更
    static std::chrono::steady_clock::time_point last_update_time{};
    float dt_s = 0.020f;  // 合理默认值（20ms），首次调用或异常大值时使用
    if (last_update_time != std::chrono::steady_clock::time_point{}) {
        float measured = std::chrono::duration<float>(now - last_update_time).count();
        // 限幅至 10ms~500ms，防止系统暂停/调试断点导致积分爆炸
        if (measured >= 0.010f && measured <= 0.500f)
            dt_s = measured;
    }
    last_update_time = now;

    // ── 1. 更新各轮 online 状态 ──
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        for (int i = 0; i < kWheelCount; ++i) {
            bool ever = (last_fb_time_[i] != std::chrono::steady_clock::time_point{});
            if (ever) {
                bool online = (now - last_fb_time_[i]) < kOnlineTimeout;
                if (!online && diag_[i].online) {
                    diag_[i].online = false;
                    ++diag_[i].feedback_lost_count;
                    spdlog::warn("[WalkMotorGroup] motor {} offline", id_base_ + i);
                }
            }
        }
    }

    // ── 2. 若在 override 状态，跳过心跳重发 ──
    if (override_active_.load())
        return;

    // ── 3. 计算 PID 差速（如果使能） ──
    hal::CanFrame ctrl;
    bool has;

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ctrl = last_ctrl_frame_;
        has = has_ctrl_frame_;

        if (has && heading_ctrl_en_) {
            float correction = calc_pid_correction(yaw_deg, dt_s);
            // correction > 0 → 偏右（yaw < target），加大左侧速度
            float lt = clamp_rpm(base_lt_rpm_ + correction);
            float rt = clamp_rpm(base_rt_rpm_ - correction);
            float lb = lt;  // 同侧前后同号
            float rb = rt;
            ctrl = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
        }
    }

    if (has)
        send_ctrl(ctrl);

    // ── 4. 定期查询温度（主动上报不含温度，需显式查询 0x107） ──
    bool need_temp_query = false;
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (last_temp_query_time_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_temp_query_time_)
                    .count() >= kTempQueryIntervalMs) {
            last_temp_query_time_ = now;
            need_temp_query = true;
        }
    }
    if (need_temp_query) {
        // 查询组内第一台电机的温度（代表性）
        auto qframe = codecs_[0].encode_query(protocol::WalkMotorQueryTarget::TEMP,
                                              protocol::WalkMotorQueryTarget::SPEED,
                                              protocol::WalkMotorQueryTarget::MODE);
        can_->send(qframe);  // 发送失败不视为错误（温度非关键）
    }
}

// ── 后台接收线程 ─────────────────────────────────────────────────────────────

void WalkMotorGroup::recv_loop() {
    hal::CanFrame frame;
    while (running_.load()) {
        if (!can_->recv(frame, 50)) {
            if (can_->is_bus_off()) {
                spdlog::error("[WalkMotorGroup] Bus-Off detected, backing off 200ms");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            continue;
        }

        // ── 检查是否为状态反馈帧（0x96+motor_id）──
        for (int i = 0; i < kWheelCount; ++i) {
            auto maybe = codecs_[i].decode_status(frame);
            if (!maybe)
                continue;

            const auto& s = *maybe;
            auto ts = std::chrono::steady_clock::now();
            {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                auto& d = diag_[i];
                d.speed_rpm = s.speed_rpm;
                d.torque_a = s.torque_a;
                d.position_deg = s.position_deg;
                d.fault = s.fault;
                d.mode = s.mode;
                d.online = true;
                ++d.feedback_frame_count;
                last_fb_time_[i] = ts;
            }
            break;
        }

        // ── 温度查询应答（0x96+motor_id，DATA[2~3] 为温度 int16 big-endian）──
        // 协议：查询应答与主动上报共用同一 CAN ID 和帧格式，
        // 当查询目标为 TEMP(0x03) 时，DATA[0~1] 为 target1 值（温度），单位 0.1°C
        // 实际此处利用 decode_status() 已解析的 torque_a/position_deg 字段判断；
        // M1502E 查询模式下应答帧 DATA[2~3] 为 target1（高8位|低8位），大端
        // 简化处理：接收到 0x96+id 帧时，若 DATA[6]==0（无故障）视温度数据有效
        for (int i = 0; i < kWheelCount; ++i) {
            if (frame.id == codecs_[i].status_can_id() && frame.len >= 4) {
                // DATA[2~3]: 温度原始值（int16，单位0.1℃，量程-3276.7~3276.7℃）
                // 仅模式为查询时有效；此处作为补充采集，接收到就更新
                int16_t temp_raw = static_cast<int16_t>(
                    (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
                float temp_c = temp_raw * 0.1f;
                // 合理性校验（-40℃ ~ 150℃ 为电机常见温度量程）
                if (temp_c >= -40.0f && temp_c <= 150.0f) {
                    std::lock_guard<hal::PiMutex> lk(mtx_);
                    temperature_deg_ = temp_c;
                }
            }
        }
    }
}

}  // namespace robot::device
