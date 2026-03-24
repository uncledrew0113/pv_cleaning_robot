#include <chrono>
#include <thread>

#include "pv_cleaning_robot/device/walk_motor_group.h"

namespace robot::device {

// ── 构造 / 析构 ─────────────────────────────────────────────────────────────

WalkMotorGroup::WalkMotorGroup(std::shared_ptr<hal::ICanBus> can, uint8_t id_base)
    : can_(std::move(can))
    , id_base_(id_base)
    , ctrl_id_((id_base <= 4u) ? protocol::kWalkMotorCtrlIdGroup1
                               : protocol::kWalkMotorCtrlIdGroup2)
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
    // 使用栈数组避免 std::vector 的堆分配
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
        std::lock_guard<std::mutex> lk(mtx_);
        ++ctrl_frame_count_;
        return DeviceError::OK;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    ++ctrl_err_count_;
    return DeviceError::COMM_TIMEOUT;
}

// ── 模式控制 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::set_mode_all(protocol::WalkMotorMode mode) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    // 0x105 批量帧，一帧覆盖4台电机，非本组 slot 填 ENABLE（默认安全完成）
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
    // 输入钳位
    auto clamp = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    lt = clamp(lt, -210.0f, 210.0f);
    rt = clamp(rt, -210.0f, 210.0f);
    lb = clamp(lb, -210.0f, 210.0f);
    rb = clamp(rb, -210.0f, 210.0f);

    // 一帧覆盖4台电机
    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
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
        std::lock_guard<std::mutex> lk(mtx_);
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
        std::lock_guard<std::mutex> lk(mtx_);
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
    auto clamp_pos = [](float v) { return v < 0.0f ? 0.0f : (v > 360.0f ? 360.0f : v); };
    lt_deg = clamp_pos(lt_deg);
    rt_deg = clamp_pos(rt_deg);
    lb_deg = clamp_pos(lb_deg);
    rb_deg = clamp_pos(rb_deg);
    auto frame = protocol::WalkMotorCanCodec::encode_group_position(
        id_base_, lt_deg, rt_deg, lb_deg, rb_deg);
    {
        std::lock_guard<std::mutex> lk(mtx_);
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
        fb_byte = 0x80u;  // 查询方式（Bit7=1）
    } else {
        uint8_t period = (period_ms > 127u) ? 127u : period_ms;
        fb_byte = period & 0x7Fu;  // 主动上报方式
    }
    std::array<uint8_t, 8> fb_modes;
    fb_modes.fill(fb_byte);  // 所月7个非本组 slot 也同步设置（安全）
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

// ── 状态读取 ─────────────────────────────────────────────────────────────────

WalkMotor::Status WalkMotorGroup::get_wheel_status(Wheel w) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<WalkMotor::Status>(diag_[static_cast<int>(w)]);
}

WalkMotor::Diagnostics WalkMotorGroup::get_wheel_diagnostics(Wheel w) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_[static_cast<int>(w)];
}

WalkMotorGroup::GroupStatus WalkMotorGroup::get_group_status() const {
    std::lock_guard<std::mutex> lk(mtx_);
    GroupStatus gs;
    for (int i = 0; i < kWheelCount; ++i)
        gs.wheel[i] = static_cast<WalkMotor::Status>(diag_[i]);
    return gs;
}

WalkMotorGroup::GroupDiagnostics WalkMotorGroup::get_group_diagnostics() const {
    std::lock_guard<std::mutex> lk(mtx_);
    GroupDiagnostics gd;
    for (int i = 0; i < kWheelCount; ++i)
        gd.wheel[i] = diag_[i];
    gd.ctrl_frame_count = ctrl_frame_count_;
    gd.ctrl_err_count = ctrl_err_count_;
    return gd;
}

// ── 周期心跳 ─────────────────────────────────────────────────────────────────

void WalkMotorGroup::update() {
    if (!can_->is_open())
        return;

    auto now = std::chrono::steady_clock::now();

    // 更新各轮 online 状态
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < kWheelCount; ++i) {
            bool ever_received = (last_fb_time_[i] != std::chrono::steady_clock::time_point{});
            if (ever_received) {
                bool online = (now - last_fb_time_[i]) < kOnlineTimeout;
                if (!online && diag_[i].online) {
                    diag_[i].online = false;
                    ++diag_[i].feedback_lost_count;
                }
            }
        }
    }

    // 重发当前设定值（心跳，维持电机通信看门狗）
    hal::CanFrame ctrl;
    bool has;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ctrl = last_ctrl_frame_;
        has = has_ctrl_frame_;
    }
    if (has)
        send_ctrl(ctrl);
}

// ── 后台接收线程 ─────────────────────────────────────────────────────────────

void WalkMotorGroup::recv_loop() {
    hal::CanFrame frame;
    while (running_.load()) {
        // 50 ms 超时：快速响应 close()，不长期阻塞
        if (!can_->recv(frame, 50)) {
            // Bus-Off 时 poll() 会立即返回 POLLERR，若不加退避将导致忙等并占满 CPU。
            if (can_->is_bus_off()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            continue;
        }

        // 确定是哪台电机的反馈帧（0x96 + motor_id）
        for (int i = 0; i < kWheelCount; ++i) {
            auto maybe = codecs_[i].decode_status(frame);
            if (!maybe)
                continue;

            const auto& s = *maybe;
            auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lk(mtx_);
                auto& d = diag_[i];
                d.speed_rpm = s.speed_rpm;
                d.torque_a = s.torque_a;
                d.position_deg = s.position_deg;
                d.fault = s.fault;
                d.mode = s.mode;
                d.online = true;
                ++d.feedback_frame_count;
                last_fb_time_[i] = now;
            }
            break;  // 每帧只能属于一台电机
        }
    }
}

}  // namespace robot::device
