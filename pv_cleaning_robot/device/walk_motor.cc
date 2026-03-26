#include <chrono>
#include <thread>

#include "pv_cleaning_robot/device/walk_motor.h"

namespace robot::device {

WalkMotor::WalkMotor(std::shared_ptr<hal::ICanBus> can, uint8_t motor_id)
    : can_(std::move(can)), codec_(motor_id) {}

WalkMotor::~WalkMotor() {
    close();
}

// ── 生命周期 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotor::open() {
    if (can_->is_open())
        return DeviceError::OK;
    if (!can_->open())
        return DeviceError::NOT_OPEN;

    // 只接收本电机的状态反馈帧（0x96 + motor_id，标准帧 11-bit ID）
    // 使用栈变量调用纯虚接口，避免构造临时 vector 带来的堆分配
    const hal::CanFilter f{codec_.status_can_id(), 0x7FFu};
    if (!can_->set_filters(&f, 1u)) {
        can_->close();
        return DeviceError::NOT_OPEN;
    }

    running_.store(true);
    recv_thread_ = std::thread(&WalkMotor::recv_loop, this);
    return DeviceError::OK;
}

void WalkMotor::close() {
    running_.store(false);
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (can_->is_open())
        can_->close();
}

// ── 内部帧发送 ────────────────────────────────────────────────────────────────

DeviceError WalkMotor::send_frame(const hal::CanFrame& frame) {
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    if (!can_->send(frame)) {
        std::lock_guard<std::mutex> lk(mtx_);
        ++diag_.can_err_count;
        return DeviceError::COMM_TIMEOUT;
    }
    return DeviceError::OK;
}

// ── 控制接口 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotor::set_mode(protocol::WalkMotorMode mode) {
    return send_frame(codec_.encode_set_mode(mode));
}

DeviceError WalkMotor::enable() {
    return set_mode(protocol::WalkMotorMode::ENABLE);
}

DeviceError WalkMotor::disable() {
    return set_mode(protocol::WalkMotorMode::DISABLE);
}

DeviceError WalkMotor::set_feedback_mode(uint8_t period_ms) {
    return send_frame(codec_.encode_set_feedback(period_ms));
}

DeviceError WalkMotor::set_node_id() {
    return send_frame(codec_.encode_set_node_id());
}

DeviceError WalkMotor::set_comm_timeout(uint16_t timeout_ms) {
    return send_frame(codec_.encode_set_comm_timeout(timeout_ms));
}

DeviceError WalkMotor::reset_comm_timeout() {
    return send_frame(codec_.encode_reset_comm_timeout());
}

DeviceError WalkMotor::read_comm_timeout() {
    return send_frame(codec_.encode_read_comm_timeout());
}

DeviceError WalkMotor::set_termination(bool enable) {
    std::array<bool, 8> enables{};
    if (codec_.motor_id() >= 1u && codec_.motor_id() <= 8u)
        enables[codec_.motor_id() - 1u] = enable;
    return send_frame(protocol::WalkMotorCanCodec::encode_set_termination_batch(enables));
}

DeviceError WalkMotor::query_firmware() {
    return send_frame(protocol::WalkMotorCanCodec::encode_query_firmware());
}

DeviceError WalkMotor::set_speed(float rpm) {
    if (rpm < -210.0f || rpm > 210.0f)
        return DeviceError::INVALID_PARAM;
    auto frame = codec_.encode_speed(rpm);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        target_value_ = rpm;
        diag_.target_value = rpm;
    }
    return send_frame(frame);
}

DeviceError WalkMotor::set_current(float amps) {
    if (amps < -33.0f || amps > 33.0f)
        return DeviceError::INVALID_PARAM;
    auto frame = codec_.encode_current(amps);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        target_value_ = amps;
        diag_.target_value = amps;
    }
    return send_frame(frame);
}

DeviceError WalkMotor::set_position(float deg) {
    if (deg < 0.0f || deg > 360.0f)
        return DeviceError::INVALID_PARAM;
    auto frame = codec_.encode_position(deg);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        target_value_ = deg;
        diag_.target_value = deg;
    }
    return send_frame(frame);
}

DeviceError WalkMotor::set_open_loop(int16_t raw_value) {
    auto frame = codec_.encode_open_loop(raw_value);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ctrl_frame_ = frame;
        has_ctrl_frame_ = true;
        target_value_ = static_cast<float>(raw_value);
        diag_.target_value = target_value_;
    }
    return send_frame(frame);
}

// ── 状态读取 ─────────────────────────────────────────────────────────────────

WalkMotor::Status WalkMotor::get_status() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<Status>(diag_);
}

WalkMotor::Diagnostics WalkMotor::get_diagnostics() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_;
}

// ── 周期心跳 ─────────────────────────────────────────────────────────────────

void WalkMotor::update() {
    if (!can_->is_open())
        return;

    // 1. 更新 online 状态
    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        // last_fb_time_ 为 epoch 时表示从未收到过反馈，不判断超时
        if (last_fb_time_ != std::chrono::steady_clock::time_point{}) {
            bool currently_online = (now - last_fb_time_) < kOnlineTimeout;
            if (!currently_online && diag_.online) {
                diag_.online = false;
                ++diag_.feedback_lost_count;
            }
        }
    }

    // 2. 重发当前设定值（心跳，维持电机通信看门狗）
    hal::CanFrame ctrl;
    bool has;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ctrl = last_ctrl_frame_;
        has = has_ctrl_frame_;
    }
    if (has)
        send_frame(ctrl);
}

// ── 后台接收线程 ─────────────────────────────────────────────────────────────

void WalkMotor::recv_loop() {
    hal::CanFrame frame;
    while (running_.load()) {
        // 50 ms 超时：快速响应 close() 而不长期阻塞
        if (!can_->recv(frame, 50)) {
            // Bus-Off 时 poll() 会立即返回 POLLERR，若不加退避将导致忙等并占满 CPU。
            // 退避 200ms 后再重试，减轻 CPU 负载并给硬件恢复留出时间。
            if (can_->is_bus_off()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            continue;
        }

        auto maybe = codec_.decode_status(frame);
        if (!maybe)
            continue;

        const auto& s = *maybe;
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lk(mtx_);
            diag_.speed_rpm = s.speed_rpm;
            diag_.torque_a = s.torque_a;
            diag_.position_deg = s.position_deg;
            diag_.fault = s.fault;
            diag_.mode = s.mode;
            diag_.online = true;
            ++diag_.feedback_frame_count;
            last_fb_time_ = now;
        }
    }
}

}  // namespace robot::device
