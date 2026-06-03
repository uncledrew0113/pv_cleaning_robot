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

static bool motor_id_in_group(uint8_t id_base, uint8_t motor_id) {
    return motor_id >= id_base &&
           motor_id < static_cast<uint8_t>(id_base + WalkMotorGroup::kWheelCount);
}

static hal::CanFrame make_group_termination_frame(uint8_t motor_id) {
    std::array<bool, 8> enables{};
    enables[static_cast<std::size_t>(motor_id - 1u)] = true;
    return protocol::WalkMotorCanCodec::encode_set_termination_batch(enables);
}

// ── 构造 / 析构 ─────────────────────────────────────────────────────────────

WalkMotorGroup::WalkMotorGroup(std::shared_ptr<hal::ICanBus> can,
                               uint8_t id_base,
                               uint16_t comm_timeout_ms,
                               bool termination_init_enabled,
                               uint8_t termination_init_retry_count,
                               uint8_t termination_motor_id)
    : can_(std::move(can))
    , id_base_(id_base)
    , comm_timeout_ms_(comm_timeout_ms)
    , termination_init_enabled_(termination_init_enabled)
    , termination_init_retry_count_(termination_init_retry_count)
    , termination_motor_id_(termination_motor_id)
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
    if (id_base_ != 1u && id_base_ != 5u) {
        spdlog::error("[WalkMotorGroup] invalid id_base={} (expected 1 or 5)", id_base_);
        return DeviceError::NOT_OPEN;
    }
    closing_.store(false, std::memory_order_release);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        control_mode_ = ControlMode::Normal;
        clear_override_pending_ = false;
    }
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

    if (termination_init_enabled_) {
        if (!motor_id_in_group(id_base_, termination_motor_id_)) {
            spdlog::error("[WalkMotorGroup] invalid termination_motor_id={} for group base={}",
                          termination_motor_id_,
                          id_base_);
            running_.store(false);
            if (recv_thread_.joinable())
                recv_thread_.join();
            can_->close();
            return DeviceError::NOT_OPEN;
        }

        const uint8_t retry_count =
            termination_init_retry_count_ == 0u ? 1u : termination_init_retry_count_;
        const auto frame = make_group_termination_frame(termination_motor_id_);
        bool any_success = false;
        spdlog::info("[WalkMotorGroup] init termination motor_id={} retries={}",
                     termination_motor_id_,
                     retry_count);
        for (uint8_t i = 0; i < retry_count; ++i) {
            const bool ok = can_->send(frame);
            {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                if (ok)
                    ++ctrl_frame_count_;
                else
                    ++ctrl_err_count_;
            }
            if (ok) {
                any_success = true;
            } else {
                spdlog::warn("[WalkMotorGroup] termination init send failed attempt {}/{}",
                             static_cast<unsigned>(i + 1u),
                             static_cast<unsigned>(retry_count));
            }
        }
        if (!any_success) {
            spdlog::error(
                "[WalkMotorGroup] termination init failed for motor_id={}, bus may rely on "
                "external termination",
                termination_motor_id_);
        }
    } else {
        spdlog::info("[WalkMotorGroup] termination init disabled by config");
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
    closing_.store(true, std::memory_order_release);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        control_mode_ = ControlMode::LatchedOverride;
        clear_override_pending_ = false;
        has_normal_ctrl_frame_ = false;
        normal_ctrl_frame_ = {};
    }

    if (can_ && can_->is_open()) {
        std::lock_guard<hal::PiMutex> send_lk(send_mtx_);
        const auto account_tx = [this](bool ok) {
            std::lock_guard<hal::PiMutex> lk(mtx_);
            if (ok)
                ++ctrl_frame_count_;
            else
                ++ctrl_err_count_;
        };

        account_tx(
            can_->send(protocol::WalkMotorCanCodec::encode_group_speed(
                id_base_, 0.0f, 0.0f, 0.0f, 0.0f)));

        std::array<protocol::WalkMotorMode, 8> modes;
        modes.fill(protocol::WalkMotorMode::ENABLE);
        for (int i = 0; i < kWheelCount; ++i)
            modes[static_cast<std::size_t>(id_base_ - 1u + static_cast<uint8_t>(i))] =
                protocol::WalkMotorMode::DISABLE;
        account_tx(can_->send(protocol::WalkMotorCanCodec::encode_set_mode_batch(modes)));
    }

    running_.store(false);
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (can_ && can_->is_open())
        can_->close();
}

// ── 内部发帧 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::send_ctrl(const hal::CanFrame& frame) {
    // 通用发帧（无 override 检查）：供配置命令路径调用
    // （enable_all / set_mode_all / set_feedback_mode_all 等）。
    // 统一复用 send_mtx_，确保配置帧、紧急覆盖帧和关停帧不会交错发送。
    if (closing_.load(std::memory_order_acquire))
        return DeviceError::NOT_OPEN;
    if (!can_->is_open())
        return DeviceError::NOT_OPEN;
    std::lock_guard<hal::PiMutex> send_lk(send_mtx_);
    if (closing_.load(std::memory_order_acquire))
        return DeviceError::NOT_OPEN;
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
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;
    lt = clamp_rpm(lt);
    rt = clamp_rpm(rt);
    lb = clamp_rpm(lb);
    rb = clamp_rpm(rb);

    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = lt;
    diag_[1].target_value = rt;
    diag_[2].target_value = lb;
    diag_[3].target_value = rb;
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_speeds(const SpeedCmd& cmd) {
    return set_speeds(cmd.lt_rpm, cmd.rt_rpm, cmd.lb_rpm, cmd.rb_rpm);
}

DeviceError WalkMotorGroup::set_speed_uniform(float rpm) {
    // 物理安装：LT/RT 正转=前进，LB/RB 因安装方向相反，负转=前进
    // rpm > 0 = 车辆前进，rpm < 0 = 车辆后退
    return set_speeds(rpm, rpm, -rpm, -rpm);
}

DeviceError WalkMotorGroup::set_currents(float lt, float rt, float lb, float rb) {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;

    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ = protocol::WalkMotorCanCodec::encode_group_current(id_base_, lt, rt, lb, rb);
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = lt;
    diag_[1].target_value = rt;
    diag_[2].target_value = lb;
    diag_[3].target_value = rb;
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_open_loops(int16_t lt, int16_t rt, int16_t lb, int16_t rb) {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;

    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ = protocol::WalkMotorCanCodec::encode_group_open_loop(id_base_, lt, rt, lb, rb);
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = static_cast<float>(lt);
    diag_[1].target_value = static_cast<float>(rt);
    diag_[2].target_value = static_cast<float>(lb);
    diag_[3].target_value = static_cast<float>(rb);
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_positions(float lt_deg, float rt_deg, float lb_deg, float rb_deg) {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;
    auto cp = [](float v) { return clamp(v, 0.0f, 360.0f); };
    lt_deg = cp(lt_deg);
    rt_deg = cp(rt_deg);
    lb_deg = cp(lb_deg);
    rb_deg = cp(rb_deg);

    std::lock_guard<hal::PiMutex> lk(mtx_);
    normal_ctrl_frame_ =
        protocol::WalkMotorCanCodec::encode_group_position(id_base_, lt_deg, rt_deg, lb_deg, rb_deg);
    has_normal_ctrl_frame_ = true;
    diag_[0].target_value = lt_deg;
    diag_[1].target_value = rt_deg;
    diag_[2].target_value = lb_deg;
    diag_[3].target_value = rb_deg;
    return DeviceError::OK;
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

// ── 边缘紧急覆盖 ──────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::emergency_override(float reverse_rpm) {
    if (closing_.load(std::memory_order_acquire) || !can_->is_open())
        return DeviceError::NOT_OPEN;

    // 物理安装：LT/RT 正转=前进，LB/RB 因安装方向相反，负转=前进。
    // reverse_rpm > 0 = 车辆后退 → LT/RT=-rpm，LB/RB=+rpm（与前进方向相反）
    float lt, rt, lb, rb;
    if (reverse_rpm > 0.0f) {
        float rpm = clamp_rpm(reverse_rpm);
        lt = -rpm;
        rt = -rpm;  // 上轮后退
        lb = +rpm;
        rb = +rpm;  // 下轮后退（安装反向，正值=后退）
    } else {
        lt = rt = lb = rb = 0.0f;
    }
    const auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        control_mode_ = ControlMode::LatchedOverride;
        clear_override_pending_ = false;
    }

    {
        std::lock_guard<hal::PiMutex> lg(send_mtx_);
        const bool ok = can_->send(frame);
        std::lock_guard<hal::PiMutex> lk(mtx_);
        diag_[0].target_value = lt;
        diag_[1].target_value = rt;
        diag_[2].target_value = lb;
        diag_[3].target_value = rb;
        if (ok)
            ++ctrl_frame_count_;
        else
            ++ctrl_err_count_;
        if (!ok)
            return DeviceError::COMM_TIMEOUT;
    }

    spdlog::warn("[WalkMotorGroup] emergency_override: vehicle_reverse_rpm={:.1f}", reverse_rpm);
    return DeviceError::OK;
}

void WalkMotorGroup::clear_override() {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    clear_override_pending_ = true;
    spdlog::info("[WalkMotorGroup] clear_override requested");
}

bool WalkMotorGroup::is_override_active() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return control_mode_ == ControlMode::LatchedOverride;
}

uint32_t WalkMotorGroup::override_clear_generation() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return override_clear_generation_;
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
    return gs;
}

WalkMotorGroup::GroupDiagnostics WalkMotorGroup::get_group_diagnostics() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    GroupDiagnostics gd;
    for (int i = 0; i < kWheelCount; ++i)
        gd.wheel[i] = diag_[i];
    gd.ctrl_frame_count = ctrl_frame_count_;
    gd.ctrl_err_count = ctrl_err_count_;
    return gd;
}

// ── 周期心跳 ─────────────────────────────────────────────────────────────────

void WalkMotorGroup::update() {
    if (closing_.load(std::memory_order_acquire))
        return;
    if (!can_->is_open())
        return;
    auto now = std::chrono::steady_clock::now();

    // ── 1. 更新各轮 online 状态 ──
    // 日志推迟到锁外：spdlog::warn 可能触发堆分配和 spdlog 内部锁，
    // 在 SCHED_FIFO-80 线程内调用会引入不可预知抖动。
    uint8_t offline_mask = 0u;  // bit i = motor i 本轮刚掉线
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        for (int i = 0; i < kWheelCount; ++i) {
            bool ever = (last_fb_time_[i] != std::chrono::steady_clock::time_point{});
            if (ever) {
                bool online = (now - last_fb_time_[i]) < kOnlineTimeout;
                if (!online && diag_[i].online) {
                    diag_[i].online = false;
                    ++diag_[i].feedback_lost_count;
                    offline_mask |= static_cast<uint8_t>(1u << i);
                }
            }
        }
    }
    // 锁外打印：不在 RT 临界路径上
    for (int i = 0; i < kWheelCount; ++i) {
        if (offline_mask & (1u << i))
            spdlog::warn("[WalkMotorGroup] motor {} offline", id_base_ + i);
    }

    hal::CanFrame candidate{};
    bool should_send_normal = false;
    bool clear_applied = false;

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (control_mode_ == ControlMode::LatchedOverride && clear_override_pending_) {
            clear_override_pending_ = false;
            control_mode_ = ControlMode::Normal;
            has_normal_ctrl_frame_ = false;
            ++override_clear_generation_;
            clear_applied = true;
        }

        if (control_mode_ == ControlMode::Normal && has_normal_ctrl_frame_) {
            candidate = normal_ctrl_frame_;
            should_send_normal = true;
        }
    }

    if (clear_applied) {
        spdlog::info("[WalkMotorGroup] clear_override applied");
    }
    if (!should_send_normal)
        return;

    std::lock_guard<hal::PiMutex> send_lk(send_mtx_);
    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (control_mode_ != ControlMode::Normal || !has_normal_ctrl_frame_) {
            return;
        }
        candidate = normal_ctrl_frame_;
    }

    if (can_->send(candidate)) {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++ctrl_frame_count_;
    } else {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ++ctrl_err_count_;
    }
}

// ── 后台接收线程 ─────────────────────────────────────────────────────────────

void WalkMotorGroup::recv_loop() {
    // ── 线程自身完成 RT 提权 + CPU 绑定（SCHED_FIFO 82，CPU 5）──
    // 设计为略高于 walk_ctrl(80)：优先消化新帧，降低控制拍读取陈旧状态概率。
    {
        sched_param sp{};
        sp.sched_priority = 82;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[WalkMotorGroup] RT priority elevation failed: {}", strerror(rc));
        }
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(5, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[WalkMotorGroup] CPU 5 affinity set failed: {}", strerror(errno));
        }
        pthread_setname_np(pthread_self(), "group_recv");
    }
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
    }
}

}  // namespace robot::device
