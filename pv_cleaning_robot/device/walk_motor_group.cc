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
    last_update_time_ = {};  // close/open 重启时 dt_s 使用默认 20ms
}

// ── 内部发帧 ─────────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::send_ctrl(const hal::CanFrame& frame) {
    // 通用发帧（无 override 检查）：供配置命令路径调用
    // （enable_all / set_mode_all / set_feedback_mode_all 等）。
    // PID 控制帧的 override 保护由 update() step4 内联的 send_mtx_ 负责。
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

// ── 命令队列内部实现 ──────────────────────────────────────────────────────────

void WalkMotorGroup::enqueue_cmd(const Cmd& cmd) {
    std::lock_guard<hal::PiMutex> lk(cmd_mtx_);
    int next_head = (cmd_head_ + 1) % kCmdQueueSize;
    if (next_head == cmd_tail_) {
        // 队列满：丢弃最旧命令，不阻塞调用方线程
        spdlog::warn("[WalkMotorGroup] cmd_queue full, oldest command dropped");
        cmd_tail_ = (cmd_tail_ + 1) % kCmdQueueSize;
    }
    cmd_buf_[static_cast<size_t>(cmd_head_)] = cmd;
    cmd_head_ = next_head;
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

    Cmd cmd;
    cmd.type = Cmd::Type::SET_CTRL_FRAME;
    cmd.frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
    cmd.base_lt_rpm = lt;
    cmd.base_rt_rpm = rt;
    cmd.target_rpms = {lt, rt, lb, rb};
    enqueue_cmd(cmd);
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
    Cmd cmd;
    cmd.type = Cmd::Type::SET_CTRL_FRAME;
    cmd.frame = protocol::WalkMotorCanCodec::encode_group_current(id_base_, lt, rt, lb, rb);
    cmd.base_lt_rpm = 0.0f;
    cmd.base_rt_rpm = 0.0f;
    cmd.target_rpms = {lt, rt, lb, rb};
    enqueue_cmd(cmd);
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_open_loops(int16_t lt, int16_t rt, int16_t lb, int16_t rb) {
    Cmd cmd;
    cmd.type = Cmd::Type::SET_CTRL_FRAME;
    cmd.frame = protocol::WalkMotorCanCodec::encode_group_open_loop(id_base_, lt, rt, lb, rb);
    cmd.base_lt_rpm = 0.0f;
    cmd.base_rt_rpm = 0.0f;
    cmd.target_rpms = {static_cast<float>(lt),
                       static_cast<float>(rt),
                       static_cast<float>(lb),
                       static_cast<float>(rb)};
    enqueue_cmd(cmd);
    return DeviceError::OK;
}

DeviceError WalkMotorGroup::set_positions(float lt_deg, float rt_deg, float lb_deg, float rb_deg) {
    auto cp = [](float v) { return clamp(v, 0.0f, 360.0f); };
    lt_deg = cp(lt_deg);
    rt_deg = cp(rt_deg);
    lb_deg = cp(lb_deg);
    rb_deg = cp(rb_deg);
    Cmd cmd;
    cmd.type = Cmd::Type::SET_CTRL_FRAME;
    cmd.frame = protocol::WalkMotorCanCodec::encode_group_position(
        id_base_, lt_deg, rt_deg, lb_deg, rb_deg);
    cmd.base_lt_rpm = 0.0f;
    cmd.base_rt_rpm = 0.0f;
    cmd.target_rpms = {lt_deg, rt_deg, lb_deg, rb_deg};
    enqueue_cmd(cmd);
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

// ── 航向 PID 控制 ─────────────────────────────────────────────────────────────

void WalkMotorGroup::set_heading_pid_params(const HeadingPidParams& p) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    pid_ctrl_.set_params(p);
}

void WalkMotorGroup::enable_heading_control(bool en) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    pid_ctrl_.enable(en);
}

void WalkMotorGroup::set_target_heading(float yaw_deg) {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    pid_ctrl_.set_target(yaw_deg);
}

// ── 边缘紧急覆盖 ──────────────────────────────────────────────────────────────

DeviceError WalkMotorGroup::emergency_override(float reverse_rpm) {
    // 防线一：立即置位（seq_cst），在持锁之前操作。
    // 即使后续等待 send_mtx_，update() 的 step3 无锁检查也能立即感知并提前退出，
    // 不必等到 send_mtx_ 释放后才被拦截。
    override_active_.store(true, std::memory_order_seq_cst);
    // 取消任何待消费的 clear_override 标志：若 update() 尚未消费该标志，
    // 让它悄悄把本次 override 清掉会导致安全停止失效。
    pending_clear_override_.store(false, std::memory_order_seq_cst);

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
    auto frame = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);

    // 防线二：持锁后直接发帧，不经过 send_ctrl() 的 override 检查门。
    // 与 send_ctrl() 的 lock(send_mtx_) 互斥，保证 stop 帧严格在任何
    // 并发 PID 帧之后写入内核 TX ring（总线上的最后一条控制语义帧）。
    DeviceError ret;
    {
        std::lock_guard<hal::PiMutex> lg(send_mtx_);
        ret = can_->send(frame) ? DeviceError::OK : DeviceError::COMM_TIMEOUT;
        std::lock_guard<hal::PiMutex> lk(mtx_);
        if (ret == DeviceError::OK)
            ++ctrl_frame_count_;
        else
            ++ctrl_err_count_;
    }

    spdlog::warn("[WalkMotorGroup] emergency_override: vehicle_reverse_rpm={:.1f}", reverse_rpm);
    return ret;
}

void WalkMotorGroup::clear_override() {
    // 原子标志绕过命令队列：clear_override() 直写 pending_clear_override_，
    // update() step2-pre 优先 exchange——任意满载下均不业失
    pending_clear_override_.store(true, std::memory_order_seq_cst);
    spdlog::info("[WalkMotorGroup] clear_override requested");
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

void WalkMotorGroup::update(float yaw_deg) {
    if (!can_->is_open())
        return;

    auto now = std::chrono::steady_clock::now();

    // 计算距上次 update() 的实际时间（秒），用于 PID 微积分计算
    // 使用实测时间而非硬编码常量，适应线程执行器周期变更
    float dt_s = 0.020f;  // 合理默认値（20ms），首次调用或异常大値时使用
    if (last_update_time_ != std::chrono::steady_clock::time_point{}) {
        float measured = std::chrono::duration<float>(now - last_update_time_).count();
        // 限幅至 10ms~500ms，防止系统暂停/调试断点导致积分爆炸
        if (measured >= 0.010f && measured <= 0.500f)
            dt_s = measured;
    }
    last_update_time_ = now;

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

    // ── 2-pre. CLEAR_OVERRIDE 原子检查（优先于命令队列，任意满载下不可丢失）──────────
    if (pending_clear_override_.exchange(false, std::memory_order_acquire)) {
        {
            std::lock_guard<hal::PiMutex> lk(mtx_);
            has_ctrl_frame_ = false;
            pid_ctrl_.reset();
        }
        // release：保证上方 mtx_ 内写对其他核可见后再清 override
        override_active_.store(false, std::memory_order_release);
        spdlog::info("[WalkMotorGroup] clear_override applied");
    }

    // ── 2. 排干命令队列（update() 是唯一消费者，walk_ctrl 线程）────────────────────
    // SET_CTRL_FRAME: 更新控制帧 + PID 基础值，不触碰 override_active_
    while (true) {
        Cmd c;
        {
            std::lock_guard<hal::PiMutex> lk_cmd(cmd_mtx_);
            if (cmd_tail_ == cmd_head_)
                break;
            c = cmd_buf_[static_cast<size_t>(cmd_tail_)];
            cmd_tail_ = (cmd_tail_ + 1) % kCmdQueueSize;
        }  // 释放 cmd_mtx_，避免与 mtx_ 嵌套持锁

        std::lock_guard<hal::PiMutex> lk(mtx_);
        last_ctrl_frame_ = c.frame;
        has_ctrl_frame_ = true;
        base_lt_rpm_ = c.base_lt_rpm;
        base_rt_rpm_ = c.base_rt_rpm;
        for (int i = 0; i < kWheelCount; ++i)
            diag_[i].target_value = c.target_rpms[static_cast<size_t>(i)];
    }

    // ── 3. 若在 override 状态，跳过心跳重发 ──
    if (override_active_.load(std::memory_order_acquire))
        return;

    // ── 4. 计算 PID 差速（如果使能） ──
    hal::CanFrame ctrl;
    bool has;

    {
        std::lock_guard<hal::PiMutex> lk(mtx_);
        ctrl = last_ctrl_frame_;
        has = has_ctrl_frame_;

        if (has && pid_ctrl_.is_enabled()) {
            float correction = pid_ctrl_.compute(yaw_deg, dt_s);
            // correction > 0 → 偏右（yaw < target），加大左侧速度
            float lt = clamp_rpm(base_lt_rpm_ + correction);
            float rt = clamp_rpm(base_rt_rpm_ - correction);
            // 物理安装：LB/RB 安装方向与 LT/RT 相反，需取反才能同向运动
            float lb = -lt;
            float rb = -rt;
            ctrl = protocol::WalkMotorCanCodec::encode_group_speed(id_base_, lt, rt, lb, rb);
        }
    }

    if (has) {
        // 防线二（仅 PID 路径）：持 send_mtx_ 后双重检查 override。
        // 处理 update 通过 step3 无锁检查后被 emergency 抢占的残余窗口：
        //   emergency: store(true) → lock(send_mtx_) → send(stop) → unlock [release]
        //   update:                                              lock(send_mtx_) [acquire]
        //   happens-before 保证此处 load 必然读到 true → 丢弃 PID 帧。
        std::lock_guard<hal::PiMutex> lg(send_mtx_);
        if (!override_active_.load(std::memory_order_acquire)) {
            if (can_->send(ctrl)) {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                ++ctrl_frame_count_;
            } else {
                std::lock_guard<hal::PiMutex> lk(mtx_);
                ++ctrl_err_count_;
            }
        }
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
