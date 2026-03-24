#include <cmath>
#include <cstring>

#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"


namespace robot::protocol {

// ── 量化比例常量 ────────────────────────────────────────────────────────────────
/// int16 范围 -21000~21000 对应 -210~210 RPM
static constexpr float kSpeedScale = 100.0f;
/// int16 范围 -32767~32767 对应 -33~33 A
static constexpr float kCurrentScale = 32767.0f / 33.0f;
/// uint16 范围 0~32767 对应 0~360°
static constexpr float kPosScale = 32767.0f / 360.0f;

// ── 构造函数 ───────────────────────────────────────────────────────────────────
WalkMotorCanCodec::WalkMotorCanCodec(uint8_t motor_id) : motor_id_(motor_id) {
    // motor_id 1~4 使用 0x32；motor_id 5~8 使用 0x33
    ctrl_id_ = (motor_id_ <= 4u) ? kWalkMotorCtrlIdGroup1 : kWalkMotorCtrlIdGroup2;
    // 计算本电机在控制帧 data[] 中的起始字节偏移
    // motor 1→0, motor 2→2, motor 3→4, motor 4→6
    // motor 5→0, motor 6→2, motor 7→4, motor 8→6
    slot_ = static_cast<uint8_t>(((motor_id_ - 1u) % 4u) * 2u);
}

// ── 内部辅助 ────────────────────────────────────────────────────────────────────
hal::CanFrame WalkMotorCanCodec::make_ctrl_frame(int16_t value) const {
    hal::CanFrame frame;
    frame.id = ctrl_id_;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    // 高字节在前（big-endian）
    frame.data[slot_] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFFu);
    frame.data[slot_ + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFFu);
    return frame;
}

// ── 控制帧编码 ───────────────────────────────────────────────────────────────────
hal::CanFrame WalkMotorCanCodec::encode_speed(float rpm) const {
    return make_ctrl_frame(static_cast<int16_t>(rpm * kSpeedScale));
}

hal::CanFrame WalkMotorCanCodec::encode_current(float amps) const {
    return make_ctrl_frame(static_cast<int16_t>(amps * kCurrentScale));
}

hal::CanFrame WalkMotorCanCodec::encode_position(float deg) const {
    // 位置环只支持 0~360°，取绝对值后量化为 uint16
    auto raw = static_cast<uint16_t>(std::fabs(deg) * kPosScale);
    return make_ctrl_frame(static_cast<int16_t>(raw));
}

hal::CanFrame WalkMotorCanCodec::encode_open_loop(int16_t raw_value) const {
    return make_ctrl_frame(raw_value);
}

hal::CanFrame WalkMotorCanCodec::encode_set_mode(WalkMotorMode mode) const {
    // 0x105 是广播帧，DATA[0~7] 同时设定所有8台电机的运行模式。
    // 若其余 slot 保持 0x00，将导致所有其他电机被意外切换到 OPEN_LOOP(0x00)。
    // 安全做法：非本电机的 slot 填入 ENABLE(0x0A)，保持其他电机工作状态不变。
    std::array<WalkMotorMode, 8> modes;
    modes.fill(WalkMotorMode::ENABLE);
    modes[motor_id_ - 1u] = mode;
    return encode_set_mode_batch(modes);
}

hal::CanFrame WalkMotorCanCodec::encode_set_feedback(uint8_t period_ms) const {
    // 0x106 是广播帧，DATA[0~7] 同时设定所有8台电机的反馈方式。
    // 若其余 slot 为 0x00，含义为"推送周期 0ms"（非法值），会导致其他电机反馈异常。
    // 安全做法：非本电机的 slot 填入协议默认状态 0x0A（10ms 主动上报）。
    constexpr uint8_t kDefaultFb = 0x0Au;  // 10ms 主动上报（协议出厂默认值）
    uint8_t fb_byte;
    if (period_ms == 0u) {
        fb_byte = 0x80u;  // 查询方式（Bit7=1）
    } else {
        uint8_t period = (period_ms > 127u) ? 127u : period_ms;
        fb_byte = period & 0x7Fu;  // 主动上报方式
    }
    std::array<uint8_t, 8> fb_modes;
    fb_modes.fill(kDefaultFb);
    fb_modes[motor_id_ - 1u] = fb_byte;
    return encode_set_feedback_batch(fb_modes);
}

hal::CanFrame WalkMotorCanCodec::encode_query(WalkMotorQueryTarget t1,
                                              WalkMotorQueryTarget t2,
                                              WalkMotorQueryTarget t3,
                                              uint8_t custom) const {
    hal::CanFrame frame;
    frame.id = kWalkMotorQueryId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = motor_id_;
    frame.data[1] = static_cast<uint8_t>(t1);
    frame.data[2] = static_cast<uint8_t>(t2);
    frame.data[3] = static_cast<uint8_t>(t3);
    frame.data[4] = custom;
    // DATA[5~7] 保留，已由 memset 置 0
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_query_firmware() {
    hal::CanFrame frame;
    frame.id = kWalkMotorFwQueryId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    return frame;
}

// ── 设置电机 ID / 多功能配置帧 ─────────────────────────────────────────────

hal::CanFrame WalkMotorCanCodec::encode_set_node_id() const {
    hal::CanFrame frame;
    frame.id = kWalkMotorSetNodeId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = motor_id_;
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_set_comm_timeout(uint16_t timeout_ms) const {
    hal::CanFrame frame;
    frame.id = kWalkMotorCommTimeoutId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = motor_id_;
    frame.data[1] = static_cast<uint8_t>(CommTimeoutOp::WRITE);  // 设置
    frame.data[2] = 0x01u;                                       // 写操作
    frame.data[3] = static_cast<uint8_t>(timeout_ms >> 8);
    frame.data[4] = static_cast<uint8_t>(timeout_ms & 0xFFu);
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_reset_comm_timeout() const {
    hal::CanFrame frame;
    frame.id = kWalkMotorCommTimeoutId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = motor_id_;
    frame.data[1] = static_cast<uint8_t>(CommTimeoutOp::RESET);  // 复位
    frame.data[2] = 0x01u;                                       // 写操作
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_read_comm_timeout() const {
    hal::CanFrame frame;
    frame.id = kWalkMotorCommTimeoutId;
    frame.len = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = motor_id_;
    frame.data[1] = static_cast<uint8_t>(CommTimeoutOp::WRITE);  // DATA[1]=设置类型
    frame.data[2] = 0x00u;                                       // 读操作
    return frame;
}

// ── 4电机组同步批量编码 ────────────────────────────────────────────────────────

/// 将4个 int16 值按 big-endian 顺序打包进8字节 data[]
static void pack_group(uint8_t* data, int16_t v0, int16_t v1, int16_t v2, int16_t v3) {
    auto put = [&](int slot, int16_t v) {
        data[slot] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFFu);
        data[slot + 1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFu);
    };
    put(0, v0);
    put(2, v1);
    put(4, v2);
    put(6, v3);
}

static uint32_t group_ctrl_id(uint8_t id_base) {
    return (id_base <= 4u) ? kWalkMotorCtrlIdGroup1 : kWalkMotorCtrlIdGroup2;
}

hal::CanFrame WalkMotorCanCodec::encode_group_speed(uint8_t id_base,
                                                    float rpm0,
                                                    float rpm1,
                                                    float rpm2,
                                                    float rpm3) {
    hal::CanFrame frame;
    frame.id = group_ctrl_id(id_base);
    frame.len = 8;
    pack_group(frame.data,
               static_cast<int16_t>(rpm0 * kSpeedScale),
               static_cast<int16_t>(rpm1 * kSpeedScale),
               static_cast<int16_t>(rpm2 * kSpeedScale),
               static_cast<int16_t>(rpm3 * kSpeedScale));
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_group_current(uint8_t id_base,
                                                      float a0,
                                                      float a1,
                                                      float a2,
                                                      float a3) {
    hal::CanFrame frame;
    frame.id = group_ctrl_id(id_base);
    frame.len = 8;
    pack_group(frame.data,
               static_cast<int16_t>(a0 * kCurrentScale),
               static_cast<int16_t>(a1 * kCurrentScale),
               static_cast<int16_t>(a2 * kCurrentScale),
               static_cast<int16_t>(a3 * kCurrentScale));
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_group_open_loop(uint8_t id_base,
                                                        int16_t v0,
                                                        int16_t v1,
                                                        int16_t v2,
                                                        int16_t v3) {
    hal::CanFrame frame;
    frame.id = group_ctrl_id(id_base);
    frame.len = 8;
    pack_group(frame.data, v0, v1, v2, v3);
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_group_position(uint8_t id_base,
                                                       float deg0,
                                                       float deg1,
                                                       float deg2,
                                                       float deg3) {
    hal::CanFrame frame;
    frame.id = group_ctrl_id(id_base);
    frame.len = 8;
    auto to_raw = [](float deg) -> int16_t {
        return static_cast<int16_t>(std::fabs(deg) * kPosScale);
    };
    pack_group(frame.data, to_raw(deg0), to_raw(deg1), to_raw(deg2), to_raw(deg3));
    return frame;
}

// ── 8电机批量编码 ──────────────────────────────────────────────────────────────────

hal::CanFrame WalkMotorCanCodec::encode_set_mode_batch(const std::array<WalkMotorMode, 8>& modes) {
    hal::CanFrame frame;
    frame.id = kWalkMotorSetModeId;
    frame.len = 8;
    for (int i = 0; i < 8; ++i)
        frame.data[i] = static_cast<uint8_t>(modes[static_cast<std::size_t>(i)]);
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_set_feedback_batch(const std::array<uint8_t, 8>& fb_modes) {
    hal::CanFrame frame;
    frame.id = kWalkMotorSetFeedbackId;
    frame.len = 8;
    for (int i = 0; i < 8; ++i)
        frame.data[i] = fb_modes[static_cast<std::size_t>(i)];
    return frame;
}

hal::CanFrame WalkMotorCanCodec::encode_set_termination_batch(const std::array<bool, 8>& enables) {
    hal::CanFrame frame;
    frame.id = kWalkMotorTermResId;
    frame.len = 8;
    for (int i = 0; i < 8; ++i)
        frame.data[i] = enables[static_cast<std::size_t>(i)] ? 0x01u : 0x00u;
    return frame;
}

// ── 反馈帧解码 ──────────────────────────────────────────────────────────────────
std::optional<WalkMotorStatus> WalkMotorCanCodec::decode_status(const hal::CanFrame& frame) const {
    if (frame.id != status_can_id() || frame.len < 8)
        return std::nullopt;

    WalkMotorStatus s{};

    // DATA[0-1]: 速度 int16 big-endian
    auto speed_raw =
        static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]);
    s.speed_rpm = speed_raw / kSpeedScale;

    // DATA[2-3]: 转矩电流 int16 big-endian
    auto torque_raw =
        static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
    s.torque_a = torque_raw / kCurrentScale;

    // DATA[4-5]: 位置 uint16 big-endian
    auto pos_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
    s.position_deg = pos_raw / kPosScale;

    // DATA[6]: 故障码
    s.fault = static_cast<WalkMotorFault>(frame.data[6]);

    // DATA[7]: 当前模式
    s.mode = static_cast<WalkMotorMode>(frame.data[7]);

    return s;
}

std::optional<WalkMotorMode> WalkMotorCanCodec::decode_mode_resp(const hal::CanFrame& frame) const {
    if (frame.id != kWalkMotorModeRespBase + motor_id_ || frame.len < 1)
        return std::nullopt;
    return static_cast<WalkMotorMode>(frame.data[0]);
}
std::optional<uint8_t> WalkMotorCanCodec::decode_fb_mode_resp(const hal::CanFrame& frame) const {
    if (frame.id != kWalkMotorFbModeBase + motor_id_ || frame.len < 1)
        return std::nullopt;
    return frame.data[0];
}

std::optional<bool> WalkMotorCanCodec::decode_termination_resp(const hal::CanFrame& frame) const {
    if (frame.id != kWalkMotorTermRespBase + motor_id_ || frame.len < 2)
        return std::nullopt;
    if (frame.data[0] != motor_id_)
        return std::nullopt;
    return frame.data[1] != 0u;
}

std::optional<WalkMotorCommTimeoutResp> WalkMotorCanCodec::decode_comm_timeout_resp(
    const hal::CanFrame& frame) const {
    if (frame.id != kWalkMotorCommTimeoutRespBase + motor_id_ || frame.len < 5)
        return std::nullopt;
    if (frame.data[0] != motor_id_)
        return std::nullopt;
    WalkMotorCommTimeoutResp resp;
    resp.motor_id = frame.data[0];
    resp.op = static_cast<CommTimeoutOp>(frame.data[1]);
    resp.timeout_ms =
        static_cast<uint16_t>((static_cast<uint16_t>(frame.data[3]) << 8) | frame.data[4]);
    return resp;
}

std::optional<WalkMotorFirmwareInfo> WalkMotorCanCodec::decode_firmware(
    const hal::CanFrame& frame) const {
    if (frame.id != kWalkMotorFwRespBase + motor_id_ || frame.len < 8)
        return std::nullopt;

    WalkMotorFirmwareInfo info{};
    info.motor_id = frame.data[0];
    info.sw_major = frame.data[1];
    info.sw_minor = frame.data[2];
    info.hw_major = frame.data[3];
    info.hw_minor = frame.data[4];
    info.year = frame.data[5];
    info.month = frame.data[6];
    info.day = frame.data[7];
    return info;
}

}  // namespace robot::protocol
