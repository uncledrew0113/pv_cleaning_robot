#include <cmath>
#include <cstring>

#include "pv_cleaning_robot/protocol/imu_protocol.h"

namespace robot::protocol {

// 协议常量
static constexpr uint8_t TYPE_ACCEL = 0x51;
static constexpr uint8_t TYPE_GYRO = 0x52;
static constexpr uint8_t TYPE_EULER = 0x53;
static constexpr uint8_t TYPE_MAG = 0x54;
static constexpr uint8_t TYPE_QUAT = 0x59;

// 量化因子
static constexpr float ACCEL_SCALE = 16.0f * 9.8f / 32768.0f;  // ±16g 满量程 → m/s²
static constexpr float GYRO_SCALE = 2000.0f / 32768.0f;        // →deg/s
static constexpr float ANGLE_SCALE = 180.0f / 32768.0f;        // →deg
static constexpr float MAG_SCALE = 1.0f;                       // uT (raw)
static constexpr float QUAT_SCALE = 1.0f / 32768.0f;

static inline int16_t to_int16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
}

// ── 流式帧解析 ────────────────────────────────────────────────────────────────

void ImuProtocol::reset() {
    buf_pos_ = 0;
    synced_ = false;
    frame_ready_ = false;
    current_ = ImuData{};
}

void ImuProtocol::push_byte(uint8_t b) {
    frame_ready_ = false;

    if (!synced_) {
        if (b == FRAME_HEADER) {
            buf_[0] = b;
            buf_pos_ = 1;
            synced_ = true;
        }
        return;
    }

    buf_[buf_pos_++] = b;

    if (buf_pos_ < FRAME_LEN)
        return;

    // 校验和验证
    uint8_t sum = 0;
    for (size_t i = 0; i < FRAME_LEN - 1; ++i)
        sum += buf_[i];
    if (sum == buf_[FRAME_LEN - 1]) {
        frame_ready_ = parse_frame(buf_);
    }

    // 重置，下一帧从头开始
    synced_ = false;
    buf_pos_ = 0;
}

const ImuData& ImuProtocol::take_frame() {
    frame_ready_ = false;
    return current_;
}

bool ImuProtocol::parse_frame(const uint8_t* f) {
    uint8_t type = f[1];

    switch (type) {
        case TYPE_ACCEL:
            current_.accel[0] = to_int16(f[2], f[3]) * ACCEL_SCALE;
            current_.accel[1] = to_int16(f[4], f[5]) * ACCEL_SCALE;
            current_.accel[2] = to_int16(f[6], f[7]) * ACCEL_SCALE;
            current_.valid = true;
            return true;

        case TYPE_GYRO:
            current_.gyro[0] = to_int16(f[2], f[3]) * GYRO_SCALE * (M_PIf / 180.0f);
            current_.gyro[1] = to_int16(f[4], f[5]) * GYRO_SCALE * (M_PIf / 180.0f);
            current_.gyro[2] = to_int16(f[6], f[7]) * GYRO_SCALE * (M_PIf / 180.0f);
            current_.valid = true;
            return true;

        case TYPE_EULER:
            current_.roll_deg = to_int16(f[2], f[3]) * ANGLE_SCALE;
            current_.pitch_deg = to_int16(f[4], f[5]) * ANGLE_SCALE;
            current_.yaw_deg = to_int16(f[6], f[7]) * ANGLE_SCALE;
            current_.valid = true;
            return true;

        case TYPE_MAG:
            current_.mag[0] = to_int16(f[2], f[3]) * MAG_SCALE;
            current_.mag[1] = to_int16(f[4], f[5]) * MAG_SCALE;
            current_.mag[2] = to_int16(f[6], f[7]) * MAG_SCALE;
            return true;

        case TYPE_QUAT:
            current_.quat[0] = to_int16(f[2], f[3]) * QUAT_SCALE;  // w
            current_.quat[1] = to_int16(f[4], f[5]) * QUAT_SCALE;  // x
            current_.quat[2] = to_int16(f[6], f[7]) * QUAT_SCALE;  // y
            current_.quat[3] = to_int16(f[8], f[9]) * QUAT_SCALE;  // z (仅在11字节格式中)
            return true;

        default:
            return false;
    }
}

// ── 配置命令编码 ─────────────────────────────────────────────────────────────
// WIT Motion 寄存器写命令格式：0xFF 0xAA [reg] [data_lo] [data_hi]（固定 5 字节）

using Cmd = ImuProtocol::Cmd;

static Cmd wit_write_reg(uint8_t reg, uint16_t val) {
    return {
        0xFF, 0xAA, reg, static_cast<uint8_t>(val & 0xFF), static_cast<uint8_t>((val >> 8) & 0xFF)};
}

Cmd ImuProtocol::encode_set_rate(uint8_t rate_code) {
    return wit_write_reg(0x03, rate_code);
}

Cmd ImuProtocol::encode_set_baudrate(uint8_t baud_code) {
    return wit_write_reg(0x04, baud_code);
}

Cmd ImuProtocol::encode_save_config() {
    return wit_write_reg(0x00, 0x0000);
}

Cmd ImuProtocol::encode_reset() {
    return {0xFF, 0xAA, 0x00, 0xFF, 0x00};
}

Cmd ImuProtocol::encode_calibrate_gyro() {
    return wit_write_reg(0x01, 0x0001);
}

Cmd ImuProtocol::encode_unlock() {
    // 写寄存器前的解锁序列，有效期 10 秒
    return {0xFF, 0xAA, 0x69, 0x88, 0xB5};
}

}  // namespace robot::protocol
