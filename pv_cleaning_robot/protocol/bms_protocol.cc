#include "pv_cleaning_robot/protocol/bms_protocol.h"

#include <algorithm>

namespace robot::protocol {

// == 协议帧常量 ================================================================
static constexpr uint8_t kStart      = 0xDD;
static constexpr uint8_t kStop       = 0x77;
static constexpr uint8_t kStatusRead = 0xA5;
static constexpr uint8_t kStatusWrite= 0x5A;

static constexpr uint8_t kCmdBasicInfo = 0x03;
static constexpr uint8_t kCmdCellVolt  = 0x04;
static constexpr uint8_t kCmdVersion   = 0x05;
static constexpr uint8_t kCmdMosCtrl   = 0xE1;

// == 内部辅助 ==================================================================

/// 从大端字节对读取 uint16
static inline uint16_t be16(const uint8_t* p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) << 8u | p[1]);
}

// == 校验和 ====================================================================

uint16_t BmsProtocol::calc_checksum(uint8_t cmd, uint8_t len, const uint8_t* data)
{
    // sum = cmd + len + data[0] + ... + data[len-1]
    // checksum = (~sum + 1) & 0xFFFF  (二进制补码)
    uint16_t sum = static_cast<uint16_t>(cmd) + len;
    for (uint8_t i = 0; i < len; ++i)
        sum = static_cast<uint16_t>(sum + data[i]);
    return static_cast<uint16_t>(~sum + 1u);
}

// == 编码：主机 -> BMS =========================================================

/// 构造读请求帧（7 字节）：DD A5 [cmd] 00 [chk_H] [chk_L] 77
/// 读请求数据段为空，校验和只覆盖 cmd + len(=0)
static std::array<uint8_t, 7> make_read_frame(uint8_t cmd)
{
    uint16_t sum = static_cast<uint16_t>(cmd);   // len = 0, 无 data
    uint16_t chk = static_cast<uint16_t>(~sum + 1u);
    return {kStart, kStatusRead, cmd, 0x00u,
            static_cast<uint8_t>(chk >> 8),
            static_cast<uint8_t>(chk & 0xFFu),
            kStop};
}

std::array<uint8_t, 7> BmsProtocol::encode_read_basic_info()
{
    return make_read_frame(kCmdBasicInfo);
}

std::array<uint8_t, 7> BmsProtocol::encode_read_cell_voltages()
{
    return make_read_frame(kCmdCellVolt);
}

std::array<uint8_t, 7> BmsProtocol::encode_read_version()
{
    return make_read_frame(kCmdVersion);
}

std::array<uint8_t, 9> BmsProtocol::encode_mos_control(uint8_t mos_state)
{
    // 帧：DD 5A E1 02 00 [mos_state] [chk_H] [chk_L] 77
    // 数据段：{0x00, mos_state}，长度 = 2
    const uint8_t data[2] = {0x00u, mos_state};
    uint16_t chk = calc_checksum(kCmdMosCtrl, 2u, data);
    return {kStart, kStatusWrite, kCmdMosCtrl, 0x02u,
            0x00u, mos_state,
            static_cast<uint8_t>(chk >> 8),
            static_cast<uint8_t>(chk & 0xFFu),
            kStop};
}

// == 解码：BMS -> 主机 =========================================================

std::optional<BmsBasicInfo>
BmsProtocol::decode_basic_info(const uint8_t* data, uint8_t len)
{
    // 固定字段 23 字节（NTC 数量字段之后为可变长 NTC 温度数组）
    static constexpr uint8_t kFixedLen = 23u;
    if (len < kFixedLen) return std::nullopt;

    BmsBasicInfo info{};

    // 总电压：原始单位 10 mV → mV
    info.total_voltage_mv = static_cast<uint32_t>(be16(data + 0)) * 10u;

    // 电流：原始单位 10 mA；最高位=1 表示放电（负值）
    //   放电：magnitude = 65536 - raw；充电：magnitude = raw
    uint16_t raw_curr = be16(data + 2);
    if (raw_curr & 0x8000u) {
        info.current_ma = -static_cast<int32_t>(65536u - raw_curr) * 10;
    } else {
        info.current_ma =  static_cast<int32_t>(raw_curr) * 10;
    }

    // 剩余容量 / 标称容量：原始单位 10 mAh → mAh
    info.residual_capacity_mah = static_cast<uint32_t>(be16(data + 4)) * 10u;
    info.nominal_capacity_mah  = static_cast<uint32_t>(be16(data + 6)) * 10u;

    // 循环次数
    info.cycle_count = be16(data + 8);

    // 生产日期：bits[15:9]=年偏移，bits[8:5]=月，bits[4:0]=日
    uint16_t raw_date = be16(data + 10);
    info.production_date.day   = static_cast<uint8_t>(raw_date & 0x1Fu);
    info.production_date.month = static_cast<uint8_t>((raw_date >> 5) & 0x0Fu);
    info.production_date.year  = static_cast<uint16_t>(2000u + (raw_date >> 9));

    // 均衡状态
    info.balance_status      = be16(data + 12);
    info.balance_status_high = be16(data + 14);

    // 保护标志位
    info.protection_status = be16(data + 16);

    // 单字节字段
    info.version     = data[18];
    info.rsoc_pct    = data[19];
    info.fet_control = data[20];
    info.cell_count  = data[21];
    info.ntc_count   = data[22];

    // NTC 温度（可变长，每个探头 2 字节）
    uint8_t ntc = std::min(info.ntc_count, kBmsMaxNtc);
    if (len < static_cast<uint8_t>(kFixedLen + ntc * 2u))
        return std::nullopt;

    // NTC 原始单位 0.1K；℃ = (raw - 2731) / 10.0
    for (uint8_t i = 0; i < ntc; ++i) {
        uint16_t raw_t = be16(data + kFixedLen + i * 2u);
        info.ntc_temp_c[i] = (static_cast<float>(raw_t) - 2731.0f) / 10.0f;
    }

    return info;
}

std::optional<BmsCellVoltages>
BmsProtocol::decode_cell_voltages(const uint8_t* data, uint8_t len)
{
    // 每节 2 字节，长度必须为非零偶数
    if (len == 0u || (len & 1u) != 0u) return std::nullopt;

    BmsCellVoltages cells{};
    cells.count = std::min(static_cast<uint8_t>(len / 2u), kBmsMaxCells);
    for (uint8_t i = 0; i < cells.count; ++i)
        cells.mv[i] = be16(data + i * 2u);

    return cells;
}

std::optional<std::string>
BmsProtocol::decode_version(const uint8_t* data, uint8_t len)
{
    if (len == 0u) return std::nullopt;
    return std::string(reinterpret_cast<const char*>(data), len);
}

// == 流式帧解析器 ==============================================================

void BmsProtocol::reset()
{
    state_       = ParseState::WAIT_START;
    cmd_ = status_ = len_ = data_pos_ = chk_h_ = 0;
    frame_ready_ = false;
}

void BmsProtocol::push_byte(uint8_t b)
{
    frame_ready_ = false;

    switch (state_) {

    case ParseState::WAIT_START:
        if (b == kStart) state_ = ParseState::WAIT_CMD;
        break;

    case ParseState::WAIT_CMD:
        cmd_   = b;
        state_ = ParseState::WAIT_STATUS;
        break;

    case ParseState::WAIT_STATUS:
        status_ = b;
        state_  = ParseState::WAIT_LEN;
        break;

    case ParseState::WAIT_LEN:
        len_      = b;
        data_pos_ = 0;
        if      (len_ == 0u)         { state_ = ParseState::CHKSUM_H; }
        else if (len_ > kMaxDataLen) { state_ = ParseState::WAIT_START; }  // 超长丢帧
        else                         { state_ = ParseState::DATA; }
        break;

    case ParseState::DATA:
        data_buf_[data_pos_++] = b;
        if (data_pos_ >= len_) state_ = ParseState::CHKSUM_H;
        break;

    case ParseState::CHKSUM_H:
        chk_h_ = b;
        state_ = ParseState::CHKSUM_L;
        break;

    case ParseState::CHKSUM_L: {
        // 响应帧校验覆盖：STATUS + LEN + DATA（CMD 是帧头，不入校验）
        // 请求帧校验覆盖：CMD   + LEN + DATA（0xA5/0x5A 是帧头，不入校验）
        // 因此解析响应帧时，第一个校验字节是 status_（0x00 或 0x80），而非 cmd_。
        uint16_t expected = calc_checksum(status_, len_, data_buf_);
        uint16_t received = static_cast<uint16_t>(
            static_cast<uint16_t>(chk_h_) << 8u | b);
        state_ = (expected == received) ? ParseState::WAIT_STOP
                                        : ParseState::WAIT_START;
        break;
    }

    case ParseState::WAIT_STOP:
        if (b == kStop) frame_ready_ = true;
        // 无论帧尾是否匹配，均重置等待下一帧
        state_ = ParseState::WAIT_START;
        break;
    }
}

}  // namespace robot::protocol
