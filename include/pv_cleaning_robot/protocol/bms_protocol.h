#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace robot::protocol {

// == 容量限制 =================================================================
static constexpr uint8_t kBmsMaxCells = 32;  ///< 最大串联节数
static constexpr uint8_t kBmsMaxNtc   = 8;   ///< 最大 NTC 温度探头数量

// == 保护状态位掩码 (BmsBasicInfo::protection_status) ========================
static constexpr uint16_t kBmsProtCellOverVolt  = (1u << 0);  ///< 单体过压
static constexpr uint16_t kBmsProtCellUnderVolt = (1u << 1);  ///< 单体欠压
static constexpr uint16_t kBmsProtBatOverVolt   = (1u << 2);  ///< 总压过压
static constexpr uint16_t kBmsProtBatUnderVolt  = (1u << 3);  ///< 总压欠压
static constexpr uint16_t kBmsProtChgOverTemp   = (1u << 4);  ///< 充电高温
static constexpr uint16_t kBmsProtChgLowTemp    = (1u << 5);  ///< 充电低温
static constexpr uint16_t kBmsProtDsgOverTemp   = (1u << 6);  ///< 放电高温
static constexpr uint16_t kBmsProtDsgLowTemp    = (1u << 7);  ///< 放电低温
static constexpr uint16_t kBmsProtChgOverCurr   = (1u << 8);  ///< 充电过流
static constexpr uint16_t kBmsProtDsgOverCurr   = (1u << 9);  ///< 放电过流
static constexpr uint16_t kBmsProtShortCircuit  = (1u << 10); ///< 短路保护
static constexpr uint16_t kBmsProtForeEndIC     = (1u << 11); ///< 前端IC错误
static constexpr uint16_t kBmsProtMosSoftLock   = (1u << 12); ///< MOS 软件锁定

// == MOS 控制状态值 ===========================================================
static constexpr uint8_t kBmosMosReleaseAll     = 0x00; ///< 解除软件关断
static constexpr uint8_t kBmosMosChargeClose    = 0x01; ///< 软件关闭充电 MOS
static constexpr uint8_t kBmosMosDischargeClose = 0x02; ///< 软件关闭放电 MOS
static constexpr uint8_t kBmosMosBothClose      = 0x03; ///< 同时关闭两路 MOS

// == 数据结构 =================================================================

/// 生产日期（由 0x03 响应 Production Date 字段解码）
struct BmsDate {
    uint16_t year;   ///< 年（如 2023）
    uint8_t  month;  ///< 月（1–12）
    uint8_t  day;    ///< 日（1–31）
};

/// 0x03 响应：基本信息与状态
struct BmsBasicInfo {
    uint32_t total_voltage_mv;       ///< 总电压（mV）
    int32_t  current_ma;             ///< 电流（mA），正=充电，负=放电
    uint32_t residual_capacity_mah;  ///< 剩余容量（mAh）
    uint32_t nominal_capacity_mah;   ///< 标称容量（mAh）
    uint16_t cycle_count;            ///< 循环次数
    BmsDate  production_date;        ///< 生产日期
    uint16_t balance_status;         ///< 均衡状态（bit0=电芯1 … bit15=电芯16）
    uint16_t balance_status_high;    ///< 均衡状态（bit0=电芯17 … bit15=电芯32）
    uint16_t protection_status;      ///< 保护标志位（见 kBmsProt* 常量）
    uint8_t  version;                ///< 版本号（0x10 = v1.0）
    uint8_t  rsoc_pct;               ///< 剩余容量百分比（%）
    uint8_t  fet_control;            ///< FET 状态（bit0=充电MOS，bit1=放电MOS，1=开通）
    uint8_t  cell_count;             ///< 串联节数
    uint8_t  ntc_count;              ///< NTC 探头数量
    float    ntc_temp_c[kBmsMaxNtc]; ///< NTC 温度（℃）
};

/// 0x04 响应：各节单体电压
struct BmsCellVoltages {
    uint16_t mv[kBmsMaxCells]; ///< 各节电芯电压（mV）
    uint8_t  count;            ///< 实际节数
};

// == 编解码器 =================================================================

/**
 * @brief 嘉佰达通用协议 V4  BMS UART 编解码器
 *
 * 物理层：RS485 / UART，9600 BPS，8-N-1
 *
 * 帧格式
 * ──────
 * 读请求（主机 → BMS）：
 *   [0xDD][0xA5][cmd][0x00][chk_H][chk_L][0x77]
 *
 * 写请求（主机 → BMS）：
 *   [0xDD][0x5A][cmd][len][data×len][chk_H][chk_L][0x77]
 *
 * 响应（BMS → 主机）：
 *   [0xDD][cmd][status][len][data×len][chk_H][chk_L][0x77]
 *   status: 0x00=正常，0x80=错误
 *
 * 校验和
 * ──────
 *   sum = cmd + len + data[0] + … + data[len-1]
 *   checksum = (~sum + 1) & 0xFFFF，高字节在前
 *
 * 使用示例
 * ────────
 * @code
 *   BmsProtocol parser;
 *   auto req = BmsProtocol::encode_read_basic_info();
 *   serial.write(req.data(), req.size());
 *
 *   for (uint8_t b : rx_bytes) {
 *       parser.push_byte(b);
 *               && parser.get_cmd() == 0x03) {
 *           auto info = BmsProtocol::decode_basic_info(
 *               parser.get_data(), parser.get_data_len());
 *       }
 *   }
 * @endcode
 */
class BmsProtocol {
public:
    static constexpr size_t kMaxDataLen = 64;  ///< 最大数据段（32节×2字节）

    // -- 编码：主机 → BMS -----------------------------------------------------
    static std::array<uint8_t, 7> encode_read_basic_info();    ///< 读基本信息 0x03
    static std::array<uint8_t, 7> encode_read_cell_voltages(); ///< 读单体电压 0x04
    static std::array<uint8_t, 7> encode_read_version();       ///< 读硬件版本 0x05

    /// MOS 控制命令（写 0xE1）；mos_state 参见 kBmosMos* 常量
    static std::array<uint8_t, 9> encode_mos_control(uint8_t mos_state);

    // -- 解码：BMS → 主机 -----------------------------------------------------
    /// 解码 0x03 数据段 → 基本信息（数据不足/格式错误返回 nullopt）
    static std::optional<BmsBasicInfo>    decode_basic_info(
                                              const uint8_t* data, uint8_t len);
    /// 解码 0x04 数据段 → 单体电压列表
    static std::optional<BmsCellVoltages> decode_cell_voltages(
                                              const uint8_t* data, uint8_t len);
    /// 解码 0x05 数据段 → 硬件版本 ASCII 字符串
    static std::optional<std::string>     decode_version(
                                              const uint8_t* data, uint8_t len);

    // -- 流式帧解析器 ---------------------------------------------------------
    /// 推入一字节，内部状态机自动完成帧同步与校验
    void push_byte(uint8_t b);

    bool           frame_complete()  const { return frame_ready_; }
    uint8_t        get_cmd()         const { return cmd_; }
    bool           is_error()        const { return status_ == 0x80; }
    const uint8_t* get_data()        const { return data_buf_; }
    uint8_t        get_data_len()    const { return len_; }

    /// 清除缓冲区，重置至等待帧头状态
    void reset();

private:
    /// ~(cmd + len + sum(data)) + 1，big-endian uint16
    static uint16_t calc_checksum(uint8_t cmd, uint8_t len, const uint8_t* data);

    enum class ParseState : uint8_t {
        WAIT_START,   ///< 等待帧头 0xDD
        WAIT_CMD,     ///< 读取命令码
        WAIT_STATUS,  ///< 读取状态字节（0x00 / 0x80）
        WAIT_LEN,     ///< 读取数据段长度
        DATA,         ///< 收集数据段字节
        CHKSUM_H,     ///< 读取校验和高字节
        CHKSUM_L,     ///< 读取并验证校验和低字节
        WAIT_STOP,    ///< 验证帧尾 0x77
    };

    ParseState state_{ParseState::WAIT_START};
    uint8_t    cmd_{0};
    uint8_t    status_{0};
    uint8_t    len_{0};
    uint8_t    data_pos_{0};
    uint8_t    chk_h_{0};
    bool       frame_ready_{false};
    uint8_t    data_buf_[kMaxDataLen]{};
};

}  // namespace robot::protocol
