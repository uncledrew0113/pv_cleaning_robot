/*
 * BMS 协议层与设备层单元测试
 *
 * 测试分组：
 *   [protocol][bms]    - BmsProtocol 编码 / 解码 / 帧解析（纯内存，无 I/O）
 *   [device][bms]      - BMS 设备层（基于 BmsMockSerial，不依赖真实硬件）
 *   [integration][bms] - BMS 与真实 /dev/ttyS8 的硬件集成测试（须在目标机上运行）
 *
 * 运行方法（交叉编译后在目标机上）：
 *   ./unit_tests "[protocol][bms]"    # 只跑协议层单元测试
 *   ./unit_tests "[device][bms]"      # 只跑设备层 mock 测试
 *   ./unit_tests "[integration][bms]" # 只跑硬件集成测试（需接 ttyS8 BMS）
 */
#include <algorithm>
#include <catch2/catch.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include "mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/protocol/bms_protocol.h"

using namespace robot;

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief 构造完整 BMS 应答帧（含帧头/帧尾/校验和）
 *
 * 格式：DD [cmd] [status] [len] [data...] [chk_H] [chk_L] 77
 * 校验：sum = status + len + Σdata[i]；checksum = (~sum + 1) & 0xFFFF
 *
 * 注意：响应帧校验覆盖 STATUS+LEN+DATA（CMD 是帧头，不入校验），
 *       与请求帧 CMD+LEN+DATA 的方式对称：帧头字节（0xA5/0x5A 或 CMD）均不入校验。
 *
 * @param cmd    命令码（0x03 / 0x04 / 0x05 / 0xE1 …）
 * @param data   数据段字节列表
 * @param status 0x00=正常，0x80=错误
 */
static std::vector<uint8_t> make_bms_frame(uint8_t cmd,
                                           const std::vector<uint8_t>& data,
                                           uint8_t status = 0x00) {
    uint8_t len = static_cast<uint8_t>(data.size());
    uint16_t sum = static_cast<uint16_t>(status) + len;  // status 入校验，cmd 是帧头不入校验
    for (auto b : data)
        sum = static_cast<uint16_t>(sum + b);
    uint16_t chk = static_cast<uint16_t>(~sum + 1u);

    std::vector<uint8_t> frame;
    frame.push_back(0xDD);
    frame.push_back(cmd);
    frame.push_back(status);
    frame.push_back(len);
    frame.insert(frame.end(), data.begin(), data.end());
    frame.push_back(static_cast<uint8_t>(chk >> 8));
    frame.push_back(static_cast<uint8_t>(chk & 0xFFu));
    frame.push_back(0x77);
    return frame;
}

/**
 * @brief 0x03 基本信息数据段构造参数
 *
 * 默认值对应：5V 总压，1A 充电，SOC=80%，8 节电芯，无 NTC，无告警，
 *             生产日期 2023-03-15，循环次数 5。
 */
struct BasicInfoParams {
    uint16_t total_voltage_raw = 500;  ///< ×10 mV → 5000 mV = 5 V
    uint16_t current_raw = 100;        ///< ×10 mA = 1000 mA = 1 A（正=充电）
    uint16_t residual_mah10 = 1000;    ///< ×10 mAh = 10000 mAh
    uint16_t nominal_mah10 = 2000;     ///< ×10 mAh = 20000 mAh
    uint16_t cycles = 5;
    uint16_t date_raw = 0x2E6Fu;  ///< (23<<9)|(3<<5)|15 = 0x2E6F → 2023-03-15
    uint16_t balance = 0;
    uint16_t balance_high = 0;
    uint16_t protection = 0;
    uint8_t version = 0x10u;
    uint8_t rsoc_pct = 80u;
    uint8_t fet_control = 0x03u;  ///< bit0=充电MOS，bit1=放电MOS，1=开通
    uint8_t cell_count = 8u;
    std::vector<uint16_t> ntc_raws;  ///< 空=无 NTC；每项单位 0.1 K
};

/** 将 BasicInfoParams 序列化为原始字节数组 */
static std::vector<uint8_t> make_basic_info_data(const BasicInfoParams& p) {
    auto push16 = [](std::vector<uint8_t>& v, uint16_t x) {
        v.push_back(static_cast<uint8_t>(x >> 8));
        v.push_back(static_cast<uint8_t>(x & 0xFFu));
    };

    std::vector<uint8_t> d;
    d.reserve(23u + p.ntc_raws.size() * 2u);

    push16(d, p.total_voltage_raw);
    push16(d, p.current_raw);
    push16(d, p.residual_mah10);
    push16(d, p.nominal_mah10);
    push16(d, p.cycles);
    push16(d, p.date_raw);
    push16(d, p.balance);
    push16(d, p.balance_high);
    push16(d, p.protection);
    d.push_back(p.version);
    d.push_back(p.rsoc_pct);
    d.push_back(p.fet_control);
    d.push_back(p.cell_count);
    d.push_back(static_cast<uint8_t>(p.ntc_raws.size()));
    for (auto raw : p.ntc_raws)
        push16(d, raw);

    return d;
}

/**
 * @brief 扩展的串口 mock：
 *   1. flush_input() 真实清空 rx_data（避免跨事务污染）
 *   2. write() 自动识别 BMS 请求命令码并注入预设的应答帧
 */
struct BmsMockSerial : MockSerialPort {
    /// 命令码 → 应答帧字节（write 触发时自动装载到 rx_data）
    std::unordered_map<uint8_t, std::vector<uint8_t>> cmd_responses;

    bool flush_input() override {
        rx_data.clear();
        flush_input_called = true;
        injected_error = hal::UartResult::OK;
        return true;
    }

    int write(const uint8_t* buf, size_t len, int timeout_ms = -1) override {
        int ret = MockSerialPort::write(buf, len, timeout_ms);
        // BMS 帧格式：DD [A5/5A] [cmd] ...
        if (len >= 3u && buf[0] == 0xDDu) {
            auto it = cmd_responses.find(buf[2]);
            if (it != cmd_responses.end())
                rx_data = it->second;
        }
        return ret;
    }
};

// 注册 0x03 基本信息应答
static void register_basic_info(BmsMockSerial& mock, const BasicInfoParams& p = {}) {
    mock.cmd_responses[0x03] = make_bms_frame(0x03, make_basic_info_data(p));
}

// 注册 0x04 单体电压应答（cells 为 mV 列表）
static void register_cell_voltages(BmsMockSerial& mock, const std::vector<uint16_t>& cells) {
    std::vector<uint8_t> data;
    for (auto mv : cells) {
        data.push_back(static_cast<uint8_t>(mv >> 8));
        data.push_back(static_cast<uint8_t>(mv & 0xFFu));
    }
    mock.cmd_responses[0x04] = make_bms_frame(0x04, data);
}

// 注册 0x05 硬件版本应答
static void register_version(BmsMockSerial& mock, const std::string& ver = "V1.0") {
    std::vector<uint8_t> data(ver.begin(), ver.end());
    mock.cmd_responses[0x05] = make_bms_frame(0x05, data);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 1: BmsProtocol 编码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("BmsProtocol 读请求编码", "[protocol][bms]") {
    // 读请求校验和公式：sum = cmd + 0（len=0，无 data）; checksum = (~sum + 1) & 0xFFFF
    // 0x03 → sum=0x03 → chk=0xFFFD
    // 0x04 → sum=0x04 → chk=0xFFFC
    // 0x05 → sum=0x05 → chk=0xFFFB

    SECTION("读基本信息  0x03: DD A5 03 00 FF FD 77") {
        auto f = protocol::BmsProtocol::encode_read_basic_info();
        REQUIRE(f.size() == 7u);
        CHECK(f[0] == 0xDDu);
        CHECK(f[1] == 0xA5u);
        CHECK(f[2] == 0x03u);
        CHECK(f[3] == 0x00u);
        CHECK(f[4] == 0xFFu);
        CHECK(f[5] == 0xFDu);
        CHECK(f[6] == 0x77u);
    }

    SECTION("读单体电压  0x04: DD A5 04 00 FF FC 77") {
        auto f = protocol::BmsProtocol::encode_read_cell_voltages();
        REQUIRE(f.size() == 7u);
        CHECK(f[1] == 0xA5u);
        CHECK(f[2] == 0x04u);
        CHECK(f[4] == 0xFFu);
        CHECK(f[5] == 0xFCu);
        CHECK(f[6] == 0x77u);
    }

    SECTION("读硬件版本  0x05: DD A5 05 00 FF FB 77") {
        auto f = protocol::BmsProtocol::encode_read_version();
        REQUIRE(f.size() == 7u);
        CHECK(f[2] == 0x05u);
        CHECK(f[4] == 0xFFu);
        CHECK(f[5] == 0xFBu);
        CHECK(f[6] == 0x77u);
    }
}

TEST_CASE("BmsProtocol MOS 控制编码", "[protocol][bms]") {
    // 写请求格式：DD 5A E1 02 00 [mos_state] [chk_H] [chk_L] 77
    // sum = 0xE1 + 0x02 + 0x00 + mos_state；checksum = (~sum + 1) & 0xFFFF
    //   0x00 → sum=0xE3 → chk=0xFF1D
    //   0x01 → sum=0xE4 → chk=0xFF1C
    //   0x02 → sum=0xE5 → chk=0xFF1B
    //   0x03 → sum=0xE6 → chk=0xFF1A

    SECTION("解除关断 0x00: chk = FF 1D") {
        auto f = protocol::BmsProtocol::encode_mos_control(protocol::kBmosMosReleaseAll);
        REQUIRE(f.size() == 9u);
        CHECK(f[0] == 0xDDu);
        CHECK(f[1] == 0x5Au);
        CHECK(f[2] == 0xE1u);
        CHECK(f[3] == 0x02u);
        CHECK(f[4] == 0x00u);
        CHECK(f[5] == 0x00u);
        CHECK(f[6] == 0xFFu);
        CHECK(f[7] == 0x1Du);
        CHECK(f[8] == 0x77u);
    }

    SECTION("关闭充电 MOS 0x01: chk = FF 1C") {
        auto f = protocol::BmsProtocol::encode_mos_control(protocol::kBmosMosChargeClose);
        CHECK(f[5] == 0x01u);
        CHECK(f[6] == 0xFFu);
        CHECK(f[7] == 0x1Cu);
    }

    SECTION("关闭放电 MOS 0x02: chk = FF 1B") {
        auto f = protocol::BmsProtocol::encode_mos_control(protocol::kBmosMosDischargeClose);
        CHECK(f[5] == 0x02u);
        CHECK(f[6] == 0xFFu);
        CHECK(f[7] == 0x1Bu);
    }

    SECTION("同时关闭 0x03: chk = FF 1A") {
        auto f = protocol::BmsProtocol::encode_mos_control(protocol::kBmosMosBothClose);
        CHECK(f[5] == 0x03u);
        CHECK(f[6] == 0xFFu);
        CHECK(f[7] == 0x1Au);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 2: BmsProtocol 解码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("BmsProtocol decode_basic_info 正常解码", "[protocol][bms]") {
    // 数据段参数（默认值）：
    //   total_voltage_raw=500  → 5000 mV = 5 V
    //   current_raw=100        → +1000 mA = +1 A（充电）
    //   residual=1000          → 10000 mAh
    //   nominal=2000           → 20000 mAh
    //   cycles=5, date=2023-03-15, rsoc=80%, cell_count=8, ntc_count=0
    BasicInfoParams p;
    auto data = make_basic_info_data(p);

    auto result =
        protocol::BmsProtocol::decode_basic_info(data.data(), static_cast<uint8_t>(data.size()));
    REQUIRE(result.has_value());
    const auto& info = *result;

    SECTION("总电压：raw×10mV = 5000 mV") {
        CHECK(info.total_voltage_mv == 5000u);
    }

    SECTION("充电电流：raw×10mA = +1000 mA") {
        CHECK(info.current_ma == 1000);
    }

    SECTION("剩余 / 标称容量") {
        CHECK(info.residual_capacity_mah == 10000u);
        CHECK(info.nominal_capacity_mah == 20000u);
    }

    SECTION("循环次数") {
        CHECK(info.cycle_count == 5u);
    }

    SECTION("生产日期 2023-03-15") {
        // raw = (23<<9)|(3<<5)|15 = 0x2E6F
        // day = raw & 0x1F = 15
        // month = (raw>>5) & 0xF = 3
        // year = 2000 + (raw>>9) = 2023
        CHECK(info.production_date.year == 2023u);
        CHECK(info.production_date.month == 3u);
        CHECK(info.production_date.day == 15u);
    }

    SECTION("SOC / 版本 / FET") {
        CHECK(info.rsoc_pct == 80u);
        CHECK(info.version == 0x10u);
        CHECK(info.fet_control == 0x03u);
        CHECK(info.cell_count == 8u);
        CHECK(info.ntc_count == 0u);
    }

    SECTION("无保护告警") {
        CHECK(info.protection_status == 0u);
    }
}

TEST_CASE("BmsProtocol decode_basic_info 放电电流解码", "[protocol][bms]") {
    // 放电电流编码：bit15=1，magnitude = 65536 - raw
    // 2A 放电 = 200 单位 × 10mA; raw = 65536 - 200 = 65336 = 0xFF38
    BasicInfoParams p;
    p.current_raw = 0xFF38u;  // 放电 2A
    auto data = make_basic_info_data(p);

    auto result =
        protocol::BmsProtocol::decode_basic_info(data.data(), static_cast<uint8_t>(data.size()));
    REQUIRE(result.has_value());

    // 期望：-2000 mA
    CHECK(result->current_ma == -2000);
}

TEST_CASE("BmsProtocol decode_basic_info NTC 温度解码", "[protocol][bms]") {
    // NTC 温度公式：℃ = (raw_0.1K - 2731) / 10.0
    //   25.0℃ → raw = 2731 + 250 = 2981 = 0x0BA5
    //   30.0℃ → raw = 2731 + 300 = 3031 = 0x0BD7
    BasicInfoParams p;
    p.ntc_raws = {2981u, 3031u};  // 25℃, 30℃
    p.cell_count = 4u;
    auto data = make_basic_info_data(p);

    auto result =
        protocol::BmsProtocol::decode_basic_info(data.data(), static_cast<uint8_t>(data.size()));
    REQUIRE(result.has_value());
    const auto& info = *result;

    CHECK(info.ntc_count == 2u);
    CHECK(info.ntc_temp_c[0] == Approx(25.0f).margin(0.1f));
    CHECK(info.ntc_temp_c[1] == Approx(30.0f).margin(0.1f));
}

TEST_CASE("BmsProtocol decode_basic_info 保护标志位解码", "[protocol][bms]") {
    // 设置：单体过压(bit0) + 放电过流(bit9) = 0x0201
    using namespace protocol;
    BasicInfoParams p;
    p.protection = static_cast<uint16_t>(kBmsProtCellOverVolt | kBmsProtDsgOverCurr);
    auto data = make_basic_info_data(p);

    auto result =
        protocol::BmsProtocol::decode_basic_info(data.data(), static_cast<uint8_t>(data.size()));
    REQUIRE(result.has_value());

    CHECK((result->protection_status & kBmsProtCellOverVolt) != 0u);
    CHECK((result->protection_status & kBmsProtDsgOverCurr) != 0u);
    CHECK((result->protection_status & kBmsProtBatOverVolt) == 0u);
    CHECK((result->protection_status & kBmsProtShortCircuit) == 0u);
}

TEST_CASE("BmsProtocol decode_basic_info 数据段不足返回 nullopt", "[protocol][bms]") {
    SECTION("空数据") {
        auto r = protocol::BmsProtocol::decode_basic_info(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("固定字段不足 23 字节") {
        std::vector<uint8_t> short_data(22u, 0x00u);
        auto r = protocol::BmsProtocol::decode_basic_info(short_data.data(),
                                                          static_cast<uint8_t>(short_data.size()));
        CHECK_FALSE(r.has_value());
    }

    SECTION("声称有 NTC 但数据不够") {
        BasicInfoParams p;
        p.ntc_raws = {2981u};  // 声称 1 个 NTC（需 2 额外字节），但截断
        auto data = make_basic_info_data(p);
        data.pop_back();  // 截掉最后 1 字节使数据不足

        auto r = protocol::BmsProtocol::decode_basic_info(data.data(),
                                                          static_cast<uint8_t>(data.size()));
        CHECK_FALSE(r.has_value());
    }
}

TEST_CASE("BmsProtocol decode_cell_voltages", "[protocol][bms]") {
    SECTION("4 节电芯：3600/3650/3620/3580 mV") {
        // 每节 2字节大端：3600=0x0E10, 3650=0x0E42, 3620=0x0E24, 3580=0x0DFC
        const uint8_t data[] = {0x0Eu,
                                0x10u,  // 3600
                                0x0Eu,
                                0x42u,  // 3650
                                0x0Eu,
                                0x24u,  // 3620
                                0x0Du,
                                0xFCu};  // 3580
        auto r = protocol::BmsProtocol::decode_cell_voltages(data, 8u);
        REQUIRE(r.has_value());
        CHECK(r->count == 4u);
        CHECK(r->mv[0] == 3600u);
        CHECK(r->mv[1] == 3650u);
        CHECK(r->mv[2] == 3620u);
        CHECK(r->mv[3] == 3580u);
    }

    SECTION("奇数字节返回 nullopt") {
        const uint8_t data[] = {0x0Eu, 0x10u, 0x0Eu};
        auto r = protocol::BmsProtocol::decode_cell_voltages(data, 3u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("空数据返回 nullopt") {
        auto r = protocol::BmsProtocol::decode_cell_voltages(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("最大节数截断（超过 kBmsMaxCells 节时只取前 32 节）") {
        // 构造 34 节 × 2 字节 = 68 字节数据
        std::vector<uint8_t> data(68u, 0x00u);
        for (size_t i = 0; i < 34u; ++i) {
            data[i * 2] = 0x0Eu;  // ~3584+ mV
            data[i * 2 + 1] = static_cast<uint8_t>(i);
        }
        auto r = protocol::BmsProtocol::decode_cell_voltages(data.data(),
                                                             static_cast<uint8_t>(data.size()));
        REQUIRE(r.has_value());
        CHECK(r->count == protocol::kBmsMaxCells);  // 截断为 32
    }
}

TEST_CASE("BmsProtocol decode_version", "[protocol][bms]") {
    SECTION("ASCII 字符串正常返回") {
        const uint8_t data[] = {'V', '1', '.', '0'};
        auto r = protocol::BmsProtocol::decode_version(data, 4u);
        REQUIRE(r.has_value());
        CHECK(*r == "V1.0");
    }

    SECTION("空数据返回 nullopt") {
        auto r = protocol::BmsProtocol::decode_version(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 3: BmsProtocol 流式帧解析器（push_byte）测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("BmsProtocol push_byte 帧解析", "[protocol][bms]") {
    protocol::BmsProtocol parser;

    // 构造一个合法的 0x03 应答帧（最简：23字节数据段，ntc=0）
    BasicInfoParams p;
    p.rsoc_pct = 60u;
    auto valid_frame = make_bms_frame(0x03u, make_basic_info_data(p));

    SECTION("推入完整合法帧后 frame_complete()==true") {
        for (auto b : valid_frame)
            parser.push_byte(b);

        CHECK(parser.frame_complete());
        CHECK(parser.get_cmd() == 0x03u);
        CHECK_FALSE(parser.is_error());
        CHECK(parser.get_data_len() == 23u);
    }

    SECTION("校验和错误帧：frame_complete() 不置位") {
        // 破坏倒数第2字节（chk_H）
        auto bad_frame = valid_frame;
        bad_frame[bad_frame.size() - 3] ^= 0xFFu;  // 翻转 chk_H

        for (auto b : bad_frame)
            parser.push_byte(b);

        CHECK_FALSE(parser.frame_complete());
    }

    SECTION("帧尾字节错误：frame_complete() 不置位") {
        auto bad_frame = valid_frame;
        bad_frame.back() = 0x00u;  // 破坏 0x77

        for (auto b : bad_frame)
            parser.push_byte(b);

        CHECK_FALSE(parser.frame_complete());
    }

    SECTION("错误状态帧（status=0x80）：is_error()==true") {
        auto err_frame = make_bms_frame(0xE1u, {}, 0x80u);  // 空数据 + 错误状态
        for (auto b : err_frame)
            parser.push_byte(b);

        CHECK(parser.frame_complete());
        CHECK(parser.is_error());
        CHECK(parser.get_cmd() == 0xE1u);
    }

    SECTION("连续两帧：第一帧完成后 reset 再推第二帧") {
        for (auto b : valid_frame)
            parser.push_byte(b);
        REQUIRE(parser.frame_complete());

        // 构造第二帧（不同 SOC）
        BasicInfoParams p2;
        p2.rsoc_pct = 95u;
        auto frame2 = make_bms_frame(0x03u, make_basic_info_data(p2));

        parser.reset();
        CHECK_FALSE(parser.frame_complete());

        for (auto b : frame2)
            parser.push_byte(b);

        CHECK(parser.frame_complete());
        auto info =
            protocol::BmsProtocol::decode_basic_info(parser.get_data(), parser.get_data_len());
        REQUIRE(info.has_value());
        CHECK(info->rsoc_pct == 95u);
    }

    SECTION("乱码字节后自动同步下一帧") {
        // 先推入一段乱码
        const uint8_t garbage[] = {0x12u, 0x34u, 0x56u, 0x78u, 0xAAu};
        for (auto b : garbage)
            parser.push_byte(b);

        // 再推入合法帧，解析器应能自动重新同步
        for (auto b : valid_frame)
            parser.push_byte(b);

        CHECK(parser.frame_complete());
        CHECK(parser.get_cmd() == 0x03u);
    }

    SECTION("0x04 单体电压帧解码验证") {
        auto cell_frame = make_bms_frame(0x04u, {0x0Eu, 0x10u, 0x0Eu, 0x42u});
        for (auto b : cell_frame)
            parser.push_byte(b);

        REQUIRE(parser.frame_complete());
        CHECK(parser.get_cmd() == 0x04u);
        CHECK(parser.get_data_len() == 4u);

        auto cells =
            protocol::BmsProtocol::decode_cell_voltages(parser.get_data(), parser.get_data_len());
        REQUIRE(cells.has_value());
        CHECK(cells->count == 2u);
        CHECK(cells->mv[0] == 3600u);
        CHECK(cells->mv[1] == 3650u);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 4: BMS 设备层 mock 测试
// ═══════════════════════════════════════════════════════════════════════════

