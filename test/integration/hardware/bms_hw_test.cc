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

TEST_CASE("BMS 集成测试 - ttyS8 串口连接与数据读取", "[integration][bms]") {
    using namespace std::chrono_literals;

    hal::UartConfig cfg;
    cfg.baudrate = 9600;
    cfg.data_bits = 8;
    cfg.parity = 'N';
    cfg.stop_bits = 1;
    cfg.write_timeout_ms = 500;

    auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS8", cfg);
    device::BMS bms(serial, 95.0f, 15.0f);

    SECTION("串口打开与 BMS 首次握手") {
        bool opened = (bms.open() == device::DeviceError::OK);
        REQUIRE(opened);
        spdlog::info("[BMS 集成] 串口打开成功");

        auto diag = bms.get_diagnostics();
        spdlog::info("[BMS 集成] 硬件版本: {}", diag.hw_version);
    }

    SECTION("基本数据合理性验证") {
        REQUIRE(bms.open() == device::DeviceError::OK);

        bms.update();
        bms.update();

        auto data = bms.get_data();
        REQUIRE(data.valid);

        spdlog::info("[BMS 集成] SOC={:.1f}%  电压={:.2f}V  电流={:.3f}A  温度={:.1f}℃",
                     data.soc_pct,
                     data.voltage_v,
                     data.current_a,
                     data.temperature_c);
        spdlog::info("[BMS 集成] 充电={} 充满={} 低电量={} 告警=0x{:04X}",
                     data.charging,
                     data.fully_charged,
                     data.low_battery,
                     data.alarm_flags);

        CHECK(data.soc_pct >= 0.0f);
        CHECK(data.soc_pct <= 100.0f);
        CHECK(data.voltage_v > 0.0f);
        CHECK(data.temperature_c > -40.0f);
        CHECK(data.temperature_c < 80.0f);
    }

    SECTION("单体电压读取（4 次 update 触发 0x04 指令）") {
        REQUIRE(bms.open() == device::DeviceError::OK);

        for (int i = 0; i < 4; ++i) {
            bms.update();
            std::this_thread::sleep_for(200ms);
        }

        auto diag = bms.get_diagnostics();

        spdlog::info("[BMS 集成] 串联节数={}  最高电芯={:.3f}V  最低电芯={:.3f}V",
                     diag.cell_count,
                     diag.cell_voltage_max_v,
                     diag.cell_voltage_min_v);

        if (diag.cell_voltages.count > 0u) {
            CHECK(diag.cell_voltage_max_v > 0.0f);
            CHECK(diag.cell_voltage_min_v > 0.0f);
            CHECK(diag.cell_voltage_max_v >= diag.cell_voltage_min_v);

            for (uint8_t i = 0; i < diag.cell_voltages.count; ++i) {
                spdlog::info("[BMS 集成]   cell[{}] = {} mV", i, diag.cell_voltages.mv[i]);
            }
        }
    }

    SECTION("NTC 温度读取") {
        REQUIRE(bms.open() == device::DeviceError::OK);

        auto diag = bms.get_diagnostics();
        spdlog::info("[BMS 集成] NTC 探头数量={}", diag.ntc_count);
        for (uint8_t i = 0; i < diag.ntc_count; ++i) {
            spdlog::info("[BMS 集成]   NTC[{}] = {:.1f} ℃", i, diag.ntc_temps_c[i]);
            CHECK(diag.ntc_temps_c[i] > -40.0f);
            CHECK(diag.ntc_temps_c[i] < 80.0f);
        }
    }

    SECTION("持续 10 次 update 无崩溃，通信错误率低于 40%") {
        REQUIRE(bms.open() == device::DeviceError::OK);

        constexpr int kRounds = 10;
        for (int i = 0; i < kRounds; ++i) {
            bms.update();
            std::this_thread::sleep_for(300ms);
        }

        auto diag = bms.get_diagnostics();
        spdlog::info(
            "[BMS 集成] update_count={} error_count={}", diag.update_count, diag.error_count);

        // 至少 60% 成功率
        CHECK(diag.update_count > static_cast<uint32_t>(kRounds * 0.6));
    }

    SECTION("诊断数据完整性打印") {
        REQUIRE(bms.open() == device::DeviceError::OK);
        for (int i = 0; i < 4; ++i) {
            bms.update();
            std::this_thread::sleep_for(200ms);
        }

        auto diag = bms.get_diagnostics();
        spdlog::info("[BMS 集成] ──── 完整诊断数据 ────");
        spdlog::info("  SOC           = {:.1f}%", diag.soc_pct);
        spdlog::info("  总电压        = {:.3f} V", diag.voltage_v);
        spdlog::info("  电流          = {:.3f} A", diag.current_a);
        spdlog::info("  剩余容量      = {:.2f} Ah", diag.remaining_capacity_ah);
        spdlog::info("  标称容量      = {:.2f} Ah", diag.nominal_capacity_ah);
        spdlog::info("  循环次数      = {}", diag.cycle_count);
        spdlog::info("  串联节数      = {}", diag.cell_count);
        spdlog::info("  NTC 数量      = {}", diag.ntc_count);
        spdlog::info("  保护标志      = 0x{:04X}", diag.alarm_flags);
        spdlog::info("  硬件版本      = {}", diag.hw_version);
        spdlog::info("  最高单体电压  = {:.3f} V", diag.cell_voltage_max_v);
        spdlog::info("  最低单体电压  = {:.3f} V", diag.cell_voltage_min_v);
        spdlog::info("  update/error  = {}/{}", diag.update_count, diag.error_count);
        SUCCEED();
    }
}
