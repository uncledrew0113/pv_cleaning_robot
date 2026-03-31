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

TEST_CASE("BMS 设备层 - 生命周期", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();

    SECTION("串口打开失败时 open() 返回 false") {
        mock->open_result = false;
        device::BMS bms(mock);
        CHECK(bms.open() != device::DeviceError::OK);
        CHECK_FALSE(bms.get_data().valid);
    }

    SECTION("串口打开成功，open() 返回 true（即使 BMS 无响应）") {
        // 不注册任何命令响应 → transact 超时，但 open() 仍返回 true
        device::BMS bms(mock);
        CHECK(bms.open() == device::DeviceError::OK);  // BMS open() 在 open失败时才返回false
    }
}

TEST_CASE("BMS 设备层 - update() 读取基本数据", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();

    // 注册版本和基本信息响应
    register_version(*mock, "V1.0");
    BasicInfoParams p;
    p.rsoc_pct = 75u;
    p.total_voltage_raw = 4800u;  // 48000 mV = 48 V（典型锂电池组）
    p.current_raw = 200u;         // 2000 mA = 2 A 充电
    p.residual_mah10 = 3000u;     // 30000 mAh
    p.nominal_mah10 = 4000u;      // 40000 mAh
    p.cycles = 42u;
    register_basic_info(*mock, p);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    auto data = bms.get_data();
    CHECK(data.valid);
    CHECK(data.soc_pct == Approx(75.0f).margin(0.01f));
    CHECK(data.voltage_v == Approx(48.0f).margin(0.01f));
    CHECK(data.current_a == Approx(2.0f).margin(0.01f));
    CHECK(data.charging);
    CHECK_FALSE(data.low_battery);
    CHECK_FALSE(data.fully_charged);
    CHECK(data.alarm_flags == 0u);
}

TEST_CASE("BMS 设备层 - 放电状态识别", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);

    BasicInfoParams p;
    p.current_raw = 0xFF9Cu;  // 放电 1A：raw=0xFF9C → -1000 mA
    register_basic_info(*mock, p);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    auto data = bms.get_data();
    CHECK(data.valid);
    CHECK(data.current_a < 0.0f);
    CHECK_FALSE(data.charging);
}

TEST_CASE("BMS 设备层 - 低电量告警", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);

    BasicInfoParams p;
    p.rsoc_pct = 10u;  // 10% < 默认低电量阈值 15%
    register_basic_info(*mock, p);

    device::BMS bms(mock, 95.0f, 15.0f);
    REQUIRE(bms.open() == device::DeviceError::OK);

    CHECK(bms.is_low_battery());
    CHECK(bms.get_data().low_battery);
}

TEST_CASE("BMS 设备层 - 充满状态检测（需 kFullChargeConfirm 次连续确认）", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();

    // SOC=96% >= full_soc=95%，电流接近 0（充电 MOS 关闭时 FET bit0=0）
    BasicInfoParams p;
    p.rsoc_pct = 96u;
    p.current_raw = 0x0002u;  // 20 mA（|I| < 0.5A 阈值）
    p.fet_control = 0x02u;    // 只有放电 MOS 开通，充电 MOS 关闭
    register_version(*mock);
    register_basic_info(*mock, p);

    device::BMS bms(mock, 95.0f, 15.0f);
    REQUIRE(bms.open() == device::DeviceError::OK);

    // 首次 open() 内已读取一次，需再读 5 次凑满 kFullChargeConfirm=6
    for (int i = 0; i < 5; ++i)
        bms.update();

    CHECK(bms.is_fully_charged());
    CHECK(bms.get_data().fully_charged);
}

TEST_CASE("BMS 设备层 - 保护告警标志", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);

    using namespace protocol;
    BasicInfoParams p;
    p.protection = static_cast<uint16_t>(kBmsProtCellOverVolt | kBmsProtChgOverTemp);
    register_basic_info(*mock, p);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    CHECK(bms.has_alarm());
    auto data = bms.get_data();
    CHECK((data.alarm_flags & device::BMS::PROT_CELL_OVERVOLT) != 0u);
    CHECK((data.alarm_flags & device::BMS::PROT_CHG_OVERTEMP) != 0u);
    CHECK((data.alarm_flags & device::BMS::PROT_BAT_UNDERVOLT) == 0u);
}

TEST_CASE("BMS 设备层 - 单体电压读取与最值计算", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);
    register_basic_info(*mock);

    // 4 节电芯：3600 / 3650 / 3580 / 3630 mV
    register_cell_voltages(*mock, {3600u, 3650u, 3580u, 3630u});

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    // 触发 4 次 update：第 4 次会额外读取单体电压（update_cycle_ % 4 == 0）
    for (int i = 0; i < 4; ++i)
        bms.update();

    auto diag = bms.get_diagnostics();
    CHECK(diag.cell_voltages.count >= 4u);
    CHECK(diag.cell_voltage_max_v == Approx(3.650f).margin(0.001f));
    CHECK(diag.cell_voltage_min_v == Approx(3.580f).margin(0.001f));
}

TEST_CASE("BMS 设备层 - NTC 温度与最高温度字段", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);

    // 3 个 NTC：25℃ / 28℃ / 32℃ → 最高温度应为 32℃
    BasicInfoParams p;
    p.ntc_raws = {2981u, 3011u, 3051u};  // 25 / 28 / 32 ℃
    p.cell_count = 4u;
    register_basic_info(*mock, p);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    auto data = bms.get_data();
    CHECK(data.temperature_c == Approx(32.0f).margin(0.1f));  // 最高温为代表值

    auto diag = bms.get_diagnostics();
    CHECK(diag.ntc_count == 3u);
    CHECK(diag.ntc_temps_c[0] == Approx(25.0f).margin(0.1f));
    CHECK(diag.ntc_temps_c[1] == Approx(28.0f).margin(0.1f));
    CHECK(diag.ntc_temps_c[2] == Approx(32.0f).margin(0.1f));
}

TEST_CASE("BMS 设备层 - 通信超时导致 error_count 递增", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    // 不注册任何响应 → transact 必然超时

    device::BMS bms(mock);
    bms.open();  // open() 失败但不 abort

    // 调用 3 次 update() → 每次 read_basic_info 失败 → error_count += 3
    for (int i = 0; i < 3; ++i)
        bms.update();

    auto diag = bms.get_diagnostics();
    CHECK(diag.error_count >= 3u);
}

TEST_CASE("BMS 设备层 - update_count 反映成功读取次数", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);
    register_basic_info(*mock);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);  // open() 内读一次 → update_count=1

    bms.update();  // +1
    bms.update();  // +1

    auto diag = bms.get_diagnostics();
    CHECK(diag.update_count >= 3u);
    CHECK(diag.hw_version == "V1.0");
}

TEST_CASE("BMS 设备层 - mos_control() 成功应答", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);
    register_basic_info(*mock);

    // 注册 MOS 控制命令应答（空数据段，status=0x00）
    mock->cmd_responses[0xE1u] = make_bms_frame(0xE1u, {});

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    CHECK(bms.mos_control(device::BMS::MOS_RELEASE_ALL) == device::DeviceError::OK);
}

TEST_CASE("BMS 设备层 - mos_control() BMS 返回错误状态", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);
    register_basic_info(*mock);

    // 注册 MOS 控制命令应答（status=0x80 错误）
    mock->cmd_responses[0xE1u] = make_bms_frame(0xE1u, {}, 0x80u);

    device::BMS bms(mock);
    REQUIRE(bms.open() == device::DeviceError::OK);

    // BMS 返回错误应答，mos_control 应返回 false
    CHECK(bms.mos_control(device::BMS::MOS_BOTH_CLOSE) != device::DeviceError::OK);
}

TEST_CASE("BMS 设备层 - get_data/get_diagnostics 线程安全", "[device][bms]") {
    auto mock = std::make_shared<BmsMockSerial>();
    register_version(*mock);
    register_basic_info(*mock);

    auto bms = std::make_shared<device::BMS>(mock);
    REQUIRE(bms->open() == device::DeviceError::OK);

    // 在更新线程执行的同时从另一线程读数据，不应崩溃
    std::atomic<bool> stop{false};
    std::thread updater([&] {
        for (int i = 0; i < 20 && not stop.load(); ++i) {
            bms->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    for (int i = 0; i < 20; ++i) {
        auto data = bms->get_data();
        auto diag = bms->get_diagnostics();
        (void)data;
        (void)diag;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    stop = true;
    updater.join();
    SUCCEED();  // 无崩溃 / 无死锁即通过
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 5: BMS 硬件集成测试（需接 /dev/ttyS8，9600-8-N-1）
// ═══════════════════════════════════════════════════════════════════════════

