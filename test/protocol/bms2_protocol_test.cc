/*
 * BMS2 协议层与设备层单元测试（嘉佰达 Modbus RTU 协议）
 *
 * 测试分组：
 *   [protocol][bms2]    - Bms2Protocol 解码（纯内存，无 I/O）
 *   [device][bms2]      - BMS 设备层 Modbus 路径（基于 MockModbusMaster，不依赖真实硬件）
 *   [integration][bms2] - BMS2 与真实 Modbus 设备的硬件集成测试（须在目标机上运行）
 *
 * 运行方法（交叉编译后在目标机上）：
 *   ./unit_tests "[protocol][bms2]"    # 只跑协议层单元测试
 *   ./unit_tests "[device][bms2]"      # 只跑设备层 mock 测试
 *   ./unit_tests "[integration][bms2]" # 只跑硬件集成测试（需接 BMS2 设备）
 *
 * 寄存器地址约定（相对于 kBms2RegStatusBase=0x38 的偏移）：
 *   kBms2StatusBlockLen = 46 个寄存器（0x38~0x65）
 *   kBms2AlarmBlockLen  =  7 个寄存器（0x6D~0x73）
 *   kBms2MaxCells       = 48，kBms2MaxNtc = 8
 *
 * 电流符号约定（BMS2 协议原始，与 BMS1 相反）：
 *   raw_to_current(raw) = (raw - 30000) × 0.1A；正值=放电，负值=充电
 *   device::BMS 在写入 diag_.current_a 时取反，确保向外暴露的值与 BMS1 一致（正=充电）。
 */
#include <algorithm>
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

#include "mock/mock_modbus_master.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/protocol/bms2_protocol.h"

using namespace robot;

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具 - 协议层
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief 主状态块（0x38~0x65）构造参数
 *
 * 默认值：48V 总压，空载，SOC=80%，16 节电芯，3 个 NTC，无告警。
 *
 * 电流编码：raw=30000 → 0A；raw=29950 → -5A（充电）；raw=30050 → +5A（放电）
 * 温度编码：raw → raw − 40 = ℃；25℃ → raw=65；30℃ → raw=70
 * SOC 编码：raw ÷ 10 = %；800 → 80.0%
 * RTC 编码（0x61）：high=年偏置（年-2000），low=月；0x1803 → 2024-03
 */
struct Bms2StatusParams {
    uint16_t total_voltage_raw = 480u;  ///< × 0.1V → 48.0V
    uint16_t current_raw = 30000u;      ///< Offset 30000；0A，正=放电
    uint16_t soc_raw = 800u;            ///< ÷ 10 = 80.0%
    uint16_t life_counter = 42u;        ///< 心跳包（0x3B）
    uint16_t cell_count = 16u;          ///< 串联节数
    uint16_t ntc_count = 3u;            ///< NTC 探头数
    uint16_t cell_max_mv = 3650u;       ///< mV
    uint16_t cell_max_idx = 3u;         ///< 1-based
    uint16_t cell_min_mv = 3580u;       ///< mV
    uint16_t cell_min_idx = 12u;        ///< 1-based
    uint16_t cell_delta_mv = 70u;       ///< mV
    uint16_t avg_cell_mv = 3620u;       ///< mV
    uint16_t temp_max_raw = 65u;        ///< 65-40=25°C
    uint16_t temp_max_idx = 1u;
    uint16_t temp_min_raw = 60u;  ///< 60-40=20°C
    uint16_t temp_min_idx = 3u;
    uint16_t temp_delta = 5u;         ///< °C
    uint16_t charge_status = 0u;      ///< 0=Idle 1=充电 2=放电
    uint16_t charger_detect = 0u;     ///< 1=有充电器
    uint16_t load_detect = 1u;        ///< 1=有负载
    uint16_t remain_cap_raw = 1000u;  ///< × 0.1Ah → 100.0Ah
    uint16_t cycle_count = 37u;       ///< 循环次数
    uint16_t balance_status = 0u;     ///< 0=关
    uint16_t balance_bits0 = 0u;      ///< bit0-15 = 电芯 1-16 均衡位
    uint16_t balance_bits1 = 0u;
    uint16_t balance_bits2 = 0u;
    uint16_t charge_mos = 1u;  ///< 1=开通
    uint16_t discharge_mos = 1u;
    uint16_t precharge_mos = 0u;
    uint16_t heat_mos = 0u;
    uint16_t fan_mos = 0u;
    uint16_t power_w = 960u;       ///< W
    uint16_t energy_wh = 10000u;   ///< Wh
    uint16_t mos_temp_raw = 70u;   ///< 30°C
    uint16_t env_temp_raw = 65u;   ///< 25°C
    uint16_t heat_temp_raw = 60u;  ///< 20°C
    uint16_t heat_current = 0u;    ///< A
    uint16_t current_limit_status = 0u;
    uint16_t current_limit_raw = 30000u;  ///< 0A（Offset 30000）
    uint16_t rtc_ym = 0x1803u;            ///< 高字节=24(2024-2000)，低字节=3(月)
    uint16_t rtc_dh = 0x160Au;            ///< 高字节=22(日)，低字节=10(时)
    uint16_t rtc_ms = 0x1E2Du;            ///< 高字节=30(分)，低字节=45(秒)
    uint16_t remaining_time_min = 120u;
    uint16_t di_do_status = 0u;
};

/** 将 Bms2StatusParams 写入 46 元素寄存器数组（kBms2StatusBlockLen = 0x2E = 46） */
static void fill_status_regs(uint16_t* regs, const Bms2StatusParams& p = {}) {
    std::memset(regs, 0, protocol::kBms2StatusBlockLen * sizeof(uint16_t));
    using namespace protocol::bms2_off;

    regs[kTotalVoltage] = p.total_voltage_raw;
    regs[kCurrent] = p.current_raw;
    regs[kSoc] = p.soc_raw;
    regs[kLifeCounter] = p.life_counter;
    regs[kCellCount] = p.cell_count;
    regs[kNtcCount] = p.ntc_count;
    regs[kCellMaxMv] = p.cell_max_mv;
    regs[kCellMaxIdx] = p.cell_max_idx;
    regs[kCellMinMv] = p.cell_min_mv;
    regs[kCellMinIdx] = p.cell_min_idx;
    regs[kCellDelta] = p.cell_delta_mv;
    regs[kTempMax] = p.temp_max_raw;
    regs[kTempMaxIdx] = p.temp_max_idx;
    regs[kTempMin] = p.temp_min_raw;
    regs[kTempMinIdx] = p.temp_min_idx;
    regs[kTempDelta] = p.temp_delta;
    regs[kChargeStatus] = p.charge_status;
    regs[kChargerDetect] = p.charger_detect;
    regs[kLoadDetect] = p.load_detect;
    regs[kRemainCap] = p.remain_cap_raw;
    regs[kCycleCount] = p.cycle_count;
    regs[kBalanceStatus] = p.balance_status;
    regs[kBalanceBits0] = p.balance_bits0;
    regs[kBalanceBits1] = p.balance_bits1;
    regs[kBalanceBits2] = p.balance_bits2;
    regs[kChargeMos] = p.charge_mos;
    regs[kDischargeMos] = p.discharge_mos;
    regs[kPreChargeMos] = p.precharge_mos;
    regs[kHeatMos] = p.heat_mos;
    regs[kFanMos] = p.fan_mos;
    regs[kAvgCellMv] = p.avg_cell_mv;
    regs[kPowerW] = p.power_w;
    regs[kEnergyWh] = p.energy_wh;
    regs[kMosTemp] = p.mos_temp_raw;
    regs[kEnvTemp] = p.env_temp_raw;
    regs[kHeatTemp] = p.heat_temp_raw;
    regs[kHeatCurrent] = p.heat_current;
    // offset 0x26 undefined，保持 0
    regs[kCurrentLimitStatus] = p.current_limit_status;
    regs[kCurrentLimitCurrent] = p.current_limit_raw;
    regs[kRtcYearMonth] = p.rtc_ym;
    regs[kRtcDayHour] = p.rtc_dh;
    regs[kRtcMinSec] = p.rtc_ms;
    regs[kRemainingTime] = p.remaining_time_min;
    regs[kDiDoStatus] = p.di_do_status;
}

// ═══════════════════════════════════════════════════════════════════════════
//  测试辅助工具 - 设备层 Mock
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief BMS2 专用 Modbus mock
 *
 * 重写 read_registers() 使其按地址分派到各预设寄存器块，
 * 无需填充 MockModbusMaster::registers 表。
 * 通过各 *_fail 标志可模拟特定地址读取失败。
 */
struct BmsMockModbus2 : MockModbusMaster {
    uint16_t status_regs[protocol::kBms2StatusBlockLen]{};
    uint16_t temp_regs[protocol::kBms2MaxNtc]{};
    uint16_t alarm_regs[protocol::kBms2AlarmBlockLen]{};
    uint16_t cell_regs[protocol::kBms2MaxCells]{};

    bool status_fail{false};  ///< 为 true 时，主状态块读取返回 -1
    bool temp_fail{false};
    bool alarm_fail{false};
    bool cell_fail{false};

    int read_registers(int /*slave*/, int addr, int count, uint16_t* out) override {
        auto fill = [&](const uint16_t* src, int src_size) {
            for (int i = 0; i < count && i < src_size; ++i)
                out[i] = src[i];
        };

        if (addr == static_cast<int>(protocol::kBms2RegStatusBase)) {
            if (status_fail) {
                injected_error = hal::ModbusResult::TIMEOUT;
                return -1;
            }
            fill(status_regs, static_cast<int>(protocol::kBms2StatusBlockLen));
            injected_error = hal::ModbusResult::OK;
            return count;
        }
        if (addr == static_cast<int>(protocol::kBms2RegTempBase)) {
            if (temp_fail) {
                injected_error = hal::ModbusResult::TIMEOUT;
                return -1;
            }
            fill(temp_regs, static_cast<int>(protocol::kBms2MaxNtc));
            injected_error = hal::ModbusResult::OK;
            return count;
        }
        if (addr == static_cast<int>(protocol::kBms2RegAlarmBase)) {
            if (alarm_fail) {
                injected_error = hal::ModbusResult::TIMEOUT;
                return -1;
            }
            fill(alarm_regs, static_cast<int>(protocol::kBms2AlarmBlockLen));
            injected_error = hal::ModbusResult::OK;
            return count;
        }
        if (addr == static_cast<int>(protocol::kBms2RegCellBase)) {
            if (cell_fail) {
                injected_error = hal::ModbusResult::TIMEOUT;
                return -1;
            }
            fill(cell_regs, static_cast<int>(protocol::kBms2MaxCells));
            injected_error = hal::ModbusResult::OK;
            return count;
        }
        // 未知地址：返回 0
        std::memset(out, 0, static_cast<size_t>(count) * sizeof(uint16_t));
        return count;
    }
};

/** 用 Bms2StatusParams 初始化 BmsMockModbus2 的主状态块 */
static void setup_status(BmsMockModbus2& mock, const Bms2StatusParams& p = {}) {
    fill_status_regs(mock.status_regs, p);
}

/** 向 mock 温度块填入若干 NTC 原始值（raw = temp + 40） */
static void setup_temps(BmsMockModbus2& mock, std::initializer_list<uint16_t> raws) {
    std::memset(mock.temp_regs, 0, sizeof(mock.temp_regs));
    int i = 0;
    for (auto raw : raws)
        mock.temp_regs[i++] = raw;
}

/** 向 mock 电芯块填入若干节电芯电压（mV） */
static void setup_cells(BmsMockModbus2& mock, std::initializer_list<uint16_t> mv_list) {
    std::memset(mock.cell_regs, 0, sizeof(mock.cell_regs));
    int i = 0;
    for (auto mv : mv_list)
        mock.cell_regs[i++] = mv;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 1: Bms2Protocol::decode_basic_info 解码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("Bms2Protocol decode_basic_info 正常解码", "[protocol][bms2]") {
    uint16_t regs[protocol::kBms2StatusBlockLen];
    fill_status_regs(regs);  // 默认参数

    auto result = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
    REQUIRE(result.has_value());
    const auto& info = *result;

    SECTION("总电压：raw × 0.1V = 48.0V") {
        CHECK(info.total_voltage_v == Approx(48.0f).margin(0.01f));
    }

    SECTION("电流：raw=30000 → 0A") {
        CHECK(info.current_a == Approx(0.0f).margin(0.01f));
    }

    SECTION("SOC：800 ÷ 10 = 80.0%") {
        CHECK(info.soc_pct == Approx(80.0f).margin(0.01f));
    }

    SECTION("剩余容量：1000 × 0.1 = 100.0Ah") {
        CHECK(info.remaining_ah == Approx(100.0f).margin(0.01f));
    }

    SECTION("功率与能量") {
        CHECK(info.power_w == Approx(960.0f).margin(0.5f));
        CHECK(info.energy_wh == Approx(10000.0f).margin(0.5f));
    }

    SECTION("心跳包与循环次数") {
        CHECK(info.life_counter == 42u);
        CHECK(info.cycle_count == 37u);
    }

    SECTION("节数与 NTC 数") {
        CHECK(info.cell_count == 16u);
        CHECK(info.ntc_count == 3u);
    }

    SECTION("单体电压统计") {
        CHECK(info.cell_max_mv == 3650u);
        CHECK(info.cell_max_idx == 3u);
        CHECK(info.cell_min_mv == 3580u);
        CHECK(info.cell_min_idx == 12u);
        CHECK(info.cell_delta_mv == 70u);
        CHECK(info.avg_cell_mv == 3620u);
    }

    SECTION("温度统计：raw−40=℃") {
        CHECK(info.temp_max_c == Approx(25.0f).margin(0.01f));  // 65-40
        CHECK(info.temp_min_c == Approx(20.0f).margin(0.01f));  // 60-40
        CHECK(info.temp_delta_c == Approx(5.0f).margin(0.01f));
        CHECK(info.mos_temp_c == Approx(30.0f).margin(0.01f));   // 70-40
        CHECK(info.env_temp_c == Approx(25.0f).margin(0.01f));   // 65-40
        CHECK(info.heat_temp_c == Approx(20.0f).margin(0.01f));  // 60-40
    }

    SECTION("充电状态：0 → Idle") {
        CHECK(info.charge_status == protocol::Bms2ChargeStatus::Idle);
    }

    SECTION("MOS 状态") {
        CHECK(info.charge_mos_on);
        CHECK(info.discharge_mos_on);
        CHECK_FALSE(info.precharge_mos_on);
        CHECK_FALSE(info.heat_mos_on);
        CHECK_FALSE(info.fan_mos_on);
    }

    SECTION("RTC：2024-03-22 10:30:45") {
        // rtc_ym=0x1803：high=0x18=24→2024，low=0x03=3
        // rtc_dh=0x160A：high=0x16=22，low=0x0A=10
        // rtc_ms=0x1E2D：high=0x1E=30，low=0x2D=45
        CHECK(info.rtc.year == 2024u);
        CHECK(info.rtc.month == 3u);
        CHECK(info.rtc.day == 22u);
        CHECK(info.rtc.hour == 10u);
        CHECK(info.rtc.minute == 30u);
        CHECK(info.rtc.second == 45u);
    }

    SECTION("剩余时间与 DI/DO") {
        CHECK(info.remaining_time_min == 120u);
        CHECK(info.di_do_status == 0u);
    }
}

TEST_CASE("Bms2Protocol decode_basic_info 充电电流解码", "[protocol][bms2]") {
    // raw=29500 → (29500-30000)×0.1 = -50.0A（充电，负值）
    uint16_t regs[protocol::kBms2StatusBlockLen];
    Bms2StatusParams p;
    p.current_raw = 29500u;
    fill_status_regs(regs, p);

    auto result = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
    REQUIRE(result.has_value());
    CHECK(result->current_a == Approx(-50.0f).margin(0.01f));
}

TEST_CASE("Bms2Protocol decode_basic_info 放电电流解码", "[protocol][bms2]") {
    // raw=30200 → (30200-30000)×0.1 = +20.0A（放电，正值）
    uint16_t regs[protocol::kBms2StatusBlockLen];
    Bms2StatusParams p;
    p.current_raw = 30200u;
    fill_status_regs(regs, p);

    auto result = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
    REQUIRE(result.has_value());
    CHECK(result->current_a == Approx(20.0f).margin(0.01f));
}

TEST_CASE("Bms2Protocol decode_basic_info 充电状态枚举", "[protocol][bms2]") {
    uint16_t regs[protocol::kBms2StatusBlockLen];

    SECTION("0 → Idle") {
        Bms2StatusParams p;
        p.charge_status = 0u;
        fill_status_regs(regs, p);
        auto r = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
        REQUIRE(r.has_value());
        CHECK(r->charge_status == protocol::Bms2ChargeStatus::Idle);
    }

    SECTION("1 → Charging") {
        Bms2StatusParams p;
        p.charge_status = 1u;
        fill_status_regs(regs, p);
        auto r = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
        REQUIRE(r.has_value());
        CHECK(r->charge_status == protocol::Bms2ChargeStatus::Charging);
    }

    SECTION("2 → Discharging") {
        Bms2StatusParams p;
        p.charge_status = 2u;
        fill_status_regs(regs, p);
        auto r = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
        REQUIRE(r.has_value());
        CHECK(r->charge_status == protocol::Bms2ChargeStatus::Discharging);
    }
}

TEST_CASE("Bms2Protocol decode_basic_info 均衡位图拼接", "[protocol][bms2]") {
    // 电芯 1（bit0 of bits0）+ 电芯 17（bit0 of bits1）+ 电芯 33（bit0 of bits2）
    uint16_t regs[protocol::kBms2StatusBlockLen];
    Bms2StatusParams p;
    p.balance_bits0 = 0x0001u;  // 电芯 1 均衡中
    p.balance_bits1 = 0x0001u;  // 电芯 17 均衡中
    p.balance_bits2 = 0x0001u;  // 电芯 33 均衡中
    fill_status_regs(regs, p);

    auto r = protocol::Bms2Protocol::decode_basic_info(regs, protocol::kBms2StatusBlockLen);
    REQUIRE(r.has_value());

    // balance_bitmap bit0 = cell1, bit16 = cell17, bit32 = cell33
    CHECK((r->balance_bitmap & (1ULL << 0u)) != 0u);
    CHECK((r->balance_bitmap & (1ULL << 16u)) != 0u);
    CHECK((r->balance_bitmap & (1ULL << 32u)) != 0u);
    CHECK((r->balance_bitmap & (1ULL << 1u)) == 0u);
}

TEST_CASE("Bms2Protocol decode_basic_info 数量不足返回 nullopt", "[protocol][bms2]") {
    uint16_t regs[protocol::kBms2StatusBlockLen]{};

    SECTION("count=0") {
        auto r = protocol::Bms2Protocol::decode_basic_info(regs, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("count < kBms2StatusBlockLen（差 1 个）") {
        auto r = protocol::Bms2Protocol::decode_basic_info(
            regs, static_cast<uint16_t>(protocol::kBms2StatusBlockLen - 1));
        CHECK_FALSE(r.has_value());
    }

    SECTION("nullptr 指针") {
        auto r = protocol::Bms2Protocol::decode_basic_info(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 2: Bms2Protocol::decode_cell_voltages 解码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("Bms2Protocol decode_cell_voltages 正常解码", "[protocol][bms2]") {
    // 4 节：3600 / 3650 / 3580 / 3630 mV；1 LSB = 1mV
    const uint16_t cells[] = {3600u, 3650u, 3580u, 3630u};
    auto r = protocol::Bms2Protocol::decode_cell_voltages(cells, 4u);

    REQUIRE(r.has_value());
    CHECK(r->count == 4u);
    CHECK(r->mv[0] == 3600u);
    CHECK(r->mv[1] == 3650u);
    CHECK(r->mv[2] == 3580u);
    CHECK(r->mv[3] == 3630u);
}

TEST_CASE("Bms2Protocol decode_cell_voltages 边界条件", "[protocol][bms2]") {
    SECTION("count=0 → nullopt") {
        auto r = protocol::Bms2Protocol::decode_cell_voltages(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("count > kBms2MaxCells(48) → nullopt") {
        const uint16_t cells[50]{};
        auto r = protocol::Bms2Protocol::decode_cell_voltages(
            cells, static_cast<uint16_t>(protocol::kBms2MaxCells + 1u));
        CHECK_FALSE(r.has_value());
    }

    SECTION("count=1（最小合法）") {
        const uint16_t cells[] = {3700u};
        auto r = protocol::Bms2Protocol::decode_cell_voltages(cells, 1u);
        REQUIRE(r.has_value());
        CHECK(r->count == 1u);
        CHECK(r->mv[0] == 3700u);
    }

    SECTION("count=kBms2MaxCells（最大合法）") {
        uint16_t cells[protocol::kBms2MaxCells];
        for (uint16_t i = 0; i < protocol::kBms2MaxCells; ++i)
            cells[i] = static_cast<uint16_t>(3600u + i);

        auto r = protocol::Bms2Protocol::decode_cell_voltages(cells, protocol::kBms2MaxCells);
        REQUIRE(r.has_value());
        CHECK(r->count == protocol::kBms2MaxCells);
        CHECK(r->mv[47] == static_cast<uint16_t>(3600u + 47u));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 3: Bms2Protocol::decode_temperatures 解码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("Bms2Protocol decode_temperatures 正常解码", "[protocol][bms2]") {
    // 3 个探头：25°C=raw65，30°C=raw70，-10°C=raw30
    const uint16_t raws[] = {65u, 70u, 30u};
    auto r = protocol::Bms2Protocol::decode_temperatures(raws, 3u);

    REQUIRE(r.has_value());
    CHECK(r->count == 3u);
    CHECK(r->temp_c[0] == Approx(25.0f).margin(0.01f));
    CHECK(r->temp_c[1] == Approx(30.0f).margin(0.01f));
    CHECK(r->temp_c[2] == Approx(-10.0f).margin(0.01f));
}

TEST_CASE("Bms2Protocol decode_temperatures 边界条件", "[protocol][bms2]") {
    SECTION("count=0 → nullopt") {
        auto r = protocol::Bms2Protocol::decode_temperatures(nullptr, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("count > kBms2MaxNtc(8) → nullopt") {
        const uint16_t raws[10]{};
        auto r = protocol::Bms2Protocol::decode_temperatures(
            raws, static_cast<uint16_t>(protocol::kBms2MaxNtc + 1u));
        CHECK_FALSE(r.has_value());
    }

    SECTION("count=kBms2MaxNtc（最大合法）") {
        uint16_t raws[protocol::kBms2MaxNtc];
        for (uint16_t i = 0; i < protocol::kBms2MaxNtc; ++i)
            raws[i] = static_cast<uint16_t>(65u + i);

        auto r = protocol::Bms2Protocol::decode_temperatures(raws, protocol::kBms2MaxNtc);
        REQUIRE(r.has_value());
        CHECK(r->count == protocol::kBms2MaxNtc);
        CHECK(r->temp_c[0] == Approx(25.0f).margin(0.01f));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Part 4: Bms2Protocol::decode_alarm_flags 解码测试
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("Bms2Protocol decode_alarm_flags 全零寄存器 - 无告警", "[protocol][bms2]") {
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);

    REQUIRE(r.has_value());
    CHECK(r->cell_overvolt_level == 0u);
    CHECK(r->cell_undervolt_level == 0u);
    CHECK_FALSE(r->short_circuit);
    CHECK_FALSE(r->afe_chip_fault);
    CHECK_FALSE(r->eeprom_fault);
    CHECK_FALSE(r->heat_fault);
}

TEST_CASE("Bms2Protocol decode_alarm_flags 单体过压告警等级", "[protocol][bms2]") {
    // reg[0] 低字节 bit[2:0]=cell_overvolt，bit[5:3]=cell_undervolt
    // 设 cell_overvolt_level=3：lo byte = 0x03 → reg[0] = 0x0003
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[0] = 0x0003u;  // cell_overvolt_level=3（高危）

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->cell_overvolt_level == 3u);
    CHECK(r->cell_undervolt_level == 0u);
    CHECK_FALSE(r->smart_charger_ok);
}

TEST_CASE("Bms2Protocol decode_alarm_flags 单体欠压 + 充电器连接", "[protocol][bms2]") {
    // reg[0] 低字节 bit[5:3]=cell_undervolt=2，bit6=smart_charger_ok
    // lo byte = (2u<<3) | (1u<<6) = 0x10 | 0x40 = 0x50
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[0] = 0x0050u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->cell_undervolt_level == 2u);
    CHECK(r->smart_charger_ok);
    CHECK(r->cell_overvolt_level == 0u);
}

TEST_CASE("Bms2Protocol decode_alarm_flags 短路告警", "[protocol][bms2]") {
    // reg[2] 低字节 bit6=short_circuit → lo=0x40 → reg[2]=0x0040
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[2] = 0x0040u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->short_circuit);
    CHECK(r->bat_overvolt_level == 0u);
}

TEST_CASE("Bms2Protocol decode_alarm_flags 放电过流告警等级", "[protocol][bms2]") {
    // reg[2] 高字节 bit[5:3]=dsg_overcurr=5 → hi byte=(5<<3)=0x28 → reg[2]=0x2800
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[2] = 0x2800u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->dsg_overcurr_level == 5u);
    CHECK(r->chg_overcurr_level == 0u);
}

TEST_CASE("Bms2Protocol decode_alarm_flags AFE 硬件故障", "[protocol][bms2]") {
    // reg[5] 高字节（hi）：bit0=afe_chip_fault，bit1=afe_comm_fault
    // hi=0x03 → reg[5]=0x0300
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[5] = 0x0300u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->afe_chip_fault);
    CHECK(r->afe_comm_fault);
    CHECK_FALSE(r->afe_sample_fault);
}

TEST_CASE("Bms2Protocol decode_alarm_flags EEPROM + MOS 故障", "[protocol][bms2]") {
    // reg[6] 低字节：bit1=eeprom_fault，bit4=chg_mos_fault
    // lo=0x12 → reg[6]=0x0012（高字节0）
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[6] = 0x0012u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->eeprom_fault);
    CHECK_FALSE(r->flash_fault);
    CHECK(r->chg_mos_fault);
    CHECK_FALSE(r->dsg_mos_fault);
}

TEST_CASE("Bms2Protocol decode_alarm_flags MOS 控制状态 + 加热工作", "[protocol][bms2]") {
    // reg[6] 高字节：bit0=comm_chg_mos_off，bit5=heat_active，bit6=current_limit_active
    // hi=0x61 → reg[6]=0x6100（高字节在高位）
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};
    regs[6] = 0x6100u;

    auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, protocol::kBms2AlarmBlockLen);
    REQUIRE(r.has_value());
    CHECK(r->comm_chg_mos_off);
    CHECK(r->heat_active);
    CHECK(r->current_limit_active);
    CHECK_FALSE(r->comm_dsg_mos_off);
}

TEST_CASE("Bms2Protocol decode_alarm_flags 数量不足返回 nullopt", "[protocol][bms2]") {
    uint16_t regs[protocol::kBms2AlarmBlockLen]{};

    SECTION("count=0") {
        auto r = protocol::Bms2Protocol::decode_alarm_flags(regs, 0u);
        CHECK_FALSE(r.has_value());
    }

    SECTION("count < kBms2AlarmBlockLen（差 1 个）") {
        auto r = protocol::Bms2Protocol::decode_alarm_flags(
            regs, static_cast<uint16_t>(protocol::kBms2AlarmBlockLen - 1));
        CHECK_FALSE(r.has_value());
    }
}

