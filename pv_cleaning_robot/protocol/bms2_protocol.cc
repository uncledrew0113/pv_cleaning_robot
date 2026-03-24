#include <algorithm>

#include "pv_cleaning_robot/protocol/bms2_protocol.h"

namespace robot::protocol {

// == 解码：寄存器值 → 物理量 ===================================================

std::optional<Bms2BasicInfo> Bms2Protocol::decode_basic_info(const uint16_t* regs, uint16_t count) {
    if (count < kBms2StatusBlockLen)
        return std::nullopt;

    Bms2BasicInfo info{};

    // -- 电气量 ----------------------------------------------------------------
    // 总电压：1 LSB = 0.1V
    info.total_voltage_v = static_cast<float>(regs[bms2_off::kTotalVoltage]) * 0.1f;

    // 电流：Offset 30000，1 LSB = 0.1A，正值 = 放电，负值 = 充电
    info.current_a = raw_to_current(regs[bms2_off::kCurrent]);

    // SOC：0~1000 对应 0.0%~100.0%，1 LSB = 0.1%
    info.soc_pct = raw_to_soc(regs[bms2_off::kSoc]);

    // 剩余容量：1 LSB = 0.1Ah
    info.remaining_ah = static_cast<float>(regs[bms2_off::kRemainCap]) * 0.1f;

    // 功率、能量：1 LSB = 1W / 1Wh
    info.power_w = static_cast<float>(regs[bms2_off::kPowerW]);
    info.energy_wh = static_cast<float>(regs[bms2_off::kEnergyWh]);

    // -- 计数与节数 -------------------------------------------------------------
    info.life_counter = regs[bms2_off::kLifeCounter];  // 0x3B 心跳包
    info.cycle_count = regs[bms2_off::kCycleCount];    // 0x4C 循环次数
    info.cell_count =
        static_cast<uint8_t>(std::min<uint16_t>(regs[bms2_off::kCellCount], kBms2MaxCells));
    info.ntc_count =
        static_cast<uint8_t>(std::min<uint16_t>(regs[bms2_off::kNtcCount], kBms2MaxNtc));

    // -- 单体电压统计 -----------------------------------------------------------
    info.cell_max_mv = regs[bms2_off::kCellMaxMv];
    info.cell_max_idx = static_cast<uint8_t>(regs[bms2_off::kCellMaxIdx]);
    info.cell_min_mv = regs[bms2_off::kCellMinMv];
    info.cell_min_idx = static_cast<uint8_t>(regs[bms2_off::kCellMinIdx]);
    info.cell_delta_mv = regs[bms2_off::kCellDelta];
    info.avg_cell_mv = regs[bms2_off::kAvgCellMv];

    // -- 温度统计 ---------------------------------------------------------------
    // 所有温度寄存器编码：实际温度 = raw - 40（offset -40 补偿低温）
    info.temp_max_c = raw_to_temp(regs[bms2_off::kTempMax]);
    info.temp_max_idx = static_cast<uint8_t>(regs[bms2_off::kTempMaxIdx]);
    info.temp_min_c = raw_to_temp(regs[bms2_off::kTempMin]);
    info.temp_min_idx = static_cast<uint8_t>(regs[bms2_off::kTempMinIdx]);
    info.temp_delta_c = static_cast<float>(regs[bms2_off::kTempDelta]);
    info.mos_temp_c = raw_to_temp(regs[bms2_off::kMosTemp]);
    info.env_temp_c = raw_to_temp(regs[bms2_off::kEnvTemp]);
    info.heat_temp_c = raw_to_temp(regs[bms2_off::kHeatTemp]);  // 0x5C 加热温度

    // -- 充放电状态 -------------------------------------------------------------
    info.charge_status = static_cast<Bms2ChargeStatus>(regs[bms2_off::kChargeStatus] & 0x03u);
    info.charger_present = (regs[bms2_off::kChargerDetect] != 0u);
    info.load_present = (regs[bms2_off::kLoadDetect] != 0u);

    // -- MOS 与加热状态 ---------------------------------------------------------
    info.charge_mos_on = (regs[bms2_off::kChargeMos] != 0u);
    info.discharge_mos_on = (regs[bms2_off::kDischargeMos] != 0u);
    info.precharge_mos_on = (regs[bms2_off::kPreChargeMos] != 0u);
    info.heat_mos_on = (regs[bms2_off::kHeatMos] != 0u);
    info.fan_mos_on = (regs[bms2_off::kFanMos] != 0u);  // 0x56 风扇 MOS
    info.heat_current_a = static_cast<float>(regs[bms2_off::kHeatCurrent]);
    // 0x5F 限流状态
    info.current_limit_on = (regs[bms2_off::kCurrentLimitStatus] != 0u);
    // 0x60 限流电流：同主电流编码，Offset 30000，0.1A/bit
    info.current_limit_a = raw_to_current(regs[bms2_off::kCurrentLimitCurrent]);

    // -- RTC (0x61~0x63) -------------------------------------------------------
    // 格式：0x61 高字节=年(偏置+2000), 低字节=月
    //      0x62 高字节=日,     低字节=时
    //      0x63 高字节=分,     低字节=秒
    // 示例：2020-08-15 08:30:56 → {0x1408, 0x0F08, 0x1E38}
    {
        uint16_t ym = regs[bms2_off::kRtcYearMonth];
        uint16_t dh = regs[bms2_off::kRtcDayHour];
        uint16_t ms = regs[bms2_off::kRtcMinSec];
        info.rtc.year = static_cast<uint16_t>(2000u + (ym >> 8));
        info.rtc.month = static_cast<uint8_t>(ym & 0xFFu);
        info.rtc.day = static_cast<uint8_t>(dh >> 8);
        info.rtc.hour = static_cast<uint8_t>(dh & 0xFFu);
        info.rtc.minute = static_cast<uint8_t>(ms >> 8);
        info.rtc.second = static_cast<uint8_t>(ms & 0xFFu);
    }

    info.remaining_time_min = regs[bms2_off::kRemainingTime];  // 0x64 剩余时间
    info.di_do_status = regs[bms2_off::kDiDoStatus];           // 0x65 DI/DO 状态

    // -- 均衡状态 ---------------------------------------------------------------
    info.balance_status = static_cast<uint8_t>(regs[bms2_off::kBalanceStatus] & 0x03u);
    // 三个均衡位寄存器各覆盖 16 节，拼成 48-bit 位图
    // bit0 = 电芯1，bit1 = 电芯2，…，bit47 = 电芯48
    info.balance_bitmap = (static_cast<uint64_t>(regs[bms2_off::kBalanceBits0])) |
                          (static_cast<uint64_t>(regs[bms2_off::kBalanceBits1]) << 16u) |
                          (static_cast<uint64_t>(regs[bms2_off::kBalanceBits2]) << 32u);

    return info;
}

std::optional<Bms2CellVoltages> Bms2Protocol::decode_cell_voltages(const uint16_t* regs,
                                                                   uint16_t count) {
    if (count == 0u || count > kBms2MaxCells)
        return std::nullopt;

    Bms2CellVoltages cells{};
    cells.count = static_cast<uint8_t>(count);
    // 每个寄存器值即为该节电芯电压（mV），1 LSB = 1 mV
    for (uint8_t i = 0; i < cells.count; ++i) {
        cells.mv[i] = regs[i];
    }
    return cells;
}

std::optional<Bms2Temperatures> Bms2Protocol::decode_temperatures(const uint16_t* regs,
                                                                  uint16_t count) {
    if (count == 0u || count > kBms2MaxNtc)
        return std::nullopt;

    Bms2Temperatures temps{};
    temps.count = static_cast<uint8_t>(count);
    // 温度编码：实际温度（℃）= 寄存器值 - 40
    for (uint8_t i = 0; i < temps.count; ++i) {
        temps.temp_c[i] = raw_to_temp(regs[i]);
    }
    return temps;
}

// == 解码：告警 / 故障状态块 ====================================================

std::optional<Bms2AlarmFlags>
Bms2Protocol::decode_alarm_flags(const uint16_t* regs, uint16_t count)
{
    if (count < kBms2AlarmBlockLen) return std::nullopt;

    Bms2AlarmFlags f{};

    // 辅助：提取低字节 / 高字节
    auto lo = [&](int i) -> uint8_t { return static_cast<uint8_t>(regs[i] & 0xFFu); };
    auto hi = [&](int i) -> uint8_t { return static_cast<uint8_t>(regs[i] >> 8); };
    // 辅助：从字节中提取 3-bit 告警等级
    auto l3 = [](uint8_t b) -> uint8_t { return b & 0x07u; };           // bits[2:0]
    auto h3 = [](uint8_t b) -> uint8_t { return (b >> 3u) & 0x07u; };  // bits[5:3]

    // -- 0x6D（index 0）：单体电压告警 & 智能设备连接 --------------------------
    f.cell_overvolt_level  = l3(lo(0));  f.cell_undervolt_level = h3(lo(0));
    f.smart_charger_ok     = (lo(0) >> 6u) & 1u;
    f.smart_charger_fail   = (lo(0) >> 7u) & 1u;
    f.cell_delta_level     = l3(hi(0));  f.chg_overtemp_level   = h3(hi(0));
    f.smart_discharger_ok  = (hi(0) >> 6u) & 1u;
    f.smart_discharger_fail= (hi(0) >> 7u) & 1u;

    // -- 0x6E（index 1）：充放电温度告警 & MOS温度故障 -------------------------
    f.chg_lowtemp_level    = l3(lo(1));  f.dsg_overtemp_level   = h3(lo(1));
    f.chg_mos_overtemp     = (lo(1) >> 6u) & 1u;
    f.chg_mos_temp_fault   = (lo(1) >> 7u) & 1u;
    f.dsg_lowtemp_level    = l3(hi(1));  f.temp_delta_level     = h3(hi(1));
    f.dsg_mos_overtemp     = (hi(1) >> 6u) & 1u;
    f.dsg_mos_temp_fault   = (hi(1) >> 7u) & 1u;

    // -- 0x6F（index 2）：总压告警 & 短路 & 过流 & 禁止充放电 -------------------
    f.bat_overvolt_level   = l3(lo(2));  f.bat_undervolt_level  = h3(lo(2));
    f.short_circuit        = (lo(2) >> 6u) & 1u;
    f.chg_overcurr_level   = l3(hi(2));  f.dsg_overcurr_level   = h3(hi(2));
    f.undervolt_chg_inhibit= (hi(2) >> 6u) & 1u;
    f.overvolt_dsg_inhibit = (hi(2) >> 7u) & 1u;

    // -- 0x70（index 3）：SOC/SOH告警 & 并联通信 & MOS过温 & 热失控 ------------
    f.soc_low_level        = l3(lo(3));  f.soh_low_level        = h3(lo(3));
    f.parallel_comm_ok     = (lo(3) >> 6u) & 1u;
    f.parallel_comm_fail   = (lo(3) >> 7u) & 1u;
    f.mos_overtemp_level   = l3(hi(3));  f.thermal_runaway_level= h3(hi(3));

    // -- 0x71（index 4）：全部预留，跳过 ---------------------------------------

    // -- 0x72（index 5）：AFE / 采样 / 检测故障（高字节）-----------------------
    {
        uint8_t h = hi(5);
        f.afe_chip_fault      = (h >> 0u) & 1u;  f.afe_comm_fault    = (h >> 1u) & 1u;
        f.afe_sample_fault    = (h >> 2u) & 1u;  f.volt_detect_fault = (h >> 3u) & 1u;
        f.volt_wire_off       = (h >> 4u) & 1u;  f.total_volt_fault  = (h >> 5u) & 1u;
        f.curr_detect_fault   = (h >> 6u) & 1u;  f.temp_detect_fault = (h >> 7u) & 1u;
    }

    // -- 0x73（index 6）：低字节=硬件故障 / 高字节=控制状态 --------------------
    {
        uint8_t l = lo(6);
        f.temp_wire_off      = (l >> 0u) & 1u;  f.eeprom_fault       = (l >> 1u) & 1u;
        f.flash_fault        = (l >> 2u) & 1u;  f.rtc_fault          = (l >> 3u) & 1u;
        f.chg_mos_fault      = (l >> 4u) & 1u;  f.dsg_mos_fault      = (l >> 5u) & 1u;
        f.pre_chg_mos_fault  = (l >> 6u) & 1u;  f.precharge_fail     = (l >> 7u) & 1u;
    }
    {
        uint8_t h = hi(6);
        f.comm_chg_mos_off     = (h >> 0u) & 1u;  f.comm_dsg_mos_off  = (h >> 1u) & 1u;
        f.sw_chg_mos_off       = (h >> 2u) & 1u;  f.sw_dsg_mos_off    = (h >> 3u) & 1u;
        f.fan_active           = (h >> 4u) & 1u;  f.heat_active        = (h >> 5u) & 1u;
        f.current_limit_active = (h >> 6u) & 1u;  f.heat_fault         = (h >> 7u) & 1u;
    }

    return f;
}

}  // namespace robot::protocol
