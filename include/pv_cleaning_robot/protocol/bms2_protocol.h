#pragma once
#include <cstdint>
#include <optional>

namespace robot::protocol {

// == 容量限制 =================================================================
static constexpr uint8_t kBms2MaxCells = 48;  ///< 最大串联节数（寄存器 0x00~0x2F）
static constexpr uint8_t kBms2MaxNtc = 8;     ///< 最大温度探头数（寄存器 0x30~0x37）

// == Modbus 从机地址 ===========================================================
/// 协议文档标注：请求帧地址 0x81（十进制 129），响应帧地址 0x51（十进制 81）。
/// 注意：部分文档将十进制 "81" 误写为十六进制 "0x81"；实际通信时
/// 从机 ID 应以所用设备的出厂配置为准，默认值取响应帧中的 0x51。
static constexpr uint8_t kBms2DefaultSlaveId = 0x01;

// == 寄存器地址常量 ============================================================

/// @defgroup reg_cells 电芯电压块（0x00 ~ 0x2F）
/// 每个寄存器对应一节电芯电压，单位 mV（1 LSB = 1 mV）
static constexpr uint16_t kBms2RegCellBase = 0x0000u;   ///< 起始地址
static constexpr uint16_t kBms2CellBlockLen = 0x0030u;  ///< 48 个寄存器

/// @defgroup reg_temps 温度传感器块（0x30 ~ 0x37）
/// 单位 1 = 1℃+40；实际温度 = 寄存器值 - 40
static constexpr uint16_t kBms2RegTempBase = 0x0030u;   ///< 起始地址
static constexpr uint16_t kBms2TempBlockLen = 0x0008u;  ///< 8 个寄存器

/// @defgroup reg_status 主状态块（0x38 ~ 0x65，46 个寄存器）
static constexpr uint16_t kBms2RegStatusBase = 0x0038u;   ///< 起始地址
static constexpr uint16_t kBms2StatusBlockLen = 0x002Eu;  ///< 46 个寄存器（0x38~0x65）

/// 唤醒源寄存器（独立读取，与主状态块不连续）
static constexpr uint16_t kBms2RegWakeupSource = 0x006Bu;

/// 唤醒源各 bit 含义（kBms2RegWakeupSource 的位掩码）
static constexpr uint16_t kBms2WakeupKeySwitch = (1u << 0u);  ///< bit0：钥匙唤醒
static constexpr uint16_t kBms2WakeupButton = (1u << 1u);     ///< bit1：按键唤醒
static constexpr uint16_t kBms2Wakeup485 = (1u << 2u);        ///< bit2：485 唤醒
static constexpr uint16_t kBms2WakeupCan = (1u << 3u);        ///< bit3：CAN 唤醒
static constexpr uint16_t kBms2WakeupCurrent = (1u << 4u);    ///< bit4：电流唤醒

/// 新功能 / 告警 / 故障状态块（0x6D ~ 0x73，7 个寄存器）
static constexpr uint16_t kBms2RegAlarmBase = 0x006Du;
static constexpr uint16_t kBms2AlarmBlockLen = 0x0007u;

/// 通信接口寄存器（1=485，2=UART）
static constexpr uint16_t kBms2RegCommInterface = 0x007Eu;

/// 主状态块内各字段偏移（相对于 kBms2RegStatusBase）
namespace bms2_off {
static constexpr uint16_t kTotalVoltage = 0x00u;  ///< 0x38 总电压，0.1V/bit
static constexpr uint16_t kCurrent = 0x01u;  ///< 0x39 电流，0.1A/bit，Offset 30000，>30000=放电
static constexpr uint16_t kSoc = 0x02u;      ///< 0x3A SOC，0-1000，÷10 = %
static constexpr uint16_t kLifeCounter = 0x03u;  ///< 0x3B 心跳包（LIFE，每秒递增）
static constexpr uint16_t kCellCount = 0x04u;    ///< 0x3C 串联节数
static constexpr uint16_t kNtcCount = 0x05u;     ///< 0x3D 温度传感器数量
static constexpr uint16_t kCellMaxMv = 0x06u;    ///< 0x3E 最高单体电压，mV
static constexpr uint16_t kCellMaxIdx = 0x07u;   ///< 0x3F 最高单体序号（1-based）
static constexpr uint16_t kCellMinMv = 0x08u;    ///< 0x40 最低单体电压，mV
static constexpr uint16_t kCellMinIdx = 0x09u;   ///< 0x41 最低单体序号（1-based）
static constexpr uint16_t kCellDelta = 0x0Au;    ///< 0x42 单体压差，mV
static constexpr uint16_t kTempMax = 0x0Bu;      ///< 0x43 最高温度，raw-40=℃
static constexpr uint16_t kTempMaxIdx = 0x0Cu;   ///< 0x44 最高温度探头序号
static constexpr uint16_t kTempMin = 0x0Du;      ///< 0x45 最低温度，raw-40=℃
static constexpr uint16_t kTempMinIdx = 0x0Eu;   ///< 0x46 最低温度探头序号
static constexpr uint16_t kTempDelta = 0x0Fu;    ///< 0x47 温差，℃
static constexpr uint16_t kChargeStatus = 0x10u;  ///< 0x48 充电状态（0=停止 1=充电 2=放电）
static constexpr uint16_t kChargerDetect = 0x11u;  ///< 0x49 充电器检测（0=无 1=有）
static constexpr uint16_t kLoadDetect = 0x12u;     ///< 0x4A 负载检测（0=无 1=有）
static constexpr uint16_t kRemainCap = 0x13u;      ///< 0x4B 剩余容量，0.1Ah/bit
static constexpr uint16_t kCycleCount = 0x14u;     ///< 0x4C 循环次数
static constexpr uint16_t kBalanceStatus = 0x15u;  ///< 0x4D 均衡状态（0=关 1=充电均衡 2=放电均衡）
// 0x4E（偏移 0x16）：未定义，跳过
static constexpr uint16_t kBalanceBits0 = 0x17u;  ///< 0x4F 均衡状态位 bit0-15（电芯 1-16）
static constexpr uint16_t kBalanceBits1 = 0x18u;  ///< 0x50 均衡状态位 bit0-15（电芯 17-32）
static constexpr uint16_t kBalanceBits2 = 0x19u;  ///< 0x51 均衡状态位 bit0-15（电芯 33-48）
static constexpr uint16_t kChargeMos = 0x1Au;     ///< 0x52 充电 MOS 状态（0=关 1=开）
static constexpr uint16_t kDischargeMos = 0x1Bu;  ///< 0x53 放电 MOS 状态
static constexpr uint16_t kPreChargeMos = 0x1Cu;  ///< 0x54 预充电 MOS 状态
static constexpr uint16_t kHeatMos = 0x1Du;       ///< 0x55 加热 MOS 状态
static constexpr uint16_t kFanMos = 0x1Eu;        ///< 0x56 风扇 MOS 状态
static constexpr uint16_t kAvgCellMv = 0x1Fu;     ///< 0x57 平均单体电压，mV
static constexpr uint16_t kPowerW = 0x20u;        ///< 0x58 功率，W
static constexpr uint16_t kEnergyWh = 0x21u;      ///< 0x59 能量，Wh
static constexpr uint16_t kMosTemp = 0x22u;       ///< 0x5A MOS 温度，raw-40=℃
static constexpr uint16_t kEnvTemp = 0x23u;       ///< 0x5B 环境温度，raw-40=℃
static constexpr uint16_t kHeatTemp = 0x24u;      ///< 0x5C 加热温度，raw-40=℃
static constexpr uint16_t kHeatCurrent = 0x25u;   ///< 0x5D 加热电流，1A/bit
// 0x5E（偏移 0x26）：未定义，跳过
static constexpr uint16_t kCurrentLimitStatus = 0x27u;  ///< 0x5F 限流状态（1=启用 0=关闭）
static constexpr uint16_t kCurrentLimitCurrent =
    0x28u;  ///< 0x60 限流电流，0.1A/bit，Offset 30000（同主电流编码）
// RTC（0x61~0x63）：每个寄存器高字节在前、低字节在后
static constexpr uint16_t kRtcYearMonth = 0x29u;   ///< 0x61 高字节=年(+2000)，低字节=月
static constexpr uint16_t kRtcDayHour = 0x2Au;     ///< 0x62 高字节=日，低字节=时
static constexpr uint16_t kRtcMinSec = 0x2Bu;      ///< 0x63 高字节=分，低字节=秒
static constexpr uint16_t kRemainingTime = 0x2Cu;  ///< 0x64 剩余时间，min
static constexpr uint16_t kDiDoStatus = 0x2Du;  ///< 0x65 DI/DO 状态，bit0-7=DI1-8，bit8-15=DO1-8
}  // namespace bms2_off

// == 充放电状态枚举 ============================================================
enum class Bms2ChargeStatus : uint8_t {
    Idle = 0,         ///< 待机
    Charging = 1,     ///< 充电中
    Discharging = 2,  ///< 放电中
};

// == 数据结构 =================================================================

/**
 * @brief 电芯电压数组
 *
 * 由寄存器块 0x00~0x2F 解码，每节 1mV 精度。
 * 实际有效节数由调用方读取 kBms2RegStatusBase + bms2_off::kCellCount 确定。
 */
struct Bms2CellVoltages {
    uint8_t count;               ///< 实际解码的节数
    uint16_t mv[kBms2MaxCells];  ///< 各节电压（mV）
};

/**
 * @brief 温度传感器数组
 *
 * 由寄存器块 0x30~0x37 解码，温度偏置 -40℃（实际温度 = 寄存器值 - 40）。
 * 实际有效探头数由调用方读取 kBms2RegStatusBase + bms2_off::kNtcCount 确定。
 */
struct Bms2Temperatures {
    uint8_t count;              ///< 实际解码的探头数
    float temp_c[kBms2MaxNtc];  ///< 各路温度（℃）
};

/**
 * @brief 新功能 / 告警 / 故障状态（寄存器 0x6D ~ 0x73，7 个寄存器）
 *
 * 每个寄存器的高/低字节含义独立；告警等级为 3-bit 编码（0=无，越大越严重）。
 * 使用 decode_alarm_flags() 解码，传入从 kBms2RegAlarmBase 起读取的 kBms2AlarmBlockLen 个寄存器。
 */
struct Bms2AlarmFlags {
    // === 0x6D 低字节：单体电压告警 & 智能充电器 ===
    uint8_t cell_overvolt_level;   ///< 单体过压告警等级（bit2:0）
    uint8_t cell_undervolt_level;  ///< 单体欠压告警等级（bit5:3）
    bool smart_charger_ok;         ///< 智能充电器已连接（bit6）
    bool smart_charger_fail;       ///< 智能充电器连接失败（bit7）
    // === 0x6D 高字节：压差 & 充电高温 & 智能放电设备 ===
    uint8_t cell_delta_level;    ///< 压差过大告警等级（bit2:0）
    uint8_t chg_overtemp_level;  ///< 充电高温告警等级（bit5:3）
    bool smart_discharger_ok;    ///< 智能放电设备已连接（bit6）
    bool smart_discharger_fail;  ///< 智能放电设备连接失败（bit7）

    // === 0x6E 低字节：充放电温度告警 & 充电 MOS 温度故障 ===
    uint8_t chg_lowtemp_level;   ///< 充电低温告警等级（bit2:0）
    uint8_t dsg_overtemp_level;  ///< 放电高温告警等级（bit5:3）
    bool chg_mos_overtemp;       ///< 充电 MOS 温度过高（bit6）
    bool chg_mos_temp_fault;     ///< 充电 MOS 温度检测故障（bit7）
    // === 0x6E 高字节：放电低温 & 温差 & 放电 MOS 温度故障 ===
    uint8_t dsg_lowtemp_level;  ///< 放电低温告警等级（bit2:0）
    uint8_t temp_delta_level;   ///< 温差过大告警等级（bit5:3）
    bool dsg_mos_overtemp;      ///< 放电 MOS 温度过高（bit6）
    bool dsg_mos_temp_fault;    ///< 放电 MOS 温度检测故障（bit7）

    // === 0x6F 低字节：总压告警 & 短路 ===
    uint8_t bat_overvolt_level;   ///< 总压过高告警等级（bit2:0）
    uint8_t bat_undervolt_level;  ///< 总压过低告警等级（bit5:3）
    bool short_circuit;           ///< 短路保护（bit6）
    // === 0x6F 高字节：过流告警 & 禁止充放电 ===
    uint8_t chg_overcurr_level;  ///< 充电过流告警等级（bit2:0）
    uint8_t dsg_overcurr_level;  ///< 放电过流告警等级（bit5:3）
    bool undervolt_chg_inhibit;  ///< 低压禁止充电（bit6）
    bool overvolt_dsg_inhibit;   ///< 高压禁止放电（bit7）

    // === 0x70 低字节：SOC/SOH 告警 & 并联通信 ===
    uint8_t soc_low_level;    ///< SOC 过低告警等级（bit2:0）
    uint8_t soh_low_level;    ///< SOH 过低告警等级（bit5:3）
    bool parallel_comm_ok;    ///< 并联通信成功（bit6）
    bool parallel_comm_fail;  ///< 并联通信失败（bit7）
    // === 0x70 高字节：MOS 过温 & 热失控告警 ===
    uint8_t mos_overtemp_level;     ///< MOS 温度过高告警等级（bit2:0）
    uint8_t thermal_runaway_level;  ///< 热失控告警等级（bit5:3）

    // 0x71：全部预留，无字段

    // === 0x72 高字节：AFE / 采样 / 检测故障 ===
    bool afe_chip_fault;     ///< AFE 芯片故障（bit0）
    bool afe_comm_fault;     ///< AFE 通信故障（bit1）
    bool afe_sample_fault;   ///< AFE 采样故障（bit2）
    bool volt_detect_fault;  ///< 电压检测故障（bit3）
    bool volt_wire_off;      ///< 电压采集线掉线（bit4）
    bool total_volt_fault;   ///< 总压检测故障（bit5）
    bool curr_detect_fault;  ///< 电流检测故障（bit6）
    bool temp_detect_fault;  ///< 温度检测故障（bit7）

    // === 0x73 低字节：采集线 / 存储 / MOS 硬件故障 ===
    bool temp_wire_off;      ///< 温度采集线掉线（bit0）
    bool eeprom_fault;       ///< EEPROM 故障（bit1）
    bool flash_fault;        ///< Flash 故障（bit2）
    bool rtc_fault;          ///< RTC 故障（bit3）
    bool chg_mos_fault;      ///< 充电 MOS 故障（bit4）
    bool dsg_mos_fault;      ///< 放电 MOS 故障（bit5）
    bool pre_chg_mos_fault;  ///< 预充 MOS 故障（bit6）
    bool precharge_fail;     ///< 预充失败（bit7）
    // === 0x73 高字节：MOS 控制来源 & 工作状态 ===
    bool comm_chg_mos_off;      ///< 通信指令控制充电 MOS 断开（bit0）
    bool comm_dsg_mos_off;      ///< 通信指令控制放电 MOS 断开（bit1）
    bool sw_chg_mos_off;        ///< 开关控制充电 MOS 断开（bit2）
    bool sw_dsg_mos_off;        ///< 开关控制放电 MOS 断开（bit3）
    bool fan_active;            ///< 风扇工作中（bit4）
    bool heat_active;           ///< 加热工作中（bit5）
    bool current_limit_active;  ///< 限流模块工作中（bit6）
    bool heat_fault;            ///< 加热故障（bit7）
};

/// RTC 实时时钟（由寄存器 0x61~0x63 解码）
struct Bms2Rtc {
    uint16_t year;   ///< 年（寄存器高字节 + 2000，如 0x14 → 2020）
    uint8_t month;   ///< 月（寄存器低字节，1~12）
    uint8_t day;     ///< 日（下一寄存器高字节，1~31）
    uint8_t hour;    ///< 时（下一寄存器低字节，0~23）
    uint8_t minute;  ///< 分（再下一寄存器高字节，0~59）
    uint8_t second;  ///< 秒（再下一寄存器低字节，0~59）
};

/**
 * @brief 主状态信息
 *
 * 由寄存器块 0x38~0x65（共 46 个寄存器）解码。
 *
 * 电流符号约定：正值 = 放电，负值 = 充电（与 BMS1 相反，遵循硬件原始定义）。
 */
struct Bms2BasicInfo {
    // -- 电气量 ---------------------------------------------------------------
    float total_voltage_v;  ///< 总电压（V）；原始: raw × 0.1
    float current_a;        ///< 电流（A）；原始: (raw−30000) × 0.1，正=放电
    float soc_pct;          ///< SOC（%）；原始: raw ÷ 10
    float remaining_ah;     ///< 剩余容量（Ah）；原始: raw × 0.1
    float power_w;          ///< 实时功率（W）
    float energy_wh;        ///< 累计能量（Wh）

    // -- 单体电压统计 ----------------------------------------------------------
    uint16_t cell_max_mv;    ///< 最高单体电压（mV）
    uint8_t cell_max_idx;    ///< 最高单体序号（1-based）
    uint16_t cell_min_mv;    ///< 最低单体电压（mV）
    uint8_t cell_min_idx;    ///< 最低单体序号（1-based）
    uint16_t cell_delta_mv;  ///< 最大压差（mV）
    uint16_t avg_cell_mv;    ///< 平均单体电压（mV）

    // -- 温度统计 --------------------------------------------------------------
    float temp_max_c;      ///< 最高温度（℃）
    uint8_t temp_max_idx;  ///< 最高温度探头序号
    float temp_min_c;      ///< 最低温度（℃）
    uint8_t temp_min_idx;  ///< 最低温度探头序号
    float temp_delta_c;    ///< 最大温差（℃）
    float mos_temp_c;      ///< MOS 温度（℃）
    float env_temp_c;      ///< 环境温度（℃）
    float heat_temp_c;     ///< 加热温度（℃）

    // -- 计数统计 --------------------------------------------------------------
    uint16_t life_counter;  ///< 心跳包计数（每秒递增，0x3B）
    uint16_t cycle_count;   ///< 循环次数（0x4C）
    uint8_t cell_count;     ///< 串联节数
    uint8_t ntc_count;      ///< 温度传感器数量

    // -- 状态标志 --------------------------------------------------------------
    Bms2ChargeStatus charge_status;  ///< 充放电状态
    bool charger_present;            ///< 充电器已接入
    bool load_present;               ///< 负载已接入
    bool charge_mos_on;              ///< 充电 MOS 状态
    bool discharge_mos_on;           ///< 放电 MOS 状态
    bool precharge_mos_on;           ///< 预充电 MOS 状态
    bool heat_mos_on;                ///< 加热 MOS 状态
    bool fan_mos_on;                 ///< 风扇 MOS 状态
    float heat_current_a;            ///< 加热电流（A，0x5D）
    bool current_limit_on;           ///< 限流状态（0x5F）
    float current_limit_a;           ///< 限流电流（A，0x60，正=放电方向）

    // -- RTC ------------------------------------------------------------------
    Bms2Rtc rtc;  ///< 实时时钟（0x61~0x63）

    // -- 其他状态 -------------------------------------------------------------
    uint16_t remaining_time_min;  ///< 剩余时间（分钟，0x64）
    uint16_t di_do_status;        ///< DI/DO 状态位（0x65，bit0-7=DI1-8，bit8-15=DO1-8）

    // -- 均衡 ------------------------------------------------------------------
    uint8_t balance_status;   ///< 均衡状态（0=关 1=充电均衡 2=放电均衡）
    uint64_t balance_bitmap;  ///< 各节均衡标志（bit N = 第 N+1 节）
};

// == 编解码器 =================================================================

/**
 * @brief 嘉佰达 BMS2 Modbus RTU 协议编解码器（纯静态工具类）
 *
 * 物理层：RS485，9600 BPS，8-N-1
 * 从机地址：默认 0x51（见 kBms2DefaultSlaveId）
 * 通信由 hal::IModbusMaster 负责，本类仅处理寄存器值 ↔ 物理量的转换。
 *
 * 典型读取流程（设备层调用）
 * ─────────────────────────
 * @code
 *   uint16_t status_regs[kBms2StatusBlockLen];
 *   int n = modbus->read_registers(kBms2DefaultSlaveId,
 *                                   kBms2RegStatusBase,
 *                                   kBms2StatusBlockLen,
 *                                   status_regs);
 *   if (n == kBms2StatusBlockLen) {
 *       auto info = Bms2Protocol::decode_basic_info(status_regs, n);
 *       if (info) { ... }
 *   }
 *
 *   uint16_t cell_regs[kBms2CellBlockLen];
 *   n = modbus->read_registers(kBms2DefaultSlaveId,
 *                               kBms2RegCellBase,
 *                               info->cell_count,      // 只读实际节数
 *                               cell_regs);
 *   auto cells = Bms2Protocol::decode_cell_voltages(cell_regs, n);
 * @endcode
 */
class Bms2Protocol {
   public:
    Bms2Protocol() = delete;  // 纯静态工具类，不可实例化

    // -- 解码 -----------------------------------------------------------------

    /**
     * @brief 解码主状态块（寄存器 0x38~0x65）
     * @param regs  从寄存器 kBms2RegStatusBase 开始读取的连续寄存器数组
     * @param count 实际读取数量，须 ≥ kBms2StatusBlockLen（46）
     * @return 解码成功返回 Bms2BasicInfo；数量不足返回 nullopt
     */
    static std::optional<Bms2BasicInfo> decode_basic_info(const uint16_t* regs, uint16_t count);

    /**
     * @brief 解码电芯电压块（寄存器 0x00~0x2F）
     * @param regs   从寄存器 kBms2RegCellBase 开始读取的数组
     * @param count  实际读取数量（即电芯节数），须在 [1, kBms2MaxCells] 范围内
     * @return 解码成功返回 Bms2CellVoltages；count 为 0 或超范围返回 nullopt
     */
    static std::optional<Bms2CellVoltages> decode_cell_voltages(const uint16_t* regs,
                                                                uint16_t count);

    /**
     * @brief 解码温度传感器块（寄存器 0x30~0x37）
     * @param regs   从寄存器 kBms2RegTempBase 开始读取的数组
     * @param count  实际读取数量（即探头数），须在 [1, kBms2MaxNtc] 范围内
     * @return 解码成功返回 Bms2Temperatures；count 为 0 或超范围返回 nullopt
     */
    static std::optional<Bms2Temperatures> decode_temperatures(const uint16_t* regs,
                                                               uint16_t count);

    /**
     * @brief 解码告警 / 故障状态块（寄存器 0x6D ~ 0x73）
     * @param regs  从寄存器 kBms2RegAlarmBase 开始读取的数组
     * @param count 实际读取数量，须 ≥ kBms2AlarmBlockLen（7）
     * @return 解码成功返回 Bms2AlarmFlags；数量不足返回 nullopt
     */
    static std::optional<Bms2AlarmFlags> decode_alarm_flags(const uint16_t* regs, uint16_t count);

    // -- 单位换算辅助 ---------------------------------------------------------

    /// 将温度寄存器原始值转换为摄氏度（所有温度寄存器均适用）
    static constexpr float raw_to_temp(uint16_t raw) noexcept {
        return static_cast<float>(raw) - 40.0f;
    }

    /// 将电流寄存器原始值（Offset 30000）转换为安培
    /// 正值 = 放电，负值 = 充电
    static constexpr float raw_to_current(uint16_t raw) noexcept {
        return (static_cast<float>(raw) - 30000.0f) * 0.1f;
    }

    /// 将 SOC 寄存器原始值转换为百分比（0.0 ~ 100.0）
    static constexpr float raw_to_soc(uint16_t raw) noexcept {
        return static_cast<float>(raw) / 10.0f;
    }
};

}  // namespace robot::protocol
