#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/protocol/bms2_protocol.h"
#include "pv_cleaning_robot/protocol/bms_protocol.h"

namespace robot::device {

/**
 * @brief 嘉佰达 BMS 设备（同时支持通用协议 V4 / UART 和 Modbus RTU 两种总线）
 *
 * 协议类型由构造函数决定，运行期不可切换：
 *   - kUart   (嘉佰达通用协议 V4)：依赖 hal::ISerialPort + protocol::BmsProtocol
 *   - kModbus (嘉佰达 Modbus RTU)：依赖 hal::IModbusMaster + protocol::Bms2Protocol
 *
 * 对外公有接口（BatteryData / Diagnostics / update() / get_*）在两种协议下语义一致。
 * 以下字段在两种协议下均有效：
 *   voltage_v, current_a（正=充电，负=放电）, soc_pct, temperature_c,
 *   alarm_flags, remaining_capacity_ah, cycle_count, cell_count, ntc_count,
 *   ntc_temps_c, cell_voltage_max_v, cell_voltage_min_v.
 * 仅 kUart 有效：cell_voltages, hw_version, nominal_capacity_ah.
 * 仅 kModbus 有效：cell_voltages2, alarm_detail2, power_w, energy_wh.
 *
 * 线程模型：
 *   外部定时器以约 500 ms 周期调用 update()，内部完成同步 I/O。
 *   所有 get_*() 方法无阻塞 I/O，可在任意线程安全调用。
 */
class BMS {
   public:
    /// 协议类型（由构造函数决定）
    enum class ProtocolType { kUart, kModbus };

    // == 保护标志透传（直接复用 protocol 层常量）===============================
    using ProtFlag = uint16_t;
    static constexpr ProtFlag PROT_CELL_OVERVOLT = protocol::kBmsProtCellOverVolt;
    static constexpr ProtFlag PROT_CELL_UNDERVOLT = protocol::kBmsProtCellUnderVolt;
    static constexpr ProtFlag PROT_BAT_OVERVOLT = protocol::kBmsProtBatOverVolt;
    static constexpr ProtFlag PROT_BAT_UNDERVOLT = protocol::kBmsProtBatUnderVolt;
    static constexpr ProtFlag PROT_CHG_OVERTEMP = protocol::kBmsProtChgOverTemp;
    static constexpr ProtFlag PROT_CHG_LOWTEMP = protocol::kBmsProtChgLowTemp;
    static constexpr ProtFlag PROT_DSG_OVERTEMP = protocol::kBmsProtDsgOverTemp;
    static constexpr ProtFlag PROT_DSG_LOWTEMP = protocol::kBmsProtDsgLowTemp;
    static constexpr ProtFlag PROT_CHG_OVERCURR = protocol::kBmsProtChgOverCurr;
    static constexpr ProtFlag PROT_DSG_OVERCURR = protocol::kBmsProtDsgOverCurr;
    static constexpr ProtFlag PROT_SHORT_CIRCUIT = protocol::kBmsProtShortCircuit;
    static constexpr ProtFlag PROT_MOS_SOFT_LOCK = protocol::kBmsProtMosSoftLock;

    // == MOS 控制命令值（透传 protocol 层常量）==================================
    static constexpr uint8_t MOS_RELEASE_ALL = protocol::kBmosMosReleaseAll;
    static constexpr uint8_t MOS_CHARGE_CLOSE = protocol::kBmosMosChargeClose;
    static constexpr uint8_t MOS_DISCHARGE_CLOSE = protocol::kBmosMosDischargeClose;
    static constexpr uint8_t MOS_BOTH_CLOSE = protocol::kBmosMosBothClose;

    // == 数据结构 ===============================================================

    /// 常驻生产数据（service 层及业务逻辑使用）
    struct BatteryData {
        float soc_pct;         ///< 剩余容量百分比（%）
        float voltage_v;       ///< 总电压（V）
        float current_a;       ///< 电流（A），正=充电，负=放电
        float temperature_c;   ///< 所有 NTC 探头最高温度（℃）
        uint16_t alarm_flags;  ///< 保护/告警标志位（见 PROT_* 常量）
        bool charging;         ///< true=充电中
        bool fully_charged;    ///< SOC≥阈值 且 |I|<0.5A 持续 3 秒
        bool low_battery;      ///< SOC≤低电量阈值
        bool valid;            ///< 至少完成一次成功读取
    };

    /// 完整诊断数据（开发调试、云端上报使用）
    struct Diagnostics : BatteryData {
        float remaining_capacity_ah;  ///< 剩余容量（Ah）
        float nominal_capacity_ah;    ///< 标称容量（Ah）（kUart 有效；kModbus=0）
        uint32_t cycle_count;         ///< 循环次数
        uint8_t cell_count;           ///< 串联节数
        uint8_t ntc_count;            ///< NTC 探头数量
        float ntc_temps_c[protocol::kBmsMaxNtc];  ///< 各探头温度（℃）
        float cell_voltage_max_v;                 ///< 最高单体电压（V）
        float cell_voltage_min_v;                 ///< 最低单体电压（V）
        protocol::BmsCellVoltages cell_voltages;  ///< 各节电压（mV）（kUart 专用）
        std::string hw_version;  ///< 硬件版本字符串（kUart 专用，0x05 读取）
        // -- kModbus 专用字段 -------------------------------------------------
        protocol::Bms2CellVoltages cell_voltages2;  ///< 各节电压（mV）（kModbus 专用）
        protocol::Bms2AlarmFlags alarm_detail2;  ///< 完整告警/故障状态（kModbus 专用）
        float power_w;                           ///< 实时功率（W）（kModbus 专用）
        float energy_wh;                         ///< 累计能量（Wh）（kModbus 专用）
        // -- 通用计数 ---------------------------------------------------------
        uint32_t update_count;  ///< 成功 update() 次数
        uint32_t error_count;   ///< 通信错误（超时/校验失败）次数
    };

    // == 构造 / 析构 ============================================================

    /// 嘉佰达通用协议 V4（UART / RS485）
    /// @param serial    已配置 9600-8-N-1 的串口实例（调用前无需 open）
    /// @param full_soc  视为充满的 SOC 阈值，默认 95%
    /// @param low_soc   低电量 SOC 阈值，默认 15%
    BMS(std::shared_ptr<hal::ISerialPort> serial, float full_soc = 95.0f, float low_soc = 15.0f);

    /// 嘉佰达 Modbus RTU 协议（RS485）
    /// @param modbus     已配置 9600-8-N-1 的 Modbus 主站实例（调用前无需 open）
    /// @param slave_id   从机地址，默认 kBms2DefaultSlaveId (0x51)
    /// @param full_soc   视为充满的 SOC 阈值，默认 95%
    /// @param low_soc    低电量 SOC 阈值，默认 15%
    BMS(std::shared_ptr<hal::IModbusMaster> modbus,
        uint8_t slave_id = protocol::kBms2DefaultSlaveId,
        float full_soc = 95.0f,
        float low_soc = 15.0f);

    ~BMS();

    // == 生命周期 ===============================================================

    /// 打开通信接口（kUart：打开串口并读取硬件版本；kModbus：打开 Modbus 连接）
    DeviceError open();

    /// 关闭通信接口
    void close();

    /// 返回当前使用的协议类型
    ProtocolType protocol_type() const noexcept {
        return protocol_type_;
    }

    // == 周期更新（由外部 ~500ms 定时器调用）====================================

    /// 发送 0x03 请求读取基本信息；每 4 次额外发送 0x04 读取单体电压
    void update();

    // == 数据访问（无 I/O，线程安全）============================================

    BatteryData get_data() const;
    Diagnostics get_diagnostics() const;

    bool is_fully_charged() const;
    bool is_low_battery() const;
    bool has_alarm() const;  ///< 任意保护标志位置位

    // == 控制命令 ===============================================================

    /// 发送 MOS 控制命令（0xE1）；仅 kUart 协议有效，kModbus 返回 NOT_SUPPORTED。
    /// mos_state 见 MOS_* 常量
    DeviceError mos_control(uint8_t mos_state);

   private:
    // -- kUart 专用 -----------------------------------------------------------
    // 发送 req 帧后阻塞等待完整应答帧（含校验），超时返回 false
    bool transact(const uint8_t* req, size_t req_len, int timeout_ms = 300);
    bool read_basic_info_uart();     // 命令 0x03
    bool read_cell_voltages_uart();  // 命令 0x04

    // -- kModbus 专用 ---------------------------------------------------------
    bool read_basic_info_modbus();
    bool read_cell_voltages_modbus();
    /// 带休眠唤醒重试的 Modbus 寄存器读取（同 transact() 的休眠逻辑）
    int modbus_read_regs(int addr, int count, uint16_t* out);

    // -- 成员变量 --------------------------------------------------------------
    ProtocolType protocol_type_;

    std::shared_ptr<hal::ISerialPort> serial_;  ///< kUart 专用
    protocol::BmsProtocol parser_;              ///< kUart 专用

    std::shared_ptr<hal::IModbusMaster> modbus_;  ///< kModbus 专用
    uint8_t slave_id_{0};                         ///< kModbus 专用

    float full_soc_;
    float low_soc_;

    mutable std::mutex mtx_;
    std::mutex uart_tx_mtx_;
    Diagnostics diag_{};

    int full_charge_count_{0};
    static constexpr int kFullChargeConfirm = 6;  // 6×500ms≈3s

    int update_cycle_{0};  // 每 4 次 update() 读一次单体电压（降低总线占用）

    // 休眠机制（kUart / kModbus 共用）：
    //   BMS 在 1 分钟无通讯后自动休眠；第一次请求触发唤醒，BMS 无响应（正常超时），
    //   等待 kWakeupDelayMs 后重试正式请求。
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_comm_time_{Clock::time_point::min()};  ///< 上次通讯成功时刻
    static constexpr int kSleepTimeoutSec = 55;  ///< BMS 休眠阈值留 5s 余量（实际 60s）
    static constexpr int kWakeupDelayMs = 200;   ///< 唤醒帧发出后等待 BMS 启动的时间
    static constexpr int kMaxRetries = 3;        ///< 唤醒重试上限（唤醒帧 + 正式帧）
};

}  // namespace robot::device
