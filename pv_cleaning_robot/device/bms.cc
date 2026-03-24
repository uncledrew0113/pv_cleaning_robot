#include <algorithm>
#include <chrono>
#include <cmath>
#include <spdlog/spdlog.h>
#include <thread>

#include "pv_cleaning_robot/device/bms.h"

namespace robot::device {

BMS::BMS(std::shared_ptr<hal::ISerialPort> serial, float full_soc, float low_soc)
    : protocol_type_(ProtocolType::kUart),
      serial_(std::move(serial)),
      full_soc_(full_soc),
      low_soc_(low_soc) {}

BMS::BMS(std::shared_ptr<hal::IModbusMaster> modbus, uint8_t slave_id, float full_soc,
         float low_soc)
    : protocol_type_(ProtocolType::kModbus),
      modbus_(std::move(modbus)),
      slave_id_(slave_id),
      full_soc_(full_soc),
      low_soc_(low_soc) {}

BMS::~BMS() {
    close();
}

// == 生命周期 ==================================================================

DeviceError BMS::open() {
    if (protocol_type_ == ProtocolType::kUart) {
        if (not serial_->open()) {
            spdlog::error("[BMS] 串口打开失败");
            return DeviceError::NOT_OPEN;
        }

        // 读取硬件版本（0x05），失败不中止启动流程
        {
            auto req = protocol::BmsProtocol::encode_read_version();
            if (transact(req.data(), req.size(), 500) and not parser_.is_error() and
                parser_.get_cmd() == 0x05) {
                auto ver = protocol::BmsProtocol::decode_version(parser_.get_data(),
                                                                  parser_.get_data_len());
                if (ver) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    diag_.hw_version = *ver;
                    spdlog::info("[BMS] 硬件版本: {}", *ver);
                }
            }
        }

        // 首次读取基本信息，失败在下次 update() 中重试
        if (not read_basic_info_uart()) {
            spdlog::warn("[BMS] 首次数据读取失败，将在下次 update() 重试");
        }
        return DeviceError::OK;
    } else {
        if (not modbus_->open()) {
            spdlog::error("[BMS2] Modbus 端口打开失败");
            return DeviceError::NOT_OPEN;
        }

        if (not read_basic_info_modbus()) {
            spdlog::warn("[BMS2] 首次数据读取失败，将在下次 update() 重试");
        }
        return DeviceError::OK;
    }
}

void BMS::close() {
    if (protocol_type_ == ProtocolType::kUart) {
        serial_->close();
    } else {
        modbus_->close();
    }
}

// == 核心事务 ==================================================================

bool BMS::transact(const uint8_t* req, size_t req_len, int timeout_ms) {
    using namespace std::chrono;

    // 判断 BMS 是否可能已休眠：从未通讯过、或距上次成功超过 kSleepTimeoutSec
    auto now = Clock::now();
    bool may_be_sleeping =
        (last_comm_time_ == Clock::time_point::min()) ||
        (duration_cast<seconds>(now - last_comm_time_).count() >= kSleepTimeoutSec);

    if (may_be_sleeping) {
        spdlog::debug("[BMS] 判断可能处于休眠，将在第一帧超时后等待 {} ms 再重试", kWakeupDelayMs);
    }

    // attempt=0：如果可能休眠，此帧份演唤醒帧角色（BMS 不回复，正常超时）
    // attempt=1：BMS 已唤醒，此帧应正常应答
    // 如果不可能休眠，只执行一次（attempt=0 即正式帧）
    int max_attempts = may_be_sleeping ? (kMaxRetries + 1) : 1;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        parser_.reset();
        serial_->flush_input();

        if (serial_->write(req, req_len) < static_cast<int>(req_len)) {
            spdlog::warn("[BMS] 写入请求帧失败");
            return false;  // 串口级错误，重试无意义
        }

        bool is_wakeup = (may_be_sleeping && attempt < kMaxRetries);
        if (is_wakeup) {
            spdlog::info(
                "[BMS] 唤醒帧 {}/{} 已发送 (cmd={:#04x})", attempt + 1, max_attempts, req[2]);
        }

        // 等待应答：每次批量读取最多 64 字节，单次 read 超时 100ms（与 VTIME 精度对齐）
        // libserialport 在 Linux 下 VTIME 精度为 0.1s，传入 <100ms 会退化为非阻塞，
        // 使用 ≥100ms 的超时确保 read() 真正阻塞等待数据，避免空转丢过应答字节。
        uint64_t total_rx = 0;
        auto deadline = Clock::now() + milliseconds(timeout_ms);
        while (Clock::now() < deadline) {
            auto remaining_ms = duration_cast<milliseconds>(deadline - Clock::now()).count();
            int read_ms = static_cast<int>(std::min<long long>(remaining_ms, 100LL));
            if (read_ms <= 0)
                break;

            uint8_t buf[64];
            int n = serial_->read(buf, sizeof(buf), read_ms);
            if (n > 0) {
                total_rx += static_cast<uint64_t>(n);
                for (int i = 0; i < n; ++i) {
                    parser_.push_byte(buf[i]);
                    if (parser_.frame_complete()) {
                        spdlog::info("[BMS] 帧接收成功 attempt={}/{} rx_bytes={}",
                                     attempt + 1,
                                     max_attempts,
                                     total_rx);
                        last_comm_time_ = Clock::now();
                        return true;
                    }
                }
            }
        }

        if (is_wakeup) {
            // 超时但仍是唤醒阶段：收到 >0 字节说明 BMS 已应答部分数据（帧可能不完整）
            spdlog::info("[BMS] 唤醒帧 {}/{} 超时 (rx_bytes={})，等待 {} ms 后重试",
                         attempt + 1,
                         max_attempts,
                         total_rx,
                         kWakeupDelayMs);
            std::this_thread::sleep_for(milliseconds(kWakeupDelayMs));
        } else {
            spdlog::debug("[BMS] transact 超时 attempt={}/{} (cmd={:#04x} rx_bytes={})",
                          attempt + 1,
                          max_attempts,
                          req[2],
                          total_rx);
        }
    }
    return false;
}

// == 读取命令 ==================================================================

bool BMS::read_basic_info_uart() {
    auto req = protocol::BmsProtocol::encode_read_basic_info();
    if (not transact(req.data(), req.size()))  // NOLINT
        return false;
    if (parser_.is_error() or parser_.get_cmd() != 0x03)
        return false;

    auto info =
        protocol::BmsProtocol::decode_basic_info(parser_.get_data(), parser_.get_data_len());
    if (not info)
        return false;

    std::lock_guard<std::mutex> lk(mtx_);

    diag_.voltage_v = static_cast<float>(info->total_voltage_mv) / 1000.0f;
    diag_.current_a = static_cast<float>(info->current_ma) / 1000.0f;
    diag_.soc_pct = static_cast<float>(info->rsoc_pct);
    diag_.alarm_flags = info->protection_status;
    diag_.remaining_capacity_ah = static_cast<float>(info->residual_capacity_mah) / 1000.0f;
    diag_.nominal_capacity_ah = static_cast<float>(info->nominal_capacity_mah) / 1000.0f;
    diag_.cycle_count = info->cycle_count;
    diag_.cell_count = info->cell_count;
    diag_.ntc_count = info->ntc_count;

    // 取所有 NTC 探头的最高温度作为代表值
    uint8_t ntc_n = std::min(info->ntc_count, protocol::kBmsMaxNtc);
    float max_t = -273.0f;
    for (uint8_t i = 0; i < ntc_n; ++i) {
        diag_.ntc_temps_c[i] = info->ntc_temp_c[i];
        if (info->ntc_temp_c[i] > max_t)
            max_t = info->ntc_temp_c[i];
    }
    diag_.temperature_c = (ntc_n > 0) ? max_t : 0.0f;

    diag_.charging = (diag_.current_a > 0.05f);
    diag_.low_battery = (diag_.soc_pct <= low_soc_);
    diag_.valid = true;

    // 充满检测：SOC 达阈值且电流趋近 0 持续 kFullChargeConfirm 次
    if (diag_.soc_pct >= full_soc_ and std::fabs(diag_.current_a) < 0.5f) {
        ++full_charge_count_;
    } else {
        full_charge_count_ = 0;
    }
    diag_.fully_charged = (full_charge_count_ >= kFullChargeConfirm);

    ++diag_.update_count;
    return true;
}

bool BMS::read_cell_voltages_uart() {
    auto req = protocol::BmsProtocol::encode_read_cell_voltages();
    if (not transact(req.data(), req.size(), 500))
        return false;
    if (parser_.is_error() or parser_.get_cmd() != 0x04)
        return false;

    auto cells =
        protocol::BmsProtocol::decode_cell_voltages(parser_.get_data(), parser_.get_data_len());
    if (not cells)
        return false;

    std::lock_guard<std::mutex> lk(mtx_);
    diag_.cell_voltages = *cells;

    // 计算最高/最低单体电压
    if (cells->count > 0) {
        uint16_t vmax = cells->mv[0];
        uint16_t vmin = cells->mv[0];
        for (uint8_t i = 1; i < cells->count; ++i) {
            if (cells->mv[i] > vmax)
                vmax = cells->mv[i];
            if (cells->mv[i] < vmin)
                vmin = cells->mv[i];
        }
        diag_.cell_voltage_max_v = static_cast<float>(vmax) / 1000.0f;
        diag_.cell_voltage_min_v = static_cast<float>(vmin) / 1000.0f;
    }

    return true;
}

// == 周期更新 ==================================================================

void BMS::update() {
    ++update_cycle_;

    bool ok = (protocol_type_ == ProtocolType::kUart) ? read_basic_info_uart()
                                                       : read_basic_info_modbus();
    if (not ok) {
        std::lock_guard<std::mutex> lk(mtx_);
        ++diag_.error_count;
    }

    // 每 4 次（约 2s）读一次单体电压，降低 9600 bps 总线占用
    if (update_cycle_ % 4 == 0) {
        if (protocol_type_ == ProtocolType::kUart) {
            read_cell_voltages_uart();
        } else {
            read_cell_voltages_modbus();
        }
    }
}

// == 数据访问 ==================================================================

BMS::BatteryData BMS::get_data() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<BatteryData>(diag_);
}

BMS::Diagnostics BMS::get_diagnostics() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_;
}

bool BMS::is_fully_charged() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_.fully_charged;
}

bool BMS::is_low_battery() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_.low_battery;
}

bool BMS::has_alarm() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_.alarm_flags != 0;
}

// == 控制命令 ==================================================================

DeviceError BMS::mos_control(uint8_t mos_state) {
    if (protocol_type_ == ProtocolType::kModbus) {
        spdlog::warn("[BMS2] mos_control 在 Modbus 协议下不支持（所有寄存器只读）");
        return DeviceError::NOT_SUPPORTED;
    }
    auto req = protocol::BmsProtocol::encode_mos_control(mos_state);
    if (not transact(req.data(), req.size(), 500))
        return DeviceError::COMM_TIMEOUT;
    if (parser_.is_error()) {
        spdlog::warn("[BMS] mos_control({:#04x}) BMS 返回错误应答", mos_state);
        return DeviceError::EXEC_FAILED;
    }
    spdlog::info("[BMS] mos_control({:#04x}) 成功", mos_state);
    return DeviceError::OK;
}

// == BMS2 (Modbus RTU) 读取命令 ================================================

// 带休眠唤醒重试的 Modbus 寄存器读取，逻辑等同 transact() 的休眠检测：
//   - 距上次成功通讯超过 kSleepTimeoutSec 秒时认为 BMS 可能已休眠
//   - 第一次发请求作为唤醒报文，BMS 不响应则超时；等待 kWakeupDelayMs 后重试
int BMS::modbus_read_regs(int addr, int count, uint16_t* out) {
    using namespace std::chrono;

    auto now = Clock::now();
    bool may_be_sleeping =
        (last_comm_time_ == Clock::time_point::min()) ||
        (duration_cast<seconds>(now - last_comm_time_).count() >= kSleepTimeoutSec);

    if (may_be_sleeping) {
        spdlog::debug("[BMS2] 判断可能处于休眠，将在第一次超时后等待 {} ms 再重试", kWakeupDelayMs);
    }

    // attempt=0：若可能休眠，此次作为唤醒报文（BMS 不回复，正常超时）
    // attempt=1..：BMS 已唤醒，此次应正常应答
    int max_attempts = may_be_sleeping ? kMaxRetries : 1;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        int n = modbus_->read_registers(slave_id_, addr, count, out);
        if (n == count) {
            last_comm_time_ = Clock::now();
            if (may_be_sleeping && attempt > 0) {
                spdlog::info("[BMS2] 唤醒成功 attempt={}/{} addr={:#04x}",
                             attempt + 1, max_attempts, addr);
            }
            return n;
        }

        if (may_be_sleeping && attempt < max_attempts - 1) {
            spdlog::info("[BMS2] 唤醒查询 {}/{} 超时 (addr={:#04x})，等待 {} ms 后重试",
                         attempt + 1, max_attempts, addr, kWakeupDelayMs);
            std::this_thread::sleep_for(milliseconds(kWakeupDelayMs));
        } else {
            spdlog::debug("[BMS2] read_registers 失败 attempt={}/{} addr={:#04x}",
                          attempt + 1, max_attempts, addr);
        }
    }
    return -1;
}

bool BMS::read_basic_info_modbus() {
    // 1. 读取主状态块（0x38，46 个寄存器）
    uint16_t status_regs[protocol::kBms2StatusBlockLen];
    int n = modbus_read_regs(protocol::kBms2RegStatusBase,
                             protocol::kBms2StatusBlockLen, status_regs);
    if (n != static_cast<int>(protocol::kBms2StatusBlockLen)) {
        spdlog::warn("[BMS2] 主状态块读取失败 (n={})", n);
        return false;
    }

    auto info =
        protocol::Bms2Protocol::decode_basic_info(status_regs, static_cast<uint16_t>(n));
    if (not info)
        return false;

    // 2. 读取温度块（0x30，ntc_count 个寄存器）
    uint8_t ntc_n = std::min(info->ntc_count, protocol::kBms2MaxNtc);
    float ntc_temps[protocol::kBms2MaxNtc]{};
    if (ntc_n > 0) {
        uint16_t temp_regs[protocol::kBms2MaxNtc];
        int tn = modbus_read_regs(protocol::kBms2RegTempBase, ntc_n, temp_regs);
        if (tn == static_cast<int>(ntc_n)) {
            auto temps =
                protocol::Bms2Protocol::decode_temperatures(temp_regs, static_cast<uint16_t>(tn));
            if (temps) {
                for (uint8_t i = 0; i < ntc_n; ++i)
                    ntc_temps[i] = temps->temp_c[i];
            }
        } else {
            spdlog::debug("[BMS2] 温度块读取失败 (n={})", tn);
        }
    }

    // 3. 读取告警/故障状态块（0x6D，7 个寄存器）
    std::optional<protocol::Bms2AlarmFlags> alarm_opt;
    {
        uint16_t alarm_regs[protocol::kBms2AlarmBlockLen];
        int an = modbus_read_regs(protocol::kBms2RegAlarmBase,
                                  protocol::kBms2AlarmBlockLen, alarm_regs);
        if (an == static_cast<int>(protocol::kBms2AlarmBlockLen)) {
            alarm_opt =
                protocol::Bms2Protocol::decode_alarm_flags(alarm_regs, static_cast<uint16_t>(an));
        } else {
            spdlog::debug("[BMS2] 告警块读取失败 (n={})", an);
        }
    }

    std::lock_guard<std::mutex> lk(mtx_);

    // BMS2 电流约定：正=放电，负=充电；内部统一正=充电
    diag_.voltage_v = info->total_voltage_v;
    diag_.current_a = -info->current_a;  // 翻转符号，与 BMS1 约定一致
    diag_.soc_pct = info->soc_pct;
    diag_.remaining_capacity_ah = info->remaining_ah;
    diag_.nominal_capacity_ah = 0.0f;  // BMS2 无标称容量寄存器
    diag_.cycle_count = info->cycle_count;
    diag_.cell_count = info->cell_count;
    diag_.ntc_count = ntc_n;

    // NTC 温度
    float max_t = -273.0f;
    for (uint8_t i = 0; i < ntc_n; ++i) {
        diag_.ntc_temps_c[i] = ntc_temps[i];
        if (ntc_temps[i] > max_t)
            max_t = ntc_temps[i];
    }
    // 若没读到 NTC 块，退化使用 BasicInfo 中的统计最高温
    diag_.temperature_c = (ntc_n > 0) ? max_t : info->temp_max_c;

    // 单体电压统计（来自主状态块中的统计寄存器）
    diag_.cell_voltage_max_v = static_cast<float>(info->cell_max_mv) / 1000.0f;
    diag_.cell_voltage_min_v = static_cast<float>(info->cell_min_mv) / 1000.0f;

    // BMS2 扩展字段
    diag_.power_w = info->power_w;
    diag_.energy_wh = info->energy_wh;

    // 告警标志：将 BMS2 告警等级映射到统一的 alarm_flags 位图
    if (alarm_opt) {
        diag_.alarm_detail2 = *alarm_opt;
        uint16_t flags = 0u;
        if (alarm_opt->cell_overvolt_level > 0)  flags |= PROT_CELL_OVERVOLT;
        if (alarm_opt->cell_undervolt_level > 0) flags |= PROT_CELL_UNDERVOLT;
        if (alarm_opt->bat_overvolt_level > 0)   flags |= PROT_BAT_OVERVOLT;
        if (alarm_opt->bat_undervolt_level > 0)  flags |= PROT_BAT_UNDERVOLT;
        if (alarm_opt->chg_overtemp_level > 0)   flags |= PROT_CHG_OVERTEMP;
        if (alarm_opt->chg_lowtemp_level > 0)    flags |= PROT_CHG_LOWTEMP;
        if (alarm_opt->dsg_overtemp_level > 0)   flags |= PROT_DSG_OVERTEMP;
        if (alarm_opt->dsg_lowtemp_level > 0)    flags |= PROT_DSG_LOWTEMP;
        if (alarm_opt->chg_overcurr_level > 0)   flags |= PROT_CHG_OVERCURR;
        if (alarm_opt->dsg_overcurr_level > 0)   flags |= PROT_DSG_OVERCURR;
        if (alarm_opt->short_circuit)            flags |= PROT_SHORT_CIRCUIT;
        diag_.alarm_flags = flags;
    }

    diag_.charging = (diag_.current_a > 0.05f);
    diag_.low_battery = (diag_.soc_pct <= low_soc_);
    diag_.valid = true;

    // 充满检测：SOC 达阈值且电流趋近 0 持续 kFullChargeConfirm 次
    if (diag_.soc_pct >= full_soc_ and std::fabs(diag_.current_a) < 0.5f) {
        ++full_charge_count_;
    } else {
        full_charge_count_ = 0;
    }
    diag_.fully_charged = (full_charge_count_ >= kFullChargeConfirm);

    ++diag_.update_count;
    return true;
}

bool BMS::read_cell_voltages_modbus() {
    // 读取前先取当前 cell_count（加锁后立即释放）
    uint8_t cell_n;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cell_n = diag_.cell_count;
    }

    if (cell_n == 0 || cell_n > protocol::kBms2MaxCells) {
        spdlog::debug("[BMS2] cell_count={} 无效，跳过电芯电压读取", cell_n);
        return false;
    }

    uint16_t cell_regs[protocol::kBms2MaxCells];
    int n = modbus_read_regs(protocol::kBms2RegCellBase, cell_n, cell_regs);
    if (n != static_cast<int>(cell_n)) {
        spdlog::warn("[BMS2] 电芯电压块读取失败 (n={})", n);
        return false;
    }

    auto cells =
        protocol::Bms2Protocol::decode_cell_voltages(cell_regs, static_cast<uint16_t>(n));
    if (not cells)
        return false;

    std::lock_guard<std::mutex> lk(mtx_);
    diag_.cell_voltages2 = *cells;
    return true;
}

}  // namespace robot::device
