/**
 * @file bms.cc
 * @brief 嘉佰达 BMS 设备实现。
 *
 * 本文件实现 BMS UART 请求/应答事务、休眠唤醒兼容、基本信息和单体电压读取以及 MOS 控制。
 * 通信失败会累计诊断错误计数，周期线程继续在后续 update() 中重试。
 */
#include <algorithm>
#include <chrono>
#include <spdlog/spdlog.h>
#include <thread>

#include "pv_cleaning_robot/device/bms.h"

namespace robot::device {

BMS::BMS(std::shared_ptr<hal::ISerialPort> serial,
         float full_soc,
         float low_soc,
         float charging_current_threshold_a)
    : serial_(std::move(serial))
    , full_soc_(full_soc)
    , low_soc_(low_soc)
    , charging_current_threshold_a_(charging_current_threshold_a) {}

BMS::~BMS() {
    close();
}

DeviceError BMS::open() {
    clear_stop_request();
    if (not serial_->open()) {
        spdlog::error("[BMS] 串口打开失败");
        return DeviceError::NOT_OPEN;
    }

    // 读取硬件版本（0x05），失败不中止启动流程。
    {
        auto req = protocol::BmsProtocol::encode_read_version();
        if (transact(req.data(), req.size(), 500) and not parser_.is_error() and
            parser_.get_cmd() == 0x05) {
            auto ver =
                protocol::BmsProtocol::decode_version(parser_.get_data(), parser_.get_data_len());
            if (ver) {
                std::lock_guard<std::mutex> lk(mtx_);
                diag_.hw_version = *ver;
                spdlog::info("[BMS] 硬件版本: {}", *ver);
            }
        }
    }

    // 首次读取基本信息，失败在下次 update() 中重试。
    if (not read_basic_info_uart()) {
        spdlog::warn("[BMS] 首次数据读取失败，将在下次 update() 重试");
    }
    return DeviceError::OK;
}

void BMS::close() {
    request_stop();
    serial_->close();
}

void BMS::request_stop() {
    stop_requested_.store(true, std::memory_order_release);
}

void BMS::clear_stop_request() {
    stop_requested_.store(false, std::memory_order_release);
}

bool BMS::stop_requested() const {
    return stop_requested_.load(std::memory_order_acquire);
}

bool BMS::sleep_interruptible(std::chrono::milliseconds duration) const {
    using namespace std::chrono;
    const auto deadline = Clock::now() + duration;
    while (Clock::now() < deadline) {
        if (stop_requested()) {
            return false;
        }
        const auto remaining = duration_cast<milliseconds>(deadline - Clock::now());
        std::this_thread::sleep_for(std::min(remaining, milliseconds(20)));
    }
    return !stop_requested();
}

bool BMS::transact(const uint8_t* req, size_t req_len, int timeout_ms) {
    std::lock_guard<std::mutex> lock(uart_tx_mtx_);
    using namespace std::chrono;

    if (stop_requested()) {
        return false;
    }

    // 判断 BMS 是否可能已休眠：从未通讯过，或距上次成功超过 kSleepTimeoutSec。
    auto now = Clock::now();
    bool may_be_sleeping =
        (last_comm_time_ == Clock::time_point::min()) ||
        (duration_cast<seconds>(now - last_comm_time_).count() >= kSleepTimeoutSec);

    if (may_be_sleeping) {
        spdlog::debug("[BMS] 判断可能处于休眠，将在第一帧超时后等待 {} ms 再重试", kWakeupDelayMs);
    }

    // 休眠场景下前几次请求作为唤醒帧，允许正常超时；最后一次按正式请求等待应答。
    int max_attempts = may_be_sleeping ? (kMaxRetries + 1) : 1;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        if (stop_requested()) {
            return false;
        }
        parser_.reset();
        serial_->flush_input();

        if (stop_requested()) {
            return false;
        }
        if (serial_->write(req, req_len) < static_cast<int>(req_len)) {
            spdlog::warn("[BMS] 写入请求帧失败");
            return false;  // 串口级错误，重试无意义
        }

        bool is_wakeup = (may_be_sleeping && attempt < kMaxRetries);
        if (is_wakeup) {
            spdlog::info(
                "[BMS] 唤醒帧 {}/{} 已发送 (cmd={:#04x})", attempt + 1, max_attempts, req[2]);
        }

        // 等待应答：每次批量读取最多 64 字节，单次 read 超时 100ms（与 VTIME 精度对齐）。
        // libserialport 在 Linux 下 VTIME 精度为 0.1s，传入 <100ms 会退化为非阻塞，
        // 使用 ≥100ms 的超时确保 read() 真正阻塞等待数据，避免空转丢过应答字节。
        uint64_t total_rx = 0;
        auto deadline = Clock::now() + milliseconds(timeout_ms);
        while (Clock::now() < deadline) {
            if (stop_requested()) {
                return false;
            }
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
                        spdlog::debug("[BMS] 帧接收成功 attempt={}/{} rx_bytes={}",
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
            if (!sleep_interruptible(milliseconds(kWakeupDelayMs))) {
                return false;
            }
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

    // 取所有 NTC 探头的最高温度作为代表值。
    uint8_t ntc_n = std::min(info->ntc_count, protocol::kBmsMaxNtc);
    float max_t = -273.0f;
    for (uint8_t i = 0; i < ntc_n; ++i) {
        diag_.ntc_temps_c[i] = info->ntc_temp_c[i];
        if (info->ntc_temp_c[i] > max_t)
            max_t = info->ntc_temp_c[i];
    }
    diag_.temperature_c = (ntc_n > 0) ? max_t : 0.0f;

    diag_.charging = (diag_.current_a > charging_current_threshold_a_);
    diag_.low_battery = (diag_.soc_pct <= low_soc_);
    diag_.valid = true;

    diag_.fully_charged = (diag_.soc_pct >= full_soc_);

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

    // 计算最高/最低单体电压。
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

void BMS::update() {
    if (stop_requested()) {
        return;
    }
    ++update_cycle_;

    if (!read_basic_info_uart()) {
        std::lock_guard<std::mutex> lk(mtx_);
        ++diag_.error_count;
    }

    // 每 4 次（约 2s）读一次单体电压，降低 9600 bps 总线占用。
    if (!stop_requested() && update_cycle_ % 4 == 0)
        read_cell_voltages_uart();
}

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

DeviceError BMS::mos_control(uint8_t mos_state) {
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

}  // namespace robot::device
