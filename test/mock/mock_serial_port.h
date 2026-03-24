#pragma once
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include <cstring>
#include <vector>

/// @brief ISerialPort 手工 mock，用于单元测试，不依赖真实硬件
struct MockSerialPort : robot::hal::ISerialPort {
    // ── 可配置的返回值 ──────────────────────────────────────
    bool open_result{true};
    int  write_return{-2};   // -2 = 成功并返回 len
    int  read_return{-2};    // -2 = 返回 rx_data 内容; 0 = 超时；-1 = 错误
    robot::hal::UartResult injected_error{robot::hal::UartResult::OK};

    // ── 模拟 RX 数据 ───────────────────────────────────────
    std::vector<uint8_t> rx_data;         // 下次 read() 返回的数据
    std::vector<uint8_t> tx_captured;     // write() 收到的数据

    // ── 调用记录 ───────────────────────────────────────────
    bool opened{false};
    bool flush_input_called{false};
    bool flush_output_called{false};

    bool open() override { opened = true; injected_error = robot::hal::UartResult::OK; return open_result; }
    void close() override { opened = false; }
    bool is_open() const override { return opened; }

    int write(const uint8_t* buf, size_t len, int /*timeout_ms*/ = -1) override {
        if (write_return == -2) {
            tx_captured.insert(tx_captured.end(), buf, buf + len);
            injected_error = robot::hal::UartResult::OK;
            return static_cast<int>(len);
        }
        injected_error = (write_return >= 0) ? robot::hal::UartResult::OK
                                             : robot::hal::UartResult::SYS_ERROR;
        return write_return;
    }

    int read(uint8_t* buf, size_t max_len, int /*timeout_ms*/) override {
        if (read_return == 0) {
            injected_error = robot::hal::UartResult::TIMEOUT;
            return 0;
        }
        if (read_return == -1) {
            injected_error = robot::hal::UartResult::SYS_ERROR;
            return -1;
        }
        // 默认：返回 rx_data 中排队的数据
        if (rx_data.empty()) {
            injected_error = robot::hal::UartResult::TIMEOUT;
            return 0;
        }
        size_t n = std::min(max_len, rx_data.size());
        std::memcpy(buf, rx_data.data(), n);
        rx_data.erase(rx_data.begin(), rx_data.begin() + static_cast<int>(n));
        injected_error = robot::hal::UartResult::OK;
        return static_cast<int>(n);
    }

    bool flush_input()  override { flush_input_called  = true; injected_error = robot::hal::UartResult::OK; return true; }
    bool flush_output() override { flush_output_called = true; injected_error = robot::hal::UartResult::OK; return true; }
    int  bytes_available() override { return static_cast<int>(rx_data.size()); }

    robot::hal::UartResult get_last_error() const override { return injected_error; }
};
