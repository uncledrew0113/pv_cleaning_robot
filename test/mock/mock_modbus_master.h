#pragma once
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include <map>
#include <vector>

/// @brief IModbusMaster 手工 mock，用于单元测试，不依赖真实硬件
struct MockModbusMaster : robot::hal::IModbusMaster {
    // ── 可配置的返回值 ──────────────────────────────────────
    bool open_result{true};
    int  read_reg_return{0};    // >=0 = 返回寄存器数; -1 = 失败
    int  write_reg_return{0};   // 0 = 成功; -1 = 失败

    robot::hal::ModbusResult injected_error{robot::hal::ModbusResult::OK};

    // ── 模拟寄存器存储 ─────────────────────────────────────
    std::map<int, uint16_t> registers;  // addr -> value

    // ── 调用记录 ───────────────────────────────────────────
    bool opened{false};
    struct WriteCall { int slave_id; int addr; uint16_t val; };
    std::vector<WriteCall> write_calls;

    bool open()  override { opened = true;  injected_error = robot::hal::ModbusResult::OK; return open_result; }
    void close() override { opened = false; }
    bool is_open() const override { return opened; }

    int read_registers(int /*slave*/, int addr, int count, uint16_t* out) override {
        if (read_reg_return < 0) {
            injected_error = robot::hal::ModbusResult::TIMEOUT;
            return -1;
        }
        for (int i = 0; i < count; ++i) {
            auto it = registers.find(addr + i);
            out[i] = (it != registers.end()) ? it->second : 0;
        }
        injected_error = robot::hal::ModbusResult::OK;
        return count;
    }

    int write_register(int slave_id, int addr, uint16_t val) override {
        write_calls.push_back({slave_id, addr, val});
        if (write_reg_return < 0) {
            injected_error = robot::hal::ModbusResult::TIMEOUT;
            return -1;
        }
        registers[addr] = val;
        injected_error = robot::hal::ModbusResult::OK;
        return 0;
    }

    int write_registers(int /*slave*/, int addr, int count, const uint16_t* vals) override {
        if (write_reg_return < 0) {
            injected_error = robot::hal::ModbusResult::TIMEOUT;
            return -1;
        }
        for (int i = 0; i < count; ++i) registers[addr + i] = vals[i];
        injected_error = robot::hal::ModbusResult::OK;
        return 0;
    }

    void set_timeout_ms(int) override {}
    robot::hal::ModbusResult get_last_error() const override { return injected_error; }
};
