#include <cerrno>
#include <chrono>
#include <modbus/modbus.h>
#include <spdlog/spdlog.h>
#include <thread>

#include "pv_cleaning_robot/driver/libmodbus_master.h"

namespace robot::driver {

namespace {
constexpr int kMaxRetry = 3;
constexpr int kRetryDelayMs[kMaxRetry] = {0, 500, 1000};
}  // namespace

LibModbusMaster::LibModbusMaster(std::string port_name, hal::ModbusConfig config)
    : port_name_(std::move(port_name)), config_(config) {}

LibModbusMaster::~LibModbusMaster() {
    close();
}

hal::ModbusResult LibModbusMaster::map_errno_to_result(int err) {
    if (err == ETIMEDOUT)
        return hal::ModbusResult::TIMEOUT;
    if (err == EBADF || err == EIO || err == ENXIO)
        return hal::ModbusResult::DISCONNECTED;
    // libmodbus 将 Modbus 协议的应用层异常码映射为了特定的错误码范围 (EMBXILFUN及以上)
    if (err >= MODBUS_ENOBASE)
        return hal::ModbusResult::EXCEPTION;
    return hal::ModbusResult::SYS_ERROR;
}

bool LibModbusMaster::open() {
    // 始终持有锁，防止多线程并发时的指针悬挂
    std::lock_guard<std::mutex> lock(bus_mutex_);

    if (is_open()) {
        return true;  // 已经打开则直接返回
    }
    if (!ctx_) {
        ctx_ = modbus_new_rtu(port_name_.c_str(),
                              config_.baudrate,
                              config_.parity,
                              config_.data_bits,
                              config_.stop_bits);
        if (!ctx_) {
            spdlog::error("[LibModbusMaster] modbus_new_rtu({}) failed: {}",
                          port_name_,
                          modbus_strerror(errno));
            last_error_.store(hal::ModbusResult::SYS_ERROR);
            return false;
        }

        // ─── 优化点 2：嵌入式专供，尝试开启内核级 RS485 硬件控制模式 ───
        // 使得底层串口驱动能够自动拉高/拉低 RTS (DE/RE) 引脚，实现真正的半双工通信
        if (modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485) == -1) {
            // 注意：使用自带硬件自动换向的 USB-RS485 转换器时可能会报错（不支持此 ioctl）。
            // 降级为 debug 日志，不阻断执行流程。但如果是主控原生引脚，这一步必须由内核接管。
            spdlog::debug(
                "[LibModbusMaster] RS485 mode natively unsupported on {} (assuming auto-direction "
                "hardware): {}",
                port_name_,
                modbus_strerror(errno));
        }

        modbus_set_response_timeout(ctx_, 0, 500000);  // 默认 500ms
        modbus_set_byte_timeout(ctx_, 0, 50000);       // 默认 50ms 字节间隙
    }

    if (modbus_connect(ctx_) == 0) {
        connected_.store(true);
        last_error_.store(hal::ModbusResult::OK);
        spdlog::info("[LibModbusMaster] opened: {} @ {} baud", port_name_, config_.baudrate);
        return true;
    }

    // ─── 优化点 1：移除底层睡眠重试，贯彻“快速失败”原则 ───
    // 连接失败直接清理现场并退出，让外层设备管理状态机负责非阻塞的定时重连
    int err = errno;
    spdlog::warn(
        "[LibModbusMaster] modbus_connect({}) failed: {}", port_name_, modbus_strerror(err));

    modbus_free(ctx_);
    ctx_ = nullptr;
    connected_.store(false);
    last_error_.store(map_errno_to_result(err));

    return false;
}

void LibModbusMaster::close() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        connected_.store(false);
        spdlog::debug("[LibModbusMaster] closed: {}", port_name_);
    }
}

bool LibModbusMaster::is_open() const {
    // connected_ 是 atomic<bool>，无锁读取无数据竞争
    return connected_.load();
}

// 模板定义必须在所有调用旹之前（同一翻译单元内实例化）
template <typename F>
int LibModbusMaster::execute_with_retry(int slave_id, const char* op_name, F&& modbus_func) {
    std::lock_guard<std::mutex> lock(bus_mutex_);

    if (!ctx_) {
        last_error_.store(hal::ModbusResult::DISCONNECTED);
        return -1;
    }

    // Modbus RTU 合法从站地址：0（广播写入）~247；248-255 为协议保留地址
    if (slave_id < 0 || slave_id > 247) {
        spdlog::warn("[LibModbusMaster] invalid slave_id={} (valid: 0-247)", slave_id);
        last_error_.store(hal::ModbusResult::SYS_ERROR);
        return -1;
    }

    if (modbus_set_slave(ctx_, slave_id) < 0) {
        last_error_.store(hal::ModbusResult::SYS_ERROR);
        return -1;
    }

    int ret = modbus_func();
    if (ret >= 0) {
        last_error_.store(hal::ModbusResult::OK);
        return ret;
    }

    // 发生错误，分析原因
    int err = errno;
    auto result_status = map_errno_to_result(err);
    // ─── 优化点 3：日志降级，防止 Timeout 刷爆 eMMC ───
    if (result_status == hal::ModbusResult::TIMEOUT) {
        spdlog::debug("[LibModbusMaster] {}(slave={}) timeout.", op_name, slave_id);
    } else {
        spdlog::warn(
            "[LibModbusMaster] {}(slave={}) failed: {}", op_name, slave_id, modbus_strerror(err));
    }

    // 如果是底层硬件离线 (设备被拔出、严重电磁干扰导致串口驱动失效)，尝试自动复位连接
    if (result_status == hal::ModbusResult::DISCONNECTED) {
        spdlog::info("[LibModbusMaster] attempting to recover connection on {}", port_name_);
        modbus_close(ctx_);
        connected_.store(false);
        if (modbus_connect(ctx_) == 0) {
            connected_.store(true);
            modbus_flush(ctx_);                // 清空收发缓冲区残留数据
            modbus_set_slave(ctx_, slave_id);  // 重新设置 slave id
            ret = modbus_func();               // 重新执行一次
            if (ret >= 0) {
                spdlog::info("[LibModbusMaster] recovery successful");
                last_error_.store(hal::ModbusResult::OK);
                return ret;
            }
            err = errno;  // 重连后执行仍失败，更新为最新 errno
        } else {
            // modbus_connect 本身失败，捕获其 errno 覆盖旧值
            err = errno;
            spdlog::warn(
                "[LibModbusMaster] reconnect failed on {}: {}", port_name_, modbus_strerror(err));
        }
    }

    last_error_.store(map_errno_to_result(err));
    return -1;
}

int LibModbusMaster::read_registers(int slave_id, int addr, int count, uint16_t* out) {
    // FC03 单次最多读取 125 个保持寄存器（Modbus 规范上限）
    if (!out || addr < 0 || count <= 0 || count > 125) {
        spdlog::warn(
            "[LibModbusMaster] read_registers invalid args: addr={} count={}", addr, count);
        last_error_.store(hal::ModbusResult::SYS_ERROR);
        return -1;
    }
    return execute_with_retry(slave_id, "read_registers", [&]() {
        return modbus_read_registers(ctx_, addr, count, out);
    });
}

int LibModbusMaster::write_register(int slave_id, int addr, uint16_t val) {
    if (addr < 0) {
        spdlog::warn("[LibModbusMaster] write_register invalid addr={}", addr);
        last_error_.store(hal::ModbusResult::SYS_ERROR);
        return -1;
    }
    return execute_with_retry(
        slave_id, "write_register", [&]() { return modbus_write_register(ctx_, addr, val); });
}

int LibModbusMaster::write_registers(int slave_id, int addr, int count, const uint16_t* vals) {
    // FC16 单次最多写入 123 个保持寄存器（Modbus 规范上限）
    if (!vals || addr < 0 || count <= 0 || count > 123) {
        spdlog::warn(
            "[LibModbusMaster] write_registers invalid args: addr={} count={}", addr, count);
        last_error_.store(hal::ModbusResult::SYS_ERROR);
        return -1;
    }
    return execute_with_retry(slave_id, "write_registers", [&]() {
        // 优化：消除 std::vector 的内存分配。
        // 如果你的 libmodbus 版本较老，头文件没有使用 const uint16_t*，这里使用 const_cast
        // 保证兼容性
        return modbus_write_registers(ctx_, addr, count, const_cast<uint16_t*>(vals));
    });
}

void LibModbusMaster::set_timeout_ms(int ms) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    if (!ctx_)
        return;
    modbus_set_response_timeout(ctx_, ms / 1000, static_cast<uint32_t>((ms % 1000) * 1000));
}

}  // namespace robot::driver
