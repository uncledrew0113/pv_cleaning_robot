#include <cerrno>
#include <libserialport.h>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/driver/libserialport_port.h"

namespace robot::driver {

LibSerialPort::LibSerialPort(std::string port_name, hal::UartConfig config)
    : port_name_(std::move(port_name)), config_(config) {}

LibSerialPort::~LibSerialPort() {
    close();
}

bool LibSerialPort::check_sp_return(int ret_code, const char* operation) {
    if (ret_code != SP_OK) {
        char* err_msg = sp_last_error_message();
        spdlog::error("[LibSerialPort] {} failed on {}: {}", operation, port_name_, err_msg);
        sp_free_error_message(err_msg);  // 必须调用！
        last_error_.store(hal::UartResult::SYS_ERROR);
        return false;
    }
    return true;
}

bool LibSerialPort::open() {
    // 防止重入：若已打开则先安全关闭，避免旧 sp_port* 被覆写泻漏
    std::unique_lock<std::shared_mutex> lk(port_rwlock_);

    // 防止重入：若已打开则先安全关闭（调用无锁内部函数避免死锁）
    if (connected_.load()) {
        close_locked();
    }

    sp_return ret = sp_get_port_by_name(port_name_.c_str(), &port_);
    if (!check_sp_return(ret, "sp_get_port_by_name")) {
        port_ = nullptr;
        return false;
    }

    ret = sp_open(port_, SP_MODE_READ_WRITE);
    if (!check_sp_return(ret, "sp_open")) {
        sp_free_port(port_);
        port_ = nullptr;
        return false;
    }

    // 配置阶段，失败则回滚
    if (!check_sp_return(sp_set_baudrate(port_, config_.baudrate), "sp_set_baudrate") ||
        !check_sp_return(sp_set_bits(port_, config_.data_bits), "sp_set_bits") ||
        !check_sp_return(sp_set_stopbits(port_, config_.stop_bits), "sp_set_stopbits")) {
        close_locked();
        return false;
    }

    sp_parity parity = SP_PARITY_NONE;
    if (config_.parity == 'E' || config_.parity == 'e')
        parity = SP_PARITY_EVEN;
    else if (config_.parity == 'O' || config_.parity == 'o')
        parity = SP_PARITY_ODD;

    if (!check_sp_return(sp_set_parity(port_, parity), "sp_set_parity") ||
        !check_sp_return(
            sp_set_flowcontrol(port_,
                               config_.flow_control ? SP_FLOWCONTROL_RTSCTS : SP_FLOWCONTROL_NONE),
            "sp_set_flowcontrol")) {
        close_locked();
        return false;
    }

    connected_.store(true, std::memory_order_release);
    last_error_.store(hal::UartResult::OK);
    spdlog::info("[LibSerialPort] opened: {} @ {} baud {}{}{}{}",
                 port_name_,
                 config_.baudrate,
                 config_.data_bits,
                 config_.parity,
                 config_.stop_bits,
                 config_.flow_control ? " RTS/CTS" : "");
    return true;
}

void LibSerialPort::close_locked() {
    connected_.store(false);
    if (port_) {
        sp_close(port_);
        sp_free_port(port_);
        port_ = nullptr;
        spdlog::debug("[LibSerialPort] closed: {}", port_name_);
    }
}

void LibSerialPort::close() {
    // 持锁等待所有进行中的 write()/read() 完成，再释放 port_ 资源。
    // 这确保: close() 返回后任何对 port_ 的访问都已终止。
    std::unique_lock<std::shared_mutex> lk(port_rwlock_);
    close_locked();
}

bool LibSerialPort::is_open() const {
    // 通过 atomic 标志返回，避免裸指针读取引发的数据竞争
    return connected_.load();
}

int LibSerialPort::write(const uint8_t* buf, size_t len, int timeout_ms) {
    std::shared_lock<std::shared_mutex> lk(port_rwlock_);
    if (!connected_.load(std::memory_order_acquire)) {
        last_error_.store(hal::UartResult::DISCONNECTED);
        return -1;
    }

    // timeout_ms == 0 时 sp_blocking_write 退化为非阻塞写，由调用方明确传入
    int actual_timeout = (timeout_ms < 0) ? config_.write_timeout_ms : timeout_ms;
    sp_return ret = sp_blocking_write(port_, buf, len, actual_timeout);

    if (ret > 0) {
        last_error_.store(hal::UartResult::OK);
        return static_cast<int>(ret);
    } else if (ret == 0) {
        last_error_.store(hal::UartResult::TIMEOUT);
        return 0;
    } else {
        int saved_errno = errno;
        char* err_msg = sp_last_error_message();
        spdlog::warn("[LibSerialPort] write error on {}: {}", port_name_, err_msg);
        sp_free_error_message(err_msg);
        // 与 read() 对称：SP_ERR_FAIL + EIO 表示设备（如 ttyUSB）被拔出
        if (ret == SP_ERR_FAIL && saved_errno == EIO) {
            last_error_.store(hal::UartResult::DISCONNECTED);
        } else {
            last_error_.store(hal::UartResult::SYS_ERROR);
        }
        return -1;
    }
}

int LibSerialPort::read(uint8_t* buf, size_t max_len, int timeout_ms) {
    std::shared_lock<std::shared_mutex> lk(port_rwlock_);
    if (!connected_.load(std::memory_order_acquire)) {
        last_error_.store(hal::UartResult::DISCONNECTED);
        return -1;
    }
    sp_return ret = sp_blocking_read(port_, buf, max_len, timeout_ms);

    if (ret > 0) {
        last_error_.store(hal::UartResult::OK);
        return static_cast<int>(ret);
    } else if (ret == 0) {
        last_error_.store(hal::UartResult::TIMEOUT);
        return 0;
    } else {
        int saved_errno = errno;
        char* err_msg = sp_last_error_message();
        spdlog::warn("[LibSerialPort] read error on {}: {}", port_name_, err_msg);
        sp_free_error_message(err_msg);  // 必须调用！
        // 当底层返回 SP_ERR_FAIL 且 errno 为 EIO 时，大概率是物理设备(如 ttyUSB)断开了
        if (ret == SP_ERR_FAIL && saved_errno == EIO) {
            last_error_.store(hal::UartResult::DISCONNECTED);
        } else {
            last_error_.store(hal::UartResult::SYS_ERROR);
        }
        return -1;
    }
}

bool LibSerialPort::flush_input() {
    std::shared_lock<std::shared_mutex> lk(port_rwlock_);
    if (!connected_.load(std::memory_order_acquire)) {
        last_error_.store(hal::UartResult::DISCONNECTED);
        return false;
    }
    bool success = (sp_flush(port_, SP_BUF_INPUT) == SP_OK);
    last_error_.store(success ? hal::UartResult::OK : hal::UartResult::SYS_ERROR);
    return success;
}

bool LibSerialPort::flush_output() {
    std::shared_lock<std::shared_mutex> lk(port_rwlock_);
    if (!connected_.load(std::memory_order_acquire)) {
        last_error_.store(hal::UartResult::DISCONNECTED);
        return false;
    }
    bool success = (sp_flush(port_, SP_BUF_OUTPUT) == SP_OK);
    last_error_.store(success ? hal::UartResult::OK : hal::UartResult::SYS_ERROR);
    return success;
}

int LibSerialPort::bytes_available() {
    std::shared_lock<std::shared_mutex> lk(port_rwlock_);
    if (!connected_.load(std::memory_order_acquire)) {
        last_error_.store(hal::UartResult::DISCONNECTED);
        return -1;
    }
    sp_return ret = sp_input_waiting(port_);
    if (ret >= 0) {
        last_error_.store(hal::UartResult::OK);
        return static_cast<int>(ret);
    } else {
        last_error_.store(hal::UartResult::SYS_ERROR);
        return -1;
    }
}

}  // namespace robot::driver
