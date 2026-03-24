#pragma once
#include <atomic>
#include <mutex>
#include <string>

#include "pv_cleaning_robot/hal/i_modbus_master.h"

// 前置声明，避免在头文件暴露 libmodbus 类型
struct _modbus;
typedef struct _modbus modbus_t;

namespace robot::driver {

/// @brief libmodbus Modbus RTU 主站实现
/// 内部完整处理：帧构建 → CRC16 → 发送 → 等待 → 校验 → 解包
class LibModbusMaster final : public hal::IModbusMaster {
   public:
    explicit LibModbusMaster(std::string port_name, hal::ModbusConfig config = hal::ModbusConfig{});
    ~LibModbusMaster() override;

    bool open() override;
    void close() override;
    bool is_open() const override;

    int read_registers(int slave_id, int addr, int count, uint16_t* out) override;
    int write_register(int slave_id, int addr, uint16_t val) override;
    int write_registers(int slave_id, int addr, int count, const uint16_t* vals) override;

    void set_timeout_ms(int ms) override;
    hal::ModbusResult get_last_error() const override {
        return last_error_.load();
    }

   private:
    std::string port_name_;
    hal::ModbusConfig config_;
    modbus_t* ctx_{nullptr};
    // is_open() 无锁读取，用 atomic 避免数据竞争
    std::atomic<bool> connected_{false};

    // 保证同一总线上事务的串行化和并发安全
    mutable std::mutex bus_mutex_;
    std::atomic<hal::ModbusResult> last_error_{hal::ModbusResult::OK};

    // 内部泛型执行引擎：统一处理加锁、设置从站、执行动作、错误转换和重连机制
    // 使用模板而非 std::function，避免 lambda 闭包 > 16 bytes 时的堆分配（500ms 热路径优化）
    template<typename F>
    int execute_with_retry(int slave_id, const char* op_name, F&& modbus_func);

    // 错误码转换辅助函数
    hal::ModbusResult map_errno_to_result(int err);
};

}  // namespace robot::driver
