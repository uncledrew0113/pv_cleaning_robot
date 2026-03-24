#pragma once
#include <cstdint>

namespace robot::device {

/// @brief 设备层操作结果码（Level-3 应用语义判定）
enum class DeviceError : int {
    OK             = 0,   ///< 操作成功
    COMM_TIMEOUT   = 1,   ///< Level-1: 通信超时（libmodbus/CAN recv 超时）
    COMM_CRC       = 2,   ///< Level-1: CRC 校验失败
    COMM_NO_RESP   = 3,   ///< Level-1: 无响应
    EXEC_FAILED    = 4,   ///< Level-3: 设备收到指令但执行失败（状态寄存器报错）
    OVERCURRENT    = 5,   ///< Level-3: 过流保护
    OVERVOLTAGE    = 6,   ///< Level-3: 过压
    UNDERVOLTAGE   = 7,   ///< Level-3: 欠压
    OVERTEMP       = 8,   ///< Level-3: 过温
    STALL          = 9,   ///< Level-3: 堵转
    NOT_OPEN       = 10,  ///< 设备未打开
    INVALID_PARAM  = 11,  ///< 调用方传参越界
    NOT_SUPPORTED  = 12,  ///< 功能不支持
};

/// 将 DeviceError 转为可读字符串
inline const char* device_error_str(DeviceError e) {
    switch (e) {
        case DeviceError::OK:            return "OK";
        case DeviceError::COMM_TIMEOUT:  return "COMM_TIMEOUT";
        case DeviceError::COMM_CRC:      return "COMM_CRC";
        case DeviceError::COMM_NO_RESP:  return "COMM_NO_RESP";
        case DeviceError::EXEC_FAILED:   return "EXEC_FAILED";
        case DeviceError::OVERCURRENT:   return "OVERCURRENT";
        case DeviceError::OVERVOLTAGE:   return "OVERVOLTAGE";
        case DeviceError::UNDERVOLTAGE:  return "UNDERVOLTAGE";
        case DeviceError::OVERTEMP:      return "OVERTEMP";
        case DeviceError::STALL:         return "STALL";
        case DeviceError::NOT_OPEN:      return "NOT_OPEN";
        case DeviceError::INVALID_PARAM: return "INVALID_PARAM";
        case DeviceError::NOT_SUPPORTED: return "NOT_SUPPORTED";
        default:                         return "UNKNOWN";
    }
}

}  // namespace robot::device
