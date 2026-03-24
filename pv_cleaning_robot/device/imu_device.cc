#include <algorithm>
#include <chrono>
#include <ctime>
#include <thread>

#include "pv_cleaning_robot/device/imu_device.h"

namespace robot::device {

ImuDevice::ImuDevice(std::shared_ptr<hal::ISerialPort> serial) : serial_(std::move(serial)) {}

ImuDevice::~ImuDevice() {
    close();
}

bool ImuDevice::open() {
    if (!serial_->open())
        return false;
    running_.store(true);
    read_thread_ = std::thread(&ImuDevice::read_loop, this);
    return true;
}

void ImuDevice::close() {
    running_.store(false);
    if (read_thread_.joinable())
        read_thread_.join();
    serial_->close();
}

DeviceError ImuDevice::set_output_rate(int hz) {
    // WIT Motion 频率码映射（RRATE 寄存器，0x03）
    uint8_t rate_code = 0x06;  // 默认 50 Hz
    if (hz <= 1)
        rate_code = 0x01;
    else if (hz <= 2)
        rate_code = 0x02;
    else if (hz <= 5)
        rate_code = 0x03;
    else if (hz <= 10)
        rate_code = 0x04;
    else if (hz <= 20)
        rate_code = 0x05;
    else if (hz <= 50)
        rate_code = 0x06;
    else if (hz <= 100)
        rate_code = 0x07;
    else if (hz <= 200)
        rate_code = 0x08;

    auto cmd = protocol::ImuProtocol::encode_set_rate(rate_code);
    return send_write_cmd(cmd, 150);
}

DeviceError ImuDevice::calibrate_gyro() {
    auto cmd = protocol::ImuProtocol::encode_calibrate_gyro();
    return send_write_cmd(cmd, 3000);  // 校准需较长等待
}

DeviceError ImuDevice::save_config() {
    auto cmd = protocol::ImuProtocol::encode_save_config();
    return send_write_cmd(cmd, 200);
}

DeviceError ImuDevice::reset() {
    auto cmd = protocol::ImuProtocol::encode_reset();
    return send_write_cmd(cmd, 500);
}

ImuDevice::ImuData ImuDevice::get_latest() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<ImuData>(diag_);
}

ImuDevice::Diagnostics ImuDevice::get_diagnostics() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_;
}

void ImuDevice::read_loop() {
    uint8_t byte_buf[64];
    auto t_last_stat = std::chrono::steady_clock::now();
    uint32_t frames_since_last = 0;

    while (running_.load()) {
        int n = serial_->read(byte_buf, sizeof(byte_buf), 20);
        if (n <= 0)
            continue;

        for (int i = 0; i < n; ++i) {
            parser_.push_byte(byte_buf[i]);
            if (parser_.frame_complete()) {
                auto& frame = parser_.take_frame();
                if (frame.valid) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    // 拷贝协议层数据到设备层结构
                    std::copy(
                        std::begin(frame.accel), std::end(frame.accel), std::begin(diag_.accel));
                    std::copy(std::begin(frame.gyro), std::end(frame.gyro), std::begin(diag_.gyro));
                    std::copy(std::begin(frame.mag), std::end(frame.mag), std::begin(diag_.mag));
                    std::copy(std::begin(frame.quat), std::end(frame.quat), std::begin(diag_.quat));
                    diag_.roll_deg = frame.roll_deg;
                    diag_.pitch_deg = frame.pitch_deg;
                    diag_.yaw_deg = frame.yaw_deg;
                    diag_.timestamp_us = frame.timestamp_us;
                    diag_.valid = true;
                    ++diag_.frame_count;
                    ++frames_since_last;
                }
            }
        }

        // 每秒更新一次实测帧率
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_last_stat).count();
        if (elapsed >= 1.0) {
            std::lock_guard<std::mutex> lk(mtx_);
            diag_.frame_rate_hz = static_cast<float>(frames_since_last / elapsed);
            frames_since_last = 0;
            t_last_stat = now;
        }
    }
}

DeviceError ImuDevice::send_command(const protocol::ImuProtocol::Cmd& cmd, int wait_ms) {
    if (!serial_->is_open())
        return DeviceError::NOT_OPEN;
    int written = serial_->write(cmd.data(), cmd.size());
    if (written < 0 || static_cast<size_t>(written) < cmd.size()) {
        return DeviceError::COMM_TIMEOUT;
    }
    // 等待命令生效
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    return DeviceError::OK;
}

DeviceError ImuDevice::send_write_cmd(const protocol::ImuProtocol::Cmd& cmd, int wait_ms) {
    // 写寄存器前必须先发解锁指令，有效期 10 秒
    auto unlock = protocol::ImuProtocol::encode_unlock();
    auto err = send_command(unlock, 50);
    if (err != DeviceError::OK)
        return err;
    return send_command(cmd, wait_ms);
}

}  // namespace robot::device
