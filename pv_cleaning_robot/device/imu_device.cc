#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <spdlog/spdlog.h>

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
    // WIT Motion RRATE 寄存器（地址 0x03）频率码，来自官方 REG.h / imu.md
    // 出厂默认值：0x06 = 10 Hz
    // 0x01=0.2Hz  0x02=0.5Hz  0x03=1Hz   0x04=2Hz   0x05=5Hz
    // 0x06=10Hz   0x07=20Hz   0x08=50Hz  0x09=100Hz 0x0B=200Hz
    uint8_t rate_code = 0x08;  // 默认 50 Hz
    if (hz <= 1)
        rate_code = 0x03;  // 1 Hz
    else if (hz <= 2)
        rate_code = 0x04;  // 2 Hz
    else if (hz <= 5)
        rate_code = 0x05;  // 5 Hz
    else if (hz <= 10)
        rate_code = 0x06;  // 10 Hz
    else if (hz <= 20)
        rate_code = 0x07;  // 20 Hz
    else if (hz <= 50)
        rate_code = 0x08;  // 50 Hz
    else if (hz <= 100)
        rate_code = 0x09;  // 100 Hz
    else
        rate_code = 0x0B;  // 200 Hz

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
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return static_cast<ImuData>(diag_);
}

ImuDevice::Diagnostics ImuDevice::get_diagnostics() const {
    std::lock_guard<hal::PiMutex> lk(mtx_);
    return diag_;
}

void ImuDevice::read_loop() {
    // ── 线程自身完成 RT 提权 + CPU 绑定 ──
    {
        sched_param sp{};
        sp.sched_priority = 68;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[ImuDevice] RT priority elevation failed: {}", strerror(rc));
        }
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(6, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            spdlog::warn("[ImuDevice] CPU 6 affinity set failed: {}", strerror(errno));
        }
        pthread_setname_np(pthread_self(), "imu_read");
    }
    uint8_t byte_buf[64];
    auto t_last_stat = std::chrono::steady_clock::now();
    uint32_t frames_since_last = 0;

    while (running_.load()) {
        if (is_configuring_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        int n = serial_->read(byte_buf, sizeof(byte_buf), 20);
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                parser_.push_byte(byte_buf[i]);
                if (parser_.frame_complete()) {
                    auto& frame = parser_.take_frame();
                    if (frame.valid) {
                        std::lock_guard<hal::PiMutex> lk(mtx_);
                        // 拷贝协议层数据到设备层结构
                        std::copy(std::begin(frame.accel),
                                  std::end(frame.accel),
                                  std::begin(diag_.accel));
                        std::copy(
                            std::begin(frame.gyro), std::end(frame.gyro), std::begin(diag_.gyro));
                        std::copy(
                            std::begin(frame.mag), std::end(frame.mag), std::begin(diag_.mag));
                        std::copy(
                            std::begin(frame.quat), std::end(frame.quat), std::begin(diag_.quat));
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
        }

        // 每秒更新一次实测帧率
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_last_stat).count();
        if (elapsed >= 1.0) {
            std::lock_guard<hal::PiMutex> lk(mtx_);
            diag_.frame_rate_hz = static_cast<float>(frames_since_last / elapsed);
            if (frames_since_last == 0) {
                diag_.valid = false;
            }
            frames_since_last = 0;
            t_last_stat = now;
        }
    }
}

DeviceError ImuDevice::send_command(const protocol::ImuProtocol::Cmd& cmd, int wait_ms) {
    if (!serial_->is_open())
        return DeviceError::NOT_OPEN;
    // 1. 挂起后台 read_loop 线程
    is_configuring_.store(true, std::memory_order_release);

    // 2. 等待后台线程进入 sleep，并清空残留数据
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    serial_->flush_input();

    // 3. 发送命令（盲发，开环控制）
    int written = serial_->write(cmd.data(), cmd.size());

    // 4. 等待传感器内部处理完成（配置生效需要时间）
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

    // 5. 恢复后台 read_loop 线程
    is_configuring_.store(false, std::memory_order_release);

    if (written < 0 || static_cast<size_t>(written) < cmd.size()) {
        return DeviceError::COMM_TIMEOUT;
    }

    return DeviceError::OK;
}

DeviceError ImuDevice::send_write_cmd(const protocol::ImuProtocol::Cmd& cmd, int wait_ms) {
    // 写寄存器前必须先发解锁指令，有效期 10 秒
    auto unlock = protocol::ImuProtocol::encode_unlock();
    auto err = send_command(unlock, 50);
    if (err != DeviceError::OK)
        return err;
    err = send_command(cmd, wait_ms);
    if (err != DeviceError::OK)
        return err;
    // 写入 Flash，掉电不丢失。WIT Motion 保存命令需 200ms 等待
    auto save = protocol::ImuProtocol::encode_save_config();
    return send_command(save, 200);
}

}  // namespace robot::device
