#include <cstring>
#include "pv_cleaning_robot/device/gps_device.h"
#include <chrono>

namespace robot::device {

GpsDevice::GpsDevice(std::shared_ptr<hal::ISerialPort> serial)
    : serial_(std::move(serial))
{
}

GpsDevice::~GpsDevice()
{
    close();
}

bool GpsDevice::open()
{
    if (!serial_->open()) return false;
    running_.store(true);
    read_thread_ = std::thread(&GpsDevice::read_loop, this);
    return true;
}

void GpsDevice::close()
{
    running_.store(false);
    if (read_thread_.joinable()) read_thread_.join();
    serial_->close();
}

DeviceError GpsDevice::set_output_rate(int hz)
{
    // PMTK314 设置 NMEA 输出句子类型，PMTK220 设置更新率
    // 常见模组（MTK 芯片）：$PMTK220,<ms>*<checksum>\r\n
    int interval_ms = 1000;
    if      (hz >= 10) interval_ms = 100;
    else if (hz >= 5)  interval_ms = 200;
    else               interval_ms = 1000;

    // 计算 NMEA 校验和
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "PMTK220,%d", interval_ms);
    uint8_t cksum = 0;
    for (int i = 0; i < len; ++i) cksum ^= static_cast<uint8_t>(buf[i]);

    char cmd[80];
    snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", buf, cksum);
    auto* ptr = reinterpret_cast<const uint8_t*>(cmd);
    int written = serial_->write(ptr, strlen(cmd));
    if (written < 0) return DeviceError::COMM_TIMEOUT;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return DeviceError::OK;
}

DeviceError GpsDevice::hot_restart()
{
    // PMTK101: 热启动（保留历书/星历）
    const char* cmd = "$PMTK101*32\r\n";
    auto* ptr = reinterpret_cast<const uint8_t*>(cmd);
    int written = serial_->write(ptr, strlen(cmd));
    if (written < 0) return DeviceError::COMM_TIMEOUT;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return DeviceError::OK;
}

DeviceError GpsDevice::cold_restart()
{
    // PMTK103: 冷启动（清除所有辅助数据）
    const char* cmd = "$PMTK103*30\r\n";
    auto* ptr = reinterpret_cast<const uint8_t*>(cmd);
    int written = serial_->write(ptr, strlen(cmd));
    if (written < 0) return DeviceError::COMM_TIMEOUT;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    return DeviceError::OK;
}

GpsDevice::GpsData GpsDevice::get_latest() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<GpsData>(diag_);
}

GpsDevice::Diagnostics GpsDevice::get_diagnostics() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_;
}

void GpsDevice::read_loop()
{
    uint8_t byte_buf[128];
    

    while (running_.load()) {
        int n = serial_->read(byte_buf, sizeof(byte_buf), 50);
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            char c = static_cast<char>(byte_buf[i]);
            if (c == '\n') {
                if (!line_buf_.empty()) {
                    // 尝试解析
                    parser_.parse_sentence(line_buf_);
                    const auto& d = parser_.get_data();

                    std::lock_guard<std::mutex> lk(mtx_);
                    ++diag_.sentence_count;

                    bool was_fixed = diag_.valid;
                    // 拷贝 GpsData 字段
                    static_cast<GpsData&>(diag_) = d;

                    if (!d.valid && was_fixed) {
                        ++diag_.fix_loss_count;
                    }
                    line_buf_.clear();
                }
            } else if (c != '\r') {
                if (line_buf_.size() < 256) {  // 防止行缓冲区溢出
                    line_buf_ += c;
                } else {
                    // 异常：过长的行，清空缓冲
                    line_buf_.clear();
                    std::lock_guard<std::mutex> lk(mtx_);
                    ++diag_.parse_error_count;
                }
            }
        }
    }
}

} // namespace robot::device
