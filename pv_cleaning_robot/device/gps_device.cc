// @file gps_device.cc
// @brief GpsDevice 封装 GPS 数据源，提供统一的 open/close/重启接口。

#include "pv_cleaning_robot/device/gps_device.h"

namespace robot::device {

/// @brief 构造函数：使用串口 GPS 数据源。
GpsDevice::GpsDevice(std::shared_ptr<hal::ISerialPort> serial)
    : source_(std::make_unique<SerialGpsSource>(
          SerialGpsSourceConfig{std::move(serial)},
          [this](const GpsData& data) { on_source_message(data); },
          [this]() { on_source_parse_error(); },
          [this]() { on_source_message_count(); })) {}

GpsDevice::GpsDevice(std::unique_ptr<IGpsSource> source) : source_(std::move(source)) {}

std::shared_ptr<GpsDevice> GpsDevice::create_gpsd(const GpsdSourceConfig& cfg) {
    auto* owner = new GpsDevice();
    owner->source_ = std::make_unique<GpsdGpsSource>(
        cfg,
        [owner](const GpsData& data) { owner->on_source_message(data); },
        [owner]() { owner->on_source_parse_error(); },
        [owner]() { owner->on_source_message_count(); });
    return std::shared_ptr<GpsDevice>(owner);
}

GpsDevice::~GpsDevice() {
    close();
}

/// @brief 打开当前 GPS 数据源实例。
bool GpsDevice::open() {
    return source_ && source_->open();
}

/// @brief 请求当前 GPS 数据源尽快停止后台读取。
void GpsDevice::request_stop() {
    if (source_)
        source_->request_stop();
}

/// @brief 关闭当前 GPS 数据源。
void GpsDevice::close() {
    if (source_)
        source_->close();
}

/// @brief 设置 GPS 数据输出速率。
DeviceError GpsDevice::set_output_rate(int hz) {
    if (!source_)
        return DeviceError::NOT_OPEN;
    return source_->set_output_rate(hz);
}

/// @brief 执行 GPS 热重启。
DeviceError GpsDevice::hot_restart() {
    if (!source_)
        return DeviceError::NOT_OPEN;
    return source_->hot_restart();
}

/// @brief 执行 GPS 冷重启。
DeviceError GpsDevice::cold_restart() {
    if (!source_)
        return DeviceError::NOT_OPEN;
    return source_->cold_restart();
}

/// @brief 返回最近一条有效 GPS 数据。
GpsDevice::GpsData GpsDevice::get_latest() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<GpsData>(diag_);
}

/// @brief 返回 GPS 诊断统计数据，包括解析错误和失锁次数。
GpsDevice::Diagnostics GpsDevice::get_diagnostics() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return diag_;
}

void GpsDevice::on_source_message(const GpsData& data) {
    std::lock_guard<std::mutex> lk(mtx_);
    const bool was_fixed = diag_.valid;
    static_cast<GpsData&>(diag_) = data;
    if (was_fixed && !data.valid) {
        ++diag_.fix_loss_count;
    }
}

void GpsDevice::on_source_parse_error() {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.parse_error_count;
}

void GpsDevice::on_source_message_count() {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.sentence_count;
}

}  // namespace robot::device
