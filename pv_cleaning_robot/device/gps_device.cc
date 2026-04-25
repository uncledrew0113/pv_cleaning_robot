#include "pv_cleaning_robot/device/gps_device.h"

namespace robot::device {

GpsDevice::GpsDevice(std::shared_ptr<hal::ISerialPort> serial)
    : source_(std::make_unique<SerialGpsSource>(
          SerialGpsSourceConfig{std::move(serial)},
          [this](const GpsData& data) { on_source_message(data); },
          [this]() { on_source_parse_error(); },
          [this]() { on_source_message_count(); }))
{
}

GpsDevice::GpsDevice(std::unique_ptr<IGpsSource> source)
    : source_(std::move(source))
{
}

std::shared_ptr<GpsDevice> GpsDevice::create_gpsd(const GpsdSourceConfig& cfg)
{
    auto* owner = new GpsDevice();
    owner->source_ = std::make_unique<GpsdGpsSource>(
        cfg,
        [owner](const GpsData& data) { owner->on_source_message(data); },
        [owner]() { owner->on_source_parse_error(); },
        [owner]() { owner->on_source_message_count(); });
    return std::shared_ptr<GpsDevice>(owner);
}

GpsDevice::~GpsDevice()
{
    close();
}

bool GpsDevice::open()
{
    return source_ && source_->open();
}

void GpsDevice::close()
{
    if (source_) source_->close();
}

DeviceError GpsDevice::set_output_rate(int hz)
{
    if (!source_) return DeviceError::NOT_OPEN;
    return source_->set_output_rate(hz);
}

DeviceError GpsDevice::hot_restart()
{
    if (!source_) return DeviceError::NOT_OPEN;
    return source_->hot_restart();
}

DeviceError GpsDevice::cold_restart()
{
    if (!source_) return DeviceError::NOT_OPEN;
    return source_->cold_restart();
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

void GpsDevice::on_source_message(const GpsData& data)
{
    std::lock_guard<std::mutex> lk(mtx_);
    const bool was_fixed = diag_.valid;
    static_cast<GpsData&>(diag_) = data;
    if (was_fixed && !data.valid) {
        ++diag_.fix_loss_count;
    }
}

void GpsDevice::on_source_parse_error()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.parse_error_count;
}

void GpsDevice::on_source_message_count()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.sentence_count;
}

}  // namespace robot::device
