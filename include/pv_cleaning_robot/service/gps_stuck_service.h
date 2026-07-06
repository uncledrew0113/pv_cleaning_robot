/*
 * GPS 卡滞检测服务接口。
 *
 * 本服务只在清扫任务运行期间启用，用 GPS 质量和速度持续性判断“机器人运行但位置未变化”。
 * 恢复流程或非任务状态必须暂停监控，避免急停、倒车、驱动重启期间产生误报。
 */
/**
 * @file gps_stuck_service.h
 * @brief GPS 卡滞检测服务接口。
 *
 * 本服务在任务运行期间根据 GPS 位置变化判断机器人是否长时间未移动。检测结果进入
 * DiagnosticsCollector，再由 ErrorManager 统一升级为故障。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"

namespace robot::service {

struct GpsStuckConfig {
    uint8_t min_fix_quality{2};
    uint8_t min_satellites_used{6};
    float max_hdop{1.2f};
    float max_pdop{2.5f};
    float moving_speed_mps{0.03f};
    int moving_speed_confirm_samples{2};
    std::chrono::milliseconds stuck_timeout{std::chrono::milliseconds(8000)};
    std::chrono::milliseconds sample_stale_timeout{std::chrono::milliseconds(3000)};
};

enum class GpsStuckState {
    kDisabled,
    kWaitingForGps,
    kGpsQualityPoor,
    kMonitoring,
    kStuck,
};

struct GpsStuckStatus {
    GpsStuckState state{GpsStuckState::kWaitingForGps};
    bool robot_stuck_detected{false};
    std::string reason{"waiting_for_gps"};
    uint64_t last_utc_timestamp_ms{0};
    float latest_speed_m_s{0.0f};
    float latest_hdop{0.0f};
    float latest_pdop{0.0f};
    uint8_t latest_satellites_used{0};
    uint8_t latest_fix_quality{0};
};

/// @brief 基于普通 GPS 的机器人卡滞检测服务。
class GpsStuckService : public middleware::IRunnable {
   public:
    using GpsData = device::GpsDevice::GpsData;
    using GpsReader = std::function<GpsData()>;
    using ClockFn = std::function<std::chrono::steady_clock::time_point()>;
    using Config = GpsStuckConfig;
    using State = GpsStuckState;
    using Status = GpsStuckStatus;

    explicit GpsStuckService(std::shared_ptr<device::GpsDevice> gps);
    GpsStuckService(std::shared_ptr<device::GpsDevice> gps, Config config);
    explicit GpsStuckService(GpsReader gps_reader);
    GpsStuckService(GpsReader gps_reader, Config config);
    GpsStuckService(GpsReader gps_reader, ClockFn clock);
    GpsStuckService(GpsReader gps_reader, ClockFn clock, Config config);

    void update() override;
    void reset();
    void clear_stuck_detection();
    void set_monitoring_enabled(bool enabled);
    Status get_status() const;

   private:
    bool has_basic_fix(const GpsData& gps) const;
    bool has_acceptable_quality(const GpsData& gps) const;
    void reset_tracking_locked();

    GpsReader gps_reader_;
    ClockFn clock_;
    Config config_;

    mutable std::mutex mtx_;
    Status status_{};
    bool enabled_{true};
    bool has_last_sample_{false};
    uint64_t last_sample_utc_ms_{0};
    std::chrono::steady_clock::time_point last_sample_at_{};
    bool tracking_started_{false};
    std::chrono::steady_clock::time_point last_motion_at_{};
    int consecutive_speed_samples_{0};
};

}  // namespace robot::service
