#pragma once
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace robot::service {

/// @brief 健康上报服务（生产模式，1 Hz）
///
/// 定期汇总 Status 结构（精简数据），通过 CloudService 上报 ThingsBoard。
/// 相对于 DiagnosticsCollector，数据量小，适合 4G/LoRaWAN 低带宽场景。
class HealthService : public middleware::IRunnable {
public:
    HealthService(std::shared_ptr<device::WalkMotor>  walk,
                  std::shared_ptr<device::BrushMotor> brush,
                  std::shared_ptr<device::BMS>        bms,
                  std::shared_ptr<device::ImuDevice>  imu,
                  std::shared_ptr<device::GpsDevice>  gps,
                  std::shared_ptr<CloudService>       cloud);

    void update() override;  ///< 由 ThreadExecutor (1Hz) 调用

private:
    std::string build_payload() const;

    std::shared_ptr<device::WalkMotor>  walk_;
    std::shared_ptr<device::BrushMotor> brush_;
    std::shared_ptr<device::BMS>        bms_;
    std::shared_ptr<device::ImuDevice>  imu_;
    std::shared_ptr<device::GpsDevice>  gps_;
    std::shared_ptr<CloudService>       cloud_;
    mutable nlohmann::json j_; ///< 预分配 JSON 键树，build_payload() 中复用
};

} // namespace robot::service
