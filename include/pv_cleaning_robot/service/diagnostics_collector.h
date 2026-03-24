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

namespace robot::service {

/// @brief 诊断数据采集器（开发模式，可配置频率）
///
/// 收集所有设备的 Diagnostics 结构（完整调试数据），
/// 适合开发/调试阶段快速定位问题。
/// 生产部署时通过 config.json 切换到 HealthService。
class DiagnosticsCollector : public middleware::IRunnable {
public:
    DiagnosticsCollector(std::shared_ptr<device::WalkMotor>  walk,
                         std::shared_ptr<device::BrushMotor> brush,
                         std::shared_ptr<device::BMS>        bms,
                         std::shared_ptr<device::ImuDevice>  imu,
                         std::shared_ptr<device::GpsDevice>  gps,
                         std::shared_ptr<CloudService>       cloud);

    void update() override;  ///< 由 ThreadExecutor 以配置频率调用

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
