#pragma once
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <string>

namespace robot::service {

/// @brief 遥测上报服务（HEALTH / DIAGNOSTICS 双模式，1 Hz）
///
/// HEALTH 模式（生产）：精简 Status 字段，适合 4G/LoRaWAN 低带宽场景。
/// DIAGNOSTICS 模式（开发）：完整 Diagnostics 字段，便于快速定位问题。
/// 通过 config.json 的 diagnostics.mode 字段在启动时选择，无需更换类。
///
/// 当 local_log_path 非空时，update() 在每次上报时额外把 JSON payload 以
/// JSONL 格式追加写入本地文件，完全独立于 MQTT/LoRaWAN，离线测试阶段也可用。
class HealthService : public middleware::IRunnable {
public:
    enum class Mode { HEALTH, DIAGNOSTICS };

    HealthService(std::shared_ptr<device::WalkMotorGroup> walk,
                  std::shared_ptr<device::BrushMotor>     brush,
                  std::shared_ptr<device::BMS>            bms,
                  std::shared_ptr<device::ImuDevice>      imu,
                  std::shared_ptr<device::GpsDevice>      gps,
                  std::shared_ptr<CloudService>           cloud,
                  Mode                                    mode = Mode::HEALTH,
                  std::string                             local_log_path = "");

    void update() override;  ///< 由 ThreadExecutor 调用

private:
    std::string build_payload() const;

    std::shared_ptr<device::WalkMotorGroup> walk_;
    std::shared_ptr<device::BrushMotor>     brush_;
    std::shared_ptr<device::BMS>        bms_;
    std::shared_ptr<device::ImuDevice>  imu_;
    std::shared_ptr<device::GpsDevice>  gps_;
    std::shared_ptr<CloudService>       cloud_;
    Mode                                mode_;
    /// 预分配 JSON 键树，build_payload() 中复用以减少内存分配。
    /// 线程约束：update()/build_payload() 必须由单一线程（ThreadExecutor）调用，
    /// 不支持并发 build_payload()。若需多线程访问，须在调用方加锁。
    mutable nlohmann::json              j_;
    std::ofstream                       local_log_file_; ///< 本地 JSONL 落盘文件（local_log_path 非空时打开）
};

} // namespace robot::service
