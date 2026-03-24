/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-23 00:06:17
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/health_service.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include <nlohmann/json.hpp>

#include "pv_cleaning_robot/service/health_service.h"

namespace robot::service {

HealthService::HealthService(std::shared_ptr<device::WalkMotor> walk,
                             std::shared_ptr<device::BrushMotor> brush,
                             std::shared_ptr<device::BMS> bms,
                             std::shared_ptr<device::ImuDevice> imu,
                             std::shared_ptr<device::GpsDevice> gps,
                             std::shared_ptr<CloudService> cloud)
    : walk_(std::move(walk))
    , brush_(std::move(brush))
    , bms_(std::move(bms))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , cloud_(std::move(cloud)) {
    // 预建 JSON 键树：键名字符串只分配一次，后续 update() 只改数值
    j_ = {{"walk", {{"rpm", 0}, {"current", 0.0f}, {"fault", false}}},
          {"brush", {{"running", false}, {"fault", false}}},
          {"battery", {{"soc", 0.0f}, {"voltage", 0.0f}, {"charging", false}, {"alarm", false}}},
          {"imu", {{"pitch", 0.0f}, {"roll", 0.0f}, {"valid", false}}},
          {"gps", {{"lat", 0.0}, {"lon", 0.0}, {"fix", 0}, {"valid", false}}}};
}

void HealthService::update() {
    cloud_->publish_telemetry(build_payload());
}

std::string HealthService::build_payload() const {
    auto ws = walk_->get_status();
    auto bs = brush_->get_status();
    auto bms = bms_->get_data();
    auto imu = imu_->get_latest();
    auto gps = gps_->get_latest();

    // 就地更新预分配 JSON（键已存在，不触发堆分配）
    j_["walk"]["rpm"] = ws.speed_rpm;
    j_["walk"]["current"] = ws.torque_a;
    j_["walk"]["fault"] = (ws.fault != protocol::WalkMotorFault::NONE);

    j_["brush"]["running"] = bs.running;
    j_["brush"]["fault"] = bs.fault;

    j_["battery"]["soc"] = bms.soc_pct;
    j_["battery"]["voltage"] = bms.voltage_v;
    j_["battery"]["charging"] = bms.charging;
    j_["battery"]["alarm"] = bms.alarm_flags != 0;

    j_["imu"]["pitch"] = imu.pitch_deg;
    j_["imu"]["roll"] = imu.roll_deg;
    j_["imu"]["valid"] = imu.valid;

    j_["gps"]["lat"] = gps.latitude;
    j_["gps"]["lon"] = gps.longitude;
    j_["gps"]["fix"] = gps.fix_quality;
    j_["gps"]["valid"] = gps.valid;

    return j_.dump();
}

}  // namespace robot::service
