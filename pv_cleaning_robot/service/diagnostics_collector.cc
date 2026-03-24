#include <nlohmann/json.hpp>

#include "pv_cleaning_robot/service/diagnostics_collector.h"

namespace robot::service {

DiagnosticsCollector::DiagnosticsCollector(std::shared_ptr<device::WalkMotor> walk,
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
    j_ = {{"walk",
           {{"rpm", 0.0f},
            {"target_rpm", 0.0f},
            {"current", 0.0f},
            {"can_errors", 0},
            {"fault", false},
            {"fault_code", 0},
            {"online", false}}},
          {"brush",
           {{"rpm", 0},
            {"target", 0},
            {"current", 0.0f},
            {"voltage", 0.0f},
            {"temp", 0.0f},
            {"stalls", 0},
            {"comm_err", 0}}},
          {"bms",
           {{"soc", 0.0f},
            {"voltage", 0.0f},
            {"current", 0.0f},
            {"temp", 0.0f},
            {"cell_max", 0.0f},
            {"cell_min", 0.0f},
            {"remain_ah", 0.0f},
            {"cycles", 0},
            {"alarm", 0}}},
          {"imu",
           {{"accel", {0.0f, 0.0f, 0.0f}},
            {"gyro", {0.0f, 0.0f, 0.0f}},
            {"pitch", 0.0f},
            {"roll", 0.0f},
            {"yaw", 0.0f},
            {"frame_rate", 0},
            {"parse_errors", 0}}},
          {"gps",
           {{"lat", 0.0},
            {"lon", 0.0},
            {"alt", 0.0f},
            {"speed", 0.0f},
            {"sats", 0},
            {"hdop", 0.0f},
            {"fix", 0},
            {"sentences", 0}}}};
}

void DiagnosticsCollector::update() {
    cloud_->publish_telemetry(build_payload());
}

std::string DiagnosticsCollector::build_payload() const {
    auto wd = walk_->get_diagnostics();
    auto bd = brush_->get_diagnostics();
    auto bms = bms_->get_diagnostics();
    auto imu = imu_->get_diagnostics();
    auto gps = gps_->get_diagnostics();

    j_["walk"]["rpm"] = wd.speed_rpm;
    j_["walk"]["target_rpm"] = wd.target_value;
    j_["walk"]["current"] = wd.torque_a;
    j_["walk"]["can_errors"] = wd.can_err_count;
    j_["walk"]["fault"] = (wd.fault != protocol::WalkMotorFault::NONE);
    j_["walk"]["fault_code"] = static_cast<int>(wd.fault);
    j_["walk"]["online"] = wd.online;

    // 辊刷电机
    j_["brush"]["rpm"] = bd.actual_rpm;
    j_["brush"]["target"] = bd.target_rpm;
    j_["brush"]["current"] = bd.current_a;
    j_["brush"]["voltage"] = bd.bus_voltage_v;
    j_["brush"]["temp"] = bd.temperature_c;
    j_["brush"]["stalls"] = bd.stall_count;
    j_["brush"]["comm_err"] = bd.comm_error_count;

    // BMS
    j_["bms"]["soc"] = bms.soc_pct;
    j_["bms"]["voltage"] = bms.voltage_v;
    j_["bms"]["current"] = bms.current_a;
    j_["bms"]["temp"] = bms.temperature_c;
    j_["bms"]["cell_max"] = bms.cell_voltage_max_v;
    j_["bms"]["cell_min"] = bms.cell_voltage_min_v;
    j_["bms"]["remain_ah"] = bms.remaining_capacity_ah;
    j_["bms"]["cycles"] = bms.cycle_count;
    j_["bms"]["alarm"] = bms.alarm_flags;

    // IMU
    j_["imu"]["accel"] = {imu.accel[0], imu.accel[1], imu.accel[2]};
    j_["imu"]["gyro"] = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    j_["imu"]["pitch"] = imu.pitch_deg;
    j_["imu"]["roll"] = imu.roll_deg;
    j_["imu"]["yaw"] = imu.yaw_deg;
    j_["imu"]["frame_rate"] = imu.frame_rate_hz;
    j_["imu"]["parse_errors"] = imu.parse_error_count;

    // GPS
    j_["gps"]["lat"] = gps.latitude;
    j_["gps"]["lon"] = gps.longitude;
    j_["gps"]["alt"] = gps.altitude_m;
    j_["gps"]["speed"] = gps.speed_m_s;
    j_["gps"]["sats"] = gps.satellites_used;
    j_["gps"]["hdop"] = gps.hdop;
    j_["gps"]["fix"] = gps.fix_quality;
    j_["gps"]["sentences"] = gps.sentence_count;

    return j_.dump();
}

}  // namespace robot::service
