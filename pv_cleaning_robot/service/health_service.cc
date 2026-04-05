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
#include <filesystem>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/service/health_service.h"

namespace robot::service {

HealthService::HealthService(std::shared_ptr<device::WalkMotorGroup> walk,
                             std::shared_ptr<device::BrushMotor>     brush,
                             std::shared_ptr<device::BMS>            bms,
                             std::shared_ptr<device::ImuDevice>      imu,
                             std::shared_ptr<device::GpsDevice>      gps,
                             std::shared_ptr<CloudService>           cloud,
                             Mode                                    mode,
                             std::string                             local_log_path)
    : walk_(std::move(walk))
    , brush_(std::move(brush))
    , bms_(std::move(bms))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , cloud_(std::move(cloud))
    , mode_(mode) {
    // 本地 JSONL 日志文件（仅 local_log_path 非空时开启，独立于 MQTT/LoRaWAN）
    if (!local_log_path.empty()) {
        std::filesystem::create_directories(
            std::filesystem::path(local_log_path).parent_path());
        local_log_file_.open(local_log_path, std::ios::app);
    }
    // 预建 JSON 键树：键名字符串只分配一次，后续 update() 只改数值
    if (mode_ == Mode::HEALTH) {
        // HEALTH 模式：精简字段（avg rpm/current/fault + 温度）
        j_ = {{"walk",    {{"rpm", 0.0f}, {"current", 0.0f}, {"fault", false}, {"temp", 0.0f}}},
              {"brush",   {{"running", false}, {"fault", false}}},
              {"battery", {{"soc", 0.0f}, {"voltage", 0.0f}, {"charging", false}, {"alarm", false}}},
              {"imu",     {{"pitch", 0.0f}, {"roll", 0.0f}, {"valid", false}}},
              {"gps",     {{"lat", 0.0}, {"lon", 0.0}, {"fix", 0}, {"valid", false}}}};
    } else {
        // DIAGNOSTICS 模式：每轮独立诊断字段
        nlohmann::json wdummy = {{"rpm",0.0f},{"target",0.0f},{"current",0.0f},
                                  {"can_err",0},{"fault",false},{"fault_code",0},{"online",false}};
        j_ = {{"walk",
               {{"lt", wdummy}, {"rt", wdummy}, {"lb", wdummy}, {"rb", wdummy},
                {"temp", 0.0f}, {"ctrl_frames", 0u}, {"ctrl_err", 0u}}},
              {"brush",
               {{"rpm", 0}, {"target", 0}, {"current", 0.0f},
                {"voltage", 0.0f}, {"temp", 0.0f}, {"stalls", 0}, {"comm_err", 0}}},
              {"bms",
               {{"soc", 0.0f}, {"voltage", 0.0f}, {"current", 0.0f}, {"temp", 0.0f},
                {"cell_max", 0.0f}, {"cell_min", 0.0f}, {"remain_ah", 0.0f},
                {"cycles", 0}, {"alarm", 0}}},
              {"imu",
               {{"accel", {0.0f, 0.0f, 0.0f}}, {"gyro", {0.0f, 0.0f, 0.0f}},
                {"pitch", 0.0f}, {"roll", 0.0f}, {"yaw", 0.0f},
                {"frame_rate", 0}, {"parse_errors", 0}}},
              {"gps",
               {{"lat", 0.0}, {"lon", 0.0}, {"alt", 0.0f}, {"speed", 0.0f},
                {"sats", 0}, {"hdop", 0.0f}, {"fix", 0}, {"sentences", 0}}}};
    }
}

void HealthService::update() {
    const std::string payload = build_payload();
    if (cloud_) cloud_->publish_telemetry(payload);  // cloud_ 为 nullptr 时（单元测试场景）跳过
    // 本地 JSONL 落盘：每条记录一行，独立于网络，离线测试直接 cat 查看
    if (local_log_file_.is_open()) {
        local_log_file_ << payload << '\n';
        if (!local_log_file_.flush()) {
            // flush 失败通常意味着磁盘满或 I/O 错误；关闭文件停止反复写失败
            spdlog::error("[HealthService] local log flush failed (disk full?), closing file");
            local_log_file_.close();
        }
    }
}

std::string HealthService::build_payload() const {
    if (mode_ == Mode::DIAGNOSTICS) {
        // ── 完整诊断模式：每轮独立字段 ────────────────────────
        auto gd  = walk_->get_group_diagnostics();
        auto bd  = brush_->get_diagnostics();
        auto bms = bms_->get_diagnostics();
        auto imu = imu_->get_diagnostics();
        auto gps = gps_->get_diagnostics();

        // 每轮进行独立诊断字段填充
        static constexpr const char* kWn[4] = {"lt", "rt", "lb", "rb"};
        for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
            auto& wd = gd.wheel[i];
            j_["walk"][kWn[i]]["rpm"]        = wd.speed_rpm;
            j_["walk"][kWn[i]]["target"]     = wd.target_value;
            j_["walk"][kWn[i]]["current"]    = wd.torque_a;
            j_["walk"][kWn[i]]["can_err"]    = wd.can_err_count;
            j_["walk"][kWn[i]]["fault"]      = (wd.fault != protocol::WalkMotorFault::NONE);
            j_["walk"][kWn[i]]["fault_code"] = static_cast<int>(wd.fault);
            j_["walk"][kWn[i]]["online"]     = wd.online;
        }
        j_["walk"]["temp"]        = 0.0f;  // 温度查询尚未实现，占位
        j_["walk"]["ctrl_frames"] = gd.ctrl_frame_count;
        j_["walk"]["ctrl_err"]    = gd.ctrl_err_count;

        // 辊刷电机
        j_["brush"]["rpm"]      = bd.actual_rpm;
        j_["brush"]["target"]   = bd.target_rpm;
        j_["brush"]["current"]  = bd.current_a;
        j_["brush"]["voltage"]  = bd.bus_voltage_v;
        j_["brush"]["temp"]     = bd.temperature_c;
        j_["brush"]["stalls"]   = bd.stall_count;
        j_["brush"]["comm_err"] = bd.comm_error_count;

        // BMS
        j_["bms"]["soc"]       = bms.soc_pct;
        j_["bms"]["voltage"]   = bms.voltage_v;
        j_["bms"]["current"]   = bms.current_a;
        j_["bms"]["temp"]      = bms.temperature_c;
        j_["bms"]["cell_max"]  = bms.cell_voltage_max_v;
        j_["bms"]["cell_min"]  = bms.cell_voltage_min_v;
        j_["bms"]["remain_ah"] = bms.remaining_capacity_ah;
        j_["bms"]["cycles"]    = bms.cycle_count;
        j_["bms"]["alarm"]     = bms.alarm_flags;

        // IMU
        j_["imu"]["accel"]        = {imu.accel[0], imu.accel[1], imu.accel[2]};
        j_["imu"]["gyro"]         = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
        j_["imu"]["pitch"]        = imu.pitch_deg;
        j_["imu"]["roll"]         = imu.roll_deg;
        j_["imu"]["yaw"]          = imu.yaw_deg;
        j_["imu"]["frame_rate"]   = imu.frame_rate_hz;
        j_["imu"]["parse_errors"] = imu.parse_error_count;

        // GPS
        j_["gps"]["lat"]       = gps.latitude;
        j_["gps"]["lon"]       = gps.longitude;
        j_["gps"]["alt"]       = gps.altitude_m;
        j_["gps"]["speed"]     = gps.speed_m_s;
        j_["gps"]["sats"]      = gps.satellites_used;
        j_["gps"]["hdop"]      = gps.hdop;
        j_["gps"]["fix"]       = gps.fix_quality;
        j_["gps"]["sentences"] = gps.sentence_count;
    } else {
        // ── 精简健康模式：4轮平均状态 ────────────────────────
        auto gs  = walk_->get_group_status();
        auto bs  = brush_->get_status();
        auto bms = bms_->get_data();
        auto imu = imu_->get_latest();
        auto gps = gps_->get_latest();

        // 先2轮平均转速和电流；任意轮故障则标志故障
        float avg_rpm = 0.0f, avg_torque = 0.0f;
        bool any_fault = false;
        for (auto& w : gs.wheel) {
            avg_rpm    += w.speed_rpm;
            avg_torque += w.torque_a;
            any_fault  |= (w.fault != protocol::WalkMotorFault::NONE);
        }
        avg_rpm    /= static_cast<float>(device::WalkMotorGroup::kWheelCount);
        avg_torque /= static_cast<float>(device::WalkMotorGroup::kWheelCount);

        j_["walk"]["rpm"]     = avg_rpm;
        j_["walk"]["current"] = avg_torque;
        j_["walk"]["fault"]   = any_fault;
        j_["walk"]["temp"]    = 0.0f;  // 温度查询尚未实现，占位

        j_["brush"]["running"] = bs.running;
        j_["brush"]["fault"]   = bs.fault;

        j_["battery"]["soc"]      = bms.soc_pct;
        j_["battery"]["voltage"]  = bms.voltage_v;
        j_["battery"]["charging"] = bms.charging;
        j_["battery"]["alarm"]    = bms.alarm_flags != 0;

        j_["imu"]["pitch"] = imu.pitch_deg;
        j_["imu"]["roll"]  = imu.roll_deg;
        j_["imu"]["valid"] = imu.valid;

        j_["gps"]["lat"]   = gps.latitude;
        j_["gps"]["lon"]   = gps.longitude;
        j_["gps"]["fix"]   = gps.fix_quality;
        j_["gps"]["valid"] = gps.valid;
    }
    return j_.dump();
}

}  // namespace robot::service
