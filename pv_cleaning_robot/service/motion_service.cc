/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:03:29
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-23 00:07:02
 * @FilePath: /pv_cleaning_robot/pv_cleaning_robot/service/motion_service.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#include "pv_cleaning_robot/service/motion_service.h"

namespace robot::service {

MotionService::MotionService(std::shared_ptr<device::WalkMotor> walk,
                             std::shared_ptr<device::BrushMotor> brush,
                             middleware::EventBus& bus,
                             Config cfg)
    : walk_(std::move(walk)), brush_(std::move(brush)), bus_(bus), cfg_(cfg) {}

bool MotionService::start_cleaning() {
    if (walk_->enable() != device::DeviceError::OK)
        return false;
    if (walk_->set_speed(cfg_.clean_speed_rpm) != device::DeviceError::OK)
        return false;
    brush_->set_rpm(cfg_.brush_rpm);
    brush_->start();
    return true;
}

void MotionService::stop_cleaning() {
    brush_->stop();
    walk_->set_speed(0.0f);
}

bool MotionService::start_returning() {
    brush_->stop();
    if (walk_->set_speed(-cfg_.return_speed_rpm) != device::DeviceError::OK)
        return false;
    return true;
}

void MotionService::emergency_stop() {
    brush_->stop();
    walk_->disable();
}

bool MotionService::set_walk_speed(float rpm) {
    return walk_->set_speed(rpm) == device::DeviceError::OK;
}

bool MotionService::is_moving() const {
    return std::abs(walk_->get_status().speed_rpm) > 5.0f;
}

bool MotionService::is_brush_running() const {
    return brush_->get_status().running;
}

void MotionService::update() {
    walk_->update();
    brush_->update();
}

}  // namespace robot::service
