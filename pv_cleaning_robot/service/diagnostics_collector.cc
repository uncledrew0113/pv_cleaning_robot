#include "pv_cleaning_robot/service/diagnostics_collector.h"

#include <chrono>
#include <utility>

#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"

namespace robot::service {

DiagnosticsCollector::DiagnosticsCollector(std::shared_ptr<device::WalkMotorGroup> walk,
                                           std::shared_ptr<device::BrushMotor> brush,
                                           std::shared_ptr<device::BMS> bms,
                                           std::shared_ptr<device::ImuDevice> imu,
                                           std::shared_ptr<device::GpsDevice> gps,
                                           std::shared_ptr<GpsStuckService> gps_stuck)
    : walk_(std::move(walk))
    , brush_(std::move(brush))
    , bms_(std::move(bms))
    , imu_(std::move(imu))
    , gps_(std::move(gps))
    , gps_stuck_(std::move(gps_stuck)) {}

uint64_t DiagnosticsCollector::steady_now_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

uint64_t DiagnosticsCollector::system_epoch_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

domain::StreamHealth DiagnosticsCollector::update_stream(StreamState& state,
                                                         bool enabled,
                                                         uint32_t count,
                                                         uint64_t now_ms) {
    if (!enabled) {
        return domain::StreamHealth{};
    }

    // 第一次采样视为该数据流已经在当前时刻有效，避免启动期因没有旧计数误报。
    if (!state.initialized) {
        state.initialized = true;
        state.previous_count = count;
        state.last_update_ms = now_ms;
    } else if (count != state.previous_count) {
        // 使用“是否变化”而不是“大于前值”，保证 uint32_t 回绕后仍能更新时间戳。
        state.previous_count = count;
        state.last_update_ms = now_ms;
    }

    return domain::StreamHealth{true, state.last_update_ms};
}

void DiagnosticsCollector::update() {
    update_from_input(read_input_from_devices(), steady_now_ms(), system_epoch_ms());
}

DiagnosticsCollector::Input DiagnosticsCollector::read_input_from_devices() const {
    Input input{};
    if (walk_) {
        input.walk_status = walk_->get_group_status();
        input.walk_diagnostics = walk_->get_group_diagnostics();
        input.walk_feedback_expected = true;
        for (auto i = 0U; i < input.walk_feedback_enabled.size(); ++i) {
            input.walk_feedback_enabled[i] = true;
            input.walk_feedback_frame_count[i] =
                input.walk_diagnostics.wheel[i].feedback_frame_count;
            input.walk_stall_active =
                input.walk_stall_active ||
                input.walk_diagnostics.wheel[i].fault == protocol::WalkMotorFault::STALL;
        }
    }
    if (brush_) {
        input.brush_status = brush_->get_status();
        input.brush_diagnostics = brush_->get_diagnostics();
        input.brush_comm_error_count = input.brush_diagnostics.comm_error_count;
    }
    if (bms_) {
        input.bms_data = bms_->get_data();
        input.bms_diagnostics = bms_->get_diagnostics();
        input.bms_update_count = input.bms_diagnostics.update_count;
    }
    if (imu_) {
        input.imu_data = imu_->get_latest();
        input.imu_diagnostics = imu_->get_diagnostics();
        input.imu_enabled = true;
        input.imu_frame_count = input.imu_diagnostics.frame_count;
    }
    if (gps_) {
        input.gps_data = gps_->get_latest();
        input.gps_diagnostics = gps_->get_diagnostics();
        input.gps_enabled = true;
        input.gps_sentence_count = input.gps_diagnostics.sentence_count;
    }
    if (gps_stuck_) {
        input.gps_stuck = gps_stuck_->get_status().robot_stuck_detected;
    }
    return input;
}

DiagnosticsCollector::Snapshot DiagnosticsCollector::update_from_input(const Input& input,
                                                                       uint64_t now_ms,
                                                                       uint64_t epoch_ms) {
    std::lock_guard<std::mutex> lk(mtx_);

    Snapshot snapshot{};
    snapshot.ts_ms = now_ms;
    // 兼容旧测试入口：未显式传 epoch_ms 时退回 now_ms。生产 update() 始终传入
    // system_clock epoch，避免 ThingsBoard 把 telemetry 落到 1970 年时间轴。
    snapshot.epoch_ms = epoch_ms == 0u ? now_ms : epoch_ms;
    snapshot.walk_status = input.walk_status;
    snapshot.walk_diagnostics = input.walk_diagnostics;
    snapshot.brush_status = input.brush_status;
    snapshot.brush_diagnostics = input.brush_diagnostics;
    snapshot.bms_data = input.bms_data;
    snapshot.bms_diagnostics = input.bms_diagnostics;
    snapshot.imu_data = input.imu_data;
    snapshot.imu_diagnostics = input.imu_diagnostics;
    snapshot.gps_data = input.gps_data;
    snapshot.gps_diagnostics = input.gps_diagnostics;

    snapshot.error.bms_update = update_stream(bms_update_state_,
                                              true,
                                              input.bms_update_count,
                                              now_ms);
    snapshot.error.brush.error_count = input.brush_comm_error_count;
    snapshot.error.brush.enabled = true;
    snapshot.error.brush_fault_active = input.brush_diagnostics.fault_code != 0;

    snapshot.error.gps = update_stream(gps_state_,
                                       input.gps_enabled,
                                       input.gps_sentence_count,
                                       now_ms);
    snapshot.error.imu = update_stream(imu_state_,
                                       input.imu_enabled,
                                       input.imu_frame_count,
                                       now_ms);

    snapshot.error.walk_feedback_expected = input.walk_feedback_expected;
    for (auto i = 0U; i < snapshot.error.walk_feedback.size(); ++i) {
        snapshot.error.walk_feedback[i] =
            update_stream(walk_feedback_states_[i],
                          input.walk_feedback_enabled[i],
                          input.walk_feedback_frame_count[i],
                          now_ms);
    }

    snapshot.error.walk_stall_active = input.walk_stall_active;
    snapshot.error.gps_stuck = input.gps_stuck;
    latest_ = snapshot;
    return latest_;
}

DiagnosticsCollector::Snapshot DiagnosticsCollector::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_;
}

domain::DiagnosticsSnapshot DiagnosticsCollector::error_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_.error;
}

}  // namespace robot::service
