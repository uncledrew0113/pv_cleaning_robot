/**
 * RobotFsm 状态机单元测试
 * [app][fsm]
 *
 * 测试策略：
 *   - 使用真实 WalkMotorGroup(MockCanBus) + BrushMotor(MockSerialPort)
 *     构造 MotionService（不调用 open，不启动后台线程）
 *   - NavService 使用真实构造（不调用 update()）
 *   - FaultService + EventBus 使用真实实例
 *   - 验证 dispatch<>() 后 current_state() 的正确转换
 */
#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

using namespace robot::app;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;

namespace {

MotionService::Config make_motion_config() {
    MotionService::Config cfg;
    cfg.heading_pid_en = false;
    return cfg;
}

EvScheduleStart make_schedule_start(bool at_parking_side, bool at_far_end, float passes) {
    EvScheduleStart evt;
    evt.at_parking_side = at_parking_side;
    evt.at_far_end = at_far_end;
    evt.passes = passes;
    return evt;
}

EvScheduleStart start_from_parking_side(float passes) {
    return make_schedule_start(true, false, passes);
}

}  // namespace

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct FsmFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockSerialPort> brush_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<BrushMotor> brush{
        std::make_shared<BrushMotor>(brush_serial, 0, 8192.0f, true, 0.5f)};
    std::shared_ptr<MockSerialPort> imu_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<ImuDevice> imu{std::make_shared<ImuDevice>(imu_serial)};
    std::shared_ptr<MockSerialPort> gps_serial{std::make_shared<MockSerialPort>()};
    std::shared_ptr<GpsDevice> gps{std::make_shared<GpsDevice>(gps_serial)};
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault{std::make_shared<FaultService>(bus)};
    RobotFsm fsm;

    FsmFixture()
        : motion(std::make_shared<MotionService>(group,
                                                 brush,
                                                 nullptr,
                                                 bus,
                                                 make_motion_config()))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , fsm(motion, nav, fault, bus) {
        can->open_result = true;
        can->send_result = true;
        brush_serial->open_result = true;
        brush->open();
        // 初始化 FSM
        fsm.dispatch(EvInitDone{});
    }
};

// ────────────────────────────────────────────────────────────────
// 初始状态
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: EvInitDone 后状态 == Idle", "[app][fsm]") {
    FsmFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// 调度触发 → SelfCheck → CleanFwd（从停机位启动）
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: EvScheduleStart(from parking side) → CleanFwd", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("FSM: EvScheduleStart(from far end only) → Idle（首版拒绝对侧端点启动）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(make_schedule_start(false, true, 1.0f));
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM: EvScheduleStart(passes=0.5) → Idle（首版拒绝非整数趟）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(0.5f));
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM: EvScheduleStart(unknown position) → Idle（自检失败）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(make_schedule_start(false, false, 1.0f));
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// CleanFwd → CleanReturn → CleanFwd … （往复清扫）
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvFarEndLimitSettled-> CleanReturn（1趟中）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
}

TEST_CASE("FSM: CleanReturn -EvParkingSideLimitSettled-> CleanFwd（还有趟数）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 任务完成 → Charging
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: N=1 往返完成后 → Charging", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});  // 到达对侧端点，开始返程
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvParkingSideLimitSettled{});  // 回到停机位，完成 1 个整数趟
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 故障转换
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvFaultP0-> Fault", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFaultP0{});
    REQUIRE(f.fsm.current_state() == "Fault");
}

TEST_CASE("FSM: CleanFwd -EvFaultP1-> Returning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFaultP1{});
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("FSM: CleanFwd -EvFaultP2-> CleanFwd（不转状态）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFaultP2{});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("FSM: Fault -EvFaultReset-> Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFaultP0{});
    REQUIRE(f.fsm.current_state() == "Fault");
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM: EvFaultReset outside Fault does not overwrite outward state", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 低电量返回
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvLowBattery-> Returning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvLowBattery{});
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("FSM: CleanFwd -EvPauseTask-> Paused", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    f.fsm.dispatch(EvPauseTask{});
    REQUIRE(f.fsm.current_state() == "Paused");
}

TEST_CASE("FSM: Paused task resumes to previous direction", "[app][fsm]") {
    SECTION("resume forward task") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(1.0f));
        f.fsm.dispatch(EvPauseTask{});
        REQUIRE(f.fsm.current_state() == "Paused");

        f.fsm.dispatch(EvResumeTask{});
        REQUIRE(f.fsm.current_state() == "CleanFwd");
    }

    SECTION("resume return task") {
        FsmFixture f;
        f.fsm.dispatch(start_from_parking_side(2.0f));
        f.fsm.dispatch(EvFarEndLimitSettled{});
        REQUIRE(f.fsm.current_state() == "CleanReturn");

        f.fsm.dispatch(EvPauseTask{});
        REQUIRE(f.fsm.current_state() == "Paused");

        f.fsm.dispatch(EvResumeTask{});
        REQUIRE(f.fsm.current_state() == "CleanReturn");
    }
}

TEST_CASE("FSM: manual return ends task at parking side", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(2.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fsm.dispatch(EvManualReturn{});
    REQUIRE(f.fsm.current_state() == "Returning");

    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

TEST_CASE("FSM: terminate transitions active task to Terminated", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    f.fsm.dispatch(EvTerminateTask{});
    REQUIRE(f.fsm.current_state() == "Terminated");
}

TEST_CASE("FSM: Terminated -EvFaultReset-> Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvTerminateTask{});
    REQUIRE(f.fsm.current_state() == "Terminated");

    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM: EvLowBattery in Idle does not overwrite outward state", "[app][fsm]") {
    FsmFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");

    f.fsm.dispatch(EvLowBattery{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

TEST_CASE("FSM: Returning -EvParkingSideLimitSettled-> Charging", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvLowBattery{});
    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// Charging → Idle
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: Charging -EvChargeDone-> Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
    f.fsm.dispatch(EvChargeDone{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// Charging → 再次调度
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: Charging 状态可再次 EvScheduleStart", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(start_from_parking_side(1.0f));
    f.fsm.dispatch(EvFarEndLimitSettled{});
    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");

    // 第二次调度
    f.fsm.dispatch(start_from_parking_side(1.0f));
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}
