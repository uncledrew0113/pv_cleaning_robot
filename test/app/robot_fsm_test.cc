/**
 * RobotFsm 状态机单元测试
 * [app][fsm]
 *
 * 测试策略：
 *   - 使用真实 WalkMotorGroup(MockCanBus) + BrushMotor(MockModbusMaster)
 *     构造 MotionService（不调用 open，不启动后台线程）
 *   - NavService 使用真实构造（不调用 update()）
 *   - FaultService + EventBus 使用真实实例
 *   - 验证 dispatch<>() 后 current_state() 的正确转换
 */
#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_modbus_master.h"
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

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct FsmFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(modbus, 1)};
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
                                                 MotionService::Config{.heading_pid_en = false}))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , fsm(motion, nav, fault, bus) {
        can->open_result = true;
        can->send_result = true;
        modbus->open_result = true;
        modbus->write_reg_return = 0;
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
// 调度触发 → SelfCheck → CleanFwd（at_home=true）
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: EvScheduleStart(at_home=true) → CleanFwd", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .at_front = false, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("FSM: EvScheduleStart(at_front=true) → CleanReturn（N=0.5 恢复）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = false, .at_front = true, .passes = 0.5f});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
}

TEST_CASE("FSM: EvScheduleStart(at_home=false, at_front=false) → Idle（自检失败）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = false, .at_front = false, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// CleanFwd → CleanReturn → CleanFwd … （往复清扫）
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvFrontLimitSettled-> CleanReturn（1趟中）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
}

TEST_CASE("FSM: CleanReturn -EvRearLimitSettled-> CleanFwd（还有趟数）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 任务完成 → Charging
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: N=0.5 单程 CleanFwd -EvFrontLimit-> Charging", "[app][fsm]") {
    FsmFixture f;
    // N=0.5 → 1 个半趟（0.5*2=1），到前端即完成
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 0.5f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

TEST_CASE("FSM: N=1 往返完成后 → Charging", "[app][fsm]") {
    FsmFixture f;
    // N=1 → 2个半趟
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFrontLimitSettled{});  // 完成半趟1
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvRearLimitSettled{});  // 完成半趟2
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 故障转换
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvFaultP0-> Fault", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFaultP0{});
    REQUIRE(f.fsm.current_state() == "Fault");
}

TEST_CASE("FSM: CleanFwd -EvFaultP1-> Returning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFaultP1{});
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("FSM: CleanFwd -EvFaultP2-> CleanFwd（不转状态）", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFaultP2{});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

TEST_CASE("FSM: Fault -EvFaultReset-> Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvFaultP0{});
    REQUIRE(f.fsm.current_state() == "Fault");
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// 低电量返回
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: CleanFwd -EvLowBattery-> Returning", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvLowBattery{});
    REQUIRE(f.fsm.current_state() == "Returning");
}

TEST_CASE("FSM: Returning -EvRearLimitSettled-> Charging", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    f.fsm.dispatch(EvLowBattery{});
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// Charging → Idle
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: Charging -EvChargeDone-> Idle", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 0.5f});
    f.fsm.dispatch(EvFrontLimitSettled{});  // → Charging（N=0.5）
    REQUIRE(f.fsm.current_state() == "Charging");
    f.fsm.dispatch(EvChargeDone{});
    REQUIRE(f.fsm.current_state() == "Idle");
}

// ────────────────────────────────────────────────────────────────
// Charging → 再次调度
// ────────────────────────────────────────────────────────────────
TEST_CASE("FSM: Charging 状态可再次 EvScheduleStart", "[app][fsm]") {
    FsmFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 0.5f});
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");

    // 第二次调度
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}
