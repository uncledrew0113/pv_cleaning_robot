/**
 * 主任务链集成测试
 * [integration][task_chain]
 *
 * 覆盖场景：
 *   1. 完整 N=1 往返清扫任务链（Scheduler → FSM → Motion → FSM → Charging）
 *   2. P0 故障中断任务链（FSM → Fault → 复位 → Idle）
 *   3. P1 故障触发安全返回链（FSM → Returning → Charging）
 *   4. SafetyMonitor 触发 → emergency_override → LimitSettledEvent → FSM 转换
 *   5. FaultHandler + FSM 联动
 *
 * 关键约束（来自 CONCURRENCY.md）：
 *   - EventBus 回调不能再调用 publish()
 *   - 仅验证状态机路径与 CAN/Modbus 帧存在，不验实时时序
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <filesystem>
#include <thread>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_gpio_pin.h"
#include "../mock/mock_modbus_master.h"
#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/middleware/data_cache.h"

using namespace robot::app;
using robot::device::BrushMotor;
using robot::device::GpsDevice;
using robot::device::ImuDevice;
using robot::device::LimitSide;
using robot::device::LimitSwitch;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::middleware::SafetyMonitor;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;
using robot::service::SchedulerService;
using FaultEvent = FaultService::FaultEvent;
using FaultLevel = FaultEvent::Level;

// ────────────────────────────────────────────────────────────────
// 完整任务链构建辅助
// ────────────────────────────────────────────────────────────────
struct TaskChainFixture {
    // 底层 mock
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    std::shared_ptr<MockSerialPort> imu_sp{std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort> gps_sp{std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockGpioPin> front_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin> rear_pin{std::make_shared<MockGpioPin>()};

    // 设备
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(modbus, 1)};
    std::shared_ptr<ImuDevice> imu{std::make_shared<ImuDevice>(imu_sp)};
    std::shared_ptr<GpsDevice> gps{std::make_shared<GpsDevice>(gps_sp)};
    std::shared_ptr<LimitSwitch> front_sw{
        std::make_shared<LimitSwitch>(front_pin, LimitSide::FRONT)};
    std::shared_ptr<LimitSwitch> rear_sw{std::make_shared<LimitSwitch>(rear_pin, LimitSide::REAR)};

    // 服务 & 中间件
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault_svc{std::make_shared<FaultService>(bus)};
    SchedulerService scheduler;
    SafetyMonitor safety_mon;

    // App 层
    RobotFsm fsm;
    std::vector<FaultEvent> dispatched_faults;
    FaultHandler fault_handler;

    TaskChainFixture()
        : motion(std::make_shared<MotionService>(group,
                                                 brush,
                                                 nullptr,
                                                 bus,
                                                 MotionService::Config{.heading_pid_en = false}))
        , nav(std::make_shared<NavService>(group, imu, gps))
        , safety_mon(group, front_sw, rear_sw, bus)
        , fsm(motion, nav, fault_svc, bus)
        , fault_handler(motion, bus, [this](FaultEvent e) {
            dispatched_faults.push_back(e);
            // 将故障转发到 FSM（在回调外异步，此处简化为同步调用）
            if (e.level == FaultLevel::P0)
                fsm.dispatch(EvFaultP0{});
            else if (e.level == FaultLevel::P1)
                fsm.dispatch(EvFaultP1{});
        }) {
        can->open_result = true;
        can->send_result = true;
        modbus->open_result = true;
        modbus->write_reg_return = 0;
        front_pin->open_result = true;
        rear_pin->open_result = true;
        brush->open();
        front_sw->open();
        rear_sw->open();
        fault_handler.start_listening();
        fsm.dispatch(EvInitDone{});
    }
};

// ────────────────────────────────────────────────────────────────
// 场景 1：完整 N=1 往返任务链
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: N=1 完整往返任务链", "[integration][task_chain]") {
    TaskChainFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");

    // 调度触发 → CleanFwd
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE_FALSE(f.can->sent_frames.empty());  // motion 已发 CAN 帧

    // 前端限位到达 → CleanReturn
    f.fsm.dispatch(EvFrontLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // 尾端限位到达 → Charging（N=1 完成）
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 2：P0 故障中断
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P0 故障中断清扫任务 → Fault → Reset → Idle", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 通过 FaultService 上报 P0（FaultHandler → dispatch EvFaultP0）
    f.fault_svc->report(FaultLevel::P0, 0x1001, "CAN lost");
    REQUIRE(f.fsm.current_state() == "Fault");

    // 人工复位
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");

    REQUIRE_FALSE(f.dispatched_faults.empty());
    REQUIRE(f.dispatched_faults[0].level == FaultLevel::P0);
}

// ────────────────────────────────────────────────────────────────
// 场景 3：P1 故障安全返回链
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P1 故障 → Returning → Charging", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    f.fsm.dispatch(EvFrontLimitSettled{});  // 进入 CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // P1 故障（FaultHandler → dispatch EvFaultP1 → Returning）
    f.fault_svc->report(FaultLevel::P1, 0x2001, "BMS low");
    REQUIRE(f.fsm.current_state() == "Returning");

    // 安全返回到停机位
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 4：SafetyMonitor 触发 → LimitSettledEvent → FSM 转换
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: SafetyMonitor 触发 LimitSettledEvent → FSM 响应",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 订阅 LimitSettledEvent 并转发给 FSM
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::FRONT)
                f.fsm.dispatch(EvFrontLimitSettled{});
            else
                f.fsm.dispatch(EvRearLimitSettled{});
        });

    // 启动安全监控
    f.safety_mon.start();

    // 模拟前端 GPIO 触发
    if (f.front_pin->registered_cb) {
        f.front_pin->simulate_edge();
    }

    // 等待 SafetyMonitor monitor_loop 延迟 180ms 后发布事件
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    f.safety_mon.stop();

    // FSM 应已收到 EvFrontLimitSettled 并转为 CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");
}

// ────────────────────────────────────────────────────────────────
// 场景 5：Scheduler 触发 → FSM 启动清扫
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: SchedulerService tick() 触发 FSM 清扫启动", "[integration][task_chain]") {
    TaskChainFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");

    // 配置当前分钟触发
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* lm = std::localtime(&t);
    f.scheduler.add_window({lm->tm_hour, lm->tm_min});

    // Scheduler 回调直接调用 FSM dispatch
    f.scheduler.set_on_task_start(
        [&] { f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f}); });

    f.scheduler.tick();
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 场景 6：N=2 完整 4 趟任务
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: N=2 完整 4 趟任务链", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    // 4 个半趟
    f.fsm.dispatch(EvFrontLimitSettled{});  // 半趟1
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvRearLimitSettled{});  // 半趟2
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    f.fsm.dispatch(EvFrontLimitSettled{});  // 半趟3
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvRearLimitSettled{});  // 半趟4 → Charging
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 7：P1 故障发生在 CleanFwd 阶段（未到前端）
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P1 故障发生在 CleanFwd → Returning → Charging",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // P1 故障在正向清扫期间触发（尚未到达前端限位）
    f.fault_svc->report(FaultLevel::P1, 0x2002, "slope_too_steep");
    REQUIRE(f.fsm.current_state() == "Returning");

    // 返回停机位
    f.fsm.dispatch(EvRearLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 8：P0 故障复位后可重新启动清扫
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P0 故障复位后重新启动清扫 → CleanFwd",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // P0 故障 → Fault
    f.fault_svc->report(FaultLevel::P0, 0x1001, "comm_lost");
    REQUIRE(f.fsm.current_state() == "Fault");

    // 复位 → Idle
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");

    // 重新下发任务 → CleanFwd
    f.fsm.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 场景 9：heading_pid_en=true 完整 N=1 任务链（FSM 路径不受 PID 影响）
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: heading_pid_en=true 完整 N=1 任务链", "[integration][task_chain][pid]") {
    TaskChainFixture f;
    // 临时用 PID 使能的 MotionService 替换 Fixture 内的 motion
    f.can->opened = true;  // 允许 send_ctrl() 通过 is_open() 检查
    MotionService::Config cfg_pid{
        .clean_speed_rpm  = 300.0f,
        .return_speed_rpm = 300.0f,
        .brush_rpm        = 1000,
        .return_brush_rpm = 1000,
        .heading_pid_en   = true
    };
    auto motion_pid =
        std::make_shared<MotionService>(f.group, f.brush, nullptr, f.bus, cfg_pid);
    RobotFsm fsm_pid(motion_pid, f.nav, f.fault_svc, f.bus);
    fsm_pid.dispatch(EvInitDone{});
    REQUIRE(fsm_pid.current_state() == "Idle");

    fsm_pid.dispatch(EvScheduleStart{.at_home = true, .passes = 1.0f});
    REQUIRE(fsm_pid.current_state() == "CleanFwd");
    REQUIRE_FALSE(f.can->sent_frames.empty());  // PID 路径仍发出 CAN 帧

    fsm_pid.dispatch(EvFrontLimitSettled{});
    REQUIRE(fsm_pid.current_state() == "CleanReturn");

    fsm_pid.dispatch(EvRearLimitSettled{});
    REQUIRE(fsm_pid.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 10：DataCache 遥测本地 JSONL 持久化
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: 遥测写入本地 JSONL、confirm_sent 后断电可恢复",
          "[integration][task_chain][data_cache]") {
    const std::string path = "/tmp/pv_test_cache_tc.jsonl";
    std::filesystem::remove(path);

    // 写入 2 条记录，确认 1 条，销毁
    {
        robot::middleware::DataCache cache{path};
        REQUIRE(cache.open());
        REQUIRE(cache.push("telemetry/status", R"({"rpm":300,"state":"CleanFwd"})"));
        REQUIRE(cache.push("telemetry/imu",    R"({"yaw":5.1,"pitch":2.0})"));
        REQUIRE(cache.size() == 2);

        auto batch = cache.pop_batch(10);
        REQUIRE(batch.size() == 2);
        REQUIRE(batch[0].topic == "telemetry/status");
        REQUIRE(batch[1].topic == "telemetry/imu");

        // 确认第一条已发送
        cache.confirm_sent({batch[0].id});
        REQUIRE(cache.size() == 1);
    }

    // 重新加载：未确认的第二条应持久化
    {
        robot::middleware::DataCache cache2{path};
        REQUIRE(cache2.open());
        REQUIRE(cache2.size() == 1);
        auto b = cache2.pop_batch(1);
        REQUIRE(b.size() == 1);
        REQUIRE(b[0].topic == "telemetry/imu");
    }

    std::filesystem::remove(path);
}
