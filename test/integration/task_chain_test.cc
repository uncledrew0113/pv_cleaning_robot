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
 *   - 仅验证状态机路径与 CAN/串口指令存在，不验实时时序
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <filesystem>
#include <thread>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_gpio_pin.h"
#include "../mock/mock_serial_port.h"
#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/robot_supervisor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"
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
using robot::service::ConfigService;
using robot::service::CommandTracker;
using robot::service::FaultService;
using robot::service::MotionService;
using robot::service::NavService;
using robot::service::SchedulerService;
using FaultEvent = FaultService::FaultEvent;
using FaultLevel = FaultEvent::Level;
namespace fs = std::filesystem;

// ────────────────────────────────────────────────────────────────
// 完整任务链构建辅助
// ────────────────────────────────────────────────────────────────
struct TaskChainFixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_task_chain_supervisor")};

    // 底层 mock
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<MockSerialPort> imu_sp{std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort> gps_sp{std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockSerialPort> brush_sp{std::make_shared<MockSerialPort>()};
    std::shared_ptr<MockGpioPin> left_pin{std::make_shared<MockGpioPin>()};
    std::shared_ptr<MockGpioPin> right_pin{std::make_shared<MockGpioPin>()};

    // 设备
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<BrushMotor> brush{
        std::make_shared<BrushMotor>(brush_sp, 0)};
    std::shared_ptr<ImuDevice> imu{std::make_shared<ImuDevice>(imu_sp)};
    std::shared_ptr<GpsDevice> gps{std::make_shared<GpsDevice>(gps_sp)};
    std::shared_ptr<LimitSwitch> left_sw{
        std::make_shared<LimitSwitch>(left_pin, LimitSide::LEFT)};
    std::shared_ptr<LimitSwitch> right_sw{std::make_shared<LimitSwitch>(right_pin, LimitSide::RIGHT)};

    // 服务 & 中间件
    EventBus bus;
    std::shared_ptr<MotionService> motion;
    std::shared_ptr<NavService> nav;
    std::shared_ptr<FaultService> fault_svc{std::make_shared<FaultService>(bus)};
    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    std::shared_ptr<CommandTracker> command_tracker{std::make_shared<CommandTracker>()};
    SafetyMonitor safety_mon;

    // App 层
    RobotFsm fsm;
    std::shared_ptr<RobotSupervisor> supervisor;
    std::vector<FaultEvent> dispatched_faults;
    FaultHandler fault_handler;

    TaskChainFixture()
        : motion([this] {
            MotionService::Config cfg;
            cfg.heading_pid_en = false;
            return std::make_shared<MotionService>(group, brush, nullptr, bus, cfg);
        }())
        , nav(std::make_shared<NavService>(group, imu, gps))
        , safety_mon(group, left_sw, right_sw, bus)
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
        can->opened = true;
        brush_sp->open_result = true;
        left_pin->open_result = true;
        right_pin->open_result = true;
        brush->open();
        left_sw->open();
        right_sw->open();
        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "parking_side": "left"
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})",
                                            R"({})");
        REQUIRE(cfg.load());
        scheduler.clear_windows();
        scheduler.add_window({8, 0});
        cfg.apply_active_runtime_schedules(scheduler);
        motion->set_parking_side_query(
            [this]() { return cfg.active_runtime_config().parking_side; });
        motion->set_runtime_config_query([this]() { return cfg.active_runtime_config(); });
        fault_handler.start_listening();
        fsm.dispatch(EvInitDone{});
        supervisor = std::make_shared<RobotSupervisor>(
            std::shared_ptr<RobotFsm>(&fsm, [](RobotFsm*) {}),
            cfg,
            command_tracker,
            fault_svc,
            nav);
    }

    ~TaskChainFixture() {
        tb_test_support::cleanup_split_config_paths(paths);
    }
};

// ────────────────────────────────────────────────────────────────
// 场景 1：完整 N=1 往返任务链
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: N=1 完整往返任务链", "[integration][task_chain]") {
    TaskChainFixture f;
    REQUIRE(f.fsm.current_state() == "Idle");

    // 调度触发 → CleanFwd
    f.fsm.dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    REQUIRE_FALSE(f.can->sent_frames.empty());  // motion 已发 CAN 帧

    // 对侧限位到达 → CleanReturn
    f.fsm.dispatch(EvFarEndLimitSettled{});
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // 停机侧限位到达 → Charging（N=1 完成）
    f.fsm.dispatch(EvParkingSideLimitSettled{});
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 2：P0 故障中断
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P0 故障中断清扫任务 → Fault → Reset → Idle", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
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
// 场景 3：P1 故障进入 Fault
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P1 故障 → Fault", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    f.fsm.dispatch(EvFarEndLimitSettled{});  // 进入 CleanReturn
    REQUIRE(f.fsm.current_state() == "CleanReturn");

    // P1 故障（FaultHandler → dispatch EvFaultP1 → Fault）
    f.fault_svc->report(FaultLevel::P1, 0x2001, "BMS low");
    REQUIRE(f.fsm.current_state() == "Fault");
}

// ────────────────────────────────────────────────────────────────
// 场景 4：SafetyMonitor 触发 → LimitSettledEvent → FSM 转换
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: SafetyMonitor 触发 LimitSettledEvent → FSM 响应",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // 订阅 LimitSettledEvent 并转发给 FSM
    f.bus.subscribe<SafetyMonitor::LimitSettledEvent>(
        [&](const SafetyMonitor::LimitSettledEvent& e) {
            if (e.side == LimitSide::LEFT)
                f.fsm.dispatch(EvFarEndLimitSettled{});
            else
                f.fsm.dispatch(EvParkingSideLimitSettled{});
        });

    // 启动安全监控
    f.safety_mon.start();

    // 模拟前端 GPIO 触发
    if (f.left_pin->registered_cb) {
        f.left_pin->simulate_edge();
    }

    // 等待 SafetyMonitor monitor_loop 延迟 180ms 后发布事件
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    f.safety_mon.stop();

    // FSM 应已收到 EvFarEndLimitSettled 并转为 CleanReturn
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
    f.scheduler.set_on_window_hit(
        [&] { REQUIRE(f.supervisor->start_task(true, true, 80.0f)); });

    f.scheduler.tick();
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 场景 6：N=2 完整 4 趟任务
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: N=2 完整 4 趟任务链", "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    // 4 个半趟
    f.fsm.dispatch(EvFarEndLimitSettled{});  // 半趟1
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvParkingSideLimitSettled{});  // 半趟2
    REQUIRE(f.fsm.current_state() == "CleanFwd");
    f.fsm.dispatch(EvFarEndLimitSettled{});  // 半趟3
    REQUIRE(f.fsm.current_state() == "CleanReturn");
    f.fsm.dispatch(EvParkingSideLimitSettled{});  // 半趟4 → Charging
    REQUIRE(f.fsm.current_state() == "Charging");
}

// ────────────────────────────────────────────────────────────────
// 场景 7：P1 故障发生在 CleanFwd 阶段（未到前端）
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P1 故障发生在 CleanFwd → Fault",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 2.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // P1 故障在正向清扫期间触发（尚未到达对侧限位）
    f.fault_svc->report(FaultLevel::P1, 0x2002, "slope_too_steep");
    REQUIRE(f.fsm.current_state() == "Fault");
}

// ────────────────────────────────────────────────────────────────
// 场景 8：P0 故障复位后可重新启动清扫
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: P0 故障复位后重新启动清扫 → CleanFwd",
          "[integration][task_chain]") {
    TaskChainFixture f;
    f.fsm.dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");

    // P0 故障 → Fault
    f.fault_svc->report(FaultLevel::P0, 0x1001, "comm_lost");
    REQUIRE(f.fsm.current_state() == "Fault");

    // 复位 → Idle
    f.fsm.dispatch(EvFaultReset{});
    REQUIRE(f.fsm.current_state() == "Idle");

    // 重新下发任务 → CleanFwd
    f.fsm.dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(f.fsm.current_state() == "CleanFwd");
}

// ────────────────────────────────────────────────────────────────
// 场景 9：视觉 PID 使能时，完整 N=1 任务链（FSM 路径不受 PID 影响）
// ────────────────────────────────────────────────────────────────
TEST_CASE("TaskChain: visual PID enabled 完整 N=1 任务链", "[integration][task_chain][pid]") {
    TaskChainFixture f;
    // 临时用 PID 使能的 MotionService 替换 Fixture 内的 motion
    f.can->opened = true;  // 允许 send_ctrl() 通过 is_open() 检查
    MotionService::Config cfg_pid;
    cfg_pid.clean_speed_rpm = 300.0f;
    cfg_pid.return_speed_rpm = 300.0f;
    cfg_pid.brush_rpm = 1000;
    cfg_pid.return_brush_rpm = 1000;
    cfg_pid.heading_pid_en = true;
    auto motion_pid =
        std::make_shared<MotionService>(f.group, f.brush, nullptr, f.bus, cfg_pid);
    RobotFsm fsm_pid(motion_pid, f.nav, f.fault_svc, f.bus);
    fsm_pid.dispatch(EvInitDone{});
    REQUIRE(fsm_pid.current_state() == "Idle");

    fsm_pid.dispatch(EvScheduleStart{true, false, 1.0f});
    REQUIRE(fsm_pid.current_state() == "CleanFwd");
    REQUIRE_FALSE(f.can->sent_frames.empty());  // PID 路径仍发出 CAN 帧

    fsm_pid.dispatch(EvFarEndLimitSettled{});
    REQUIRE(fsm_pid.current_state() == "CleanReturn");

    fsm_pid.dispatch(EvParkingSideLimitSettled{});
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
