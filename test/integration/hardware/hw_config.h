// test/integration/hardware/hw_config.h
#pragma once
/**
 * @file hw_config.h
 * @brief 硬件测试公共 Fixture
 *
 * DeviceFixture       — Driver + Device 层，用于限位和电机单元测试
 * FullSystemFixture   — 全层栈，用于集成测试（BrushMotor 用 MockModbusMaster）
 *
 * 硬件接线（与 config/config.json 对齐）：
 *   CAN      : can0，行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1，WIT Motion，9600 baud
 *   BMS      : /dev/ttyS8，嘉佰达通用协议 V4，9600 baud
 *   GPIO     : gpiochip5 line0=前限位，line1=后限位
 */
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

// Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"

// Device
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Middleware
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"

// Service
#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/fault_service.h"
#include "pv_cleaning_robot/service/health_service.h"
#include "pv_cleaning_robot/service/motion_service.h"
#include "pv_cleaning_robot/service/nav_service.h"

// App
#include "pv_cleaning_robot/app/fault_handler.h"
#include "pv_cleaning_robot/app/robot_fsm.h"
#include "pv_cleaning_robot/app/watchdog_mgr.h"

// Mock（辊刷电机未安装）
#include "mock/mock_modbus_master.h"

namespace hw {

// ── 硬件接线常量（与 config/config.json 对齐）────────────────────────────────
constexpr char     kCanIface[]    = "can0";
constexpr uint8_t  kMotorIdBase   = 1u;
constexpr uint16_t kCommTimeoutMs = 500u;   ///< update 50ms × 10 倍余量
constexpr float    kTestSpeedRpm  = 20.0f;  ///< 安全低速（测试专用）
constexpr float    kTestReturnRpm = 20.0f;

constexpr char kImuPort[] = "/dev/ttyS1";
constexpr int  kImuBaud   = 9600;

constexpr char kBmsPort[] = "/dev/ttyS8";
constexpr int  kBmsBaud   = 9600;

constexpr char     kGpioChip[] = "gpiochip5";
constexpr unsigned kFrontLine  = 0u;
constexpr unsigned kRearLine   = 1u;

constexpr int kLimitTimeoutSec  = 60;   ///< 每段（单一限位）等待最大秒数
constexpr int kOnlineTimeoutMs  = 600;  ///< 等待电机上线最大毫秒数
constexpr float kCombinedPasses = 1.0f; ///< combined 测试趟数（1=一来回，2=两来回…）

constexpr char kHealthJsonlPath[] = "/tmp/hw_system_test_health.jsonl"; ///< HealthService JSONL 落盘路径

// ── DeviceFixture：Driver + Device 层（限位 / 电机单元测试使用）────────────
struct DeviceFixture {
    std::shared_ptr<robot::driver::LinuxCanSocket>  can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup>  walk_group;
    std::shared_ptr<robot::driver::LibSerialPort>   imu_serial;
    std::shared_ptr<robot::device::ImuDevice>       imu;
    std::shared_ptr<robot::driver::LibSerialPort>   bms_serial;
    std::shared_ptr<robot::device::BMS>             bms;
    std::shared_ptr<robot::driver::LibGpiodPin>     front_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin>     rear_gpio;
    std::shared_ptr<robot::device::LimitSwitch>     front_sw;
    std::shared_ptr<robot::device::LimitSwitch>     rear_sw;

    DeviceFixture() {
        using namespace robot;
        can_bus    = std::make_shared<driver::LinuxCanSocket>(kCanIface);
        walk_group = std::make_shared<device::WalkMotorGroup>(
            can_bus, kMotorIdBase, kCommTimeoutMs);
        imu_serial = std::make_shared<driver::LibSerialPort>(
            kImuPort, hal::UartConfig{kImuBaud});
        imu        = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial = std::make_shared<driver::LibSerialPort>(
            kBmsPort, hal::UartConfig{kBmsBaud});
        bms        = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        front_gpio = std::make_shared<driver::LibGpiodPin>(kGpioChip, kFrontLine);
        rear_gpio  = std::make_shared<driver::LibGpiodPin>(kGpioChip, kRearLine);
        front_sw   = std::make_shared<device::LimitSwitch>(
            front_gpio, device::LimitSide::FRONT);
        rear_sw    = std::make_shared<device::LimitSwitch>(
            rear_gpio, device::LimitSide::REAR);
    }

    ~DeviceFixture() {
        if (front_sw)   front_sw->close();
        if (rear_sw)    rear_sw->close();
        if (imu)        imu->close();
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        // bms_serial 由 shared_ptr 析构时自动关闭
    }
};

// ── FullSystemFixture：全层栈（集成测试使用）─────────────────────────────────
struct FullSystemFixture : DeviceFixture {
    robot::middleware::EventBus                       event_bus;
    std::shared_ptr<MockModbusMaster>                 mock_modbus;
    std::shared_ptr<robot::device::BrushMotor>        brush;
    std::shared_ptr<robot::device::GpsDevice>         gps_dummy;
    std::unique_ptr<robot::middleware::SafetyMonitor> safety;
    std::shared_ptr<robot::service::NavService>       nav;
    std::shared_ptr<robot::service::MotionService>    motion;
    std::shared_ptr<robot::service::FaultService>     fault;
    std::shared_ptr<robot::service::HealthService>    health;  ///< null cloud，本地 JSONL 落盘
    std::unique_ptr<robot::app::WatchdogMgr>          watchdog;
    std::shared_ptr<robot::app::RobotFsm>             fsm;
    std::shared_ptr<robot::app::FaultHandler>         fault_handler;
    std::vector<robot::service::FaultService::FaultEvent> dispatched_faults;

    /// @param pid_enabled 是否开启航向 PID（clean_cycle 测试按场景传入）
    explicit FullSystemFixture(bool pid_enabled = false) : DeviceFixture() {
        using namespace robot;

        mock_modbus = std::make_shared<MockModbusMaster>();
        brush       = std::make_shared<device::BrushMotor>(mock_modbus, 1);

        // SafetyMonitor 构造时内部绑定 LimitSwitch 回调
        safety = std::make_unique<middleware::SafetyMonitor>(
            walk_group, front_sw, rear_sw, event_bus);

        // GPS 未安装：创建占位对象，不 open，NavService 会跳过 GPS 校正
        auto gps_serial_dummy = std::make_shared<driver::LibSerialPort>(
            "/dev/null", hal::UartConfig{9600});
        gps_dummy = std::make_shared<device::GpsDevice>(gps_serial_dummy);

        nav = std::make_shared<service::NavService>(
            walk_group, imu, gps_dummy, 0.3f);

        service::MotionService::Config motion_cfg;
        motion_cfg.clean_speed_rpm  = kTestSpeedRpm;
        motion_cfg.return_speed_rpm = kTestReturnRpm;
        motion_cfg.brush_rpm        = 0;   // MockModbus，不实际驱动辊刷
        motion_cfg.return_brush_rpm = 0;
        motion_cfg.edge_reverse_rpm = 0.0f;
        motion_cfg.heading_pid_en   = pid_enabled;

        motion = std::make_shared<service::MotionService>(
            walk_group, brush, imu, event_bus, motion_cfg);
        fault  = std::make_shared<service::FaultService>(event_bus);

        // WatchdogMgr：路径为空 = 不操作 /dev/watchdog
        watchdog = std::make_unique<app::WatchdogMgr>("");

        fsm = std::make_shared<app::RobotFsm>(motion, nav, fault, event_bus);

        // SafetyMonitor LimitSettledEvent → RobotFsm
        event_bus.subscribe<middleware::SafetyMonitor::LimitSettledEvent>(
            [this](const middleware::SafetyMonitor::LimitSettledEvent& evt) {
                if (evt.side == device::LimitSide::FRONT)
                    fsm->dispatch(app::EvFrontLimitSettled{});
                else
                    fsm->dispatch(app::EvRearLimitSettled{});
            });

        // FaultHandler: P0→emergency_stop+EvFaultP0，P1→stop+EvFaultP1；同时记录 dispatched_faults
        fault_handler = std::make_shared<app::FaultHandler>(
            motion, event_bus,
            [this](service::FaultService::FaultEvent e) {
                dispatched_faults.push_back(e);
                using Level = service::FaultService::FaultEvent::Level;
                if (e.level == Level::P0)
                    fsm->dispatch(app::EvFaultP0{});
                else if (e.level == Level::P1)
                    fsm->dispatch(app::EvFaultP1{});
            });
    }

    /// 初始化所有硬件并启动后台线程，dispatch EvInitDone → FSM = "Idle"
    /// @param health_jsonl_path  HealthService JSONL 落盘路径（空字符串=不落盘）
    /// @return false 表示关键硬件（walk_group）初始化失败
    bool init(const std::string& health_jsonl_path = "") {
        using robot::device::DeviceError;
        if (walk_group->open() != DeviceError::OK) {
            spdlog::error("[FullSystemFixture] walk_group open 失败");
            return false;
        }
        walk_group->set_feedback_mode_all(10u);  // 10ms 主动上报

        if (!imu->open())
            spdlog::warn("[FullSystemFixture] IMU open 失败（非致命）");

        if (bms->open() != DeviceError::OK)
            spdlog::warn("[FullSystemFixture] BMS open 失败（非致命）");

        // 限位开关：gpiochip5 不支持 IRQ，使用 1ms 软件轮询；测试中不设 RT 优先级，无 CPU 绑定
        if (!front_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] front_sw open 失败");
        if (!rear_sw->open(0, 2, 0, false))
            spdlog::warn("[FullSystemFixture] rear_sw open 失败");

        if (!safety->start()) {
            spdlog::error("[FullSystemFixture] safety monitor start 失败");
            return false;
        }
        watchdog->start();
        fault_handler->start_listening();
        start_loops_();

        // HealthService 在硬件 open 之后构造，保证传感器缓存已就绪
        if (!health_jsonl_path.empty()) {
            std::filesystem::remove(health_jsonl_path);  // 清旧文件，open() 创建新文件
            health = std::make_shared<robot::service::HealthService>(
                walk_group, brush, bms, imu, gps_dummy,
                nullptr,   // cloud = null，不需要 MQTT/LoRaWAN
                robot::service::HealthService::Mode::DIAGNOSTICS,
                health_jsonl_path);
            spdlog::info("[FullSystemFixture] HealthService 已创建: {}", health_jsonl_path);
        }

        // FSM: StateInit → Idle
        fsm->dispatch(robot::app::EvInitDone{});
        return true;
    }

    /// 轮询等待 FSM 进入指定状态，超时返回 false
    bool wait_state(const std::string& expected,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (fsm->current_state() == expected) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        spdlog::warn("[FullSystemFixture] wait_state 超时: 期望={} 实际={}",
                     expected, fsm->current_state());
        return false;
    }

    ~FullSystemFixture() {
        stop_loops_();
        if (safety)   safety->stop();
        if (motion)   motion->emergency_stop();
        if (watchdog) watchdog->stop();
    }

private:
    std::thread       walk_ctrl_thread_;
    std::thread       nav_exec_thread_;
    std::atomic<bool> loops_running_{false};

    void start_loops_() {
        loops_running_.store(true);
        walk_ctrl_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                motion->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        nav_exec_thread_ = std::thread([this] {
            while (loops_running_.load()) {
                nav->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void stop_loops_() {
        loops_running_.store(false);
        if (walk_ctrl_thread_.joinable()) walk_ctrl_thread_.join();
        if (nav_exec_thread_.joinable())  nav_exec_thread_.join();
    }
};

} // namespace hw
