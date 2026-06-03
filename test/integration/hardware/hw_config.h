// test/integration/hardware/hw_config.h
#pragma once
/**
 * @file hw_config.h
 * @brief 硬件测试公共 Fixture
 *
 * DeviceFixture       — Driver + Device 层，用于限位和电机单元测试
 * ImuGpsHealthFixture — IMU/GPS/HealthService 本地落盘测试底座
 *
 * 硬件接线（默认值与仓库 split config 对齐，可通过 hw_test_config.json 覆盖）：
 *   CAN      : can0（默认），行走电机 M1502E_111，motor_id_base=1
 *   IMU      : /dev/ttyS1（默认），WIT Motion，9600 baud
 *   GPSD     : 127.0.0.1:2947（默认），由目标机 gpsd 服务提供 TCP JSON 数据
 *   BMS      : /dev/ttyS8（默认），嘉佰达通用协议 V4，9600 baud
 *   距离传感器: /dev/ttyS9（默认），RS485 Modbus RTU，9600 baud
 *   GPIO     : gpiochip5 line0=左限位，line1=右限位，
 *              line2=左下姿态极限，line3=右下姿态极限（默认）
 */
#include <filesystem>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>

// Driver
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"

// Device
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/gps_device.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/attitude_limit_switch.h"
#include "pv_cleaning_robot/device/limit_switch.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"

// Service
#include "pv_cleaning_robot/service/health_service.h"

// Mock（滚刷电机未安装）
#include "mock/mock_can_bus.h"
#include "mock/mock_serial_port.h"
#include "pv_cleaning_robot/service/config_service.h"

namespace hw {

// ── 运行时硬件测试参数（从 hw_test_config.json 加载，缺失则用内嵌默认值）──────
/// 所有参数均可通过 hw_test_config.json 覆盖，无需重新编译
struct HwParams {
    // hardware mapping
    std::string can_iface = "can0";
    uint8_t motor_id_base = 1u;
    uint16_t comm_timeout_ms = 500u;  ///< update 50ms × 10 倍余量
    bool termination_init_enabled = true;
    uint8_t termination_init_retry_count = 3u;
    uint8_t termination_motor_id = 2u;
    std::string imu_port = "/dev/ttyS1";
    int imu_baud = 9600;
    std::string brush_port = "/dev/ttyACM0";
    int brush_baud = 115200;
    uint8_t brush_axis = 0u;
    std::string gpsd_host = "127.0.0.1";
    int gpsd_port = 2947;
    std::string gpsd_watch = "?WATCH={\"enable\":true,\"json\":true};";
    std::string bms_port = "/dev/ttyS8";
    int bms_baud = 9600;
    std::string dist_port = "/dev/ttyS9";  ///< 距离传感器 RS485 串口
    int dist_baud = 9600;
    uint8_t dist_slave_id = 1u;
    uint8_t dist_channel_count = 2u;
    std::string gpio_chip = "gpiochip5";
    unsigned left_limit_line = 0u;
    unsigned right_limit_line = 1u;
    unsigned left_attitude_limit_line = 2u;
    unsigned right_attitude_limit_line = 3u;
    // timing
    int limit_timeout_sec = 60;   ///< 每段（单一限位）等待最大秒数
    int online_timeout_ms = 600;  ///< 等待电机上线最大毫秒数
    int sweep_duration_ms = 5000;
    int loop_period_ms = 50;
    int gpsd_message_timeout_sec = 5;  ///< 期望收到首条 gpsd JSON 报文的最大等待时间
    int gpsd_fix_timeout_sec = 30;     ///< 期望拿到有效 GPS fix 的最大等待时间
    // behavior
    float test_speed_rpm = 10.0f;  ///< 安全低速（测试专用）
    float test_return_rpm = 10.0f;
    float sweep_rpm = 20.0f;
    float limit_test_rpm = 10.0f;
    float brush_test_rpm = 3000.0f;  ///< ODrive 滚刷硬件测试目标转速
    uint32_t combined_passes = 1u;   ///< 组合链路完整任务次数，1=单停机位去返一轮
    robot::domain::Endpoint primary_dock =
        robot::domain::Endpoint::B;  ///< 硬件测试运行时主停机端配置
    std::string health_jsonl_path = "/tmp/hw_system_test_health.jsonl";
    size_t health_log_max_bytes = 10u * 1024u * 1024u;  ///< HealthService 本地轮转单文件上限
    size_t health_log_max_files = 3u;  ///< HealthService 本地轮转保留文件数
    std::string pid_jsonl_path = "/tmp/hw_pid_test_metrics.jsonl";  ///< 预留调试输出路径
    float pid_max_drift_deg = 15.0f;  ///< 预留调试阈值

    /// 视觉纠偏参数，与 HeadingCorrector::Params 字段一一对应
    struct PidParams {
        std::string uds_path{"/tmp/pv_edge_tracker.sock"};
        int reconnect_interval_ms{500};
        int result_timeout_ms{500};
        float min_confidence{0.60f};
        float deadband_yaw_deg{1.0f};
        float kp{0.8f};
        float ki{0.0f};
        float kd{0.0f};
        float integral_limit{1.0f};
        float max_output{8.0f};
        float min_effective_output{1.0f};
        float yaw_alpha{0.35f};
        float output_sign{1.0f};
    } pid;

    struct CorrectionCompareConfig {
        float slow_base_rpm{15.0f};
        float yaw_slow_threshold_deg{1.0f};
        float max_output{10.0f};
        float min_effective_output{1.0f};
        float kp{5.0f};
        float ki{0.0f};
        float kd{0.0f};
        float integral_limit{1.0f};
        struct FusionConfig {
            float process_noise_angle{0.05f};
            float process_noise_bias{0.001f};
            float measurement_noise_uds{0.5f};
            float initial_angle_variance{1.0f};
            float initial_bias_variance{1.0f};
            int max_gyro_only_ms{300};
        } fusion;
    } correction_compare;
};

inline void inherit_correction_compare_pid_defaults(HwParams& p) {
    p.correction_compare.kp = p.pid.kp;
    p.correction_compare.ki = p.pid.ki;
    p.correction_compare.kd = p.pid.kd;
    p.correction_compare.integral_limit = p.pid.integral_limit;
    p.correction_compare.max_output = p.pid.max_output;
    p.correction_compare.min_effective_output = p.pid.min_effective_output;
}

/// 按优先级查找配置：1. 环境变量 HW_TEST_CONFIG  2. CWD/hw_test_config.json  3. 内嵌默认值
inline HwParams load_hw_test_config() {
    HwParams p;
    std::string path;
    const char* env = std::getenv("HW_TEST_CONFIG");
    if (env && std::filesystem::exists(env))
        path = env;
    else if (std::filesystem::exists("hw_test_config.json"))
        path = "hw_test_config.json";

    if (path.empty()) {
        spdlog::warn("[hw_config] hw_test_config.json not found — using built-in defaults");
        inherit_correction_compare_pid_defaults(p);
        return p;
    }
    try {
        robot::service::ConfigService cfg(path);
        if (!cfg.load()) {
            spdlog::warn("[hw_config] Failed to load {} — using built-in defaults", path);
            inherit_correction_compare_pid_defaults(p);
            return p;
        }
        p.can_iface = cfg.get<std::string>("hardware.can_iface", p.can_iface);
        p.motor_id_base =
            static_cast<uint8_t>(cfg.get<int>("hardware.motor_id_base", (int)p.motor_id_base));
        p.comm_timeout_ms =
            static_cast<uint16_t>(cfg.get<int>("timing.comm_timeout_ms", (int)p.comm_timeout_ms));
        p.termination_init_enabled =
            cfg.get<bool>("hardware.termination_init_enabled", p.termination_init_enabled);
        p.termination_init_retry_count =
            static_cast<uint8_t>(cfg.get<int>("hardware.termination_init_retry_count",
                                              static_cast<int>(p.termination_init_retry_count)));
        p.termination_motor_id = static_cast<uint8_t>(cfg.get<int>(
            "hardware.termination_motor_id", static_cast<int>(p.termination_motor_id)));
        p.imu_port = cfg.get<std::string>("hardware.imu_port", p.imu_port);
        p.imu_baud = cfg.get<int>("hardware.imu_baud", p.imu_baud);
        p.brush_port = cfg.get<std::string>("hardware.brush_port", p.brush_port);
        p.brush_baud = cfg.get<int>("hardware.brush_baud", p.brush_baud);
        p.brush_axis = static_cast<uint8_t>(
            cfg.get<int>("hardware.brush_axis", static_cast<int>(p.brush_axis)));
        p.gpsd_host = cfg.get<std::string>("hardware.gpsd_host", p.gpsd_host);
        p.gpsd_port = cfg.get<int>("hardware.gpsd_port", p.gpsd_port);
        p.gpsd_watch = cfg.get<std::string>("hardware.gpsd_watch", p.gpsd_watch);
        p.bms_port = cfg.get<std::string>("hardware.bms_port", p.bms_port);
        p.bms_baud = cfg.get<int>("hardware.bms_baud", p.bms_baud);
        p.dist_port = cfg.get<std::string>("hardware.dist_port", p.dist_port);
        p.dist_baud = cfg.get<int>("hardware.dist_baud", p.dist_baud);
        p.dist_slave_id =
            static_cast<uint8_t>(cfg.get<int>("hardware.dist_slave_id", (int)p.dist_slave_id));
        p.dist_channel_count = static_cast<uint8_t>(
            cfg.get<int>("hardware.dist_channel_count", (int)p.dist_channel_count));
        p.gpio_chip = cfg.get<std::string>("hardware.gpio_chip", p.gpio_chip);
        p.left_limit_line =
            static_cast<unsigned>(cfg.get<int>("hardware.left_limit_line", (int)p.left_limit_line));
        p.right_limit_line = static_cast<unsigned>(
            cfg.get<int>("hardware.right_limit_line", (int)p.right_limit_line));
        p.left_attitude_limit_line = static_cast<unsigned>(cfg.get<int>(
            "hardware.left_attitude_limit_line", (int)p.left_attitude_limit_line));
        p.right_attitude_limit_line = static_cast<unsigned>(cfg.get<int>(
            "hardware.right_attitude_limit_line", (int)p.right_attitude_limit_line));
        p.limit_timeout_sec = cfg.get<int>("timing.limit_timeout_sec", p.limit_timeout_sec);
        p.online_timeout_ms = cfg.get<int>("timing.online_timeout_ms", p.online_timeout_ms);
        p.sweep_duration_ms = cfg.get<int>("timing.sweep_duration_ms", p.sweep_duration_ms);
        p.loop_period_ms = cfg.get<int>("timing.loop_period_ms", p.loop_period_ms);
        p.gpsd_message_timeout_sec =
            cfg.get<int>("timing.gpsd_message_timeout_sec", p.gpsd_message_timeout_sec);
        p.gpsd_fix_timeout_sec =
            cfg.get<int>("timing.gpsd_fix_timeout_sec", p.gpsd_fix_timeout_sec);
        p.test_speed_rpm = cfg.get<float>("behavior.test_speed_rpm", p.test_speed_rpm);
        p.test_return_rpm = cfg.get<float>("behavior.test_return_rpm", p.test_return_rpm);
        p.sweep_rpm = cfg.get<float>("behavior.sweep_rpm", p.sweep_rpm);
        p.limit_test_rpm = cfg.get<float>("behavior.limit_test_rpm", p.limit_test_rpm);
        p.brush_test_rpm = cfg.get<float>("behavior.brush_test_rpm", p.brush_test_rpm);
        p.combined_passes = static_cast<uint32_t>(
            std::max(1, cfg.get<int>("behavior.combined_passes",
                                     static_cast<int>(p.combined_passes))));
        {
            const auto primary_dock =
                cfg.get<std::string>("behavior.primary_dock", "B");
            if (primary_dock == "A") {
                p.primary_dock = robot::domain::Endpoint::A;
            } else if (primary_dock == "B") {
                p.primary_dock = robot::domain::Endpoint::B;
            } else {
                spdlog::warn(
                    "[hw_config] invalid behavior.primary_dock='{}', fallback to B",
                    primary_dock);
                p.primary_dock = robot::domain::Endpoint::B;
            }
        }
        p.health_jsonl_path =
            cfg.get<std::string>("behavior.health_jsonl_path", p.health_jsonl_path);
        p.health_log_max_bytes = static_cast<size_t>(cfg.get<int>(
            "behavior.health_log_max_bytes", static_cast<int>(p.health_log_max_bytes)));
        p.health_log_max_files = static_cast<size_t>(cfg.get<int>(
            "behavior.health_log_max_files", static_cast<int>(p.health_log_max_files)));
        p.pid_jsonl_path = cfg.get<std::string>("behavior.pid_jsonl_path", p.pid_jsonl_path);
        p.pid_max_drift_deg = cfg.get<float>("behavior.pid_max_drift_deg", p.pid_max_drift_deg);
        p.pid.uds_path = cfg.get<std::string>("pid.uds_path", p.pid.uds_path);
        p.pid.reconnect_interval_ms =
            cfg.get<int>("pid.reconnect_interval_ms", p.pid.reconnect_interval_ms);
        p.pid.result_timeout_ms =
            cfg.get<int>("pid.result_timeout_ms", p.pid.result_timeout_ms);
        p.pid.min_confidence = cfg.get<float>("pid.min_confidence", p.pid.min_confidence);
        p.pid.deadband_yaw_deg =
            cfg.get<float>("pid.deadband_yaw_deg",
                           cfg.get<float>("pid.deadband_slope",
                                          cfg.get<float>("pid.deadband_norm",
                                                         p.pid.deadband_yaw_deg)));
        p.pid.kp = cfg.get<float>("pid.kp", p.pid.kp);
        p.pid.ki = cfg.get<float>("pid.ki", p.pid.ki);
        p.pid.kd = cfg.get<float>("pid.kd", p.pid.kd);
        p.pid.integral_limit = cfg.get<float>("pid.integral_limit", p.pid.integral_limit);
        p.pid.max_output = cfg.get<float>("pid.max_output", p.pid.max_output);
        p.pid.min_effective_output =
            cfg.get<float>("pid.min_effective_output", p.pid.min_effective_output);
        p.pid.yaw_alpha =
            cfg.get<float>("pid.yaw_alpha",
                           cfg.get<float>("pid.slope_alpha",
                                          cfg.get<float>("pid.offset_alpha",
                                                         p.pid.yaw_alpha)));
        p.pid.output_sign = cfg.get<float>("pid.output_sign", p.pid.output_sign);
        inherit_correction_compare_pid_defaults(p);
        p.correction_compare.slow_base_rpm =
            cfg.get<float>("correction_compare.slow_base_rpm",
                           p.correction_compare.slow_base_rpm);
        p.correction_compare.yaw_slow_threshold_deg =
            cfg.get<float>("correction_compare.yaw_slow_threshold_deg",
                           p.correction_compare.yaw_slow_threshold_deg);
        p.correction_compare.max_output =
            cfg.get<float>("correction_compare.max_output", p.correction_compare.max_output);
        p.correction_compare.min_effective_output =
            cfg.get<float>("correction_compare.min_effective_output",
                           p.correction_compare.min_effective_output);
        p.correction_compare.kp = cfg.get<float>("correction_compare.kp",
                                                 p.correction_compare.kp);
        p.correction_compare.ki = cfg.get<float>("correction_compare.ki",
                                                 p.correction_compare.ki);
        p.correction_compare.kd = cfg.get<float>("correction_compare.kd",
                                                 p.correction_compare.kd);
        p.correction_compare.integral_limit =
            cfg.get<float>("correction_compare.integral_limit",
                           p.correction_compare.integral_limit);
        p.correction_compare.fusion.process_noise_angle =
            cfg.get<float>("correction_compare.fusion.process_noise_angle",
                           p.correction_compare.fusion.process_noise_angle);
        p.correction_compare.fusion.process_noise_bias =
            cfg.get<float>("correction_compare.fusion.process_noise_bias",
                           p.correction_compare.fusion.process_noise_bias);
        p.correction_compare.fusion.measurement_noise_uds =
            cfg.get<float>("correction_compare.fusion.measurement_noise_uds",
                           p.correction_compare.fusion.measurement_noise_uds);
        p.correction_compare.fusion.initial_angle_variance =
            cfg.get<float>("correction_compare.fusion.initial_angle_variance",
                           p.correction_compare.fusion.initial_angle_variance);
        p.correction_compare.fusion.initial_bias_variance =
            cfg.get<float>("correction_compare.fusion.initial_bias_variance",
                           p.correction_compare.fusion.initial_bias_variance);
        p.correction_compare.fusion.max_gyro_only_ms =
            cfg.get<int>("correction_compare.fusion.max_gyro_only_ms",
                         p.correction_compare.fusion.max_gyro_only_ms);
        spdlog::debug("[hw_config] Loaded config: {}", path);
    } catch (const std::exception& e) {
        spdlog::warn("[hw_config] Exception loading config {}: {} — using built-in defaults",
                     path,
                     e.what());
    }
    return p;
}

// ── DeviceFixture：Driver + Device 层（限位 / 电机单元测试使用）────────────
struct DeviceFixture {
    HwParams p;  ///< 运行时硬件参数，必须为第一成员（构造时初始化列表先于 ctor 体）
    std::shared_ptr<robot::driver::LinuxCanSocket> can_bus;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::driver::LibSerialPort> bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibGpiodPin> left_gpio;
    std::shared_ptr<robot::driver::LibGpiodPin> right_gpio;
    std::shared_ptr<robot::device::LimitSwitch> left_sw;
    std::shared_ptr<robot::device::LimitSwitch> right_sw;

    DeviceFixture() : p(load_hw_test_config()) {
        using namespace robot;
        can_bus = std::make_shared<driver::LinuxCanSocket>(p.can_iface);
        walk_group = std::make_shared<device::WalkMotorGroup>(can_bus,
                                                              p.motor_id_base,
                                                              p.comm_timeout_ms,
                                                              p.termination_init_enabled,
                                                              p.termination_init_retry_count,
                                                              p.termination_motor_id);
        imu_serial =
            std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu = std::make_shared<device::ImuDevice>(imu_serial);
        bms_serial =
            std::make_shared<driver::LibSerialPort>(p.bms_port, hal::UartConfig{p.bms_baud});
        bms = std::make_shared<device::BMS>(bms_serial, 95.0f, 15.0f);
        left_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.left_limit_line);
        right_gpio = std::make_shared<driver::LibGpiodPin>(p.gpio_chip, p.right_limit_line);
        left_sw = std::make_shared<device::LimitSwitch>(left_gpio, device::LimitSide::LEFT);
        right_sw = std::make_shared<device::LimitSwitch>(right_gpio, device::LimitSide::RIGHT);
    }

    ~DeviceFixture() {
        if (left_sw)
            left_sw->close();
        if (right_sw)
            right_sw->close();
        if (imu)
            imu->close();
        if (walk_group) {
            walk_group->disable_all();
            walk_group->close();
        }
        // bms_serial 由 shared_ptr 析构时自动关闭
    }
};

// ── ImuGpsHealthFixture：仅 IMU + GPS + Health 本地落盘 ─────────────────────
struct ImuGpsHealthFixture {
    HwParams p;
    std::shared_ptr<MockCanBus> mock_can;
    std::shared_ptr<robot::device::WalkMotorGroup> walk_group;
    std::shared_ptr<MockSerialPort> mock_brush_serial;
    std::shared_ptr<robot::device::BrushMotor> brush;
    std::shared_ptr<MockSerialPort> mock_bms_serial;
    std::shared_ptr<robot::device::BMS> bms;
    std::shared_ptr<robot::driver::LibSerialPort> imu_serial;
    std::shared_ptr<robot::device::ImuDevice> imu;
    std::shared_ptr<robot::device::GpsDevice> gps;
    std::shared_ptr<robot::service::HealthService> health;

    ImuGpsHealthFixture() : p(load_hw_test_config()) {
        using namespace robot;

        // HealthService 仍要求 walk / brush / bms 依赖，这里用最小 mock 底座承载，
        // 不打开真实 CAN / BMS / Brush 硬件，仅为生成完整 JSON 提供静态状态。
        mock_can = std::make_shared<MockCanBus>();
        walk_group = std::make_shared<device::WalkMotorGroup>(mock_can, 1u, 200u, false, 1u, 2u);

        mock_brush_serial = std::make_shared<MockSerialPort>();
        brush = std::make_shared<device::BrushMotor>(mock_brush_serial, 0u);

        mock_bms_serial = std::make_shared<MockSerialPort>();
        bms = std::make_shared<device::BMS>(mock_bms_serial, 95.0f, 15.0f);

        imu_serial =
            std::make_shared<driver::LibSerialPort>(p.imu_port, hal::UartConfig{p.imu_baud});
        imu = std::make_shared<device::ImuDevice>(imu_serial);

        device::GpsdSourceConfig gpsd_cfg;
        gpsd_cfg.host = p.gpsd_host;
        gpsd_cfg.port = p.gpsd_port;
        gpsd_cfg.watch = p.gpsd_watch;
        gps = device::GpsDevice::create_gpsd(gpsd_cfg);
    }

    bool init(const std::string& health_jsonl_path) {
        if (!imu || !imu->open()) {
            spdlog::error("[ImuGpsHealthFixture] IMU open 失败");
            return false;
        }
        if (!gps || !gps->open()) {
            spdlog::error("[ImuGpsHealthFixture] GPS(gpsd) open 失败");
            return false;
        }

        health = std::make_shared<robot::service::HealthService>(
            walk_group,
            brush,
            bms,
            imu,
            gps,
            nullptr,
            robot::service::HealthService::Mode::DIAGNOSTICS,
            health_jsonl_path,
            p.health_log_max_bytes,
            p.health_log_max_files);
        return true;
    }

    ~ImuGpsHealthFixture() {
        if (gps)
            gps->close();
        if (imu)
            imu->close();
    }
};

}  // namespace hw
