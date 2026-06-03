# 项目调用链与影响范围索引

> 静态分析基于当前工作区源码生成。第三方库内部不展开；C++ 虚调用、lambda、`std::function`、对象成员方法会受类型解析限制，本报告用主链路、include 反向依赖和可唯一解析函数调用辅助定位影响范围。

## 1. 构建与模块边界

- 主程序目标 `pv_cleaning_robot`：`main.cc` + `driver/protocol/device/middleware/service/app` 下全部 `.cc`。
- `unit_tests` 和 `hw_tests` 都复用 `test/CMakeLists.txt` 的 `COMMON_SRCS`，所以生产源码改动会同时影响两个测试二进制的编译。
- 公共 API 头文件在 `include/pv_cleaning_robot/*`；改头文件时优先查“反向 include 索引”。
- 当前报告覆盖：生产文件 `100` 个，测试文件 `69` 个，生产模块 `57` 个，生产函数/方法定义 `650` 个。

## 2. 运行时主调用链

- **启动/依赖注入**：`main -> ConfigService::load -> Logger::init -> 构造 HAL/Driver/Device/Middleware/Service/App -> RobotController::start -> WatchdogMgr::start -> ThreadExecutor::start(walk/nav/bms/cloud) -> main loop`
- **运动任务启动**：`ThingsBoardControlPlane RPC 或 SchedulerService::tick -> RobotController::submit_command/post_schedule_window_hit -> start_command_locked -> domain::build_*_mission_context -> complete_self_check -> start_current_segment_locked -> ActionPorts.start_segment -> MotionService::start_segment -> start_cleaning_to -> WalkMotorGroup::enable_all/set_mode_all/set_speeds + BrushMotor::set_rpm`
- **运动周期控制**：`ThreadExecutor(walk_ctrl)::loop -> MotionService::update -> HeadingCorrector::compute(可选) -> WalkMotorGroup::set_speeds -> WalkMotorGroup::update -> WalkMotorCanCodec encode/decode -> LinuxCanSocket::send/recv`
- **导航/打滑检测**：`ThreadExecutor(nav)::loop -> NavService::update -> WalkMotorGroup::get_group_diagnostics + ImuDevice::get_latest + GpsDevice::get_latest -> main loop 检查 nav.get_pose().spin_free_detected -> RobotController::post_fault`
- **限位急停**：`LibGpiodPin/LimitSwitch 边沿或轮询 -> SafetyMonitor::on_limit_trigger/monitor_loop -> WalkMotorGroup::emergency_override(0) -> limit_settled_callback -> RobotController::post_limit_settled -> handle_limit_settled_locked -> 下一段 MotionService::start_segment 或任务结束`
- **看门狗故障**：`WatchdogMgr::monitor_loop timeout -> timeout callback -> MotionService::emergency_stop -> RobotController::post_watchdog_timeout -> handle_fault_locked -> FaultPolicy::decide -> FaultStopped/Recovery`
- **云端命令**：`MqttTransport::message_arrived -> NetworkManager::subscribe callback -> CloudService/ThingsBoardControlPlane RPC handler -> CommandTracker -> RobotCommandPort.submit -> RobotController::submit_command`
- **遥测上报**：`ThreadExecutor(cloud)::loop -> HealthService::update + ThingsBoardControlPlane::publish_business_telemetry + CloudService::update -> NetworkManager::publish -> MqttTransport/LoRaWANTransport::publish; DataCache 负责离线缓存/重放`
- **配置下发**：`ThingsBoardControlPlane shared attributes/RPC -> ConfigService::apply_runtime_patch/save_pending/promote -> SchedulerService windows 更新 -> RobotController ConfigPorts/MotionService runtime_config_query 后续读取`
- **BMS/滚刷采集**：`ThreadExecutor(bms)::loop -> BMS::update -> BmsProtocol/Bms2Protocol parse + BrushMotor::update -> ODrive ASCII/串口交互`

## 3. 运行时注入端口

- `RobotController::ActionPorts`：由 `main.cc` 注入到 `MotionService::start_segment/stop_cleaning/emergency_stop`、`RecoveryMotion::start`、`FaultService::clear_active_fault`。
- `RobotController::ConfigPorts`：由 `main.cc` 注入到 `ConfigService` 和 `domain::LaneConfig` 查询/提升逻辑。
- `SafetyMonitor` callbacks：限位稳定/不稳定回调在 `main.cc` 连接到 `RobotController::post_limit_settled/post_limit_unstable`。
- `SchedulerService::set_on_window_hit`：在 `main.cc` 连接到 `RobotController::post_schedule_window_hit`。
- `WatchdogMgr::set_timeout_callback`：在 `main.cc` 连接到 `MotionService::emergency_stop` 和 `RobotController::post_watchdog_timeout`。
- `ThreadExecutor::add_runnable`：把 `MotionService/NavService/BMS/BrushMotor/HealthService/CloudService/ThingsBoardControlPlane` 等对象或 lambda 放到周期线程。

## 4. 模块分层总览

### hal
- `hal/i_can_bus`
- `hal/i_gpio_pin`
- `hal/i_modbus_master`
- `hal/i_serial_port`
- `hal/pi_mutex`
### driver
- `driver/libgpiod_pin`
- `driver/libmodbus_master`
- `driver/libserialport_port`
- `driver/linux_can_socket`
### protocol
- `protocol/bms2_protocol`
- `protocol/bms_protocol`
- `protocol/distance_sensor_protocol`
- `protocol/gpsd_json_parser`
- `protocol/imu_protocol`
- `protocol/nmea_parser`
- `protocol/odrive_ascii_protocol`
- `protocol/walk_motor_can_codec`
### device
- `device/attitude_limit_switch`
- `device/bms`
- `device/brush_motor`
- `device/device_error`
- `device/distance_sensor`
- `device/gps_device`
- `device/gps_source`
- `device/gpsd_gps_source`
- `device/imu_device`
- `device/limit_switch`
- `device/serial_gps_source`
- `device/walk_motor_group`
- `device/walk_motor_types`
### middleware
- `middleware/data_cache`
- `middleware/event_bus`
- `middleware/i_network_transport`
- `middleware/logger`
- `middleware/lorawan_transport`
- `middleware/mqtt_transport`
- `middleware/network_manager`
- `middleware/safety_monitor`
- `middleware/thread_executor`
### domain
- `domain/robot_domain`
### service
- `service/cloud_service`
- `service/command_tracker`
- `service/config_service`
- `service/fault_service`
- `service/heading_corrector`
- `service/health_service`
- `service/motion_service`
- `service/nav_service`
- `service/recovery_motion`
- `service/scheduler_service`
- `service/thingsboard_control_plane`
- `service/uds_gyro_yaw_fusion`
### app
- `app/fault_detector`
- `app/fault_policy`
- `app/robot_controller`
- `app/watchdog_mgr`
### main.cc
- `main.cc/main`

## 5. 按模块影响范围反查

`直接下游` = 当前模块 include 或显式限定调用到的生产模块；`直接上游` = 生产模块中 include 或显式调用当前模块者；`传递上游` 最多追溯 4 层。对象成员调用通常通过头文件依赖体现。

### `app/fault_detector`
- 文件：`include/pv_cleaning_robot/app/fault_detector.h`, `pv_cleaning_robot/app/fault_detector.cc`
- 直接下游：`app/fault_policy`, `domain/robot_domain`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：`test/app/fault_detector_test`
### `app/fault_policy`
- 文件：`include/pv_cleaning_robot/app/fault_policy.h`, `pv_cleaning_robot/app/fault_policy.cc`
- 直接下游：`domain/robot_domain`
- 直接上游：`app/fault_detector`, `app/robot_controller`
- 传递上游：`app/fault_detector`, `app/robot_controller`, `main.cc/main`
- 测试影响：`test/app/fault_policy_test`, `test/app/robot_controller_test`
### `app/robot_controller`
- 文件：`include/pv_cleaning_robot/app/robot_controller.h`, `pv_cleaning_robot/app/robot_controller.cc`
- 直接下游：`app/fault_policy`, `domain/robot_domain`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/app/robot_controller_test`, `test/integration/hardware/system_hw_common`
### `app/watchdog_mgr`
- 文件：`include/pv_cleaning_robot/app/watchdog_mgr.h`, `pv_cleaning_robot/app/watchdog_mgr.cc`
- 直接下游：`hal/pi_mutex`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/app/watchdog_mgr_test`, `test/integration/hardware/system_hw_common`
### `device/attitude_limit_switch`
- 文件：`include/pv_cleaning_robot/device/attitude_limit_switch.h`, `pv_cleaning_robot/device/attitude_limit_switch.cc`
- 直接下游：`hal/i_gpio_pin`, `hal/pi_mutex`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：`test/device/attitude_limit_switch_test`, `test/device/limit_switch_test`, `test/integration/hardware/attitude_limit_switch_hw_test`, `test/integration/hardware/hw_config`
### `device/bms`
- 文件：`include/pv_cleaning_robot/device/bms.h`, `pv_cleaning_robot/device/bms.cc`
- 直接下游：`device/device_error`, `hal/i_serial_port`, `hal/pi_mutex`, `protocol/bms_protocol`, `protocol/imu_protocol`, `protocol/nmea_parser`
- 直接上游：`main.cc/main`, `service/health_service`
- 传递上游：`main.cc/main`, `service/health_service`
- 测试影响：`test/device/bms_device_test`, `test/integration/hardware/bms_hw_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/protocol/bms2_protocol_test`, `test/protocol/bms_protocol_test`
### `device/brush_motor`
- 文件：`include/pv_cleaning_robot/device/brush_motor.h`, `pv_cleaning_robot/device/brush_motor.cc`
- 直接下游：`device/device_error`, `driver/linux_can_socket`, `hal/i_serial_port`, `hal/pi_mutex`, `protocol/odrive_ascii_protocol`
- 直接上游：`main.cc/main`, `service/health_service`, `service/motion_service`
- 传递上游：`main.cc/main`, `service/health_service`, `service/motion_service`
- 测试影响：`test/device/brush_motor_test`, `test/integration/hardware/hw_config`, `test/service/motion_service_test`
### `device/device_error`
- 文件：`include/pv_cleaning_robot/device/device_error.h`
- 直接下游：无生产模块直接下游
- 直接上游：`device/bms`, `device/brush_motor`, `device/gps_device`, `device/gps_source`, `device/imu_device`, `device/walk_motor_group`
- 传递上游：`device/bms`, `device/brush_motor`, `device/gps_device`, `device/gps_source`, `device/gpsd_gps_source`, `device/imu_device`, `device/serial_gps_source`, `device/walk_motor_group`, `main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：未识别到直接测试
### `device/distance_sensor`
- 文件：`include/pv_cleaning_robot/device/distance_sensor.h`, `pv_cleaning_robot/device/distance_sensor.cc`
- 直接下游：`hal/i_modbus_master`, `hal/pi_mutex`, `protocol/distance_sensor_protocol`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：`test/device/distance_sensor_device_test`, `test/integration/hardware/distance_sensor_hw_test`, `test/protocol/distance_sensor_protocol_test`
### `device/gps_device`
- 文件：`include/pv_cleaning_robot/device/gps_device.h`, `pv_cleaning_robot/device/gps_device.cc`
- 直接下游：`device/device_error`, `device/gps_source`, `hal/i_serial_port`, `protocol/nmea_parser`
- 直接上游：`main.cc/main`, `service/health_service`, `service/nav_service`
- 传递上游：`main.cc/main`, `service/health_service`, `service/nav_service`
- 测试影响：`test/device/gps_device_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`
### `device/gps_source`
- 文件：`include/pv_cleaning_robot/device/gps_source.h`
- 直接下游：`device/device_error`, `protocol/gpsd_json_parser`, `protocol/nmea_parser`
- 直接上游：`device/gps_device`, `device/gpsd_gps_source`, `device/serial_gps_source`
- 传递上游：`device/gps_device`, `device/gpsd_gps_source`, `device/serial_gps_source`, `main.cc/main`, `service/health_service`, `service/nav_service`
- 测试影响：未识别到直接测试
### `device/gpsd_gps_source`
- 文件：`pv_cleaning_robot/device/gpsd_gps_source.cc`
- 直接下游：`device/gps_source`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：未识别到直接测试
### `device/imu_device`
- 文件：`include/pv_cleaning_robot/device/imu_device.h`, `pv_cleaning_robot/device/imu_device.cc`
- 直接下游：`device/device_error`, `hal/i_serial_port`, `hal/pi_mutex`, `protocol/imu_protocol`
- 直接上游：`main.cc/main`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 传递上游：`main.cc/main`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/imu_device_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/imu_hw_test`, `test/integration/hardware/system_hw_common`, `test/protocol/imu_protocol_test`
### `device/limit_switch`
- 文件：`include/pv_cleaning_robot/device/limit_switch.h`, `pv_cleaning_robot/device/limit_switch.cc`
- 直接下游：`hal/i_gpio_pin`, `hal/pi_mutex`
- 直接上游：`main.cc/main`, `middleware/safety_monitor`
- 传递上游：`main.cc/main`, `middleware/safety_monitor`, `service/command_tracker`, `service/thingsboard_control_plane`
- 测试影响：`test/device/attitude_limit_switch_test`, `test/device/limit_switch_test`, `test/integration/hardware/attitude_limit_switch_hw_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/limit_switch_hw_test`, `test/middleware/safety_monitor_test`
### `device/serial_gps_source`
- 文件：`pv_cleaning_robot/device/serial_gps_source.cc`
- 直接下游：`device/gps_source`, `hal/i_serial_port`, `protocol/nmea_parser`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：未识别到直接测试
### `device/walk_motor_group`
- 文件：`include/pv_cleaning_robot/device/walk_motor_group.h`, `pv_cleaning_robot/device/walk_motor_group.cc`
- 直接下游：`device/device_error`, `device/walk_motor_types`, `driver/linux_can_socket`, `hal/i_can_bus`, `hal/pi_mutex`, `protocol/walk_motor_can_codec`
- 直接上游：`main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 传递上游：`main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/walk_motor_group_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/walk_motor_group_hw_test`, `test/middleware/safety_monitor_test`, `test/protocol/walk_motor_codec_test`, `test/service/motion_service_test`
### `device/walk_motor_types`
- 文件：`include/pv_cleaning_robot/device/walk_motor_types.h`
- 直接下游：`protocol/walk_motor_can_codec`
- 直接上游：`device/walk_motor_group`
- 传递上游：`device/walk_motor_group`, `main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/walk_motor_group_test`, `test/protocol/walk_motor_codec_test`
### `domain/robot_domain`
- 文件：`include/pv_cleaning_robot/domain/robot_domain.h`
- 直接下游：无生产模块直接下游
- 直接上游：`app/fault_detector`, `app/fault_policy`, `app/robot_controller`, `main.cc/main`, `middleware/safety_monitor`, `service/config_service`, `service/fault_service`, `service/heading_corrector`, `service/motion_service`, `service/thingsboard_control_plane`
- 传递上游：`app/fault_detector`, `app/fault_policy`, `app/robot_controller`, `main.cc/main`, `middleware/safety_monitor`, `service/command_tracker`, `service/config_service`, `service/fault_service`, `service/heading_corrector`, `service/motion_service`, `service/thingsboard_control_plane`
- 测试影响：`test/app/fault_detector_test`, `test/app/fault_policy_test`, `test/app/robot_controller_test`, `test/domain/robot_domain_test`, `test/service/business_payload_builder_test`
### `driver/libgpiod_pin`
- 文件：`include/pv_cleaning_robot/driver/libgpiod_pin.h`, `pv_cleaning_robot/driver/libgpiod_pin.cc`
- 直接下游：`hal/i_gpio_pin`, `hal/pi_mutex`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/driver/libgpiod_pin_test`, `test/integration/hardware/hw_config`
### `driver/libmodbus_master`
- 文件：`include/pv_cleaning_robot/driver/libmodbus_master.h`, `pv_cleaning_robot/driver/libmodbus_master.cc`
- 直接下游：`hal/i_modbus_master`, `hal/pi_mutex`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/driver/libmodbus_test`, `test/integration/hardware/distance_sensor_hw_test`, `test/integration/hardware/hw_config`, `test/protocol/bms2_protocol_test`
### `driver/libserialport_port`
- 文件：`include/pv_cleaning_robot/driver/libserialport_port.h`, `pv_cleaning_robot/driver/libserialport_port.cc`
- 直接下游：`hal/i_serial_port`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/device/bms_device_test`, `test/device/imu_device_test`, `test/driver/libserialport_test`, `test/integration/hardware/bms_hw_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/imu_hw_test`, `test/protocol/bms_protocol_test`, `test/protocol/imu_protocol_test`
### `driver/linux_can_socket`
- 文件：`include/pv_cleaning_robot/driver/linux_can_socket.h`, `pv_cleaning_robot/driver/linux_can_socket.cc`
- 直接下游：`hal/i_can_bus`
- 直接上游：`device/brush_motor`, `device/walk_motor_group`, `main.cc/main`
- 传递上游：`device/brush_motor`, `device/walk_motor_group`, `main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/walk_motor_group_test`, `test/driver/linux_can_socket_test`, `test/integration/hardware/hw_config`, `test/protocol/walk_motor_codec_test`
### `hal/i_can_bus`
- 文件：`include/pv_cleaning_robot/hal/i_can_bus.h`
- 直接下游：无生产模块直接下游
- 直接上游：`device/walk_motor_group`, `driver/linux_can_socket`, `protocol/walk_motor_can_codec`
- 传递上游：`device/brush_motor`, `device/walk_motor_group`, `device/walk_motor_types`, `driver/linux_can_socket`, `main.cc/main`, `protocol/walk_motor_can_codec`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/mock/mock_can_bus`
### `hal/i_gpio_pin`
- 文件：`include/pv_cleaning_robot/hal/i_gpio_pin.h`
- 直接下游：无生产模块直接下游
- 直接上游：`device/attitude_limit_switch`, `device/limit_switch`, `driver/libgpiod_pin`
- 传递上游：`device/attitude_limit_switch`, `device/limit_switch`, `driver/libgpiod_pin`, `main.cc/main`, `middleware/safety_monitor`, `service/command_tracker`, `service/thingsboard_control_plane`
- 测试影响：`test/mock/mock_gpio_pin`
### `hal/i_modbus_master`
- 文件：`include/pv_cleaning_robot/hal/i_modbus_master.h`
- 直接下游：无生产模块直接下游
- 直接上游：`device/distance_sensor`, `driver/libmodbus_master`, `main.cc/main`
- 传递上游：`device/distance_sensor`, `driver/libmodbus_master`, `main.cc/main`
- 测试影响：`test/mock/mock_modbus_master`
### `hal/i_serial_port`
- 文件：`include/pv_cleaning_robot/hal/i_serial_port.h`
- 直接下游：无生产模块直接下游
- 直接上游：`device/bms`, `device/brush_motor`, `device/gps_device`, `device/imu_device`, `device/serial_gps_source`, `driver/libserialport_port`, `main.cc/main`, `middleware/lorawan_transport`
- 传递上游：`device/bms`, `device/brush_motor`, `device/gps_device`, `device/imu_device`, `device/serial_gps_source`, `driver/libserialport_port`, `main.cc/main`, `middleware/lorawan_transport`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/mock/mock_serial_port`
### `hal/pi_mutex`
- 文件：`include/pv_cleaning_robot/hal/pi_mutex.h`
- 直接下游：无生产模块直接下游
- 直接上游：`app/watchdog_mgr`, `device/attitude_limit_switch`, `device/bms`, `device/brush_motor`, `device/distance_sensor`, `device/imu_device`, `device/limit_switch`, `device/walk_motor_group`, `driver/libgpiod_pin`, `driver/libmodbus_master`, `middleware/event_bus`, `service/fault_service`, `service/heading_corrector`, `service/nav_service`
- 传递上游：`app/watchdog_mgr`, `device/attitude_limit_switch`, `device/bms`, `device/brush_motor`, `device/distance_sensor`, `device/imu_device`, `device/limit_switch`, `device/walk_motor_group`, `driver/libgpiod_pin`, `driver/libmodbus_master`, `main.cc/main`, `middleware/event_bus`, `middleware/mqtt_transport`, `middleware/network_manager`, `middleware/safety_monitor`, `service/cloud_service`, `service/command_tracker`, `service/fault_service`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`, `service/thingsboard_control_plane`
- 测试影响：`test/driver/pi_mutex_test`
### `main.cc/main`
- 文件：`pv_cleaning_robot/main.cc`
- 直接下游：`app/robot_controller`, `app/watchdog_mgr`, `device/bms`, `device/brush_motor`, `device/gps_device`, `device/imu_device`, `device/limit_switch`, `device/walk_motor_group`, `domain/robot_domain`, `driver/libgpiod_pin`, `driver/libmodbus_master`, `driver/libserialport_port`, `driver/linux_can_socket`, `hal/i_modbus_master`, `hal/i_serial_port`, `middleware/data_cache`, `middleware/event_bus`, `middleware/logger`, `middleware/lorawan_transport`, `middleware/mqtt_transport`, `middleware/network_manager`, `middleware/safety_monitor`, `middleware/thread_executor`, `service/cloud_service`, `service/command_tracker`, `service/config_service`, `service/fault_service`, `service/health_service`, `service/motion_service`, `service/nav_service`, `service/recovery_motion`, `service/scheduler_service`, `service/thingsboard_control_plane`
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：`test/domain/robot_domain_test`, `test/integration/hardware/hw_test_main`, `test/test_main`
### `middleware/data_cache`
- 文件：`include/pv_cleaning_robot/middleware/data_cache.h`, `pv_cleaning_robot/middleware/data_cache.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`main.cc/main`, `service/cloud_service`
- 传递上游：`main.cc/main`, `service/cloud_service`, `service/health_service`, `service/thingsboard_control_plane`
- 测试影响：`test/middleware/data_cache_test`, `test/service/thingsboard_control_plane_test`
### `middleware/event_bus`
- 文件：`include/pv_cleaning_robot/middleware/event_bus.h`
- 直接下游：`hal/pi_mutex`
- 直接上游：`main.cc/main`, `middleware/mqtt_transport`, `middleware/network_manager`, `middleware/safety_monitor`, `service/cloud_service`, `service/fault_service`, `service/motion_service`
- 传递上游：`main.cc/main`, `middleware/mqtt_transport`, `middleware/network_manager`, `middleware/safety_monitor`, `service/cloud_service`, `service/command_tracker`, `service/fault_service`, `service/health_service`, `service/motion_service`, `service/thingsboard_control_plane`
- 测试影响：`test/integration/hardware/system_hw_common`, `test/middleware/event_bus_test`, `test/middleware/safety_monitor_test`, `test/service/fault_service_test`, `test/service/motion_service_test`
### `middleware/i_network_transport`
- 文件：`include/pv_cleaning_robot/middleware/i_network_transport.h`
- 直接下游：无生产模块直接下游
- 直接上游：`middleware/lorawan_transport`, `middleware/mqtt_transport`, `middleware/network_manager`
- 传递上游：`main.cc/main`, `middleware/lorawan_transport`, `middleware/mqtt_transport`, `middleware/network_manager`, `service/cloud_service`, `service/health_service`, `service/thingsboard_control_plane`
- 测试影响：未识别到直接测试
### `middleware/logger`
- 文件：`include/pv_cleaning_robot/middleware/logger.h`, `pv_cleaning_robot/middleware/logger.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：未识别到直接测试
### `middleware/lorawan_transport`
- 文件：`include/pv_cleaning_robot/middleware/lorawan_transport.h`, `pv_cleaning_robot/middleware/lorawan_transport.cc`
- 直接下游：`hal/i_serial_port`, `middleware/i_network_transport`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：未识别到直接测试
### `middleware/mqtt_transport`
- 文件：`include/pv_cleaning_robot/middleware/mqtt_transport.h`, `pv_cleaning_robot/middleware/mqtt_transport.cc`
- 直接下游：`middleware/event_bus`, `middleware/i_network_transport`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/integration/thingsboard_test_support`, `test/middleware/mqtt_transport_test`
### `middleware/network_manager`
- 文件：`include/pv_cleaning_robot/middleware/network_manager.h`, `pv_cleaning_robot/middleware/network_manager.cc`
- 直接下游：`middleware/event_bus`, `middleware/i_network_transport`
- 直接上游：`main.cc/main`, `service/cloud_service`
- 传递上游：`main.cc/main`, `service/cloud_service`, `service/health_service`, `service/thingsboard_control_plane`
- 测试影响：`test/middleware/network_manager_test`, `test/service/thingsboard_control_plane_test`
### `middleware/safety_monitor`
- 文件：`include/pv_cleaning_robot/middleware/safety_monitor.h`, `pv_cleaning_robot/middleware/safety_monitor.cc`
- 直接下游：`device/limit_switch`, `domain/robot_domain`, `middleware/event_bus`
- 直接上游：`main.cc/main`, `service/command_tracker`
- 传递上游：`main.cc/main`, `service/command_tracker`, `service/thingsboard_control_plane`
- 测试影响：`test/integration/hardware/system_hw_common`, `test/middleware/safety_monitor_test`
### `middleware/thread_executor`
- 文件：`include/pv_cleaning_robot/middleware/thread_executor.h`, `pv_cleaning_robot/middleware/thread_executor.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`main.cc/main`, `service/cloud_service`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 传递上游：`main.cc/main`, `service/cloud_service`, `service/health_service`, `service/motion_service`, `service/nav_service`, `service/thingsboard_control_plane`
- 测试影响：`test/middleware/thread_executor_test`
### `protocol/bms2_protocol`
- 文件：`include/pv_cleaning_robot/protocol/bms2_protocol.h`, `pv_cleaning_robot/protocol/bms2_protocol.cc`
- 直接下游：无生产模块直接下游
- 直接上游：无生产模块直接上游
- 传递上游：无
- 测试影响：`test/protocol/bms2_protocol_test`
### `protocol/bms_protocol`
- 文件：`include/pv_cleaning_robot/protocol/bms_protocol.h`, `pv_cleaning_robot/protocol/bms_protocol.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`device/bms`
- 传递上游：`device/bms`, `main.cc/main`, `service/health_service`
- 测试影响：`test/device/bms_device_test`, `test/integration/hardware/bms_hw_test`, `test/protocol/bms_protocol_test`
### `protocol/distance_sensor_protocol`
- 文件：`include/pv_cleaning_robot/protocol/distance_sensor_protocol.h`, `pv_cleaning_robot/protocol/distance_sensor_protocol.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`device/distance_sensor`
- 传递上游：`device/distance_sensor`
- 测试影响：`test/integration/hardware/distance_sensor_hw_test`, `test/protocol/distance_sensor_protocol_test`
### `protocol/gpsd_json_parser`
- 文件：`include/pv_cleaning_robot/protocol/gpsd_json_parser.h`, `pv_cleaning_robot/protocol/gpsd_json_parser.cc`
- 直接下游：`protocol/nmea_parser`
- 直接上游：`device/gps_source`
- 传递上游：`device/gps_device`, `device/gps_source`, `device/gpsd_gps_source`, `device/serial_gps_source`, `main.cc/main`, `service/health_service`, `service/nav_service`
- 测试影响：`test/protocol/gpsd_json_parser_test`
### `protocol/imu_protocol`
- 文件：`include/pv_cleaning_robot/protocol/imu_protocol.h`, `pv_cleaning_robot/protocol/imu_protocol.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`device/bms`, `device/imu_device`
- 传递上游：`device/bms`, `device/imu_device`, `main.cc/main`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/imu_device_test`, `test/integration/hardware/imu_hw_test`, `test/protocol/imu_protocol_test`
### `protocol/nmea_parser`
- 文件：`include/pv_cleaning_robot/protocol/nmea_parser.h`, `pv_cleaning_robot/protocol/nmea_parser.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`device/bms`, `device/gps_device`, `device/gps_source`, `device/serial_gps_source`, `protocol/gpsd_json_parser`, `service/health_service`
- 传递上游：`device/bms`, `device/gps_device`, `device/gps_source`, `device/gpsd_gps_source`, `device/serial_gps_source`, `main.cc/main`, `protocol/gpsd_json_parser`, `service/health_service`, `service/nav_service`
- 测试影响：`test/protocol/nmea_parser_test`
### `protocol/odrive_ascii_protocol`
- 文件：`include/pv_cleaning_robot/protocol/odrive_ascii_protocol.h`, `pv_cleaning_robot/protocol/odrive_ascii_protocol.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`device/brush_motor`
- 传递上游：`device/brush_motor`, `main.cc/main`, `service/health_service`, `service/motion_service`
- 测试影响：`test/protocol/odrive_ascii_protocol_test`
### `protocol/walk_motor_can_codec`
- 文件：`include/pv_cleaning_robot/protocol/walk_motor_can_codec.h`, `pv_cleaning_robot/protocol/walk_motor_can_codec.cc`
- 直接下游：`hal/i_can_bus`
- 直接上游：`device/walk_motor_group`, `device/walk_motor_types`
- 传递上游：`device/walk_motor_group`, `device/walk_motor_types`, `main.cc/main`, `service/heading_corrector`, `service/health_service`, `service/motion_service`, `service/nav_service`
- 测试影响：`test/device/walk_motor_group_test`, `test/protocol/walk_motor_codec_test`, `test/service/motion_service_test`
### `service/cloud_service`
- 文件：`include/pv_cleaning_robot/service/cloud_service.h`, `pv_cleaning_robot/service/cloud_service.cc`
- 直接下游：`middleware/data_cache`, `middleware/event_bus`, `middleware/network_manager`, `middleware/thread_executor`
- 直接上游：`main.cc/main`, `service/health_service`, `service/thingsboard_control_plane`
- 传递上游：`main.cc/main`, `service/health_service`, `service/thingsboard_control_plane`
- 测试影响：`test/service/cloud_service_test`, `test/service/thingsboard_control_plane_test`
### `service/command_tracker`
- 文件：`include/pv_cleaning_robot/service/command_tracker.h`, `pv_cleaning_robot/service/command_tracker.cc`
- 直接下游：`middleware/safety_monitor`
- 直接上游：`main.cc/main`, `service/thingsboard_control_plane`
- 传递上游：`main.cc/main`, `service/thingsboard_control_plane`
- 测试影响：`test/service/command_tracker_test`, `test/service/thingsboard_control_plane_test`
### `service/config_service`
- 文件：`include/pv_cleaning_robot/service/config_service.h`, `pv_cleaning_robot/service/config_service.cc`
- 直接下游：`domain/robot_domain`, `service/scheduler_service`
- 直接上游：`main.cc/main`, `service/motion_service`, `service/thingsboard_control_plane`
- 传递上游：`main.cc/main`, `service/motion_service`, `service/thingsboard_control_plane`
- 测试影响：`test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/integration/thingsboard_test_support`, `test/service/config_service_runtime_patch_test`, `test/service/config_service_test`, `test/service/thingsboard_control_plane_test`
### `service/fault_service`
- 文件：`include/pv_cleaning_robot/service/fault_service.h`, `pv_cleaning_robot/service/fault_service.cc`
- 直接下游：`domain/robot_domain`, `hal/pi_mutex`, `middleware/event_bus`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/integration/hardware/system_hw_common`, `test/service/fault_service_test`
### `service/heading_corrector`
- 文件：`include/pv_cleaning_robot/service/heading_corrector.h`, `pv_cleaning_robot/service/heading_corrector.cc`
- 直接下游：`device/walk_motor_group`, `domain/robot_domain`, `hal/pi_mutex`, `service/uds_gyro_yaw_fusion`
- 直接上游：`service/motion_service`
- 传递上游：`main.cc/main`, `service/motion_service`
- 测试影响：`test/service/heading_pid_test`
### `service/health_service`
- 文件：`include/pv_cleaning_robot/service/health_service.h`, `pv_cleaning_robot/service/health_service.cc`
- 直接下游：`device/bms`, `device/brush_motor`, `device/gps_device`, `device/imu_device`, `device/walk_motor_group`, `middleware/thread_executor`, `protocol/nmea_parser`, `service/cloud_service`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/service/health_payload_builder_test`
### `service/motion_service`
- 文件：`include/pv_cleaning_robot/service/motion_service.h`, `pv_cleaning_robot/service/motion_service.cc`
- 直接下游：`device/brush_motor`, `device/imu_device`, `device/walk_motor_group`, `domain/robot_domain`, `middleware/event_bus`, `middleware/thread_executor`, `service/config_service`, `service/heading_corrector`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/integration/hardware/system_hw_common`, `test/service/motion_service_test`
### `service/nav_service`
- 文件：`include/pv_cleaning_robot/service/nav_service.h`, `pv_cleaning_robot/service/nav_service.cc`
- 直接下游：`device/gps_device`, `device/imu_device`, `device/walk_motor_group`, `hal/pi_mutex`, `middleware/thread_executor`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/integration/hardware/system_hw_common`
### `service/recovery_motion`
- 文件：`include/pv_cleaning_robot/service/recovery_motion.h`, `pv_cleaning_robot/service/recovery_motion.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/service/recovery_motion_test`
### `service/scheduler_service`
- 文件：`include/pv_cleaning_robot/service/scheduler_service.h`, `pv_cleaning_robot/service/scheduler_service.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`main.cc/main`, `service/config_service`, `service/thingsboard_control_plane`
- 传递上游：`main.cc/main`, `service/config_service`, `service/motion_service`, `service/thingsboard_control_plane`
- 测试影响：`test/service/config_service_runtime_patch_test`, `test/service/scheduler_service_test`, `test/service/thingsboard_control_plane_test`
### `service/thingsboard_control_plane`
- 文件：`include/pv_cleaning_robot/service/thingsboard_control_plane.h`, `pv_cleaning_robot/service/thingsboard_control_plane.cc`
- 直接下游：`domain/robot_domain`, `service/cloud_service`, `service/command_tracker`, `service/config_service`, `service/scheduler_service`
- 直接上游：`main.cc/main`
- 传递上游：`main.cc/main`
- 测试影响：`test/service/business_payload_builder_test`, `test/service/config_service_runtime_patch_test`, `test/service/thingsboard_control_plane_test`, `test/service/thingsboard_event_payload_builder_test`
### `service/uds_gyro_yaw_fusion`
- 文件：`include/pv_cleaning_robot/service/uds_gyro_yaw_fusion.h`, `pv_cleaning_robot/service/uds_gyro_yaw_fusion.cc`
- 直接下游：无生产模块直接下游
- 直接上游：`service/heading_corrector`
- 传递上游：`main.cc/main`, `service/heading_corrector`, `service/motion_service`
- 测试影响：`test/integration/hardware/system_hw_common`, `test/service/uds_gyro_yaw_fusion_test`

## 6. 函数/方法级直接调用索引（生产源码，降噪版）

只列同模块唯一解析调用、显式 `Class::method`/`namespace::function` 调用；不列 STL/容器常用方法和无法确定对象类型的成员调用。

### `app/fault_policy::FaultPolicy::decide`
- -> `app/fault_policy::is_p0_code`
### `app/robot_controller::RobotController::complete_self_check`
- -> `app/robot_controller::RobotController::complete_self_check_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::complete_self_check_for_test`
- -> `app/robot_controller::RobotController::complete_self_check_locked`
### `app/robot_controller::RobotController::complete_self_check_locked`
- -> `app/robot_controller::RobotController::start_current_segment_locked`
### `app/robot_controller::RobotController::handle_fault_for_test`
- -> `app/robot_controller::RobotController::handle_fault_locked`
### `app/robot_controller::RobotController::handle_limit_settled_for_test`
- -> `app/robot_controller::RobotController::handle_limit_settled_locked`
### `app/robot_controller::RobotController::handle_limit_settled_locked`
- -> `app/robot_controller::RobotController::start_current_segment_locked`
- -> `domain/robot_domain::current_segment`
### `app/robot_controller::RobotController::handle_limit_unstable_locked`
- -> `app/robot_controller::RobotController::handle_fault_locked`
### `app/robot_controller::RobotController::handle_recovery_finished_locked`
- -> `app/robot_controller::RobotController::handle_fault_locked`
- -> `app/robot_controller::RobotController::start_current_segment_locked`
### `app/robot_controller::RobotController::handle_watchdog_timeout_locked`
- -> `app/robot_controller::RobotController::handle_fault_locked`
### `app/robot_controller::RobotController::post_fault`
- -> `app/robot_controller::RobotController::handle_fault_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_for_test`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_limit_settled`
- -> `app/robot_controller::RobotController::handle_limit_settled_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_limit_unstable`
- -> `app/robot_controller::RobotController::handle_limit_unstable_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_recovery_finished`
- -> `app/robot_controller::RobotController::handle_recovery_finished_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_schedule_window_hit`
- -> `app/robot_controller::RobotController::post`
- -> `app/robot_controller::RobotController::submit_command_locked`
### `app/robot_controller::RobotController::post_tick`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::post_watchdog_timeout`
- -> `app/robot_controller::RobotController::handle_watchdog_timeout_locked`
- -> `app/robot_controller::RobotController::post`
### `app/robot_controller::RobotController::snapshot`
- -> `app/robot_controller::RobotController::state_name`
### `app/robot_controller::RobotController::start`
- -> `app/robot_controller::RobotController::loop`
### `app/robot_controller::RobotController::start_command_locked`
- -> `app/robot_controller::RobotController::build_start_mission_locked`
- -> `app/robot_controller::RobotController::validate_start_command_locked`
### `app/robot_controller::RobotController::start_current_segment_locked`
- -> `domain/robot_domain::current_segment`
### `app/robot_controller::RobotController::stop_locked`
- -> `app/robot_controller::RobotController::mission_active`
### `app/robot_controller::RobotController::submit_command`
- -> `app/robot_controller::RobotController::post`
- -> `app/robot_controller::RobotController::submit_command_locked`
### `app/robot_controller::RobotController::submit_command_locked`
- -> `app/robot_controller::RobotController::start_command_locked`
- -> `app/robot_controller::RobotController::stop_locked`
### `app/watchdog_mgr::WatchdogMgr::monitor_loop`
- -> `app/watchdog_mgr::WatchdogMgr::feed_hw_watchdog`
### `device/attitude_limit_switch::AttitudeLimitSwitch::close`
- -> `device/attitude_limit_switch::AttitudeLimitSwitch::stop_monitoring`
### `device/attitude_limit_switch::AttitudeLimitSwitch::on_edge`
- -> `hal/pi_mutex::lock`
### `device/attitude_limit_switch::AttitudeLimitSwitch::read_status`
- -> `device/attitude_limit_switch::AttitudeLimitSwitch::is_triggered`
- -> `device/attitude_limit_switch::AttitudeLimitSwitch::read_current_level`
### `device/attitude_limit_switch::AttitudeLimitSwitch::set_trigger_callback`
- -> `hal/pi_mutex::lock`
### `device/attitude_limit_switch::AttitudeLimitSwitch::start_monitoring`
- -> `device/attitude_limit_switch::AttitudeLimitSwitch::on_edge`
### `device/bms::BMS::mos_control`
- -> `device/bms::BMS::transact`
- -> `protocol/bms_protocol::is_error`
### `device/bms::BMS::open`
- -> `device/bms::BMS::read_basic_info_uart`
- -> `device/bms::BMS::transact`
- -> `protocol/bms_protocol::get_cmd`
- -> `protocol/bms_protocol::get_data_len`
- -> `protocol/bms_protocol::is_error`
- -> `protocol/nmea_parser::get_data`
### `device/bms::BMS::read_basic_info_uart`
- -> `device/bms::BMS::transact`
- -> `protocol/bms_protocol::get_cmd`
- -> `protocol/bms_protocol::get_data_len`
- -> `protocol/bms_protocol::is_error`
- -> `protocol/nmea_parser::get_data`
### `device/bms::BMS::read_cell_voltages_uart`
- -> `device/bms::BMS::transact`
- -> `protocol/bms_protocol::get_cmd`
- -> `protocol/bms_protocol::get_data_len`
- -> `protocol/bms_protocol::is_error`
- -> `protocol/nmea_parser::get_data`
### `device/bms::BMS::transact`
- -> `hal/pi_mutex::lock`
- -> `protocol/imu_protocol::frame_complete`
### `device/bms::BMS::update`
- -> `device/bms::BMS::read_basic_info_uart`
- -> `device/bms::BMS::read_cell_voltages_uart`
### `device/brush_motor::BrushMotor::clear_fault`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/brush_motor::BrushMotor::close`
- -> `device/brush_motor::BrushMotor::update_running_locked`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/brush_motor::BrushMotor::read_line_locked`
- -> `device/brush_motor::uart_error_to_device_error`
- -> `driver/linux_can_socket::get_last_error`
### `device/brush_motor::BrushMotor::request_ascii_locked`
- -> `device/brush_motor::BrushMotor::read_line_locked`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/brush_motor::BrushMotor::restart`
- -> `device/brush_motor::BrushMotor::update_running_locked`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/brush_motor::BrushMotor::set_rpm`
- -> `device/brush_motor::BrushMotor::update_running_locked`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
- -> `device/brush_motor::rpm_to_turns_per_sec`
### `device/brush_motor::BrushMotor::stop`
- -> `device/brush_motor::BrushMotor::update_running_locked`
- -> `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/brush_motor::BrushMotor::update`
- -> `device/brush_motor::BrushMotor::mark_comm_error_locked`
- -> `device/brush_motor::BrushMotor::request_ascii_locked`
- -> `device/brush_motor::BrushMotor::update_running_locked`
- -> `device/brush_motor::turns_per_sec_to_rpm`
### `device/brush_motor::BrushMotor::write_ascii_locked`
- -> `device/brush_motor::uart_error_to_device_error`
- -> `driver/linux_can_socket::get_last_error`
### `device/distance_sensor::DistanceSensor::update`
- -> `protocol/distance_sensor_protocol::DistanceSensorProtocol::decode_register`
- -> `protocol/distance_sensor_protocol::DistanceSensorProtocol::voltage_to_ma`
### `device/gps_device::GpsDevice::create_gpsd`
- -> `device/gps_device::GpsDevice::on_source_message`
- -> `device/gps_device::GpsDevice::on_source_message_count`
- -> `device/gps_device::GpsDevice::on_source_parse_error`
### `device/gpsd_gps_source::GpsdGpsSource::close`
- -> `device/gpsd_gps_source::close_socket`
### `device/gpsd_gps_source::GpsdGpsSource::connect_socket`
- -> `device/gpsd_gps_source::close_socket`
### `device/gpsd_gps_source::GpsdGpsSource::open`
- -> `device/gpsd_gps_source::GpsdGpsSource::connect_socket`
- -> `device/gpsd_gps_source::GpsdGpsSource::send_watch`
- -> `device/gpsd_gps_source::close_socket`
### `device/gpsd_gps_source::GpsdGpsSource::read_loop`
- -> `device/gpsd_gps_source::GpsdGpsSource::connect_socket`
- -> `device/gpsd_gps_source::GpsdGpsSource::handle_json_line`
- -> `device/gpsd_gps_source::GpsdGpsSource::send_watch`
- -> `device/gpsd_gps_source::close_socket`
### `device/imu_device::ImuDevice::calibrate_gyro`
- -> `device/imu_device::ImuDevice::send_write_cmd`
### `device/imu_device::ImuDevice::read_loop`
- -> `protocol/imu_protocol::frame_complete`
### `device/imu_device::ImuDevice::reset`
- -> `device/imu_device::ImuDevice::send_write_cmd`
### `device/imu_device::ImuDevice::save_config`
- -> `device/imu_device::ImuDevice::send_write_cmd`
### `device/imu_device::ImuDevice::send_write_cmd`
- -> `device/imu_device::ImuDevice::send_command`
### `device/imu_device::ImuDevice::set_output_rate`
- -> `device/imu_device::ImuDevice::send_write_cmd`
### `device/limit_switch::LimitSwitch::close`
- -> `device/limit_switch::LimitSwitch::stop_monitoring`
### `device/limit_switch::LimitSwitch::on_edge`
- -> `hal/pi_mutex::lock`
### `device/limit_switch::LimitSwitch::set_trigger_callback`
- -> `hal/pi_mutex::lock`
### `device/limit_switch::LimitSwitch::start_monitoring`
- -> `device/limit_switch::LimitSwitch::on_edge`
### `device/serial_gps_source::SerialGpsSource::cold_restart`
- -> `device/serial_gps_source::write_command`
### `device/serial_gps_source::SerialGpsSource::hot_restart`
- -> `device/serial_gps_source::write_command`
### `device/serial_gps_source::SerialGpsSource::read_loop`
- -> `protocol/nmea_parser::get_data`
### `device/serial_gps_source::SerialGpsSource::set_output_rate`
- -> `device/serial_gps_source::write_command`
### `device/walk_motor_group::WalkMotorGroup::disable_all`
- -> `device/walk_motor_group::WalkMotorGroup::set_mode_all`
### `device/walk_motor_group::WalkMotorGroup::emergency_override`
- -> `device/walk_motor_group::clamp_rpm`
### `device/walk_motor_group::WalkMotorGroup::enable_all`
- -> `device/walk_motor_group::WalkMotorGroup::set_mode_all`
### `device/walk_motor_group::WalkMotorGroup::open`
- -> `device/walk_motor_group::make_group_termination_frame`
- -> `device/walk_motor_group::motor_id_in_group`
- -> `hal/i_can_bus::set_filters`
- -> `protocol/walk_motor_can_codec::status_can_id`
### `device/walk_motor_group::WalkMotorGroup::query_firmware`
- -> `device/walk_motor_group::WalkMotorGroup::send_ctrl`
### `device/walk_motor_group::WalkMotorGroup::recv_loop`
- -> `driver/linux_can_socket::is_bus_off`
### `device/walk_motor_group::WalkMotorGroup::set_feedback_mode_all`
- -> `device/walk_motor_group::WalkMotorGroup::send_ctrl`
### `device/walk_motor_group::WalkMotorGroup::set_mode_all`
- -> `device/walk_motor_group::WalkMotorGroup::send_ctrl`
### `device/walk_motor_group::WalkMotorGroup::set_modes`
- -> `device/walk_motor_group::WalkMotorGroup::send_ctrl`
### `device/walk_motor_group::WalkMotorGroup::set_positions`
- -> `device/walk_motor_group::clamp`
### `device/walk_motor_group::WalkMotorGroup::set_speeds`
- -> `device/walk_motor_group::clamp_rpm`
### `device/walk_motor_group::WalkMotorGroup::set_terminations`
- -> `device/walk_motor_group::WalkMotorGroup::send_ctrl`
### `domain/robot_domain::build_configured_mission_context`
- -> `domain/robot_domain::endpoint_from_position`
- -> `domain/robot_domain::opposite_endpoint`
### `domain/robot_domain::build_directional_clean_context`
- -> `domain/robot_domain::opposite_endpoint`
### `domain/robot_domain::can_start_configured_mission`
- -> `domain/robot_domain::endpoint_from_position`
### `domain/robot_domain::is_at_target`
- -> `domain/robot_domain::endpoint_from_position`
### `driver/libgpiod_pin::LibGpiodPin::close`
- -> `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::is_open`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::monitor_loop`
- -> `driver/libgpiod_pin::LibGpiodPin::setup_thread_rt_`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::open`
- -> `driver/libgpiod_pin::LibGpiodPin::request_line_as_input`
- -> `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::poll_loop`
- -> `driver/libgpiod_pin::LibGpiodPin::setup_thread_rt_`
### `driver/libgpiod_pin::LibGpiodPin::read_value`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::set_edge_callback`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::start_monitoring`
- -> `driver/libgpiod_pin::LibGpiodPin::request_line_as_input`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::stop_monitoring`
- -> `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
- -> `hal/pi_mutex::lock`
### `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
- -> `driver/libgpiod_pin::LibGpiodPin::request_line_as_input`
### `driver/libgpiod_pin::LibGpiodPin::write_value`
- -> `hal/pi_mutex::lock`
### `driver/libmodbus_master::LibModbusMaster::close`
- -> `hal/pi_mutex::lock`
### `driver/libmodbus_master::LibModbusMaster::execute_with_retry`
- -> `driver/libmodbus_master::LibModbusMaster::map_errno_to_result`
- -> `hal/pi_mutex::lock`
### `driver/libmodbus_master::LibModbusMaster::open`
- -> `driver/libmodbus_master::LibModbusMaster::is_open`
- -> `driver/libmodbus_master::LibModbusMaster::map_errno_to_result`
- -> `hal/pi_mutex::lock`
### `driver/libmodbus_master::LibModbusMaster::read_input_registers`
- -> `driver/libmodbus_master::execute_with_retry`
### `driver/libmodbus_master::LibModbusMaster::read_registers`
- -> `driver/libmodbus_master::execute_with_retry`
### `driver/libmodbus_master::LibModbusMaster::set_timeout_ms`
- -> `hal/pi_mutex::lock`
### `driver/libmodbus_master::LibModbusMaster::write_register`
- -> `driver/libmodbus_master::execute_with_retry`
### `driver/libmodbus_master::LibModbusMaster::write_registers`
- -> `driver/libmodbus_master::execute_with_retry`
### `driver/libserialport_port::LibSerialPort::close`
- -> `driver/libserialport_port::LibSerialPort::close_locked`
### `driver/libserialport_port::LibSerialPort::open`
- -> `driver/libserialport_port::LibSerialPort::check_sp_return`
- -> `driver/libserialport_port::LibSerialPort::close_locked`
### `driver/linux_can_socket::LinuxCanSocket::clear_filter`
- -> `driver/linux_can_socket::LinuxCanSocket::is_open`
### `driver/linux_can_socket::LinuxCanSocket::open`
- -> `driver/linux_can_socket::LinuxCanSocket::is_open`
### `driver/linux_can_socket::LinuxCanSocket::recover`
- -> `hal/i_can_bus::set_filters`
### `driver/linux_can_socket::LinuxCanSocket::set_filters`
- -> `driver/linux_can_socket::LinuxCanSocket::clear_filter`
- -> `driver/linux_can_socket::LinuxCanSocket::is_open`
### `main.cc/main::main`
- -> `main.cc/main::safety_monitor`
### `middleware/data_cache::DataCache::append_ack_record_locked`
- -> `middleware/data_cache::DataCache::append_journal_line_locked`
- -> `middleware/data_cache::build_ack_line`
### `middleware/data_cache::DataCache::append_journal_line_locked`
- -> `middleware/data_cache::parse_json_object_line`
### `middleware/data_cache::DataCache::append_push_record_locked`
- -> `middleware/data_cache::DataCache::append_journal_line_locked`
- -> `middleware/data_cache::build_push_line`
### `middleware/data_cache::DataCache::compact_to_snapshot_locked`
- -> `middleware/data_cache::build_snapshot_line`
### `middleware/data_cache::DataCache::confirm_sent`
- -> `middleware/data_cache::DataCache::append_ack_record_locked`
- -> `middleware/data_cache::DataCache::maybe_compact_locked`
- -> `middleware/data_cache::erase_record_by_id`
### `middleware/data_cache::DataCache::maybe_compact_locked`
- -> `middleware/data_cache::DataCache::compact_to_snapshot_locked`
### `middleware/data_cache::DataCache::open`
- -> `middleware/data_cache::DataCache::compact_to_snapshot_locked`
- -> `middleware/data_cache::erase_record_by_id`
- -> `middleware/data_cache::parse_snapshot_record`
### `middleware/data_cache::DataCache::push`
- -> `middleware/data_cache::DataCache::append_ack_record_locked`
- -> `middleware/data_cache::DataCache::append_push_record_locked`
- -> `middleware/data_cache::DataCache::maybe_compact_locked`
### `middleware/lorawan_transport::LoRaWANTransport::connect`
- -> `middleware/lorawan_transport::LoRaWANTransport::send_at`
### `middleware/lorawan_transport::LoRaWANTransport::publish`
- -> `middleware/lorawan_transport::LoRaWANTransport::send_at`
### `middleware/lorawan_transport::LoRaWANTransport::wait_for`
- -> `middleware/lorawan_transport::LoRaWANTransport::send_at`
### `middleware/lorawan_transport::LoRaWANTransport::~LoRaWANTransport`
- -> `middleware/lorawan_transport::LoRaWANTransport::disconnect`
### `middleware/mqtt_transport::MqttTransport::MqttTransport`
- -> `middleware/mqtt_transport::MqttTransport::delivery_loop`
### `middleware/mqtt_transport::MqttTransport::connect`
- -> `middleware/mqtt_transport::MqttTransport::is_connected`
- -> `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
### `middleware/mqtt_transport::MqttTransport::publish`
- -> `middleware/event_bus::publish`
- -> `middleware/mqtt_transport::MqttTransport::is_connected`
### `middleware/mqtt_transport::MqttTransport::subscribe`
- -> `middleware/event_bus::subscribe`
- -> `middleware/mqtt_transport::MqttTransport::is_connected`
- -> `middleware/mqtt_transport::granted_qos_to_string`
- -> `middleware/mqtt_transport::subscribe_granted`
### `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
- -> `middleware/event_bus::subscribe`
- -> `middleware/mqtt_transport::granted_qos_to_string`
- -> `middleware/mqtt_transport::subscribe_granted`
### `middleware/mqtt_transport::MqttTransport::~MqttTransport`
- -> `middleware/mqtt_transport::MqttTransport::disconnect`
### `middleware/mqtt_transport::connected`
- -> `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
### `middleware/mqtt_transport::message_arrived`
- -> `middleware/mqtt_transport::MqttTransport::enqueue_delivery`
### `middleware/network_manager::NetworkManager::connect`
- -> `middleware/network_manager::NetworkManager::uses_mqtt`
### `middleware/network_manager::NetworkManager::disconnect`
- -> `middleware/network_manager::NetworkManager::uses_lorawan`
- -> `middleware/network_manager::NetworkManager::uses_mqtt`
### `middleware/network_manager::NetworkManager::is_connected`
- -> `middleware/network_manager::NetworkManager::uses_mqtt`
### `middleware/network_manager::NetworkManager::publish`
- -> `middleware/event_bus::publish`
- -> `middleware/network_manager::NetworkManager::uses_lorawan`
- -> `middleware/network_manager::NetworkManager::uses_mqtt`
### `middleware/network_manager::NetworkManager::subscribe`
- -> `middleware/event_bus::subscribe`
- -> `middleware/network_manager::NetworkManager::uses_lorawan`
- -> `middleware/network_manager::NetworkManager::uses_mqtt`
### `middleware/safety_monitor::SafetyMonitor::monitor_loop`
- -> `middleware/event_bus::publish`
- -> `middleware/safety_monitor::SafetyMonitor::on_limit_trigger`
- -> `middleware/safety_monitor::now_ms`
### `middleware/safety_monitor::SafetyMonitor::on_limit_trigger`
- -> `middleware/safety_monitor::now_ms`
### `middleware/safety_monitor::SafetyMonitor::start`
- -> `middleware/safety_monitor::SafetyMonitor::on_limit_trigger`
- -> `middleware/safety_monitor::to_endpoint`
### `protocol/bms2_protocol::Bms2Protocol::decode_basic_info`
- -> `protocol/bms2_protocol::raw_to_current`
- -> `protocol/bms2_protocol::raw_to_soc`
- -> `protocol/bms2_protocol::raw_to_temp`
### `protocol/bms2_protocol::Bms2Protocol::decode_temperatures`
- -> `protocol/bms2_protocol::raw_to_temp`
### `protocol/bms_protocol::BmsProtocol::decode_basic_info`
- -> `protocol/bms_protocol::be16`
### `protocol/bms_protocol::BmsProtocol::decode_cell_voltages`
- -> `protocol/bms_protocol::be16`
### `protocol/bms_protocol::BmsProtocol::encode_mos_control`
- -> `protocol/bms_protocol::BmsProtocol::calc_checksum`
### `protocol/bms_protocol::BmsProtocol::encode_read_basic_info`
- -> `protocol/bms_protocol::make_read_frame`
### `protocol/bms_protocol::BmsProtocol::encode_read_cell_voltages`
- -> `protocol/bms_protocol::make_read_frame`
### `protocol/bms_protocol::BmsProtocol::encode_read_version`
- -> `protocol/bms_protocol::make_read_frame`
### `protocol/bms_protocol::BmsProtocol::push_byte`
- -> `protocol/bms_protocol::BmsProtocol::calc_checksum`
### `protocol/gpsd_json_parser::GpsdJsonParser::parse_line`
- -> `protocol/gpsd_json_parser::find_string`
- -> `protocol/gpsd_json_parser::parse_sky`
- -> `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::extract_scalar`
- -> `protocol/gpsd_json_parser::find_value_span`
- -> `protocol/gpsd_json_parser::trim_left`
### `protocol/gpsd_json_parser::find_array`
- -> `protocol/gpsd_json_parser::find_value_span`
- -> `protocol/gpsd_json_parser::trim_left`
### `protocol/gpsd_json_parser::find_bool`
- -> `protocol/gpsd_json_parser::find_value_span`
- -> `protocol/gpsd_json_parser::trim_left`
### `protocol/gpsd_json_parser::find_double`
- -> `protocol/gpsd_json_parser::extract_scalar`
### `protocol/gpsd_json_parser::find_float`
- -> `protocol/gpsd_json_parser::find_double`
### `protocol/gpsd_json_parser::find_int`
- -> `protocol/gpsd_json_parser::extract_scalar`
### `protocol/gpsd_json_parser::find_string`
- -> `protocol/gpsd_json_parser::find_value_span`
- -> `protocol/gpsd_json_parser::trim_left`
### `protocol/gpsd_json_parser::parse_sky`
- -> `protocol/gpsd_json_parser::clamp_sat_count`
- -> `protocol/gpsd_json_parser::find_array`
- -> `protocol/gpsd_json_parser::find_bool`
- -> `protocol/gpsd_json_parser::find_float`
- -> `protocol/gpsd_json_parser::find_int`
### `protocol/gpsd_json_parser::parse_tpv`
- -> `protocol/gpsd_json_parser::find_double`
- -> `protocol/gpsd_json_parser::find_float`
- -> `protocol/gpsd_json_parser::find_int`
- -> `protocol/gpsd_json_parser::find_string`
- -> `protocol/gpsd_json_parser::parse_utc_timestamp_ms`
### `protocol/imu_protocol::ImuProtocol::encode_calibrate_gyro`
- -> `protocol/imu_protocol::wit_write_reg`
### `protocol/imu_protocol::ImuProtocol::encode_save_config`
- -> `protocol/imu_protocol::wit_write_reg`
### `protocol/imu_protocol::ImuProtocol::encode_set_baudrate`
- -> `protocol/imu_protocol::wit_write_reg`
### `protocol/imu_protocol::ImuProtocol::encode_set_rate`
- -> `protocol/imu_protocol::wit_write_reg`
### `protocol/imu_protocol::ImuProtocol::parse_frame`
- -> `protocol/imu_protocol::to_int16`
### `protocol/imu_protocol::ImuProtocol::push_byte`
- -> `protocol/imu_protocol::ImuProtocol::parse_frame`
### `protocol/nmea_parser::NmeaParser::parse_gga`
- -> `protocol/nmea_parser::NmeaParser::parse_nmea_coord`
- -> `protocol/nmea_parser::NmeaParser::split`
### `protocol/nmea_parser::NmeaParser::parse_gsa`
- -> `protocol/nmea_parser::NmeaParser::split`
### `protocol/nmea_parser::NmeaParser::parse_gsv`
- -> `protocol/nmea_parser::NmeaParser::split`
### `protocol/nmea_parser::NmeaParser::parse_rmc`
- -> `protocol/nmea_parser::NmeaParser::parse_nmea_coord`
- -> `protocol/nmea_parser::NmeaParser::split`
### `protocol/nmea_parser::NmeaParser::parse_sentence`
- -> `protocol/nmea_parser::NmeaParser::parse_gga`
- -> `protocol/nmea_parser::NmeaParser::parse_gsa`
- -> `protocol/nmea_parser::NmeaParser::parse_gsv`
- -> `protocol/nmea_parser::NmeaParser::parse_rmc`
- -> `protocol/nmea_parser::NmeaParser::validate_checksum`
### `protocol/odrive_ascii_protocol::encode_clear_errors`
- -> `protocol/odrive_ascii_protocol::encode_line`
### `protocol/odrive_ascii_protocol::encode_feedback_request`
- -> `protocol/odrive_ascii_protocol::encode_line`
### `protocol/odrive_ascii_protocol::encode_read_property`
- -> `protocol/odrive_ascii_protocol::encode_line`
- -> `protocol/odrive_ascii_protocol::property_path`
### `protocol/odrive_ascii_protocol::encode_restart`
- -> `protocol/odrive_ascii_protocol::encode_line`
### `protocol/odrive_ascii_protocol::encode_set_velocity`
- -> `protocol/odrive_ascii_protocol::encode_line`
### `protocol/odrive_ascii_protocol::parse_feedback_response`
- -> `protocol/odrive_ascii_protocol::parse_float_token`
### `protocol/odrive_ascii_protocol::parse_float_response`
- -> `protocol/odrive_ascii_protocol::parse_float_token`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::decode_status`
- -> `protocol/walk_motor_can_codec::status_can_id`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_current`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::make_ctrl_frame`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_current`
- -> `protocol/walk_motor_can_codec::group_ctrl_id`
- -> `protocol/walk_motor_can_codec::pack_group`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_open_loop`
- -> `protocol/walk_motor_can_codec::group_ctrl_id`
- -> `protocol/walk_motor_can_codec::pack_group`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_position`
- -> `protocol/walk_motor_can_codec::group_ctrl_id`
- -> `protocol/walk_motor_can_codec::pack_group`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_speed`
- -> `protocol/walk_motor_can_codec::group_ctrl_id`
- -> `protocol/walk_motor_can_codec::pack_group`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_open_loop`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::make_ctrl_frame`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_position`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::make_ctrl_frame`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_feedback`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_feedback_batch`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_mode`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_mode_batch`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_speed`
- -> `protocol/walk_motor_can_codec::WalkMotorCanCodec::make_ctrl_frame`
### `service/cloud_service::CloudService::CloudService`
- -> `middleware/event_bus::subscribe`
- -> `service/cloud_service::CloudService::on_rpc_message`
- -> `service/cloud_service::CloudService::on_shared_attributes_message`
- -> `service/cloud_service::CloudService::on_shared_attributes_response_message`
### `service/cloud_service::CloudService::flush_cache`
- -> `middleware/event_bus::publish`
### `service/cloud_service::CloudService::on_rpc_message`
- -> `middleware/event_bus::publish`
- -> `service/cloud_service::build_rpc_response`
- -> `service/cloud_service::stringify_json_value`
### `service/cloud_service::CloudService::on_shared_attributes_message`
- -> `service/cloud_service::parse_small_json_object`
### `service/cloud_service::CloudService::on_shared_attributes_response_message`
- -> `service/cloud_service::parse_small_json_object`
### `service/cloud_service::CloudService::publish_attributes`
- -> `middleware/event_bus::publish`
### `service/cloud_service::CloudService::publish_telemetry`
- -> `middleware/event_bus::publish`
### `service/cloud_service::CloudService::request_shared_attributes_snapshot`
- -> `middleware/event_bus::publish`
### `service/cloud_service::CloudService::update`
- -> `service/cloud_service::CloudService::flush_cache`
### `service/command_tracker::CommandTracker::accept`
- -> `service/command_tracker::CommandTracker::make_snapshot`
### `service/command_tracker::CommandTracker::finish_active`
- -> `middleware/safety_monitor::now_ms`
### `service/command_tracker::CommandTracker::finish_failure`
- -> `service/command_tracker::CommandTracker::finish_active`
### `service/command_tracker::CommandTracker::finish_success`
- -> `service/command_tracker::CommandTracker::finish_active`
### `service/command_tracker::CommandTracker::make_snapshot`
- -> `middleware/safety_monitor::now_ms`
### `service/command_tracker::CommandTracker::reject`
- -> `service/command_tracker::CommandTracker::make_snapshot`
### `service/config_service::ConfigService::active_runtime_config`
- -> `service/config_service::ConfigService::parse_runtime_config`
### `service/config_service::ConfigService::apply_active_runtime_schedules`
- -> `service/config_service::ConfigService::active_runtime_config`
### `service/config_service::ConfigService::apply_runtime_patch`
- -> `domain/robot_domain::endpoint_config_string`
- -> `service/config_service::ConfigService::apply_active_runtime_schedules`
- -> `service/config_service::ConfigService::apply_schedule_json`
- -> `service/config_service::ConfigService::clear_pending`
- -> `service/config_service::ConfigService::clone_document`
- -> `service/config_service::ConfigService::load_pending`
- -> `service/config_service::ConfigService::parse_endpoint_string`
- -> `service/config_service::ConfigService::parse_runtime_config`
- -> `service/config_service::ConfigService::parse_schedule_entries`
- -> `service/config_service::ConfigService::replace_and_save`
- -> `service/config_service::ConfigService::save_pending`
- -> `service/config_service::ConfigService::snapshot`
- -> `service/config_service::ConfigService::validate_runtime_config`
- -> `service/config_service::ensure_object_member`
- -> `service/config_service::is_supported_runtime_patch_field`
- -> `service/config_service::is_valid_repeat_count`
- -> `service/config_service::merge_runtime_root`
- -> `service/config_service::set_double_member`
- -> `service/config_service::set_int_member`
- -> `service/config_service::set_string_member`
- -> `service/config_service::set_uint_member`
### `service/config_service::ConfigService::apply_schedule_json`
- -> `service/config_service::ensure_object_member`
### `service/config_service::ConfigService::backup_path`
- -> `service/config_service::ConfigService::derive_companion_path`
### `service/config_service::ConfigService::get_subtree`
- -> `service/config_service::ConfigService::split_path`
- -> `service/config_service::find_path`
### `service/config_service::ConfigService::has_pending_runtime_config`
- -> `service/config_service::ConfigService::load_pending`
### `service/config_service::ConfigService::load`
- -> `service/config_service::ConfigService::backup_path`
- -> `service/config_service::ConfigService::load_json_file_into`
### `service/config_service::ConfigService::load_fixed`
- -> `service/config_service::ConfigService::load_json_file_into`
### `service/config_service::ConfigService::load_json_file_into`
- -> `service/config_service::ConfigService::read_json_file`
### `service/config_service::ConfigService::load_pending`
- -> `service/config_service::ConfigService::read_json_file`
### `service/config_service::ConfigService::parse_endpoint_string`
- -> `domain/robot_domain::endpoint_config_string`
### `service/config_service::ConfigService::parse_runtime_config`
- -> `service/config_service::ConfigService::parse_endpoint_string`
- -> `service/config_service::ConfigService::parse_schedule_entries`
- -> `service/config_service::ConfigService::validate_runtime_config`
### `service/config_service::ConfigService::pending_runtime_config`
- -> `service/config_service::ConfigService::load_pending`
- -> `service/config_service::ConfigService::parse_runtime_config`
- -> `service/config_service::ConfigService::snapshot`
- -> `service/config_service::merge_runtime_root`
### `service/config_service::ConfigService::promote_pending_runtime_to_active`
- -> `service/config_service::ConfigService::clear_pending`
- -> `service/config_service::ConfigService::load_pending`
- -> `service/config_service::ConfigService::replace_and_save`
- -> `service/config_service::ConfigService::save_pending`
- -> `service/config_service::ConfigService::snapshot`
- -> `service/config_service::merge_runtime_root`
### `service/config_service::ConfigService::read_json_file`
- -> `service/config_service::parse_json_text`
### `service/config_service::ConfigService::replace_and_save`
- -> `service/config_service::ConfigService::backup_path`
- -> `service/config_service::ConfigService::clone_document`
- -> `service/config_service::ConfigService::save_locked`
- -> `service/config_service::ConfigService::write_json_file`
### `service/config_service::ConfigService::runtime_config_to_pending_root`
- -> `domain/robot_domain::endpoint_config_string`
- -> `service/config_service::ensure_object_member`
- -> `service/config_service::set_double_member`
- -> `service/config_service::set_int_member`
- -> `service/config_service::set_string_member`
- -> `service/config_service::set_uint_member`
### `service/config_service::ConfigService::runtime_config_version`
- -> `domain/robot_domain::endpoint_config_string`
### `service/config_service::ConfigService::save`
- -> `service/config_service::ConfigService::save_locked`
### `service/config_service::ConfigService::save_locked`
- -> `service/config_service::ConfigService::write_json_file`
### `service/config_service::ConfigService::save_pending`
- -> `service/config_service::ConfigService::write_json_file`
### `service/config_service::ConfigService::save_pending_runtime_config`
- -> `service/config_service::ConfigService::runtime_config_to_pending_root`
- -> `service/config_service::ConfigService::save_pending`
- -> `service/config_service::ConfigService::validate_runtime_config`
### `service/config_service::ConfigService::snapshot`
- -> `service/config_service::ConfigService::clone_document`
### `service/config_service::get_optional_from_document`
- -> `service/config_service::ConfigService::split_path`
- -> `service/config_service::constexpr`
### `service/config_service::merge_runtime_root`
- -> `service/config_service::merge_object_members`
### `service/config_service::set`
- -> `service/config_service::ConfigService::split_path`
- -> `service/config_service::constexpr`
### `service/fault_service::FaultService::report`
- -> `middleware/event_bus::publish`
### `service/heading_corrector::HeadingCorrector::HeadingCorrector`
- -> `service/heading_corrector::HeadingCorrector::set_params`
- -> `service/heading_corrector::HeadingCorrector::start_io_thread_if_needed`
### `service/heading_corrector::HeadingCorrector::apply_correction`
- -> `device/walk_motor_group::clamp`
### `service/heading_corrector::HeadingCorrector::clamp_alpha`
- -> `device/walk_motor_group::clamp`
### `service/heading_corrector::HeadingCorrector::compute`
- -> `device/walk_motor_group::clamp`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::apply_correction`
- -> `service/heading_corrector::HeadingCorrector::latest_result_age_ms_locked`
- -> `service/heading_corrector::HeadingCorrector::low_pass`
- -> `service/heading_corrector::HeadingCorrector::reset_control_state_locked`
- -> `service/heading_corrector::apply_base_abs_rpm`
- -> `service/heading_corrector::normalize_yaw_to_control_error`
### `service/heading_corrector::HeadingCorrector::consume_socket_locked`
- -> `service/heading_corrector::HeadingCorrector::close_socket_locked`
- -> `service/heading_corrector::HeadingCorrector::ingest_json_line_locked`
### `service/heading_corrector::HeadingCorrector::debug_state`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::latest_result_age_ms_locked`
### `service/heading_corrector::HeadingCorrector::enable`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::reset_control_state_locked`
### `service/heading_corrector::HeadingCorrector::io_loop`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::connect_locked`
- -> `service/heading_corrector::HeadingCorrector::consume_socket_locked`
### `service/heading_corrector::HeadingCorrector::low_pass`
- -> `service/heading_corrector::HeadingCorrector::clamp_alpha`
### `service/heading_corrector::HeadingCorrector::reset`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::reset_control_state_locked`
### `service/heading_corrector::HeadingCorrector::set_params`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::close_socket_locked`
### `service/heading_corrector::HeadingCorrector::start_io_thread_if_needed`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::io_loop`
### `service/heading_corrector::HeadingCorrector::stop_io_thread`
- -> `hal/pi_mutex::lock`
- -> `service/heading_corrector::HeadingCorrector::close_socket_locked`
### `service/heading_corrector::HeadingCorrector::~HeadingCorrector`
- -> `service/heading_corrector::HeadingCorrector::stop_io_thread`
### `service/health_service::HealthService::build_payload`
- -> `protocol/nmea_parser::get_data`
### `service/health_service::HealthService::update`
- -> `service/health_service::HealthService::build_payload`
### `service/health_service::append_quoted`
- -> `service/health_service::append_char`
- -> `service/health_service::append_raw`
### `service/motion_service::MotionService::emergency_stop`
- -> `service/motion_service::MotionService::deactivate_walk_command`
### `service/motion_service::MotionService::handle_override_clear`
- -> `service/motion_service::MotionService::apply_speed_if_command_current`
- -> `service/motion_service::MotionService::observe_override_clear`
### `service/motion_service::MotionService::primary_dock`
- -> `service/motion_service::MotionService::snapshot_state`
### `service/motion_service::MotionService::start_cleaning_to`
- -> `service/motion_service::MotionService::activate_walk_command`
- -> `service/motion_service::MotionService::deactivate_walk_command`
- -> `service/motion_service::MotionService::enable_speed_mode`
- -> `service/motion_service::MotionService::snapshot_state`
- -> `service/motion_service::MotionService::sync_heading_pid_enabled`
- -> `service/motion_service::MotionService::sync_runtime_config`
- -> `service/motion_service::MotionService::target_direction_sign`
### `service/motion_service::MotionService::start_segment`
- -> `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::stop_cleaning`
- -> `service/motion_service::MotionService::deactivate_walk_command`
### `service/motion_service::MotionService::sync_heading_pid_enabled`
- -> `service/heading_corrector::is_enabled`
- -> `service/motion_service::MotionService::snapshot_state`
### `service/motion_service::MotionService::update`
- -> `service/motion_service::MotionService::handle_override_clear`
- -> `service/motion_service::MotionService::snapshot_state`
- -> `service/motion_service::MotionService::update_heading_correction`
### `service/motion_service::MotionService::update_heading_correction`
- -> `service/motion_service::MotionService::apply_speed_if_command_current`
- -> `service/motion_service::to_corrector_command`
- -> `service/motion_service::to_group_command`
### `service/nav_service::NavService::update`
- -> `service/nav_service::clamp01`
- -> `service/nav_service::project_gps_to_track`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::accept_rpc_command`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::rpc_reply`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_business_telemetry`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_business_payload`
- -> `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_business_telemetry`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_startup_attributes`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_attributes_payload`
- -> `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_startup_attributes`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::register_command_rpc`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::accept_rpc_command`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::reject_rpc_command`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::register_rpc_handlers`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::register_command_rpc`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::reject_rpc_command`
- -> `service/thingsboard_control_plane::ThingsBoardControlPlane::rpc_reply`
### `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_business_telemetry`
- -> `service/thingsboard_control_plane::overflow`
### `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_startup_attributes`
- -> `service/thingsboard_control_plane::overflow`
### `service/thingsboard_control_plane::write_runtime_config`
- -> `domain/robot_domain::endpoint_config_string`
- -> `service/thingsboard_control_plane::write_schedule_entries`
### `service/uds_gyro_yaw_fusion::UdsGyroYawFusion::set_params`
- -> `service/uds_gyro_yaw_fusion::non_negative`

## 7. 函数/方法级反向调用索引（生产源码，降噪版）

### `app/fault_policy::is_p0_code`
- <- `app/fault_policy::FaultPolicy::decide`
### `app/robot_controller::RobotController::build_start_mission_locked`
- <- `app/robot_controller::RobotController::start_command_locked`
### `app/robot_controller::RobotController::complete_self_check_locked`
- <- `app/robot_controller::RobotController::complete_self_check`
- <- `app/robot_controller::RobotController::complete_self_check_for_test`
### `app/robot_controller::RobotController::handle_fault_locked`
- <- `app/robot_controller::RobotController::handle_fault_for_test`
- <- `app/robot_controller::RobotController::handle_limit_unstable_locked`
- <- `app/robot_controller::RobotController::handle_recovery_finished_locked`
- <- `app/robot_controller::RobotController::handle_watchdog_timeout_locked`
- <- `app/robot_controller::RobotController::post_fault`
### `app/robot_controller::RobotController::handle_limit_settled_locked`
- <- `app/robot_controller::RobotController::handle_limit_settled_for_test`
- <- `app/robot_controller::RobotController::post_limit_settled`
### `app/robot_controller::RobotController::handle_limit_unstable_locked`
- <- `app/robot_controller::RobotController::post_limit_unstable`
### `app/robot_controller::RobotController::handle_recovery_finished_locked`
- <- `app/robot_controller::RobotController::post_recovery_finished`
### `app/robot_controller::RobotController::handle_watchdog_timeout_locked`
- <- `app/robot_controller::RobotController::post_watchdog_timeout`
### `app/robot_controller::RobotController::loop`
- <- `app/robot_controller::RobotController::start`
### `app/robot_controller::RobotController::mission_active`
- <- `app/robot_controller::RobotController::stop_locked`
### `app/robot_controller::RobotController::post`
- <- `app/robot_controller::RobotController::complete_self_check`
- <- `app/robot_controller::RobotController::post_fault`
- <- `app/robot_controller::RobotController::post_for_test`
- <- `app/robot_controller::RobotController::post_limit_settled`
- <- `app/robot_controller::RobotController::post_limit_unstable`
- <- `app/robot_controller::RobotController::post_recovery_finished`
- <- `app/robot_controller::RobotController::post_schedule_window_hit`
- <- `app/robot_controller::RobotController::post_tick`
- <- `app/robot_controller::RobotController::post_watchdog_timeout`
- <- `app/robot_controller::RobotController::submit_command`
### `app/robot_controller::RobotController::start_command_locked`
- <- `app/robot_controller::RobotController::submit_command_locked`
### `app/robot_controller::RobotController::start_current_segment_locked`
- <- `app/robot_controller::RobotController::complete_self_check_locked`
- <- `app/robot_controller::RobotController::handle_limit_settled_locked`
- <- `app/robot_controller::RobotController::handle_recovery_finished_locked`
### `app/robot_controller::RobotController::state_name`
- <- `app/robot_controller::RobotController::snapshot`
### `app/robot_controller::RobotController::stop_locked`
- <- `app/robot_controller::RobotController::submit_command_locked`
### `app/robot_controller::RobotController::submit_command_locked`
- <- `app/robot_controller::RobotController::post_schedule_window_hit`
- <- `app/robot_controller::RobotController::submit_command`
### `app/robot_controller::RobotController::validate_start_command_locked`
- <- `app/robot_controller::RobotController::start_command_locked`
### `app/watchdog_mgr::WatchdogMgr::feed_hw_watchdog`
- <- `app/watchdog_mgr::WatchdogMgr::monitor_loop`
### `device/attitude_limit_switch::AttitudeLimitSwitch::is_triggered`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::read_status`
### `device/attitude_limit_switch::AttitudeLimitSwitch::on_edge`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::start_monitoring`
### `device/attitude_limit_switch::AttitudeLimitSwitch::read_current_level`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::read_status`
### `device/attitude_limit_switch::AttitudeLimitSwitch::stop_monitoring`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::close`
### `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::update`
### `device/bms::BMS::read_cell_voltages_uart`
- <- `device/bms::BMS::update`
### `device/bms::BMS::transact`
- <- `device/bms::BMS::mos_control`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::read_cell_voltages_uart`
### `device/brush_motor::BrushMotor::mark_comm_error_locked`
- <- `device/brush_motor::BrushMotor::update`
### `device/brush_motor::BrushMotor::read_line_locked`
- <- `device/brush_motor::BrushMotor::request_ascii_locked`
### `device/brush_motor::BrushMotor::request_ascii_locked`
- <- `device/brush_motor::BrushMotor::update`
### `device/brush_motor::BrushMotor::update_running_locked`
- <- `device/brush_motor::BrushMotor::close`
- <- `device/brush_motor::BrushMotor::restart`
- <- `device/brush_motor::BrushMotor::set_rpm`
- <- `device/brush_motor::BrushMotor::stop`
- <- `device/brush_motor::BrushMotor::update`
### `device/brush_motor::BrushMotor::write_ascii_locked`
- <- `device/brush_motor::BrushMotor::clear_fault`
- <- `device/brush_motor::BrushMotor::close`
- <- `device/brush_motor::BrushMotor::request_ascii_locked`
- <- `device/brush_motor::BrushMotor::restart`
- <- `device/brush_motor::BrushMotor::set_rpm`
- <- `device/brush_motor::BrushMotor::stop`
### `device/brush_motor::rpm_to_turns_per_sec`
- <- `device/brush_motor::BrushMotor::set_rpm`
### `device/brush_motor::turns_per_sec_to_rpm`
- <- `device/brush_motor::BrushMotor::update`
### `device/brush_motor::uart_error_to_device_error`
- <- `device/brush_motor::BrushMotor::read_line_locked`
- <- `device/brush_motor::BrushMotor::write_ascii_locked`
### `device/gps_device::GpsDevice::on_source_message`
- <- `device/gps_device::GpsDevice::create_gpsd`
### `device/gps_device::GpsDevice::on_source_message_count`
- <- `device/gps_device::GpsDevice::create_gpsd`
### `device/gps_device::GpsDevice::on_source_parse_error`
- <- `device/gps_device::GpsDevice::create_gpsd`
### `device/gpsd_gps_source::GpsdGpsSource::connect_socket`
- <- `device/gpsd_gps_source::GpsdGpsSource::open`
- <- `device/gpsd_gps_source::GpsdGpsSource::read_loop`
### `device/gpsd_gps_source::GpsdGpsSource::handle_json_line`
- <- `device/gpsd_gps_source::GpsdGpsSource::read_loop`
### `device/gpsd_gps_source::GpsdGpsSource::send_watch`
- <- `device/gpsd_gps_source::GpsdGpsSource::open`
- <- `device/gpsd_gps_source::GpsdGpsSource::read_loop`
### `device/gpsd_gps_source::close_socket`
- <- `device/gpsd_gps_source::GpsdGpsSource::close`
- <- `device/gpsd_gps_source::GpsdGpsSource::connect_socket`
- <- `device/gpsd_gps_source::GpsdGpsSource::open`
- <- `device/gpsd_gps_source::GpsdGpsSource::read_loop`
### `device/imu_device::ImuDevice::send_command`
- <- `device/imu_device::ImuDevice::send_write_cmd`
### `device/imu_device::ImuDevice::send_write_cmd`
- <- `device/imu_device::ImuDevice::calibrate_gyro`
- <- `device/imu_device::ImuDevice::reset`
- <- `device/imu_device::ImuDevice::save_config`
- <- `device/imu_device::ImuDevice::set_output_rate`
### `device/limit_switch::LimitSwitch::on_edge`
- <- `device/limit_switch::LimitSwitch::start_monitoring`
### `device/limit_switch::LimitSwitch::stop_monitoring`
- <- `device/limit_switch::LimitSwitch::close`
### `device/serial_gps_source::write_command`
- <- `device/serial_gps_source::SerialGpsSource::cold_restart`
- <- `device/serial_gps_source::SerialGpsSource::hot_restart`
- <- `device/serial_gps_source::SerialGpsSource::set_output_rate`
### `device/walk_motor_group::WalkMotorGroup::send_ctrl`
- <- `device/walk_motor_group::WalkMotorGroup::query_firmware`
- <- `device/walk_motor_group::WalkMotorGroup::set_feedback_mode_all`
- <- `device/walk_motor_group::WalkMotorGroup::set_mode_all`
- <- `device/walk_motor_group::WalkMotorGroup::set_modes`
- <- `device/walk_motor_group::WalkMotorGroup::set_terminations`
### `device/walk_motor_group::WalkMotorGroup::set_mode_all`
- <- `device/walk_motor_group::WalkMotorGroup::disable_all`
- <- `device/walk_motor_group::WalkMotorGroup::enable_all`
### `device/walk_motor_group::clamp`
- <- `device/walk_motor_group::WalkMotorGroup::set_positions`
- <- `service/heading_corrector::HeadingCorrector::apply_correction`
- <- `service/heading_corrector::HeadingCorrector::clamp_alpha`
- <- `service/heading_corrector::HeadingCorrector::compute`
### `device/walk_motor_group::clamp_rpm`
- <- `device/walk_motor_group::WalkMotorGroup::emergency_override`
- <- `device/walk_motor_group::WalkMotorGroup::set_speeds`
### `device/walk_motor_group::make_group_termination_frame`
- <- `device/walk_motor_group::WalkMotorGroup::open`
### `device/walk_motor_group::motor_id_in_group`
- <- `device/walk_motor_group::WalkMotorGroup::open`
### `domain/robot_domain::current_segment`
- <- `app/robot_controller::RobotController::handle_limit_settled_locked`
- <- `app/robot_controller::RobotController::start_current_segment_locked`
### `domain/robot_domain::endpoint_config_string`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::parse_endpoint_string`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
- <- `service/config_service::ConfigService::runtime_config_version`
- <- `service/thingsboard_control_plane::write_runtime_config`
### `domain/robot_domain::endpoint_from_position`
- <- `domain/robot_domain::build_configured_mission_context`
- <- `domain/robot_domain::can_start_configured_mission`
- <- `domain/robot_domain::is_at_target`
### `domain/robot_domain::opposite_endpoint`
- <- `domain/robot_domain::build_configured_mission_context`
- <- `domain/robot_domain::build_directional_clean_context`
### `driver/libgpiod_pin::LibGpiodPin::request_line_as_input`
- <- `driver/libgpiod_pin::LibGpiodPin::open`
- <- `driver/libgpiod_pin::LibGpiodPin::start_monitoring`
- <- `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
### `driver/libgpiod_pin::LibGpiodPin::setup_thread_rt_`
- <- `driver/libgpiod_pin::LibGpiodPin::monitor_loop`
- <- `driver/libgpiod_pin::LibGpiodPin::poll_loop`
### `driver/libgpiod_pin::LibGpiodPin::stop_monitoring_locked`
- <- `driver/libgpiod_pin::LibGpiodPin::close`
- <- `driver/libgpiod_pin::LibGpiodPin::open`
- <- `driver/libgpiod_pin::LibGpiodPin::stop_monitoring`
### `driver/libmodbus_master::LibModbusMaster::is_open`
- <- `driver/libmodbus_master::LibModbusMaster::open`
### `driver/libmodbus_master::LibModbusMaster::map_errno_to_result`
- <- `driver/libmodbus_master::LibModbusMaster::execute_with_retry`
- <- `driver/libmodbus_master::LibModbusMaster::open`
### `driver/libmodbus_master::execute_with_retry`
- <- `driver/libmodbus_master::LibModbusMaster::read_input_registers`
- <- `driver/libmodbus_master::LibModbusMaster::read_registers`
- <- `driver/libmodbus_master::LibModbusMaster::write_register`
- <- `driver/libmodbus_master::LibModbusMaster::write_registers`
### `driver/libserialport_port::LibSerialPort::check_sp_return`
- <- `driver/libserialport_port::LibSerialPort::open`
### `driver/libserialport_port::LibSerialPort::close_locked`
- <- `driver/libserialport_port::LibSerialPort::close`
- <- `driver/libserialport_port::LibSerialPort::open`
### `driver/linux_can_socket::LinuxCanSocket::clear_filter`
- <- `driver/linux_can_socket::LinuxCanSocket::set_filters`
### `driver/linux_can_socket::LinuxCanSocket::is_open`
- <- `driver/linux_can_socket::LinuxCanSocket::clear_filter`
- <- `driver/linux_can_socket::LinuxCanSocket::open`
- <- `driver/linux_can_socket::LinuxCanSocket::set_filters`
### `driver/linux_can_socket::get_last_error`
- <- `device/brush_motor::BrushMotor::read_line_locked`
- <- `device/brush_motor::BrushMotor::write_ascii_locked`
### `driver/linux_can_socket::is_bus_off`
- <- `device/walk_motor_group::WalkMotorGroup::recv_loop`
### `hal/i_can_bus::set_filters`
- <- `device/walk_motor_group::WalkMotorGroup::open`
- <- `driver/linux_can_socket::LinuxCanSocket::recover`
### `hal/pi_mutex::lock`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::on_edge`
- <- `device/attitude_limit_switch::AttitudeLimitSwitch::set_trigger_callback`
- <- `device/bms::BMS::transact`
- <- `device/limit_switch::LimitSwitch::on_edge`
- <- `device/limit_switch::LimitSwitch::set_trigger_callback`
- <- `driver/libgpiod_pin::LibGpiodPin::close`
- <- `driver/libgpiod_pin::LibGpiodPin::is_open`
- <- `driver/libgpiod_pin::LibGpiodPin::monitor_loop`
- <- `driver/libgpiod_pin::LibGpiodPin::open`
- <- `driver/libgpiod_pin::LibGpiodPin::read_value`
- <- `driver/libgpiod_pin::LibGpiodPin::set_edge_callback`
- <- `driver/libgpiod_pin::LibGpiodPin::start_monitoring`
- <- `driver/libgpiod_pin::LibGpiodPin::stop_monitoring`
- <- `driver/libgpiod_pin::LibGpiodPin::write_value`
- <- `driver/libmodbus_master::LibModbusMaster::close`
- <- `driver/libmodbus_master::LibModbusMaster::execute_with_retry`
- <- `driver/libmodbus_master::LibModbusMaster::open`
- <- `driver/libmodbus_master::LibModbusMaster::set_timeout_ms`
- <- `service/heading_corrector::HeadingCorrector::compute`
- <- `service/heading_corrector::HeadingCorrector::debug_state`
- <- `service/heading_corrector::HeadingCorrector::enable`
- <- `service/heading_corrector::HeadingCorrector::io_loop`
- <- `service/heading_corrector::HeadingCorrector::reset`
- <- `service/heading_corrector::HeadingCorrector::set_params`
- <- `service/heading_corrector::HeadingCorrector::start_io_thread_if_needed`
- <- `service/heading_corrector::HeadingCorrector::stop_io_thread`
### `main.cc/main::safety_monitor`
- <- `main.cc/main::main`
### `middleware/data_cache::DataCache::append_ack_record_locked`
- <- `middleware/data_cache::DataCache::confirm_sent`
- <- `middleware/data_cache::DataCache::push`
### `middleware/data_cache::DataCache::append_journal_line_locked`
- <- `middleware/data_cache::DataCache::append_ack_record_locked`
- <- `middleware/data_cache::DataCache::append_push_record_locked`
### `middleware/data_cache::DataCache::append_push_record_locked`
- <- `middleware/data_cache::DataCache::push`
### `middleware/data_cache::DataCache::compact_to_snapshot_locked`
- <- `middleware/data_cache::DataCache::maybe_compact_locked`
- <- `middleware/data_cache::DataCache::open`
### `middleware/data_cache::DataCache::maybe_compact_locked`
- <- `middleware/data_cache::DataCache::confirm_sent`
- <- `middleware/data_cache::DataCache::push`
### `middleware/data_cache::build_ack_line`
- <- `middleware/data_cache::DataCache::append_ack_record_locked`
### `middleware/data_cache::build_push_line`
- <- `middleware/data_cache::DataCache::append_push_record_locked`
### `middleware/data_cache::build_snapshot_line`
- <- `middleware/data_cache::DataCache::compact_to_snapshot_locked`
### `middleware/data_cache::erase_record_by_id`
- <- `middleware/data_cache::DataCache::confirm_sent`
- <- `middleware/data_cache::DataCache::open`
### `middleware/data_cache::parse_json_object_line`
- <- `middleware/data_cache::DataCache::append_journal_line_locked`
### `middleware/data_cache::parse_snapshot_record`
- <- `middleware/data_cache::DataCache::open`
### `middleware/event_bus::publish`
- <- `middleware/mqtt_transport::MqttTransport::publish`
- <- `middleware/network_manager::NetworkManager::publish`
- <- `middleware/safety_monitor::SafetyMonitor::monitor_loop`
- <- `service/cloud_service::CloudService::flush_cache`
- <- `service/cloud_service::CloudService::on_rpc_message`
- <- `service/cloud_service::CloudService::publish_attributes`
- <- `service/cloud_service::CloudService::publish_telemetry`
- <- `service/cloud_service::CloudService::request_shared_attributes_snapshot`
- <- `service/fault_service::FaultService::report`
### `middleware/event_bus::subscribe`
- <- `middleware/mqtt_transport::MqttTransport::subscribe`
- <- `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
- <- `middleware/network_manager::NetworkManager::subscribe`
- <- `service/cloud_service::CloudService::CloudService`
### `middleware/lorawan_transport::LoRaWANTransport::disconnect`
- <- `middleware/lorawan_transport::LoRaWANTransport::~LoRaWANTransport`
### `middleware/lorawan_transport::LoRaWANTransport::send_at`
- <- `middleware/lorawan_transport::LoRaWANTransport::connect`
- <- `middleware/lorawan_transport::LoRaWANTransport::publish`
- <- `middleware/lorawan_transport::LoRaWANTransport::wait_for`
### `middleware/mqtt_transport::MqttTransport::delivery_loop`
- <- `middleware/mqtt_transport::MqttTransport::MqttTransport`
### `middleware/mqtt_transport::MqttTransport::disconnect`
- <- `middleware/mqtt_transport::MqttTransport::~MqttTransport`
### `middleware/mqtt_transport::MqttTransport::enqueue_delivery`
- <- `middleware/mqtt_transport::message_arrived`
### `middleware/mqtt_transport::MqttTransport::is_connected`
- <- `middleware/mqtt_transport::MqttTransport::connect`
- <- `middleware/mqtt_transport::MqttTransport::publish`
- <- `middleware/mqtt_transport::MqttTransport::subscribe`
### `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
- <- `middleware/mqtt_transport::MqttTransport::connect`
- <- `middleware/mqtt_transport::connected`
### `middleware/mqtt_transport::granted_qos_to_string`
- <- `middleware/mqtt_transport::MqttTransport::subscribe`
- <- `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
### `middleware/mqtt_transport::subscribe_granted`
- <- `middleware/mqtt_transport::MqttTransport::subscribe`
- <- `middleware/mqtt_transport::MqttTransport::subscribe_all_registered_topics`
### `middleware/network_manager::NetworkManager::uses_lorawan`
- <- `middleware/network_manager::NetworkManager::disconnect`
- <- `middleware/network_manager::NetworkManager::publish`
- <- `middleware/network_manager::NetworkManager::subscribe`
### `middleware/network_manager::NetworkManager::uses_mqtt`
- <- `middleware/network_manager::NetworkManager::connect`
- <- `middleware/network_manager::NetworkManager::disconnect`
- <- `middleware/network_manager::NetworkManager::is_connected`
- <- `middleware/network_manager::NetworkManager::publish`
- <- `middleware/network_manager::NetworkManager::subscribe`
### `middleware/safety_monitor::SafetyMonitor::on_limit_trigger`
- <- `middleware/safety_monitor::SafetyMonitor::monitor_loop`
- <- `middleware/safety_monitor::SafetyMonitor::start`
### `middleware/safety_monitor::now_ms`
- <- `middleware/safety_monitor::SafetyMonitor::monitor_loop`
- <- `middleware/safety_monitor::SafetyMonitor::on_limit_trigger`
- <- `service/command_tracker::CommandTracker::finish_active`
- <- `service/command_tracker::CommandTracker::make_snapshot`
### `middleware/safety_monitor::to_endpoint`
- <- `middleware/safety_monitor::SafetyMonitor::start`
### `protocol/bms2_protocol::raw_to_current`
- <- `protocol/bms2_protocol::Bms2Protocol::decode_basic_info`
### `protocol/bms2_protocol::raw_to_soc`
- <- `protocol/bms2_protocol::Bms2Protocol::decode_basic_info`
### `protocol/bms2_protocol::raw_to_temp`
- <- `protocol/bms2_protocol::Bms2Protocol::decode_basic_info`
- <- `protocol/bms2_protocol::Bms2Protocol::decode_temperatures`
### `protocol/bms_protocol::BmsProtocol::calc_checksum`
- <- `protocol/bms_protocol::BmsProtocol::encode_mos_control`
- <- `protocol/bms_protocol::BmsProtocol::push_byte`
### `protocol/bms_protocol::be16`
- <- `protocol/bms_protocol::BmsProtocol::decode_basic_info`
- <- `protocol/bms_protocol::BmsProtocol::decode_cell_voltages`
### `protocol/bms_protocol::get_cmd`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::read_cell_voltages_uart`
### `protocol/bms_protocol::get_data_len`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::read_cell_voltages_uart`
### `protocol/bms_protocol::is_error`
- <- `device/bms::BMS::mos_control`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::read_cell_voltages_uart`
### `protocol/bms_protocol::make_read_frame`
- <- `protocol/bms_protocol::BmsProtocol::encode_read_basic_info`
- <- `protocol/bms_protocol::BmsProtocol::encode_read_cell_voltages`
- <- `protocol/bms_protocol::BmsProtocol::encode_read_version`
### `protocol/distance_sensor_protocol::DistanceSensorProtocol::decode_register`
- <- `device/distance_sensor::DistanceSensor::update`
### `protocol/distance_sensor_protocol::DistanceSensorProtocol::voltage_to_ma`
- <- `device/distance_sensor::DistanceSensor::update`
### `protocol/gpsd_json_parser::clamp_sat_count`
- <- `protocol/gpsd_json_parser::parse_sky`
### `protocol/gpsd_json_parser::extract_scalar`
- <- `protocol/gpsd_json_parser::find_double`
- <- `protocol/gpsd_json_parser::find_int`
### `protocol/gpsd_json_parser::find_array`
- <- `protocol/gpsd_json_parser::parse_sky`
### `protocol/gpsd_json_parser::find_bool`
- <- `protocol/gpsd_json_parser::parse_sky`
### `protocol/gpsd_json_parser::find_double`
- <- `protocol/gpsd_json_parser::find_float`
- <- `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::find_float`
- <- `protocol/gpsd_json_parser::parse_sky`
- <- `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::find_int`
- <- `protocol/gpsd_json_parser::parse_sky`
- <- `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::find_string`
- <- `protocol/gpsd_json_parser::GpsdJsonParser::parse_line`
- <- `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::find_value_span`
- <- `protocol/gpsd_json_parser::extract_scalar`
- <- `protocol/gpsd_json_parser::find_array`
- <- `protocol/gpsd_json_parser::find_bool`
- <- `protocol/gpsd_json_parser::find_string`
### `protocol/gpsd_json_parser::parse_sky`
- <- `protocol/gpsd_json_parser::GpsdJsonParser::parse_line`
### `protocol/gpsd_json_parser::parse_tpv`
- <- `protocol/gpsd_json_parser::GpsdJsonParser::parse_line`
### `protocol/gpsd_json_parser::parse_utc_timestamp_ms`
- <- `protocol/gpsd_json_parser::parse_tpv`
### `protocol/gpsd_json_parser::trim_left`
- <- `protocol/gpsd_json_parser::extract_scalar`
- <- `protocol/gpsd_json_parser::find_array`
- <- `protocol/gpsd_json_parser::find_bool`
- <- `protocol/gpsd_json_parser::find_string`
### `protocol/imu_protocol::ImuProtocol::parse_frame`
- <- `protocol/imu_protocol::ImuProtocol::push_byte`
### `protocol/imu_protocol::frame_complete`
- <- `device/bms::BMS::transact`
- <- `device/imu_device::ImuDevice::read_loop`
### `protocol/imu_protocol::to_int16`
- <- `protocol/imu_protocol::ImuProtocol::parse_frame`
### `protocol/imu_protocol::wit_write_reg`
- <- `protocol/imu_protocol::ImuProtocol::encode_calibrate_gyro`
- <- `protocol/imu_protocol::ImuProtocol::encode_save_config`
- <- `protocol/imu_protocol::ImuProtocol::encode_set_baudrate`
- <- `protocol/imu_protocol::ImuProtocol::encode_set_rate`
### `protocol/nmea_parser::NmeaParser::parse_gga`
- <- `protocol/nmea_parser::NmeaParser::parse_sentence`
### `protocol/nmea_parser::NmeaParser::parse_gsa`
- <- `protocol/nmea_parser::NmeaParser::parse_sentence`
### `protocol/nmea_parser::NmeaParser::parse_gsv`
- <- `protocol/nmea_parser::NmeaParser::parse_sentence`
### `protocol/nmea_parser::NmeaParser::parse_nmea_coord`
- <- `protocol/nmea_parser::NmeaParser::parse_gga`
- <- `protocol/nmea_parser::NmeaParser::parse_rmc`
### `protocol/nmea_parser::NmeaParser::parse_rmc`
- <- `protocol/nmea_parser::NmeaParser::parse_sentence`
### `protocol/nmea_parser::NmeaParser::split`
- <- `protocol/nmea_parser::NmeaParser::parse_gga`
- <- `protocol/nmea_parser::NmeaParser::parse_gsa`
- <- `protocol/nmea_parser::NmeaParser::parse_gsv`
- <- `protocol/nmea_parser::NmeaParser::parse_rmc`
### `protocol/nmea_parser::NmeaParser::validate_checksum`
- <- `protocol/nmea_parser::NmeaParser::parse_sentence`
### `protocol/nmea_parser::get_data`
- <- `device/bms::BMS::open`
- <- `device/bms::BMS::read_basic_info_uart`
- <- `device/bms::BMS::read_cell_voltages_uart`
- <- `device/serial_gps_source::SerialGpsSource::read_loop`
- <- `service/health_service::HealthService::build_payload`
### `protocol/odrive_ascii_protocol::encode_line`
- <- `protocol/odrive_ascii_protocol::encode_clear_errors`
- <- `protocol/odrive_ascii_protocol::encode_feedback_request`
- <- `protocol/odrive_ascii_protocol::encode_read_property`
- <- `protocol/odrive_ascii_protocol::encode_restart`
- <- `protocol/odrive_ascii_protocol::encode_set_velocity`
### `protocol/odrive_ascii_protocol::parse_float_token`
- <- `protocol/odrive_ascii_protocol::parse_feedback_response`
- <- `protocol/odrive_ascii_protocol::parse_float_response`
### `protocol/odrive_ascii_protocol::property_path`
- <- `protocol/odrive_ascii_protocol::encode_read_property`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_feedback_batch`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_feedback`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_mode_batch`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_set_mode`
### `protocol/walk_motor_can_codec::WalkMotorCanCodec::make_ctrl_frame`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_current`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_open_loop`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_position`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_speed`
### `protocol/walk_motor_can_codec::group_ctrl_id`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_current`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_open_loop`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_position`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_speed`
### `protocol/walk_motor_can_codec::pack_group`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_current`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_open_loop`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_position`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::encode_group_speed`
### `protocol/walk_motor_can_codec::status_can_id`
- <- `device/walk_motor_group::WalkMotorGroup::open`
- <- `protocol/walk_motor_can_codec::WalkMotorCanCodec::decode_status`
### `service/cloud_service::CloudService::flush_cache`
- <- `service/cloud_service::CloudService::update`
### `service/cloud_service::CloudService::on_rpc_message`
- <- `service/cloud_service::CloudService::CloudService`
### `service/cloud_service::CloudService::on_shared_attributes_message`
- <- `service/cloud_service::CloudService::CloudService`
### `service/cloud_service::CloudService::on_shared_attributes_response_message`
- <- `service/cloud_service::CloudService::CloudService`
### `service/cloud_service::build_rpc_response`
- <- `service/cloud_service::CloudService::on_rpc_message`
### `service/cloud_service::parse_small_json_object`
- <- `service/cloud_service::CloudService::on_shared_attributes_message`
- <- `service/cloud_service::CloudService::on_shared_attributes_response_message`
### `service/cloud_service::stringify_json_value`
- <- `service/cloud_service::CloudService::on_rpc_message`
### `service/command_tracker::CommandTracker::finish_active`
- <- `service/command_tracker::CommandTracker::finish_failure`
- <- `service/command_tracker::CommandTracker::finish_success`
### `service/command_tracker::CommandTracker::make_snapshot`
- <- `service/command_tracker::CommandTracker::accept`
- <- `service/command_tracker::CommandTracker::reject`
### `service/config_service::ConfigService::active_runtime_config`
- <- `service/config_service::ConfigService::apply_active_runtime_schedules`
### `service/config_service::ConfigService::apply_active_runtime_schedules`
- <- `service/config_service::ConfigService::apply_runtime_patch`
### `service/config_service::ConfigService::apply_schedule_json`
- <- `service/config_service::ConfigService::apply_runtime_patch`
### `service/config_service::ConfigService::backup_path`
- <- `service/config_service::ConfigService::load`
- <- `service/config_service::ConfigService::replace_and_save`
### `service/config_service::ConfigService::clear_pending`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
### `service/config_service::ConfigService::clone_document`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::replace_and_save`
- <- `service/config_service::ConfigService::snapshot`
### `service/config_service::ConfigService::derive_companion_path`
- <- `service/config_service::ConfigService::backup_path`
### `service/config_service::ConfigService::load_json_file_into`
- <- `service/config_service::ConfigService::load`
- <- `service/config_service::ConfigService::load_fixed`
### `service/config_service::ConfigService::load_pending`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::has_pending_runtime_config`
- <- `service/config_service::ConfigService::pending_runtime_config`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
### `service/config_service::ConfigService::parse_endpoint_string`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::parse_runtime_config`
### `service/config_service::ConfigService::parse_runtime_config`
- <- `service/config_service::ConfigService::active_runtime_config`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::pending_runtime_config`
### `service/config_service::ConfigService::parse_schedule_entries`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::parse_runtime_config`
### `service/config_service::ConfigService::read_json_file`
- <- `service/config_service::ConfigService::load_json_file_into`
- <- `service/config_service::ConfigService::load_pending`
### `service/config_service::ConfigService::replace_and_save`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
### `service/config_service::ConfigService::runtime_config_to_pending_root`
- <- `service/config_service::ConfigService::save_pending_runtime_config`
### `service/config_service::ConfigService::save_locked`
- <- `service/config_service::ConfigService::replace_and_save`
- <- `service/config_service::ConfigService::save`
### `service/config_service::ConfigService::save_pending`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
- <- `service/config_service::ConfigService::save_pending_runtime_config`
### `service/config_service::ConfigService::snapshot`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::pending_runtime_config`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
### `service/config_service::ConfigService::split_path`
- <- `service/config_service::ConfigService::get_subtree`
- <- `service/config_service::get_optional_from_document`
- <- `service/config_service::set`
### `service/config_service::ConfigService::validate_runtime_config`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::parse_runtime_config`
- <- `service/config_service::ConfigService::save_pending_runtime_config`
### `service/config_service::ConfigService::write_json_file`
- <- `service/config_service::ConfigService::replace_and_save`
- <- `service/config_service::ConfigService::save_locked`
- <- `service/config_service::ConfigService::save_pending`
### `service/config_service::constexpr`
- <- `service/config_service::get_optional_from_document`
- <- `service/config_service::set`
### `service/config_service::ensure_object_member`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::apply_schedule_json`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
### `service/config_service::find_path`
- <- `service/config_service::ConfigService::get_subtree`
### `service/config_service::is_supported_runtime_patch_field`
- <- `service/config_service::ConfigService::apply_runtime_patch`
### `service/config_service::is_valid_repeat_count`
- <- `service/config_service::ConfigService::apply_runtime_patch`
### `service/config_service::merge_object_members`
- <- `service/config_service::merge_runtime_root`
### `service/config_service::merge_runtime_root`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::pending_runtime_config`
- <- `service/config_service::ConfigService::promote_pending_runtime_to_active`
### `service/config_service::parse_json_text`
- <- `service/config_service::ConfigService::read_json_file`
### `service/config_service::set_double_member`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
### `service/config_service::set_int_member`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
### `service/config_service::set_string_member`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
### `service/config_service::set_uint_member`
- <- `service/config_service::ConfigService::apply_runtime_patch`
- <- `service/config_service::ConfigService::runtime_config_to_pending_root`
### `service/heading_corrector::HeadingCorrector::apply_correction`
- <- `service/heading_corrector::HeadingCorrector::compute`
### `service/heading_corrector::HeadingCorrector::clamp_alpha`
- <- `service/heading_corrector::HeadingCorrector::low_pass`
### `service/heading_corrector::HeadingCorrector::close_socket_locked`
- <- `service/heading_corrector::HeadingCorrector::consume_socket_locked`
- <- `service/heading_corrector::HeadingCorrector::set_params`
- <- `service/heading_corrector::HeadingCorrector::stop_io_thread`
### `service/heading_corrector::HeadingCorrector::connect_locked`
- <- `service/heading_corrector::HeadingCorrector::io_loop`
### `service/heading_corrector::HeadingCorrector::consume_socket_locked`
- <- `service/heading_corrector::HeadingCorrector::io_loop`
### `service/heading_corrector::HeadingCorrector::ingest_json_line_locked`
- <- `service/heading_corrector::HeadingCorrector::consume_socket_locked`
### `service/heading_corrector::HeadingCorrector::io_loop`
- <- `service/heading_corrector::HeadingCorrector::start_io_thread_if_needed`
### `service/heading_corrector::HeadingCorrector::latest_result_age_ms_locked`
- <- `service/heading_corrector::HeadingCorrector::compute`
- <- `service/heading_corrector::HeadingCorrector::debug_state`
### `service/heading_corrector::HeadingCorrector::low_pass`
- <- `service/heading_corrector::HeadingCorrector::compute`
### `service/heading_corrector::HeadingCorrector::reset_control_state_locked`
- <- `service/heading_corrector::HeadingCorrector::compute`
- <- `service/heading_corrector::HeadingCorrector::enable`
- <- `service/heading_corrector::HeadingCorrector::reset`
### `service/heading_corrector::HeadingCorrector::set_params`
- <- `service/heading_corrector::HeadingCorrector::HeadingCorrector`
### `service/heading_corrector::HeadingCorrector::start_io_thread_if_needed`
- <- `service/heading_corrector::HeadingCorrector::HeadingCorrector`
### `service/heading_corrector::HeadingCorrector::stop_io_thread`
- <- `service/heading_corrector::HeadingCorrector::~HeadingCorrector`
### `service/heading_corrector::apply_base_abs_rpm`
- <- `service/heading_corrector::HeadingCorrector::compute`
### `service/heading_corrector::is_enabled`
- <- `service/motion_service::MotionService::sync_heading_pid_enabled`
### `service/heading_corrector::normalize_yaw_to_control_error`
- <- `service/heading_corrector::HeadingCorrector::compute`
### `service/health_service::HealthService::build_payload`
- <- `service/health_service::HealthService::update`
### `service/health_service::append_char`
- <- `service/health_service::append_quoted`
### `service/health_service::append_raw`
- <- `service/health_service::append_quoted`
### `service/motion_service::MotionService::activate_walk_command`
- <- `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::apply_speed_if_command_current`
- <- `service/motion_service::MotionService::handle_override_clear`
- <- `service/motion_service::MotionService::update_heading_correction`
### `service/motion_service::MotionService::deactivate_walk_command`
- <- `service/motion_service::MotionService::emergency_stop`
- <- `service/motion_service::MotionService::start_cleaning_to`
- <- `service/motion_service::MotionService::stop_cleaning`
### `service/motion_service::MotionService::enable_speed_mode`
- <- `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::handle_override_clear`
- <- `service/motion_service::MotionService::update`
### `service/motion_service::MotionService::observe_override_clear`
- <- `service/motion_service::MotionService::handle_override_clear`
### `service/motion_service::MotionService::snapshot_state`
- <- `service/motion_service::MotionService::primary_dock`
- <- `service/motion_service::MotionService::start_cleaning_to`
- <- `service/motion_service::MotionService::sync_heading_pid_enabled`
- <- `service/motion_service::MotionService::update`
### `service/motion_service::MotionService::start_cleaning_to`
- <- `service/motion_service::MotionService::start_segment`
### `service/motion_service::MotionService::sync_heading_pid_enabled`
- <- `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::sync_runtime_config`
- <- `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::target_direction_sign`
- <- `service/motion_service::MotionService::start_cleaning_to`
### `service/motion_service::MotionService::update_heading_correction`
- <- `service/motion_service::MotionService::update`
### `service/motion_service::to_corrector_command`
- <- `service/motion_service::MotionService::update_heading_correction`
### `service/motion_service::to_group_command`
- <- `service/motion_service::MotionService::update_heading_correction`
### `service/nav_service::clamp01`
- <- `service/nav_service::NavService::update`
### `service/nav_service::project_gps_to_track`
- <- `service/nav_service::NavService::update`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::accept_rpc_command`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::register_command_rpc`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_attributes_payload`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_startup_attributes`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_business_payload`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_business_telemetry`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::register_command_rpc`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::register_rpc_handlers`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::reject_rpc_command`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::register_command_rpc`
### `service/thingsboard_control_plane::ThingsBoardControlPlane::rpc_reply`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::accept_rpc_command`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::reject_rpc_command`
### `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_business_telemetry`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_business_telemetry`
### `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_startup_attributes`
- <- `service/thingsboard_control_plane::ThingsBoardControlPlane::publish_startup_attributes`
### `service/thingsboard_control_plane::overflow`
- <- `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_business_telemetry`
- <- `service/thingsboard_control_plane::ThingsBoardJsonCodec::build_startup_attributes`
### `service/thingsboard_control_plane::write_schedule_entries`
- <- `service/thingsboard_control_plane::write_runtime_config`
### `service/uds_gyro_yaw_fusion::non_negative`
- <- `service/uds_gyro_yaw_fusion::UdsGyroYawFusion::set_params`

## 8. 反向 include 索引

### 生产源码 include
- `app/fault_policy` <- `app/fault_detector`, `app/robot_controller`
- `app/robot_controller` <- `main.cc/main`
- `app/watchdog_mgr` <- `main.cc/main`
- `device/bms` <- `main.cc/main`, `service/health_service`
- `device/brush_motor` <- `main.cc/main`, `service/health_service`, `service/motion_service`
- `device/device_error` <- `device/bms`, `device/brush_motor`, `device/gps_device`, `device/gps_source`, `device/imu_device`, `device/walk_motor_group`
- `device/gps_device` <- `main.cc/main`, `service/health_service`, `service/nav_service`
- `device/gps_source` <- `device/gps_device`, `device/gpsd_gps_source`, `device/serial_gps_source`
- `device/imu_device` <- `main.cc/main`, `service/health_service`, `service/motion_service`, `service/nav_service`
- `device/limit_switch` <- `main.cc/main`, `middleware/safety_monitor`
- `device/walk_motor_group` <- `main.cc/main`, `service/health_service`, `service/motion_service`, `service/nav_service`
- `device/walk_motor_types` <- `device/walk_motor_group`
- `domain/robot_domain` <- `app/fault_detector`, `app/fault_policy`, `app/robot_controller`, `main.cc/main`, `middleware/safety_monitor`, `service/config_service`, `service/fault_service`, `service/heading_corrector`, `service/motion_service`, `service/thingsboard_control_plane`
- `driver/libgpiod_pin` <- `main.cc/main`
- `driver/libmodbus_master` <- `main.cc/main`
- `driver/libserialport_port` <- `main.cc/main`
- `driver/linux_can_socket` <- `main.cc/main`
- `hal/i_can_bus` <- `device/walk_motor_group`, `driver/linux_can_socket`, `protocol/walk_motor_can_codec`
- `hal/i_gpio_pin` <- `device/attitude_limit_switch`, `device/limit_switch`, `driver/libgpiod_pin`
- `hal/i_modbus_master` <- `device/distance_sensor`, `driver/libmodbus_master`, `main.cc/main`
- `hal/i_serial_port` <- `device/bms`, `device/brush_motor`, `device/gps_device`, `device/imu_device`, `device/serial_gps_source`, `driver/libserialport_port`, `main.cc/main`, `middleware/lorawan_transport`
- `hal/pi_mutex` <- `app/watchdog_mgr`, `device/attitude_limit_switch`, `device/brush_motor`, `device/distance_sensor`, `device/imu_device`, `device/limit_switch`, `device/walk_motor_group`, `driver/libgpiod_pin`, `middleware/event_bus`, `service/fault_service`, `service/nav_service`
- `middleware/data_cache` <- `main.cc/main`, `service/cloud_service`
- `middleware/event_bus` <- `main.cc/main`, `middleware/safety_monitor`, `service/cloud_service`, `service/fault_service`, `service/motion_service`
- `middleware/i_network_transport` <- `middleware/lorawan_transport`, `middleware/mqtt_transport`, `middleware/network_manager`
- `middleware/logger` <- `main.cc/main`
- `middleware/lorawan_transport` <- `main.cc/main`
- `middleware/mqtt_transport` <- `main.cc/main`
- `middleware/network_manager` <- `main.cc/main`, `service/cloud_service`
- `middleware/safety_monitor` <- `main.cc/main`
- `middleware/thread_executor` <- `main.cc/main`, `service/cloud_service`, `service/health_service`, `service/motion_service`, `service/nav_service`
- `protocol/bms_protocol` <- `device/bms`
- `protocol/distance_sensor_protocol` <- `device/distance_sensor`
- `protocol/gpsd_json_parser` <- `device/gps_source`
- `protocol/imu_protocol` <- `device/imu_device`
- `protocol/nmea_parser` <- `device/gps_device`, `device/gps_source`, `protocol/gpsd_json_parser`
- `protocol/odrive_ascii_protocol` <- `device/brush_motor`
- `protocol/walk_motor_can_codec` <- `device/walk_motor_group`, `device/walk_motor_types`
- `service/cloud_service` <- `main.cc/main`, `service/health_service`, `service/thingsboard_control_plane`
- `service/command_tracker` <- `main.cc/main`, `service/thingsboard_control_plane`
- `service/config_service` <- `main.cc/main`, `service/motion_service`, `service/thingsboard_control_plane`
- `service/fault_service` <- `main.cc/main`
- `service/heading_corrector` <- `service/motion_service`
- `service/health_service` <- `main.cc/main`
- `service/motion_service` <- `main.cc/main`
- `service/nav_service` <- `main.cc/main`
- `service/recovery_motion` <- `main.cc/main`
- `service/scheduler_service` <- `main.cc/main`, `service/config_service`, `service/thingsboard_control_plane`
- `service/thingsboard_control_plane` <- `main.cc/main`
- `service/uds_gyro_yaw_fusion` <- `service/heading_corrector`
### 测试源码 include
- `app/fault_detector` <- `test/app/fault_detector_test`
- `app/fault_policy` <- `test/app/fault_policy_test`, `test/app/robot_controller_test`
- `app/robot_controller` <- `test/app/robot_controller_test`, `test/integration/hardware/system_hw_common`
- `app/watchdog_mgr` <- `test/app/watchdog_mgr_test`, `test/integration/hardware/system_hw_common`
- `device/attitude_limit_switch` <- `test/device/attitude_limit_switch_test`, `test/integration/hardware/attitude_limit_switch_hw_test`, `test/integration/hardware/hw_config`
- `device/bms` <- `test/device/bms_device_test`, `test/integration/hardware/bms_hw_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/protocol/bms2_protocol_test`, `test/protocol/bms_protocol_test`
- `device/brush_motor` <- `test/device/brush_motor_test`, `test/integration/hardware/hw_config`, `test/service/motion_service_test`
- `device/distance_sensor` <- `test/device/distance_sensor_device_test`, `test/integration/hardware/distance_sensor_hw_test`
- `device/gps_device` <- `test/device/gps_device_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`
- `device/imu_device` <- `test/device/imu_device_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/imu_hw_test`, `test/integration/hardware/system_hw_common`, `test/protocol/imu_protocol_test`
- `device/limit_switch` <- `test/device/limit_switch_test`, `test/integration/hardware/hw_config`, `test/middleware/safety_monitor_test`
- `device/walk_motor_group` <- `test/device/walk_motor_group_test`, `test/integration/hardware/hw_config`, `test/middleware/safety_monitor_test`, `test/protocol/walk_motor_codec_test`, `test/service/motion_service_test`
- `device/walk_motor_types` <- `test/device/walk_motor_group_test`, `test/protocol/walk_motor_codec_test`
- `domain/robot_domain` <- `test/app/fault_detector_test`, `test/app/fault_policy_test`, `test/app/robot_controller_test`, `test/domain/robot_domain_test`, `test/service/business_payload_builder_test`
- `driver/libgpiod_pin` <- `test/driver/libgpiod_pin_test`, `test/integration/hardware/hw_config`
- `driver/libmodbus_master` <- `test/driver/libmodbus_test`, `test/integration/hardware/distance_sensor_hw_test`, `test/integration/hardware/hw_config`, `test/protocol/bms2_protocol_test`
- `driver/libserialport_port` <- `test/device/bms_device_test`, `test/device/imu_device_test`, `test/driver/libserialport_test`, `test/integration/hardware/bms_hw_test`, `test/integration/hardware/hw_config`, `test/integration/hardware/imu_hw_test`, `test/protocol/bms_protocol_test`, `test/protocol/imu_protocol_test`
- `driver/linux_can_socket` <- `test/device/walk_motor_group_test`, `test/driver/linux_can_socket_test`, `test/integration/hardware/hw_config`, `test/protocol/walk_motor_codec_test`
- `hal/i_can_bus` <- `test/mock/mock_can_bus`
- `hal/i_gpio_pin` <- `test/mock/mock_gpio_pin`
- `hal/i_modbus_master` <- `test/mock/mock_modbus_master`
- `hal/i_serial_port` <- `test/mock/mock_serial_port`
- `hal/pi_mutex` <- `test/driver/pi_mutex_test`
- `middleware/data_cache` <- `test/middleware/data_cache_test`, `test/service/thingsboard_control_plane_test`
- `middleware/event_bus` <- `test/integration/hardware/system_hw_common`, `test/middleware/event_bus_test`, `test/middleware/safety_monitor_test`, `test/service/fault_service_test`, `test/service/motion_service_test`
- `middleware/mqtt_transport` <- `test/integration/thingsboard_test_support`, `test/middleware/mqtt_transport_test`
- `middleware/network_manager` <- `test/middleware/network_manager_test`, `test/service/thingsboard_control_plane_test`
- `middleware/safety_monitor` <- `test/integration/hardware/system_hw_common`, `test/middleware/safety_monitor_test`
- `middleware/thread_executor` <- `test/middleware/thread_executor_test`
- `protocol/bms2_protocol` <- `test/protocol/bms2_protocol_test`
- `protocol/bms_protocol` <- `test/device/bms_device_test`, `test/integration/hardware/bms_hw_test`, `test/protocol/bms_protocol_test`
- `protocol/distance_sensor_protocol` <- `test/integration/hardware/distance_sensor_hw_test`, `test/protocol/distance_sensor_protocol_test`
- `protocol/gpsd_json_parser` <- `test/protocol/gpsd_json_parser_test`
- `protocol/imu_protocol` <- `test/device/imu_device_test`, `test/integration/hardware/imu_hw_test`, `test/protocol/imu_protocol_test`
- `protocol/nmea_parser` <- `test/protocol/nmea_parser_test`
- `protocol/odrive_ascii_protocol` <- `test/protocol/odrive_ascii_protocol_test`
- `protocol/walk_motor_can_codec` <- `test/device/walk_motor_group_test`, `test/protocol/walk_motor_codec_test`, `test/service/motion_service_test`
- `service/cloud_service` <- `test/service/cloud_service_test`, `test/service/thingsboard_control_plane_test`
- `service/command_tracker` <- `test/service/command_tracker_test`, `test/service/thingsboard_control_plane_test`
- `service/config_service` <- `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/integration/thingsboard_test_support`, `test/service/config_service_runtime_patch_test`, `test/service/config_service_test`, `test/service/thingsboard_control_plane_test`
- `service/fault_service` <- `test/integration/hardware/system_hw_common`, `test/service/fault_service_test`
- `service/heading_corrector` <- `test/service/heading_pid_test`
- `service/health_service` <- `test/integration/hardware/hw_config`, `test/integration/hardware/system_hw_common`, `test/service/health_payload_builder_test`
- `service/motion_service` <- `test/integration/hardware/system_hw_common`, `test/service/motion_service_test`
- `service/nav_service` <- `test/integration/hardware/system_hw_common`
- `service/recovery_motion` <- `test/service/recovery_motion_test`
- `service/scheduler_service` <- `test/service/config_service_runtime_patch_test`, `test/service/scheduler_service_test`, `test/service/thingsboard_control_plane_test`
- `service/thingsboard_control_plane` <- `test/service/business_payload_builder_test`, `test/service/config_service_runtime_patch_test`, `test/service/thingsboard_control_plane_test`, `test/service/thingsboard_event_payload_builder_test`
- `service/uds_gyro_yaw_fusion` <- `test/integration/hardware/system_hw_common`, `test/service/uds_gyro_yaw_fusion_test`

## 9. 修改时的最小检查路径

- 改协议层：先跑对应 `test/protocol/*`，再跑依赖它的 `test/device/*`。
- 改设备层：看直接上游里的 service/middleware/main，再跑对应 device 测试和相关 service 测试。
- 改 `WalkMotorGroup`、`MotionService`、`SafetyMonitor`、`RobotController`：同时检查运动任务启动、周期控制、限位急停、看门狗故障四条主链。
- 改 `ConfigService` 或 `ThingsBoardControlPlane`：同时检查云端命令、配置下发、调度窗口和业务遥测。
- 改 HAL/driver 头文件：必须检查 mock、driver/device 单测和 `hw_tests` 编译，因为这些接口是硬件抽象边界。
