find_package(Catch2 CONFIG REQUIRED)
find_package(Threads REQUIRED)

# ── 被测源文件集合（unit_tests 和 hw_tests 共用） ──────────────────────
set(PROJ ${PROJECT_SOURCE_DIR}/pv_cleaning_robot)

set(COMMON_SRCS
  # 协议层
  ${PROJ}/protocol/bms_protocol.cc
  ${PROJ}/protocol/bms2_protocol.cc
  ${PROJ}/protocol/imu_protocol.cc
  ${PROJ}/protocol/walk_motor_can_codec.cc
  ${PROJ}/protocol/nmea_parser.cc

  # 设备层
  ${PROJ}/device/bms.cc
  ${PROJ}/device/imu_device.cc
  ${PROJ}/device/gps_device.cc
  ${PROJ}/device/walk_motor.cc
  ${PROJ}/device/walk_motor_group.cc
  ${PROJ}/device/brush_motor.cc
  ${PROJ}/device/limit_switch.cc

  # 驱动层
  ${PROJ}/driver/libmodbus_master.cc
  ${PROJ}/driver/libserialport_port.cc
  ${PROJ}/driver/libgpiod_pin.cc
  ${PROJ}/driver/linux_can_socket.cc

  # 中间件层
  ${PROJ}/middleware/data_cache.cc
  ${PROJ}/middleware/safety_monitor.cc

  # 服务层
  ${PROJ}/service/config_service.cc
  ${PROJ}/service/fault_service.cc
  ${PROJ}/service/scheduler_service.cc
  ${PROJ}/service/nav_service.cc
  ${PROJ}/service/motion_service.cc

  # App 层
  ${PROJ}/app/robot_fsm.cc
  ${PROJ}/app/fault_handler.cc
  ${PROJ}/app/watchdog_mgr.cc
  ${PROJ}/app/clean_task.cc
)

# ── 单元测试可执行文件（纯软件路径，无需真实硬件） ──────────────────────
add_executable(unit_tests
  test_main.cc

  # 协议层
  protocol/bms_protocol_test.cc
  protocol/bms2_protocol_test.cc
  protocol/imu_protocol_test.cc
  protocol/walk_motor_codec_test.cc
  protocol/nmea_parser_test.cc

  # 设备层
  device/bms_device_test.cc
  device/imu_device_test.cc
  device/walk_motor_test.cc
  device/walk_motor_group_test.cc
  device/brush_motor_test.cc
  device/limit_switch_test.cc
  device/gps_device_test.cc

  # 驱动层（纯软件路径，无需真实硬件即可运行）
  driver/pi_mutex_test.cc
  driver/libgpiod_pin_test.cc
  driver/linux_can_socket_test.cc
  driver/libserialport_test.cc
  driver/libmodbus_test.cc

  # 中间件层
  middleware/event_bus_test.cc
  middleware/data_cache_test.cc
  middleware/safety_monitor_test.cc

  # 服务层
  service/config_service_test.cc
  service/fault_service_test.cc
  service/scheduler_service_test.cc
  service/motion_service_test.cc

  # App 层
  app/robot_fsm_test.cc
  app/fault_handler_test.cc
  app/watchdog_mgr_test.cc

  # 集成测试（非硬件，可在 QEMU 中运行）
  integration/task_chain_test.cc

  ${COMMON_SRCS}
)

target_include_directories(unit_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_SOURCE_DIR}/test/mock
)

target_link_libraries(unit_tests PRIVATE
  Catch2::Catch2
  spdlog::spdlog
  Threads::Threads
  robot_3rdparty
)

# ── 硬件集成测试可执行文件（需要真实硬件，目标机上运行） ─────────────────
add_executable(hw_tests
  integration/hardware/hw_test_main.cc
  integration/hardware/driver_hw_test.cc
  integration/hardware/bms_hw_test.cc
  integration/hardware/imu_hw_test.cc
  integration/hardware/walk_motor_hw_test.cc
  integration/hardware/full_sweep_hw_test.cc

  ${COMMON_SRCS}
)

target_include_directories(hw_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_SOURCE_DIR}/test/mock
)

target_link_libraries(hw_tests PRIVATE
  Catch2::Catch2
  spdlog::spdlog
  Threads::Threads
  robot_3rdparty
)

# ── CTest 注册 ────────────────────────────────────────────────────────
include(CTest)

# Cross-compiling note:
# Catch2 discovery runs the test binary, which usually can't execute on the host
# when targeting aarch64. We still build the binaries, but only register tests
# when an emulator is provided.
if(CMAKE_CROSSCOMPILING AND NOT CMAKE_CROSSCOMPILING_EMULATOR)
  message(STATUS "Cross-compiling without CMAKE_CROSSCOMPILING_EMULATOR: tests are built but not registered")
else()
  include(Catch)
  catch_discover_tests(unit_tests)
  catch_discover_tests(hw_tests)
endif()
