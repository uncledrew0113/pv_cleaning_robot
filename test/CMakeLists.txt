find_package(Catch2 CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(unit_tests
  test_base.cc

  # BMS 协议层 + 设备层单元测试（含 /dev/ttyS8 硬件集成测试）
  bms_test.cc

  # BMS2 (Modbus RTU) 协议层 + 设备层单元测试
  bms2_test.cc

  # IMU 协议层 + 设备层单元测试（WIT Motion UART 协议）
  imu_test.cc

  # 行走电机协议层 + 设备层单元测试（M1502E_111 CAN 协议）
  walk_motor_test.cc

  # 设备层源文件（测试中使用 MockGpioPin/MockModbusMaster 替代真实驱动）
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/limit_switch.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/brush_motor.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/bms.cc

  # 协议层源文件
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/protocol/bms_protocol.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/protocol/bms2_protocol.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/protocol/imu_protocol.cc

  # 设备层 imu_device
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/imu_device.cc

  # 行走电机协议层 + 设备层
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/protocol/walk_motor_can_codec.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/walk_motor.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/device/walk_motor_group.cc

  # 驱动层实现（硬件集成测试，需要在目标机上运行）
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/driver/libmodbus_master.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/driver/libserialport_port.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/driver/libgpiod_pin.cc
  ${PROJECT_SOURCE_DIR}/pv_cleaning_robot/driver/linux_can_socket.cc

  # 驱动层单元测试（纯软件路径 + 线程安全，无需真实硬件即可运行）
  driver_pi_mutex_test.cc
  driver_libgpiod_pin_test.cc
  driver_linux_can_socket_test.cc
  driver_libserialport_test.cc
  driver_libmodbus_test.cc
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

include(CTest)

# Cross-compiling note:
# Catch2 discovery runs the test binary, which usually can't execute on the host
# when targeting aarch64. We still build unit_tests, but only register tests when
# an emulator is provided.
if(CMAKE_CROSSCOMPILING AND NOT CMAKE_CROSSCOMPILING_EMULATOR)
  message(STATUS "Cross-compiling without CMAKE_CROSSCOMPILING_EMULATOR: unit tests are built but not registered")
else()
  include(Catch)
  catch_discover_tests(unit_tests)
endif()
