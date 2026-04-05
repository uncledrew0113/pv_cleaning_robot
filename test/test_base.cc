/*
 * @Author: UncleDrew
 * @Date: 2026-03-11 23:38:44
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-04-03 17:08:59
 * @FilePath: /pv_cleaning_robot/test/test_base.cc
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */

#include <array>
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <cassert>
#include <catch2/catch.hpp>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "mock/mock_can_bus.h"
#include "pv_cleaning_robot/device/bms.h"
#include "pv_cleaning_robot/device/imu_device.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/driver/libgpiod_pin.h"
#include "pv_cleaning_robot/driver/libmodbus_master.h"
#include "pv_cleaning_robot/driver/libserialport_port.h"
#include "pv_cleaning_robot/driver/linux_can_socket.h"
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include "pv_cleaning_robot/hal/i_gpio_pin.h"
#include "pv_cleaning_robot/hal/i_modbus_master.h"
#include "pv_cleaning_robot/hal/i_serial_port.h"
#include "pv_cleaning_robot/middleware/data_cache.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/safety_monitor.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include "pv_cleaning_robot/protocol/bms_protocol.h"
#include "pv_cleaning_robot/protocol/imu_protocol.h"
#include "pv_cleaning_robot/protocol/walk_motor_can_codec.h"
// ip link set can0 down
// ip link set can0 type can bitrate 500000
// ip link set can0 up

TEST_CASE("driver can驱动", "[driver][can]") {
    auto test_timeDT = std::chrono::milliseconds(100);
    spdlog::info("start");
    robot::driver::LinuxCanSocket can_bus("can0");
    can_bus.open();
    robot::hal::CanFrame frame_set_mode;
    frame_set_mode.id = 0x105;
    frame_set_mode.len = 8;
    frame_set_mode.data[0] = 0x02;
    frame_set_mode.data[1] = 0x02;
    frame_set_mode.data[2] = 0x02;
    frame_set_mode.data[3] = 0x02;
    frame_set_mode.data[4] = 0x02;
    frame_set_mode.data[5] = 0x02;
    frame_set_mode.data[6] = 0x02;
    frame_set_mode.data[7] = 0x02;
    frame_set_mode.is_ext = false;
    frame_set_mode.is_rtr = false;
    CHECK(can_bus.send(frame_set_mode) == true);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    spdlog::info("发送设置模式指令:{}", "速度闭环模式");
    robot::hal::CanFrame frame_set_velocity;
    frame_set_velocity.id = 0x032;
    frame_set_velocity.len = 8;
    frame_set_velocity.data[0] = 0x17;
    frame_set_velocity.data[1] = 0xD0;
    frame_set_velocity.data[2] = 0x07;
    frame_set_velocity.data[3] = 0xD0;
    frame_set_velocity.data[4] = 0x07;
    frame_set_velocity.data[5] = 0xD0;
    frame_set_velocity.data[6] = 0x07;
    frame_set_velocity.data[7] = 0xD0;
    frame_set_velocity.is_ext = false;
    frame_set_velocity.is_rtr = false;
    CHECK(can_bus.send(frame_set_velocity) == true);
    spdlog::info("发送设置速度指令:0x17D0 m/s");
    frame_set_velocity.data[0] = 0x00;
    frame_set_velocity.data[1] = 0xD0;
    CHECK(can_bus.send(frame_set_velocity) == true);
    std::this_thread::sleep_for(test_timeDT);
    spdlog::info("发送设置速度指令:0x0fD0 m/s");
    frame_set_velocity.data[0] = 0x17;
    frame_set_velocity.data[1] = 0xD0;
    CHECK(can_bus.send(frame_set_velocity) == true);
    std::this_thread::sleep_for(test_timeDT);
    spdlog::info("发送设置速度指令:0x17D0 m/s");
    frame_set_velocity.data[0] = 0x00;
    frame_set_velocity.data[1] = 0xD0;
    CHECK(can_bus.send(frame_set_velocity) == true);
    std::this_thread::sleep_for(test_timeDT);
    spdlog::info("发送设置速度指令:0x0fD0 m/s");
    frame_set_velocity.data[0] = 0x17;
    frame_set_velocity.data[1] = 0xD0;
    CHECK(can_bus.send(frame_set_velocity) == true);
    std::this_thread::sleep_for(test_timeDT);
    spdlog::info("发送设置速度指令:0x17D0 m/s");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    frame_set_velocity.id = 0x032;
    frame_set_velocity.len = 8;
    frame_set_velocity.data[0] = 0x00;
    frame_set_velocity.data[1] = 0x00;
    frame_set_velocity.data[2] = 0x00;
    frame_set_velocity.data[3] = 0x00;
    frame_set_velocity.data[4] = 0x00;
    frame_set_velocity.data[5] = 0x00;
    frame_set_velocity.data[6] = 0x00;
    frame_set_velocity.data[7] = 0x00;
    frame_set_velocity.is_ext = false;
    frame_set_velocity.is_rtr = false;
    CHECK(can_bus.send(frame_set_velocity) == true);
    can_bus.close();
}

TEST_CASE("drivercan_continue", "[driver][can]") {}

static constexpr const char* TARGET_CHIP = "gpiochip5";
static constexpr unsigned int TARGET_IN_LINE_1 = 0;   ///< DI1
static constexpr unsigned int TARGET_IN_LINE_2 = 1;   ///< DI2
static constexpr unsigned int TARGET_OUT_LINE_1 = 8;  ///< DO1
static constexpr unsigned int TARGET_OUT_LINE_2 = 9;  ///< DO2

TEST_CASE("driver gpio驱动", "[driver][gpio]") {
    robot::driver::LibGpiodPin in_pin(TARGET_CHIP, TARGET_IN_LINE_1, "test_in_lifecycle");

    robot::hal::GpioConfig in_cfg;
    in_cfg.direction = robot::hal::GpioDirection::INPUT;
    in_cfg.bias = robot::hal::GpioBias::DISABLE;
    in_cfg.debounce_ms = 2;

    robot::driver::LibGpiodPin in_pin2(TARGET_CHIP, TARGET_IN_LINE_2, "test_in_lifecycle");

    bool ok = in_pin.open(in_cfg);
    REQUIRE(ok == true);
    bool ok2 = in_pin2.open(in_cfg);
    REQUIRE(ok2 == true);
    for (size_t i = 0; i < 1000; i++) {
        spdlog::info("DI1: {}", in_pin.read_value());
        spdlog::info("DI2: {}", in_pin2.read_value());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    in_pin.close();
}

TEST_CASE("driver modbus驱动", "[driver][modbus]") {}

TEST_CASE("driver serial驱动", "[driver][serial]") {
    robot::driver::LibSerialPort serial_port("/dev/ttyS1");
    REQUIRE(serial_port.open());
    const std::string cmd1 = "GPRMC 1\r\n";
    REQUIRE(serial_port.write(reinterpret_cast<const uint8_t*>(cmd1.c_str()), cmd1.length(), 100) >
            0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t temp_buf[256];         // 物理读取缓冲区
    std::string line_accumulator;  // 逻辑行累加器

    while (true) {
        // 调用你定义的 read 函数，建议超时设为 100ms
        int n = serial_port.read(temp_buf, sizeof(temp_buf), 100);

        if (n > 0) {
            // 1. 将读取到的字节存入累加器
            line_accumulator.append(reinterpret_cast<char*>(temp_buf), n);

            // 2. 检查是否有完整的行（可能一次收到多行）
            size_t pos;
            while ((pos = line_accumulator.find('\n')) != std::string::npos) {
                // 提取一行完整 NMEA 数据
                std::string nmea_sentence = line_accumulator.substr(0, pos + 1);

                // 从累加器中移除已提取的部分
                line_accumulator.erase(0, pos + 1);

                // 3. 处理这行数据
                spdlog::info("Received NMEA sentence: {}", nmea_sentence);
            }
        } else if (n < 0) {
            // 处理错误（如断开连接）
            spdlog::error("Serial read failed, exiting...");
            break;
        }

        // 容错：防止因没读到换行符导致内存无限增长
        if (line_accumulator.size() > 1024) {
            line_accumulator.clear();
        }
    }
}

TEST_CASE("无锁队列", "[driver][nonlock]") {
    boost::lockfree::spsc_queue<int, boost::lockfree::capacity<1024>> q;

    assert(q.empty());

    for (int i = 0; i < 100; ++i) {
        assert(q.push(i));
    }

    int v;
    for (int i = 0; i < 100; ++i) {
        assert(q.pop(v));
        assert(v == i);
    }

    assert(q.empty());
}

using namespace robot;

static constexpr char kCanIface[] = "can0";
static constexpr uint8_t kMotorId = 1u;

TEST_CASE("循环加速0-10rpm", "[integration][walk_motor_test]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotorGroup group(bus, kMotorId, 0);
    REQUIRE(group.open() == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    group.enable_all();
    group.set_mode_all(protocol::WalkMotorMode::SPEED);
    group.enable_heading_control(false);
    std::this_thread::sleep_for(250ms);

    SECTION("逐渐加速 返回 OK") {
        for (int i = 0; i <= 20; i += 2) {
            auto err = group.set_speeds_extra(i, i, -i, -i);
            REQUIRE(err == device::DeviceError::OK);
            std::this_thread::sleep_for(200ms);
        }

        spdlog::info("[WalkMotor 集成] set_speed(30.0) 已发送");
    }

    std::this_thread::sleep_for(2000ms);

    SECTION("逐渐减速 返回 OK") {
        for (int i = 20; i >= 0; i -= 2) {
            auto err = group.set_speeds_extra(i, i, -i, -i);
            REQUIRE(err == device::DeviceError::OK);
            std::this_thread::sleep_for(200ms);
        }

        spdlog::info("[WalkMotor 集成] set_speed(0.0) 已发送");
    }

    SECTION("逐渐返回加速 返回 OK") {
        for (int i = 0; i <= 20; i += 2) {
            auto err = group.set_speeds_extra(-i, -i, i, i);
            REQUIRE(err == device::DeviceError::OK);
            std::this_thread::sleep_for(200ms);
        }

        spdlog::info("[WalkMotor 集成] set_speed(30.0) 已发送");
    }

    std::this_thread::sleep_for(2000ms);

    SECTION("逐渐减速 返回 OK") {
        for (int i = 20; i >= 0; i -= 2) {
            auto err = group.set_speeds_extra(-i, -i, i, i);
            REQUIRE(err == device::DeviceError::OK);
            std::this_thread::sleep_for(200ms);
        }

        spdlog::info("[WalkMotor 集成] set_speed(0.0) 已发送");
    }

    std::this_thread::sleep_for(500ms);
    group.close();
}

TEST_CASE("goto20rpm", "[integration][walk_motor_test]") {
    using namespace std::chrono_literals;

    auto bus = std::make_shared<driver::LinuxCanSocket>(kCanIface);
    device::WalkMotorGroup group(bus, kMotorId, 0);
    REQUIRE(group.open() == device::DeviceError::OK);

    // std::atomic<bool> stop_monitor{false};
    // auto monitor_thread = std::thread([&group, &stop_monitor]() -> void {
    //     while (!stop_monitor) {
    //         auto gd = group.get_group_diagnostics();
    //         for (int i = 0; i < device::WalkMotorGroup::kWheelCount; ++i) {
    //             auto& wd = gd.wheel[i];
    //             spdlog::info("rpm:{} current:{} position_deg:{} fault:{} mode:{}",
    //                          wd.speed_rpm,
    //                          wd.torque_a,
    //                          wd.position_deg,
    //                          static_cast<int>(wd.fault),
    //                          static_cast<int>(wd.mode));
    //         }
    //         std::this_thread::sleep_for(300ms);
    //     }
    // });

    float speed = 20.0f;
    std::this_thread::sleep_for(500ms);
    group.set_terminations(false, true, false, false);
    std::this_thread::sleep_for(500ms);
    // group.enable_all();
    group.set_mode_all(protocol::WalkMotorMode::SPEED);
    std::this_thread::sleep_for(500ms);
    group.enable_heading_control(false);
    std::this_thread::sleep_for(250ms);

    robot::device::DeviceError err{};
    for (int i = 0; i <= 20; i += 1) {
        err = group.set_speeds_extra(i, i, -i, -i);
        REQUIRE(err == device::DeviceError::OK);
        std::this_thread::sleep_for(100ms);
        spdlog::info("[WalkMotor 集成] set_speed(30.0) 已发送");
    }

    std::this_thread::sleep_for(12000ms);

    err = group.set_speeds_extra(0, 0, 0, 0);
    REQUIRE(err == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    spdlog::info("[WalkMotor 集成] set_speed(0.0) 已发送");

    std::this_thread::sleep_for(2000ms);

    err = group.set_speeds_extra(-speed, -speed, speed, speed);
    REQUIRE(err == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    spdlog::info("[WalkMotor 集成] set_speed(30.0) 已发送");

    std::this_thread::sleep_for(12000ms);

    err = group.set_speeds_extra(0, 0, 0, 0);
    REQUIRE(err == device::DeviceError::OK);
    std::this_thread::sleep_for(200ms);

    spdlog::info("[WalkMotor 集成] set_speed(0.0) 已发送");

    std::this_thread::sleep_for(500ms);
    // stop_monitor = true;  // 1. 先发停止信号
    // if (monitor_thread.joinable()) {
    //     monitor_thread.join();  // 2. 等待线程退出
    // }

    group.close();
}

// 全局停止标记（信号处理必须用全局）
static std::atomic<bool> g_stop_flag{false};

// Ctrl+C 信号处理函数
static void sigint_handler(int /*signal*/) {
    spdlog::warn("收到 Ctrl+C，正在停止测试...");
    g_stop_flag = true;
}

TEST_CASE("BMS循环测试-ttyS8", "[integration][bms]") {
    using namespace std::chrono_literals;

    hal::UartConfig cfg;
    cfg.baudrate = 9600;
    cfg.data_bits = 8;
    cfg.parity = 'N';
    cfg.stop_bits = 1;
    cfg.write_timeout_ms = 500;

    auto serial = std::make_shared<driver::LibSerialPort>("/dev/ttyS8", cfg);
    device::BMS bms(serial, 95.0f, 15.0f);

    std::signal(SIGINT, sigint_handler);

    REQUIRE(bms.open() == device::DeviceError::OK);

    while (!g_stop_flag) {
        bms.update();

        auto diag = bms.get_diagnostics();
        spdlog::info("  SOC           = {:.1f}%", diag.soc_pct);
        spdlog::info("  总电压        = {:.3f} V", diag.voltage_v);
        spdlog::info("  电流          = {:.3f} A", diag.current_a);
        spdlog::info("  update/error  = {}/{}", diag.update_count, diag.error_count);
        for (int i = 0; i < 10 && !g_stop_flag; ++i) {
            std::this_thread::sleep_for(10ms);
        }
    }
    // ================= 安全关闭 =================
    spdlog::info("BMS 测试已安全停止 ✅");
    bms.close();
}
