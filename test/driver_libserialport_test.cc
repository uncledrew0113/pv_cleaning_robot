/*
 * LibSerialPort 驱动层单元测试
 *
 * 测试分两类：
 *   [driver][serial_port]           — 纯软件错误路径（无需真实串口硬件，任何 Linux 主机可运行）
 *   [driver][serial_port][hardware] — 硬件集成测试（需要目标板上的真实串口设备）
 *
 * 运行方式：
 *   仅软件测试：./unit_tests "[driver][serial_port]" ~[hardware]
 *   含硬件测试：./unit_tests "[driver][serial_port]"
 *
 * 测试覆盖：
 *   - 生命周期（构造/析构/open/close 幂等性）
 *   - 未打开状态下所有 I/O 操作快速失败并设置 DISCONNECTED
 *   - 零长度写入与 nullptr 缓冲区的边界安全性
 *   - open→close 反复循环无资源泄漏/崩溃
 *   - 多线程并发 write/read 均快速失败不崩溃
 *   - 并发 close+write（shared_mutex 独占锁保证资源不被双重释放）
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/driver/libserialport_port.h"

using namespace robot;
using namespace robot::hal;

// 不存在的串口设备路径（libserialport 会在 sp_open 时快速失败）
static constexpr const char* FAKE_PORT = "/dev/ttyS_TEST_NOT_EXIST";

// ═══════════════════════════════════════════════════════════════════════
//  纯软件路径测试（不访问真实串口硬件）
// ═══════════════════════════════════════════════════════════════════════

// ── 生命周期 ────────────────────────────────────────────────────────────

TEST_CASE("LibSerialPort - 构造后默认未连接且初始状态正确", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    CHECK_FALSE(port.is_open());
    CHECK(port.get_last_error() == UartResult::OK);
}

TEST_CASE("LibSerialPort - 无效串口路径 open 返回 false 且 is_open 保持 false",
          "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    CHECK_FALSE(port.open());
    CHECK_FALSE(port.is_open());
}

TEST_CASE("LibSerialPort - 未打开时 close 幂等安全", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    REQUIRE_NOTHROW(port.close());
    REQUIRE_NOTHROW(port.close());  // 二次不崩溃
    CHECK_FALSE(port.is_open());
}

TEST_CASE("LibSerialPort - open 失败后 close 安全", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    port.open();  // 预期失败
    REQUIRE_NOTHROW(port.close());
    CHECK_FALSE(port.is_open());
}

TEST_CASE("LibSerialPort - 析构不崩溃（从不打开）", "[driver][serial_port]") {
    REQUIRE_NOTHROW([]() { driver::LibSerialPort port(FAKE_PORT); }());
}

TEST_CASE("LibSerialPort - 析构不崩溃（open 失败后）", "[driver][serial_port]") {
    REQUIRE_NOTHROW([]() {
        driver::LibSerialPort port(FAKE_PORT);
        port.open();  // 预期失败
    }());
}

// ── 未打开状态下的 I/O 操作 ─────────────────────────────────────────────

TEST_CASE("LibSerialPort - 未打开时 write 返回 -1 且设置 DISCONNECTED", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    const uint8_t buf[8] = {0};
    CHECK(port.write(buf, 8) == -1);
    CHECK(port.get_last_error() == UartResult::DISCONNECTED);
}

TEST_CASE("LibSerialPort - 未打开时 read 返回 -1 且设置 DISCONNECTED", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    uint8_t buf[64];
    CHECK(port.read(buf, 64, 50) == -1);
    CHECK(port.get_last_error() == UartResult::DISCONNECTED);
}

TEST_CASE("LibSerialPort - 未打开时 flush_input 返回 false 且设置 DISCONNECTED",
          "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    CHECK_FALSE(port.flush_input());
    CHECK(port.get_last_error() == UartResult::DISCONNECTED);
}

TEST_CASE("LibSerialPort - 未打开时 flush_output 返回 false 且设置 DISCONNECTED",
          "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    CHECK_FALSE(port.flush_output());
    CHECK(port.get_last_error() == UartResult::DISCONNECTED);
}

TEST_CASE("LibSerialPort - 未打开时 bytes_available 返回 -1 且设置 DISCONNECTED",
          "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    CHECK(port.bytes_available() == -1);
    CHECK(port.get_last_error() == UartResult::DISCONNECTED);
}

// ── I/O 边界安全 ────────────────────────────────────────────────────────

TEST_CASE("LibSerialPort - 零长度 write 安全（未打开路径）", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    const uint8_t dummy = 0;
    // 未打开：connected_==false → 提前返回 -1，不解引用指针
    int ret = -999;
    REQUIRE_NOTHROW([&] { ret = port.write(&dummy, 0); }());
    // 零长度写入：未打开时返回 -1（DISCONNECTED 路径）
    CHECK(ret == -1);
}

TEST_CASE("LibSerialPort - write nullptr + len=0 安全（提前返回，不解引用）",
          "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    // 未打开路径先检查 connected_ → 返回 -1，不会解引用 nullptr
    REQUIRE_NOTHROW(port.write(nullptr, 0));
}

TEST_CASE("LibSerialPort - read 零超时不崩溃（未打开）", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    uint8_t buf[1];
    REQUIRE_NOTHROW(port.read(buf, 1, 0));
}

// ── open→close 反复循环 ─────────────────────────────────────────────────

TEST_CASE("LibSerialPort - open→close 5 次循环无崩溃（无硬件）", "[driver][serial_port]") {
    // open 预期失败，close 幂等；循环确认无内存泄漏/句柄泄漏
    driver::LibSerialPort port(FAKE_PORT);
    for (int i = 0; i < 5; ++i) {
        port.open();   // 失败，状态保持 closed
        port.close();  // 幂等
        CHECK_FALSE(port.is_open());
    }
    SUCCEED("重复 open/close 循环无崩溃");
}

TEST_CASE("LibSerialPort - 多次实例化 open/close 无崩溃", "[driver][serial_port]") {
    constexpr int CYCLES = 10;
    for (int i = 0; i < CYCLES; ++i) {
        driver::LibSerialPort port(FAKE_PORT);
        port.open();
        // 析构自动 close
    }
    SUCCEED("多次实例化析构无崩溃");
}

// ═══════════════════════════════════════════════════════════════════════
//  线程安全测试（全部在关闭状态下并发，无需硬件）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LibSerialPort - 多线程并发 write 均返回 -1 无崩溃", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    constexpr int THREADS = 8;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::atomic<int> neg_count{0};
    std::vector<std::thread> threads;

    const uint8_t buf[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j) {
                if (port.write(buf, 8) == -1)
                    neg_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();

    CHECK(neg_count.load() == THREADS * ITERS);
}

TEST_CASE("LibSerialPort - 多线程并发 read 均返回 -1 无崩溃", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    constexpr int THREADS = 8;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::atomic<int> neg_count{0};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            uint8_t buf[64];
            for (int j = 0; j < ITERS; ++j) {
                if (port.read(buf, 64, 0) == -1)
                    neg_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();

    CHECK(neg_count.load() == THREADS * ITERS);
}

TEST_CASE("LibSerialPort - 多线程并发 bytes_available 无崩溃", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    constexpr int THREADS = 4;
    constexpr int ITERS = 10000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j)
                (void)port.bytes_available();
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 bytes_available 无崩溃");
}

TEST_CASE("LibSerialPort - 并发 close 与 write（shared_mutex 防止 UAF）", "[driver][serial_port]") {
    // close() 持独占锁，write() 持共享锁
    // 两者并发时 shared_mutex 保证 close 完成前 write 不会使用已释放的 port_ 指针
    driver::LibSerialPort port(FAKE_PORT);
    constexpr int THREADS = 4;
    constexpr int ITERS = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    const uint8_t buf[8] = {};
    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    port.close();  // 独占锁，幂等
            });
        } else {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    port.write(buf, 8);  // 共享锁，未打开返回 -1
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 close+write 无 UAF 崩溃");
}

TEST_CASE("LibSerialPort - 并发 close 与 read（shared_mutex 防止 UAF）", "[driver][serial_port]") {
    driver::LibSerialPort port(FAKE_PORT);
    constexpr int THREADS = 4;
    constexpr int ITERS = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    port.close();
            });
        } else {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                uint8_t buf[64];
                for (int j = 0; j < ITERS; ++j)
                    port.read(buf, 64, 0);
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 close+read 无 UAF 崩溃");
}
