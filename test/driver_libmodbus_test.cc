/*
 * LibModbusMaster 驱动层单元测试
 *
 * 测试分两类：
 *   [driver][modbus]           — 纯软件错误路径（无需真实 RS485 硬件，任何 Linux 主机可运行）
 *   [driver][modbus][hardware] — 硬件集成测试（需要目标板 RS485 接口 + Modbus 从站设备）
 *
 * 运行方式：
 *   仅软件测试：./unit_tests "[driver][modbus]" ~[hardware]
 *   含硬件测试：./unit_tests "[driver][modbus]"
 *
 * 测试覆盖：
 *   - 生命周期（构造/析构/open 快速失败/close 幂等性）
 *   - set_timeout_ms 在任意状态下安全（ctx_==nullptr 时提前返回）
 *   - ctx_==nullptr 时所有 I/O 操作返回 -1 + DISCONNECTED
 *   - 入参边界校验（越界 count/addr/nullptr）：先于 ctx_ 检查，返回 -1 + SYS_ERROR
 *   - 边界值合法入参（count=1/count=125 等）：通过参数检查，运行至 ctx_ 检查 → DISCONNECTED
 *   - slave_id 校验：需要 ctx_!=nullptr，测试见注释
 *   - 多线程并发 read_registers（bus_mutex_ 串行化保障）无崩溃
 *   - 并发 close+read_registers 无崩溃
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/driver/libmodbus_master.h"

using namespace robot;
using namespace robot::hal;

// 不存在的串口设备路径（modbus_connect 会快速失败）
static constexpr const char* FAKE_MODBUS_PORT = "/dev/ttyS_MODBUS_TEST_NOT_EXIST";

// ═══════════════════════════════════════════════════════════════════════
//  纯软件路径测试（不访问真实 Modbus 硬件）
// ═══════════════════════════════════════════════════════════════════════

// ── 生命周期 ────────────────────────────────────────────────────────────

TEST_CASE("LibModbusMaster - 构造后默认未打开且初始状态正确", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    CHECK_FALSE(m.is_open());
    CHECK(m.get_last_error() == ModbusResult::OK);
}

TEST_CASE("LibModbusMaster - 无效串口 open 快速失败返回 false（无重试睡眠）", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    CHECK_FALSE(m.open());
    CHECK_FALSE(m.is_open());
    // 快速失败后 ctx_ 被 modbus_free，状态为 DISCONNECTED 或 SYS_ERROR（连接错误）
    // 无论哪种，is_open() 必须为 false
}

TEST_CASE("LibModbusMaster - 未打开时 close 幂等安全", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    REQUIRE_NOTHROW(m.close());
    REQUIRE_NOTHROW(m.close());
    CHECK_FALSE(m.is_open());
}

TEST_CASE("LibModbusMaster - open 失败后 close 安全（ctx_ 已被 open 内部释放）",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    m.open();  // 预期失败，open 内部已 modbus_free(ctx_)
    REQUIRE_NOTHROW(m.close());
    CHECK_FALSE(m.is_open());
}

TEST_CASE("LibModbusMaster - 析构不崩溃（从不打开）", "[driver][modbus]") {
    REQUIRE_NOTHROW([]() { driver::LibModbusMaster m(FAKE_MODBUS_PORT); }());
}

TEST_CASE("LibModbusMaster - 析构不崩溃（open 失败后）", "[driver][modbus]") {
    REQUIRE_NOTHROW([]() {
        driver::LibModbusMaster m(FAKE_MODBUS_PORT);
        m.open();  // 预期失败
    }());
}

TEST_CASE("LibModbusMaster - 反复 open/close 循环无崩溃", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    for (int i = 0; i < 5; ++i) {
        m.open();   // 失败
        m.close();  // 幂等
        CHECK_FALSE(m.is_open());
    }
    SUCCEED("重复 open/close 循环无崩溃");
}

// ── set_timeout_ms ──────────────────────────────────────────────────────

TEST_CASE("LibModbusMaster - set_timeout_ms 在 ctx_==nullptr 时安全（提前返回）",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    // ctx_==nullptr → 提前返回，不崩溃
    REQUIRE_NOTHROW(m.set_timeout_ms(500));
    REQUIRE_NOTHROW(m.set_timeout_ms(100));
    REQUIRE_NOTHROW(m.set_timeout_ms(0));  // 边界：实现应安全处理
}

// ── ctx_==nullptr 时 I/O 操作返回 DISCONNECTED ──────────────────────────

TEST_CASE("LibModbusMaster - ctx_=null 时 read_registers 返回 -1 + DISCONNECTED",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    uint16_t out[10];
    // 合法入参，但 ctx_==nullptr → execute_with_retry 返回 DISCONNECTED
    CHECK(m.read_registers(1, 0, 10, out) == -1);
    CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
}

TEST_CASE("LibModbusMaster - ctx_=null 时 write_register 返回 -1 + DISCONNECTED",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    CHECK(m.write_register(1, 0, 0) == -1);
    CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
}

TEST_CASE("LibModbusMaster - ctx_=null 时 write_registers 返回 -1 + DISCONNECTED",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    uint16_t vals[4] = {0};
    CHECK(m.write_registers(1, 0, 4, vals) == -1);
    CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
}

// ── read_registers 入参边界校验 ─────────────────────────────────────────

TEST_CASE("LibModbusMaster - read_registers 入参边界校验", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    uint16_t out[130] = {};

    SECTION("count=0 → SYS_ERROR（Modbus 规范：单次至少读 1 个）") {
        CHECK(m.read_registers(1, 0, 0, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=-1 → SYS_ERROR（负数等同越界）") {
        CHECK(m.read_registers(1, 0, -1, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=126 → SYS_ERROR（超过 FC03 上限 125）") {
        CHECK(m.read_registers(1, 0, 126, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=INT_MAX → SYS_ERROR（极端越界）") {
        CHECK(m.read_registers(1, 0, 0x7FFFFFFF, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("addr=-1 → SYS_ERROR（寄存器地址不可为负）") {
        CHECK(m.read_registers(1, -1, 10, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("out=nullptr → SYS_ERROR（空缓冲区）") {
        CHECK(m.read_registers(1, 0, 10, nullptr) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=1 边界合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        uint16_t buf[1];
        CHECK(m.read_registers(1, 0, 1, buf) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }

    SECTION("count=125 上限合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        CHECK(m.read_registers(1, 0, 125, out) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }

    SECTION("addr=0 最小合法地址（通过参数检查，ctx_=null → DISCONNECTED）") {
        uint16_t buf[1];
        CHECK(m.read_registers(1, 0, 1, buf) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }
}

// ── write_register 入参边界校验 ─────────────────────────────────────────

TEST_CASE("LibModbusMaster - write_register 入参边界校验", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);

    SECTION("addr=-1 → SYS_ERROR") {
        CHECK(m.write_register(1, -1, 0) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("addr=-100 → SYS_ERROR") {
        CHECK(m.write_register(1, -100, 0xFFFF) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("addr=0 合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        CHECK(m.write_register(1, 0, 0) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }

    SECTION("val=0xFFFF 极大值合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        CHECK(m.write_register(1, 100, 0xFFFF) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }
}

// ── write_registers 入参边界校验 ────────────────────────────────────────

TEST_CASE("LibModbusMaster - write_registers 入参边界校验", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    uint16_t vals[130] = {};

    SECTION("count=0 → SYS_ERROR") {
        CHECK(m.write_registers(1, 0, 0, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=-1 → SYS_ERROR") {
        CHECK(m.write_registers(1, 0, -1, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=124 → SYS_ERROR（超过 FC16 上限 123）") {
        CHECK(m.write_registers(1, 0, 124, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("addr=-1 → SYS_ERROR") {
        CHECK(m.write_registers(1, -1, 4, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("vals=nullptr → SYS_ERROR") {
        CHECK(m.write_registers(1, 0, 4, nullptr) == -1);
        CHECK(m.get_last_error() == ModbusResult::SYS_ERROR);
    }

    SECTION("count=1 边界合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        uint16_t v[1] = {0xABCD};
        CHECK(m.write_registers(1, 0, 1, v) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }

    SECTION("count=123 上限合法（通过参数检查，ctx_=null → DISCONNECTED）") {
        CHECK(m.write_registers(1, 0, 123, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }

    SECTION("addr=0 + count=123 + vals valid（通过参数检查，ctx_=null → DISCONNECTED）") {
        CHECK(m.write_registers(1, 0, 123, vals) == -1);
        CHECK(m.get_last_error() == ModbusResult::DISCONNECTED);
    }
}

// ── slave_id 边界说明 ────────────────────────────────────────────────────
// NOTE: slave_id 校验在 execute_with_retry 中只有 ctx_!=nullptr 时才能到达。
//       无 ctx_ 时直接返回 DISCONNECTED，无法测试 slave_id=-1/248 触发的 SYS_ERROR。
//       此路径须在硬件集成测试或通过真实串口 stub 测试。

// ═══════════════════════════════════════════════════════════════════════
//  线程安全测试（全部在关闭状态下并发，无需硬件）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LibModbusMaster - 多线程并发 read_registers（bus_mutex_ 串行化）无崩溃",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    constexpr int THREADS = 8;
    constexpr int ITERS = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            uint16_t out[10];
            for (int j = 0; j < ITERS; ++j)
                m.read_registers(1, 0, 10, out);  // ctx_=null → DISCONNECTED，无崩溃
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 read_registers 无崩溃");
}

TEST_CASE("LibModbusMaster - 多线程并发 write_register 无崩溃", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    constexpr int THREADS = 8;
    constexpr int ITERS = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j)
                m.write_register(1, 0, static_cast<uint16_t>(j & 0xFFFF));
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 write_register 无崩溃");
}

TEST_CASE("LibModbusMaster - 并发 close + read_registers 无崩溃（bus_mutex_ 保护）",
          "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    constexpr int THREADS = 4;
    constexpr int ITERS = 1000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    m.close();  // bus_mutex_ 保护，幂等
            });
        } else {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                uint16_t out[10];
                for (int j = 0; j < ITERS; ++j)
                    m.read_registers(1, 0, 10, out);  // bus_mutex_ 保护
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 close+read_registers 无崩溃");
}

TEST_CASE("LibModbusMaster - 并发 set_timeout_ms + read_registers 无崩溃", "[driver][modbus]") {
    driver::LibModbusMaster m(FAKE_MODBUS_PORT);
    constexpr int THREADS = 4;
    constexpr int ITERS = 1000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    m.set_timeout_ms(100 + (j % 5) * 100);  // ctx_=null → 提前返回
            });
        } else {
            threads.emplace_back([&]() {
                while (!go.load(std::memory_order_acquire)) {
                }
                uint16_t out[10];
                for (int j = 0; j < ITERS; ++j)
                    m.read_registers(1, 0, 10, out);
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 set_timeout_ms+read_registers 无崩溃");
}
