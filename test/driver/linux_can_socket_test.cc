/*
 * LinuxCanSocket 驱动层单元测试
 *
 * 测试分两类：
 *   [driver][can_socket]           — 纯软件错误路径（无需真实 CAN 硬件，任何 Linux 主机可运行）
 *   [driver][can_socket][hardware] — 硬件集成测试（需要目标板上的 can0/can1 接口）
 *
 * 运行方式：
 *   仅软件测试：./unit_tests "[driver][can_socket]" ~[hardware]
 *   含硬件测试：./unit_tests "[driver][can_socket]"
 *
 * 测试覆盖：
 *   - 生命周期（构造/析构/open/close 幂等性）
 *   - 未打开状态下所有 I/O 操作的快速失败语义
 *   - 帧长度边界（len > 8 内部截断，未打开时短路返回 false）
 *   - filter_mutex_ 保护的并发 set_filters / clear_filter
 *   - 多线程并发读（atomic 状态无数据竞争）
 *   - 多线程并发 send（全部应快速失败，不崩溃）
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/driver/linux_can_socket.h"

using namespace robot;
using namespace robot::hal;

// ═══════════════════════════════════════════════════════════════════════
//  纯软件路径测试（不访问真实 CAN 硬件）
// ═══════════════════════════════════════════════════════════════════════

// ── 生命周期 ────────────────────────────────────────────────────────────

TEST_CASE("LinuxCanSocket - 构造后默认未打开且初始状态正确", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CHECK_FALSE(sock.is_open());
    CHECK_FALSE(sock.is_bus_off());
    CHECK(sock.get_last_error() == CanResult::OK);
    CHECK(sock.get_tx_drop_count() == 0u);
}

TEST_CASE("LinuxCanSocket - 无效接口 open 返回 false 且 is_open 保持 false",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_invalid_99");
    CHECK_FALSE(sock.open());
    CHECK_FALSE(sock.is_open());
}

TEST_CASE("LinuxCanSocket - 未打开时 close 幂等安全", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    REQUIRE_NOTHROW(sock.close());
    REQUIRE_NOTHROW(sock.close());  // 二次不崩溃
    CHECK_FALSE(sock.is_open());
}

TEST_CASE("LinuxCanSocket - open 失败后 close 安全（资源已被清理）", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_invalid_99");
    sock.open();  // 预期失败
    REQUIRE_NOTHROW(sock.close());
    CHECK_FALSE(sock.is_open());
}

TEST_CASE("LinuxCanSocket - 析构不崩溃（从不打开）", "[driver][can_socket]") {
    REQUIRE_NOTHROW([]() { driver::LinuxCanSocket sock("can_not_real"); }());
}

TEST_CASE("LinuxCanSocket - 析构不崩溃（open 失败后）", "[driver][can_socket]") {
    REQUIRE_NOTHROW([]() {
        driver::LinuxCanSocket sock("can_invalid_99");
        sock.open();  // 预期失败，析构时做幂等 close
    }());
}

// ── 未打开状态下的 I/O 操作 ─────────────────────────────────────────────

TEST_CASE("LinuxCanSocket - 未打开时 send 返回 false", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFrame frm;
    frm.id = 0x100;
    frm.len = 8;
    CHECK_FALSE(sock.send(frm));
}

TEST_CASE("LinuxCanSocket - 未打开时 recv 立即返回 false 且设置 SYS_ERROR",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFrame frm;
    CHECK_FALSE(sock.recv(frm, 0));
    CHECK(sock.get_last_error() == CanResult::SYS_ERROR);
}

TEST_CASE("LinuxCanSocket - 未打开时 set_filters 返回 false 且设置 SYS_ERROR",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFilter f{0x100, 0x7FF};
    CHECK_FALSE(sock.set_filters(&f, 1));
    CHECK(sock.get_last_error() == CanResult::SYS_ERROR);
}

TEST_CASE("LinuxCanSocket - 未打开时 clear_filter 返回 false 且设置 SYS_ERROR",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CHECK_FALSE(sock.clear_filter());
    CHECK(sock.get_last_error() == CanResult::SYS_ERROR);
}

TEST_CASE("LinuxCanSocket - 未打开时 recover 返回 false", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    // bus_off_ 初始 false，recover() 应快速失败不崩溃
    CHECK_FALSE(sock.recover());
}

// ── 帧长度边界 ──────────────────────────────────────────────────────────

TEST_CASE("LinuxCanSocket - len=0 的帧 send 未打开时返回 false 不崩溃", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFrame frm;
    frm.id = 0x1;
    frm.len = 0;
    CHECK_FALSE(sock.send(frm));
}

TEST_CASE("LinuxCanSocket - len=255 超长帧 send 未打开时返回 false 不崩溃（内部截断路径不可达）",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFrame frm;
    frm.id = 0x1;
    frm.len = 255;  // 内部 safe_len = min(255, 8) = 8；但未打开，直接 fd<0 返回
    CHECK_FALSE(sock.send(frm));
}

// ── set_filters 特殊参数 ────────────────────────────────────────────────

TEST_CASE("LinuxCanSocket - set_filters(nullptr, 0) 未打开时安全（early-return 路径）",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    // fd<0 → SYS_ERROR → false，不会到达 nullptr 分支
    bool ret = false;
    REQUIRE_NOTHROW([&] { ret = sock.set_filters(nullptr, 0); }());
    CHECK_FALSE(ret);
}

TEST_CASE("LinuxCanSocket - set_filters(valid, 0) 语义同 clear_filter（未打开时安全）",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    CanFilter f{0x100, 0x7FF};
    // count=0 会在有效 fd 时委托给 clear_filter；未打开则 fd<0 直接 false
    REQUIRE_NOTHROW(sock.set_filters(&f, 0));
}

// ═══════════════════════════════════════════════════════════════════════
//  线程安全测试（全部在关闭状态下并发，无需硬件）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LinuxCanSocket - 多线程并发读 atomic 状态无崩溃", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    constexpr int THREADS = 8;
    constexpr int ITERS = 20000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j) {
                (void)sock.is_open();
                (void)sock.is_bus_off();
                (void)sock.get_last_error();
                (void)sock.get_tx_drop_count();
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发读 atomic 状态无崩溃");
}

TEST_CASE("LinuxCanSocket - 多线程并发 send 均快速失败（返回 false）无崩溃",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    constexpr int THREADS = 8;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::atomic<int> false_count{0};
    std::vector<std::thread> threads;

    CanFrame frm;
    frm.id = 0x200;
    frm.len = 4;
    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j) {
                if (!sock.send(frm))
                    false_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();

    // 全部应返回 false（未打开状态）
    CHECK(false_count.load() == THREADS * ITERS);
}

TEST_CASE("LinuxCanSocket - 多线程并发 set_filters / clear_filter（filter_mutex_ 保护）无崩溃",
          "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    constexpr int THREADS = 4;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    const CanFilter filters[3] = {{0x100, 0x7FF}, {0x200, 0x7FF}, {0x300, 0x7FF}};
    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    sock.set_filters(filters, 3);  // 未打开：快速 false，不崩溃
            });
        } else {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    sock.clear_filter();  // 未打开：快速 false，不崩溃
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 set_filters/clear_filter 无崩溃");
}

TEST_CASE("LinuxCanSocket - 多线程并发 recv（未打开即时返回）无崩溃", "[driver][can_socket]") {
    driver::LinuxCanSocket sock("can_not_real");
    constexpr int THREADS = 4;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            CanFrame frm;
            for (int j = 0; j < ITERS; ++j)
                sock.recv(frm, 0);  // 未打开：立即返回 false
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 recv（未打开）无崩溃");
}

TEST_CASE("LinuxCanSocket - 快速重复 open/close 生命周期循环无崩溃", "[driver][can_socket]") {
    // open 预期失败（接口不存在），close 是幂等的；循环执行确认无资源泄漏/崩溃
    constexpr int CYCLES = 20;
    for (int i = 0; i < CYCLES; ++i) {
        driver::LinuxCanSocket sock("can_invalid_99");
        sock.open();  // 失败
        REQUIRE_NOTHROW(sock.close());
    }
    SUCCEED("重复 open/close 生命周期无崩溃");
}
