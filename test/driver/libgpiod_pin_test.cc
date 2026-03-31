/*
 * LibGpiodPin 驱动层集成测试
 *
 * 测试分两类：
 *   [driver][gpio]           — 纯软件错误路径（无需真实硬件，任何 Linux 主机可运行）
 *   [driver][gpio][hardware] — 硬件集成测试（需在目标板运行，修改下方 TARGET_* 常量）
 *
 * 硬件测试前提：
 *   1. TARGET_IN_LINE  和 TARGET_OUT_LINE 在板上短接（写输出→读输入可互验）
 *   2. 以有权限访问 /dev/gpiochipX 的用户身份运行
 *
 * 运行方式：
 *   仅软件测试：./unit_tests "[driver][gpio]" ~[hardware]
 *   含硬件测试：./unit_tests "[driver][gpio]"
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <thread>

#include "pv_cleaning_robot/driver/libgpiod_pin.h"

using namespace robot;
using namespace std::chrono_literals;

// ── 硬件测试常量（按目标板实际 GPIO 布局修改）────────────────────────────
static constexpr const char* TARGET_CHIP = "gpiochip0";
static constexpr unsigned int TARGET_IN_LINE = 10;   ///< 输入引脚编号
static constexpr unsigned int TARGET_OUT_LINE = 11;  ///< 输出引脚编号（与 IN 短接）

// ═══════════════════════════════════════════════════════════════════════
//  纯软件路径测试（不访问真实 GPIO 硬件）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LibGpiodPin - 构造后默认未打开", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    CHECK_FALSE(pin.is_open());
}

TEST_CASE("LibGpiodPin - 无效芯片名 open 返回 false 且不崩溃", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);

    SECTION("INPUT 方向") {
        hal::GpioConfig cfg;
        cfg.direction = hal::GpioDirection::INPUT;
        CHECK_FALSE(pin.open(cfg));
        CHECK_FALSE(pin.is_open());
    }

    SECTION("OUTPUT 方向") {
        hal::GpioConfig cfg;
        cfg.direction = hal::GpioDirection::OUTPUT;
        CHECK_FALSE(pin.open(cfg));
        CHECK_FALSE(pin.is_open());
    }
}

TEST_CASE("LibGpiodPin - 未打开时 close 幂等安全", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    REQUIRE_NOTHROW(pin.close());
    REQUIRE_NOTHROW(pin.close());  // 二次 close 不崩溃
    CHECK_FALSE(pin.is_open());
}

TEST_CASE("LibGpiodPin - 未打开时 read_value 返回 false", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    CHECK_FALSE(pin.read_value());
}

TEST_CASE("LibGpiodPin - 未打开时 write_value 返回 false", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    CHECK_FALSE(pin.write_value(true));
    CHECK_FALSE(pin.write_value(false));
}

TEST_CASE("LibGpiodPin - 未打开时 stop_monitoring 安全", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    REQUIRE_NOTHROW(pin.stop_monitoring());
}

TEST_CASE("LibGpiodPin - open 前 set_edge_callback 安全", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    // 注册回调不依赖 is_open()，应纯粹写内存不崩溃
    REQUIRE_NOTHROW(pin.set_edge_callback(hal::GpioEdge::RISING, []() {}));
    REQUIRE_NOTHROW(pin.set_edge_callback(hal::GpioEdge::FALLING, []() {}));
    REQUIRE_NOTHROW(pin.set_edge_callback(hal::GpioEdge::BOTH, []() {}));
}

TEST_CASE("LibGpiodPin - 未打开时 start_monitoring 立即返回不崩溃", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    pin.set_edge_callback(hal::GpioEdge::RISING, []() {});
    // is_open() 为 false，期望提前返回，不启动线程
    REQUIRE_NOTHROW(pin.start_monitoring());
    REQUIRE_NOTHROW(pin.stop_monitoring());
}

TEST_CASE("LibGpiodPin - 析构时无监控线程不崩溃", "[driver][gpio]") {
    REQUIRE_NOTHROW([] {
        driver::LibGpiodPin pin("nonexistent_chip", 0);
        // 未 open，析构应是无操作
    }());
}

TEST_CASE("LibGpiodPin - 析构时监控未启动也不崩溃", "[driver][gpio]") {
    REQUIRE_NOTHROW([] {
        driver::LibGpiodPin pin("nonexistent_chip", 0);
        pin.set_edge_callback(hal::GpioEdge::BOTH, []() {});
        // 未 open，start_monitoring 内部会提前返回
        pin.start_monitoring();
        // 析构
    }());
}

// ═══════════════════════════════════════════════════════════════════════
//  硬件集成测试（需在目标板运行，用 TARGET_* 常量对应真实引脚）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LibGpiodPin 硬件 - 输入引脚生命周期", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_IN_LINE, "test_in_lifecycle");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    cfg.debounce_ms = 0;

    SECTION("open 成功后 is_open 为 true") {
        REQUIRE(pin.open(cfg));
        CHECK(pin.is_open());
    }

    SECTION("close 后 is_open 为 false") {
        pin.open(cfg);
        pin.close();
        CHECK_FALSE(pin.is_open());
    }

    SECTION("close 后重新 open 成功") {
        REQUIRE(pin.open(cfg));
        pin.close();
        REQUIRE(pin.open(cfg));
        CHECK(pin.is_open());
    }
}

TEST_CASE("LibGpiodPin 硬件 - 上拉输入悬空读高电平", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_IN_LINE, "test_in_read");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    REQUIRE(pin.open(cfg));

    // 悬空 + 上拉：应读高（true）
    CHECK(pin.read_value() == true);
}

TEST_CASE("LibGpiodPin 硬件 - 输出引脚写值返回 true", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_OUT_LINE, "test_out_write");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::OUTPUT;
    REQUIRE(pin.open(cfg));

    CHECK(pin.write_value(true));
    CHECK(pin.write_value(false));
    CHECK(pin.write_value(true));  // 连续多次不崩溃
}

TEST_CASE("LibGpiodPin 硬件 - 输出写值可被输入引脚读取（短接验证）", "[driver][gpio][hardware]") {
    driver::LibGpiodPin in_pin(TARGET_CHIP, TARGET_IN_LINE, "test_io_in");
    driver::LibGpiodPin out_pin(TARGET_CHIP, TARGET_OUT_LINE, "test_io_out");

    hal::GpioConfig in_cfg;
    in_cfg.direction = hal::GpioDirection::INPUT;
    in_cfg.bias = hal::GpioBias::DISABLE;  // 不加偏置，完全由输出侧驱动

    hal::GpioConfig out_cfg;
    out_cfg.direction = hal::GpioDirection::OUTPUT;

    REQUIRE(in_pin.open(in_cfg));
    REQUIRE(out_pin.open(out_cfg));

    SECTION("输出高 → 输入读高") {
        out_pin.write_value(true);
        std::this_thread::sleep_for(5ms);  // 等电平稳定
        CHECK(in_pin.read_value() == true);
    }

    SECTION("输出低 → 输入读低") {
        out_pin.write_value(false);
        std::this_thread::sleep_for(5ms);
        CHECK(in_pin.read_value() == false);
    }
}

TEST_CASE("LibGpiodPin 硬件 - 输入模式禁止 write_value", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_IN_LINE, "test_write_guard");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    REQUIRE(pin.open(cfg));

    // 输入模式写值应有保护，返回 false 且不崩溃
    CHECK_FALSE(pin.write_value(true));
    CHECK_FALSE(pin.write_value(false));
    // 保护后引脚仍然正常
    CHECK(pin.is_open());
}

TEST_CASE("LibGpiodPin 硬件 - 输出模式 start_monitoring 被安全忽略", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_OUT_LINE, "test_monitor_output");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::OUTPUT;
    REQUIRE(pin.open(cfg));

    pin.set_edge_callback(hal::GpioEdge::BOTH, []() {});
    REQUIRE_NOTHROW(pin.start_monitoring());
    REQUIRE_NOTHROW(pin.stop_monitoring());

    // 监控被忽略后输出引脚仍可正常使用
    CHECK(pin.write_value(false));
    CHECK(pin.is_open());
}

TEST_CASE("LibGpiodPin 硬件 - 边沿回调在翻转后触发", "[driver][gpio][hardware]") {
    driver::LibGpiodPin in_pin(TARGET_CHIP, TARGET_IN_LINE, "test_edge_in");
    driver::LibGpiodPin out_pin(TARGET_CHIP, TARGET_OUT_LINE, "test_edge_out");

    hal::GpioConfig in_cfg;
    in_cfg.direction = hal::GpioDirection::INPUT;
    in_cfg.bias = hal::GpioBias::DISABLE;
    in_cfg.debounce_ms = 0;

    hal::GpioConfig out_cfg;
    out_cfg.direction = hal::GpioDirection::OUTPUT;

    REQUIRE(in_pin.open(in_cfg));
    REQUIRE(out_pin.open(out_cfg));

    std::atomic<int> trigger_count{0};
    in_pin.set_edge_callback(hal::GpioEdge::BOTH, [&trigger_count]() {
        trigger_count.fetch_add(1, std::memory_order_relaxed);
    });
    in_pin.start_monitoring();

    std::this_thread::sleep_for(20ms);
    out_pin.write_value(true);  // 上升沿
    std::this_thread::sleep_for(30ms);
    out_pin.write_value(false);  // 下降沿
    std::this_thread::sleep_for(50ms);

    in_pin.stop_monitoring();

    // 两次翻转至少应触发 1 次（允许驱动调度有少量延迟）
    CHECK(trigger_count.load() >= 1);
}

TEST_CASE("LibGpiodPin 硬件 - 消抖过滤窗口内快速抖动", "[driver][gpio][hardware]") {
    driver::LibGpiodPin in_pin(TARGET_CHIP, TARGET_IN_LINE, "test_debounce");
    driver::LibGpiodPin out_pin(TARGET_CHIP, TARGET_OUT_LINE, "test_debounce_out");

    hal::GpioConfig in_cfg;
    in_cfg.direction = hal::GpioDirection::INPUT;
    in_cfg.bias = hal::GpioBias::DISABLE;
    in_cfg.debounce_ms = 100;  // 100 ms 消抖窗口

    hal::GpioConfig out_cfg;
    out_cfg.direction = hal::GpioDirection::OUTPUT;

    REQUIRE(in_pin.open(in_cfg));
    REQUIRE(out_pin.open(out_cfg));

    std::atomic<int> count{0};
    in_pin.set_edge_callback(hal::GpioEdge::BOTH,
                             [&count]() { count.fetch_add(1, std::memory_order_relaxed); });
    in_pin.start_monitoring();

    // 在 100 ms 消抖窗口内快速翻转 10 次（每次间隔 5 ms）
    for (int i = 0; i < 10; ++i) {
        out_pin.write_value(i % 2 == 0);
        std::this_thread::sleep_for(5ms);
    }
    std::this_thread::sleep_for(200ms);  // 等消抖窗口彻底结束

    in_pin.stop_monitoring();

    // 10 次翻转因消抖被大量过滤，实际触发应 ≤ 2
    CHECK(count.load() <= 2);
}

TEST_CASE("LibGpiodPin 硬件 - 多次 start/stop 循环后引脚仍正常", "[driver][gpio][hardware]") {
    driver::LibGpiodPin pin(TARGET_CHIP, TARGET_IN_LINE, "test_restart");

    hal::GpioConfig cfg;
    cfg.direction = hal::GpioDirection::INPUT;
    cfg.bias = hal::GpioBias::PULL_UP;
    REQUIRE(pin.open(cfg));

    pin.set_edge_callback(hal::GpioEdge::BOTH, []() {});

    // start→stop 5 次循环：验证线程可反复创建/销毁
    for (int i = 0; i < 5; ++i) {
        REQUIRE_NOTHROW(pin.start_monitoring());
        std::this_thread::sleep_for(20ms);
        REQUIRE_NOTHROW(pin.stop_monitoring());
    }

    // 所有循环结束后引脚状态应完好
    CHECK(pin.is_open());
    CHECK(pin.read_value() == true);  // 上拉悬空 → 高
}

TEST_CASE("LibGpiodPin 硬件 - 析构时活跃监控线程自动被 join", "[driver][gpio][hardware]") {
    // 析构路径：close() → stop_monitoring() → monitor_thread_.join()
    // 不应死锁，也不应在 join 之前访问悬挂的 line_ 指针
    REQUIRE_NOTHROW([&]() {
        driver::LibGpiodPin pin(TARGET_CHIP, TARGET_IN_LINE, "test_dtor_hw");

        hal::GpioConfig cfg;
        cfg.direction = hal::GpioDirection::INPUT;
        cfg.bias = hal::GpioBias::PULL_UP;

        pin.open(cfg);
        pin.set_edge_callback(hal::GpioEdge::BOTH, []() {});
        pin.start_monitoring();
        std::this_thread::sleep_for(20ms);
        // 超出作用域：析构自动 join
    }());
}

// ═══════════════════════════════════════════════════════════════════════
//  线程安全补充测试（纯软件，无需硬件）
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("LibGpiodPin - 多线程并发 is_open 读取无数据竞争", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    constexpr int THREADS = 8;
    constexpr int ITERS = 20000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j)
                (void)pin.is_open();  // shared_mutex 保护，并发读无竞争
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 is_open 读取无崩溃");
}

TEST_CASE("LibGpiodPin - 多线程并发注册边沿回调（PiMutex cb_mutex_ 保护）", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    constexpr int THREADS = 6;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::atomic<int> invoke_count{0};
    std::vector<std::thread> threads;

    const hal::GpioEdge edges[] = {
        hal::GpioEdge::RISING, hal::GpioEdge::FALLING, hal::GpioEdge::BOTH};

    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&, i] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (int j = 0; j < ITERS; ++j) {
                // 多线程并发写 callbacks_ 数组，PiMutex 保证互斥
                pin.set_edge_callback(edges[i % 3], [&invoke_count]() {
                    invoke_count.fetch_add(1, std::memory_order_relaxed);
                });
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发注册回调无崩溃");
}

TEST_CASE("LibGpiodPin - 并发 set_edge_callback 与 start/stop_monitoring 无崩溃",
          "[driver][gpio]") {
    // 未打开时 start_monitoring 内部 is_open()==false 提前返回；
    // 并发注册回调与幂等 start/stop 不应崩溃。
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    constexpr int THREADS = 4;
    constexpr int ITERS = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j) {
                    pin.set_edge_callback(hal::GpioEdge::RISING, []() {});
                    pin.set_edge_callback(hal::GpioEdge::FALLING, []() {});
                }
            });
        } else {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j) {
                    pin.start_monitoring();  // is_open()==false → 立即返回
                    pin.stop_monitoring();   // 幂等
                }
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 set_callback+start/stop 无崩溃");
}

TEST_CASE("LibGpiodPin - stop_monitoring 重复调用幂等安全", "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    for (int i = 0; i < 10; ++i)
        REQUIRE_NOTHROW(pin.stop_monitoring());
}

TEST_CASE("LibGpiodPin - 多次实例化 open/close 循环（非硬件路径）无崩溃", "[driver][gpio]") {
    constexpr int CYCLES = 10;
    for (int i = 0; i < CYCLES; ++i) {
        driver::LibGpiodPin pin("nonexistent_chip", 0);
        hal::GpioConfig cfg;
        cfg.direction = hal::GpioDirection::INPUT;
        pin.open(cfg);  // 预期失败
        pin.close();    // 幂等
        CHECK_FALSE(pin.is_open());
    }
    SUCCEED("多次实例化 open/close 无崩溃");
}

TEST_CASE("LibGpiodPin - 多线程并发 read_value/write_value（未打开快速失败）无崩溃",
          "[driver][gpio]") {
    driver::LibGpiodPin pin("nonexistent_chip", 0);
    constexpr int THREADS = 4;
    constexpr int ITERS = 5000;
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;

    for (int i = 0; i < THREADS; ++i) {
        if (i % 2 == 0) {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    (void)pin.read_value();  // 未打开：shared_lock → false
            });
        } else {
            threads.emplace_back([&] {
                while (!go.load(std::memory_order_acquire)) {
                }
                for (int j = 0; j < ITERS; ++j)
                    pin.write_value(j % 2 == 0);  // 未打开：shared_lock → false
            });
        }
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();
    SUCCEED("并发 read_value/write_value（未打开）无崩溃");
}
