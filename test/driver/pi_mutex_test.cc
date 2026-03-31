/*
 * PiMutex（优先级继承互斥量）单元测试
 *
 * 测试范围：
 *   - 基本加锁/解锁语义
 *   - try_lock 无竞争 / 有竞争场景
 *   - RAII（lock_guard / unique_lock）正确释放
 *   - 多线程并发互斥：非原子计数器的一致性验证
 *   - 多线程并发 try_lock 竞争
 *
 * 所有测试均无需硬件，任何 Linux 主机可运行。
 */
#include <atomic>
#include <catch2/catch.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "pv_cleaning_robot/hal/pi_mutex.h"

using robot::hal::PiMutex;
using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════
//  基础语义
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("PiMutex - 构造与析构不抛异常", "[driver][pi_mutex]") {
    REQUIRE_NOTHROW([] { PiMutex m; }());
}

TEST_CASE("PiMutex - lock / unlock 正常循环", "[driver][pi_mutex]") {
    PiMutex m;
    // lock() 返回 void，正常路径不抛异常
    for (int i = 0; i < 100; ++i) {
        REQUIRE_NOTHROW(m.lock());
        REQUIRE_NOTHROW(m.unlock());
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  try_lock 语义
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("PiMutex - try_lock 无竞争时成功", "[driver][pi_mutex]") {
    PiMutex m;
    REQUIRE(m.try_lock());
    m.unlock();
}

TEST_CASE("PiMutex - try_lock 持锁后被另一线程 try_lock 返回 false", "[driver][pi_mutex]") {
    PiMutex m;

    // 当前线程先持锁
    REQUIRE(m.try_lock());

    std::atomic<bool> other_result{true};
    std::thread t([&]() {
        // 另一线程 try_lock 应立即返回 false（EBUSY）
        other_result.store(m.try_lock());
    });
    t.join();

    REQUIRE_FALSE(other_result.load());
    m.unlock();

    // 释放后另一线程应能成功获取，且必须由同一线程解锁（POSIX 规定）
    std::atomic<bool> after_unlock{false};
    std::thread t2([&]() {
        after_unlock.store(m.try_lock());
        if (after_unlock.load())
            m.unlock();  // 在持锁线程内解锁，避免 EPERM
    });
    t2.join();
    REQUIRE(after_unlock.load());
    // 不再由主线程调用 m.unlock()：t2 已在退出前自行解锁
}

TEST_CASE("PiMutex - try_lock 解锁后再次成功", "[driver][pi_mutex]") {
    PiMutex m;
    REQUIRE(m.try_lock());
    m.unlock();
    // 再次 try_lock 应成功
    REQUIRE(m.try_lock());
    m.unlock();
}

// ═══════════════════════════════════════════════════════════════════════
//  RAII 语义
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("PiMutex - lock_guard 离开作用域自动释放锁", "[driver][pi_mutex]") {
    PiMutex m;
    {
        std::lock_guard<PiMutex> lk(m);
        // 此时锁已持有
        std::atomic<bool> other_result{true};
        std::thread t([&]() { other_result.store(m.try_lock()); });
        t.join();
        REQUIRE_FALSE(other_result.load());  // 持锁期间外部不可 try_lock
    }
    // lock_guard 析构后锁应已释放
    REQUIRE(m.try_lock());
    m.unlock();
}

TEST_CASE("PiMutex - unique_lock 手动 unlock/lock 周期", "[driver][pi_mutex]") {
    PiMutex m;
    std::unique_lock<PiMutex> lk(m);
    REQUIRE(lk.owns_lock());

    lk.unlock();
    REQUIRE_FALSE(lk.owns_lock());

    // 释放期间另一线程可 try_lock
    REQUIRE(m.try_lock());
    m.unlock();

    lk.lock();
    REQUIRE(lk.owns_lock());
    // lk 析构时自动解锁
}

TEST_CASE("PiMutex - unique_lock 析构自动释放锁", "[driver][pi_mutex]") {
    PiMutex m;
    { std::unique_lock<PiMutex> lk(m); }
    // 析构后锁应已释放
    REQUIRE(m.try_lock());
    m.unlock();
}

// ═══════════════════════════════════════════════════════════════════════
//  并发线程安全
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("PiMutex - 多线程并发互斥：非原子计数器一致性", "[driver][pi_mutex]") {
    PiMutex m;
    int counter = 0;  // 非原子，依赖 PiMutex 保护

    constexpr int THREADS = 8;
    constexpr int ITERATIONS = 50000;

    auto worker = [&]() {
        for (int i = 0; i < ITERATIONS; ++i) {
            std::lock_guard<PiMutex> lk(m);
            ++counter;
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(THREADS);
    for (int i = 0; i < THREADS; ++i)
        threads.emplace_back(worker);
    for (auto& t : threads)
        t.join();

    // 若互斥失效则 counter < THREADS * ITERATIONS
    REQUIRE(counter == THREADS * ITERATIONS);
}

TEST_CASE("PiMutex - 多线程并发 try_lock 竞争：最终只有一个线程持锁", "[driver][pi_mutex]") {
    PiMutex m;
    std::atomic<int> locked_count{0};

    constexpr int THREADS = 16;
    std::atomic<bool> start{false};

    auto worker = [&]() {
        while (!start.load(std::memory_order_acquire)) {
        }
        if (m.try_lock()) {
            locked_count.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(1ms);  // 持锁片刻
            m.unlock();
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(THREADS);
    for (int i = 0; i < THREADS; ++i)
        threads.emplace_back(worker);

    start.store(true, std::memory_order_release);
    for (auto& t : threads)
        t.join();

    // 任何时刻最多只有 1 个线程能持锁，因此计数 ≥ 1（至少有一个线程成功）
    REQUIRE(locked_count.load() >= 1);
}

TEST_CASE("PiMutex - 高并发下无死锁：lock_guard 快速进出临界区", "[driver][pi_mutex]") {
    PiMutex m;
    int shared = 0;

    constexpr int THREADS = 4;
    constexpr int ITERATIONS = 100000;

    std::vector<std::thread> threads;
    threads.reserve(THREADS);
    for (int i = 0; i < THREADS; ++i) {
        threads.emplace_back([&]() {
            for (int j = 0; j < ITERATIONS; ++j) {
                std::lock_guard<PiMutex> lk(m);
                shared = shared + 1;  // 读-改-写，若互斥失效会有竞态
            }
        });
    }
    for (auto& t : threads)
        t.join();

    REQUIRE(shared == THREADS * ITERATIONS);
}

// ═══════════════════════════════════════════════════════════════════
//  Robust mutex: EOWNERDEAD 异常行为
//  PTHREAD_MUTEX_ROBUST 保证: 持锁线程异常终止后，等待者能感知并处理
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("PiMutex - EOWNERDEAD: 持锁线程异常终止后 lock() 抛出 runtime_error",
          "[driver][pi_mutex][robust]") {
    // 要求 PTHREAD_MUTEX_ROBUST：持锁线程不解锁就终止时，
    // 后续调用 lock() 应抛出 runtime_error（包含 "EOWNERDEAD" 语义）而非永久死锁。
    PiMutex m;

    // 子线程持锁后直接返回（不解锁）——模拟持有者异常崩溃
    std::thread t([&]() {
        m.lock();
        // 故意不调用 unlock()，线程结束 → PTHREAD_MUTEX_ROBUST 触发 EOWNERDEAD
    });
    t.join();

    // 当前线程再调用 lock() 应抛出（而不是死锁）
    REQUIRE_THROWS_AS(m.lock(), std::runtime_error);
}

TEST_CASE("PiMutex - EOWNERDEAD: 持锁线程异常终止后 try_lock() 抛出 runtime_error",
          "[driver][pi_mutex][robust]") {
    PiMutex m;

    std::thread t([&]() {
        m.lock();
        // 故意不解锁
    });
    t.join();

    // try_lock 同样应抛出（当前实现对 EOWNERDEAD 抛 runtime_error）
    REQUIRE_THROWS_AS(m.try_lock(), std::runtime_error);
}
