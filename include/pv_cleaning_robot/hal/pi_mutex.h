/*
 * @Author: UncleDrew
 * @Date: 2026-03-24 09:26:08
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-24 10:49:29
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/hal/pi_mutex.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <stdexcept>

namespace robot::hal {

/// @brief 支持优先级继承（PI）和持有者死亡恢复（Robust）的互斥量
///
/// 工业场景价值：
///   - PTHREAD_PRIO_INHERIT：消除 RT 线程与普通线程之间的优先级反转
///   - PTHREAD_MUTEX_ROBUST：当持锁线程被信号杀死或异常终止时，
///     其他等待线程不会永久死锁，而是收到 EOWNERDEAD 并可恢复
///
/// 使用约束：
///   - lock() 返回 false 表示前任持有者已死亡，被保护的共享数据可能处于
///     不一致状态。调用方**必须**在使用数据前执行修复/重置逻辑。
///   - 若调用方无法修复数据，应立即释放锁并上报故障，禁止继续操作。
class PiMutex {
   public:
    PiMutex() {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        // 优先级继承：持锁线程临时继承等待线程的最高优先级
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        // Robust：持有者异常终止后，等待者收到 EOWNERDEAD 而非永久死锁
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);

        int rc = pthread_mutex_init(&mutex_, &attr);
        pthread_mutexattr_destroy(&attr);
        if (rc != 0) {
            throw std::runtime_error(std::string("[PiMutex] pthread_mutex_init failed: ") +
                                     std::strerror(rc));
        }
    }

    ~PiMutex() {
        pthread_mutex_destroy(&mutex_);
    }

    PiMutex(const PiMutex&) = delete;
    PiMutex& operator=(const PiMutex&) = delete;

    // 适配 std::lock_guard，必须返回 void
    void lock() {
        int rc = pthread_mutex_lock(&mutex_);
        if (rc == 0) {
            return;  // 正常加锁
        }

        if (rc == EOWNERDEAD) {
            // 前任持有者已死亡，内核将锁的所有权转交。
            // 标记为 consistent 防止后续永远 ENOTRECOVERABLE，
            // 并通过异常强制打断 lock_guard，防止操作可能已损坏的共享状态。
            pthread_mutex_consistent(&mutex_);
            throw std::runtime_error(
                "[PiMutex] lock acquired but previous owner died. State inconsistent!");
        }

        throw std::runtime_error(std::string("[PiMutex] pthread_mutex_lock failed: ") +
                                 std::strerror(rc));
    }

    void unlock() {
        int rc = pthread_mutex_unlock(&mutex_);
        if (rc != 0) {
            throw std::runtime_error(std::string("[PiMutex] pthread_mutex_unlock failed: ") +
                                     std::strerror(rc));
        }
    }

    bool try_lock() {
        int rc = pthread_mutex_trylock(&mutex_);
        if (rc == 0)
            return true;

        if (rc == EOWNERDEAD) {
            pthread_mutex_consistent(&mutex_);
            throw std::runtime_error(
                "[PiMutex] trylock acquired but previous owner died. State inconsistent!");
        }
        return false;
    }

   private:
    pthread_mutex_t mutex_{};
};

}  // namespace robot::hal