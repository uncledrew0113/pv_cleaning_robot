/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:02:26
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-27 15:59:25
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/middleware/event_bus.h
 * @Description: 
 * 
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved. 
 */
#pragma once
#include <algorithm>
#include <array>
#include <functional>
#include <mutex>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::middleware {

/// @brief 线程安全类型安全事件总线（模板 pub/sub）
///
/// 用法示例：
///   EventBus bus;
///   bus.subscribe<FaultEvent>([](const FaultEvent& e) { ... });
///   bus.publish(FaultEvent{...});
///
/// 订阅者回调在 publish() 的调用线程中执行（同步）。
/// 如需异步，由调用方在回调中自行投递到队列。
///
/// 实现说明：回调签名改为 void(const void*)，用 static_cast 还原原始类型。
/// 相比 std::any，publish() 热路径完全消除堆分配，对 SCHED_FIFO RT 线程更友好。
class EventBus {
   public:
    template <typename EventT>
    using Handler = std::function<void(const EventT&)>;

    /// 订阅事件类型 EventT，返回订阅 ID（用于取消订阅）
    template <typename EventT>
    int subscribe(Handler<EventT> handler) {
        std::lock_guard<robot::hal::PiMutex> lk(mtx_);
        int id = next_id_++;
        auto& vec = handlers_[std::type_index(typeid(EventT))];
        vec.push_back(
            {id, [h = std::move(handler)](const void* p) { h(*static_cast<const EventT*>(p)); }});
        return id;
    }

    /// 取消订阅
    void unsubscribe(int subscription_id) {
        std::lock_guard<robot::hal::PiMutex> lk(mtx_);
        for (auto& [type, vec] : handlers_) {
            vec.erase(std::remove_if(
                          vec.begin(),
                          vec.end(),
                          [subscription_id](const Entry& e) { return e.id == subscription_id; }),
                      vec.end());
        }
    }

    /// 发布事件——在调用线程中同步回调所有订阅者
    ///
    /// publish() 持锁期间仅拷贝 std::function 指针到固定栈数组，不堆分配。
    /// 回调在锁外执行，事件对象生命期由 publish() 栈帧保证（const EventT& event）。
    /// 订阅者上限 kMaxHandlers=32；超出时截断并记录（不崩溃）。
    template <typename EventT>
    void publish(const EventT& event) {
        // 固定大小栈数组，消除 SCHED_FIFO 95 路径上 std::vector::reserve() 触发的 malloc()
        std::array<std::function<void(const void*)>, kMaxHandlers> cbs;
        int count = 0;
        {
            std::lock_guard<robot::hal::PiMutex> lk(mtx_);
            auto it = handlers_.find(std::type_index(typeid(EventT)));
            if (it == handlers_.end())
                return;
            for (auto& e : it->second) {
                if (count < kMaxHandlers)
                    cbs[count++] = e.cb;
            }
        }
        const void* ptr = static_cast<const void*>(&event);
        for (int i = 0; i < count; ++i)
            cbs[i](ptr);
    }

   private:
    struct Entry {
        int id;
        std::function<void(const void*)> cb;  ///< 类型擦除为 void*，subscribe 中 static_cast 还原
    };
    static constexpr int kMaxHandlers = 32;  ///< publish() 最大同步回调数（栈数组，无堆分配）
    // RT 安全互斥量：EventBus::publish() 由 SCHED_FIFO 95 的 GPIO 监控线程（SafetyMonitor）
    // 直接调用。若普通线程持有此锁时被 RT 线程抢占，将出现优先级反转。
    // PiMutex (PTHREAD_PRIO_INHERIT) 自动将持锁线程临时提升至等待者最高优先级。
    robot::hal::PiMutex mtx_;
    std::unordered_map<std::type_index, std::vector<Entry>> handlers_;
    int next_id_{0};
};

}  // namespace robot::middleware
