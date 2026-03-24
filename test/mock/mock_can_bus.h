#pragma once
#include "pv_cleaning_robot/hal/i_can_bus.h"
#include <vector>

/// @brief ICanBus 手工 mock，用于单元测试，不依赖真实硬件
struct MockCanBus : robot::hal::ICanBus {
    // ── 可配置的返回值 ──────────────────────────────────────
    bool open_result{true};
    bool send_result{true};
    bool recv_result{false};
    robot::hal::CanFrame recv_frame{};
    bool recover_result{true};

    // ── 可注入的错误状态 ────────────────────────────────────
    robot::hal::CanResult injected_error{robot::hal::CanResult::OK};
    bool injected_bus_off{false};

    // ── 调用记录 ───────────────────────────────────────────
    bool opened{false};
    bool closed{false};
    int  recv_call_count{0};
    std::vector<robot::hal::CanFrame>  sent_frames;
    std::vector<robot::hal::CanFilter> last_set_filters;
    bool clear_filter_called{false};
    bool recover_called{false};

    bool open() override {
        if (open_result) opened = true;
        injected_error = robot::hal::CanResult::OK;
        return open_result;
    }
    void close() override { opened = false; closed = true; }
    bool is_open() const override { return opened; }

    bool send(const robot::hal::CanFrame& f) override {
        sent_frames.push_back(f);
        return send_result;
    }
    bool recv(robot::hal::CanFrame& frame, int) override {
        ++recv_call_count;
        if (recv_result) { frame = recv_frame; injected_error = robot::hal::CanResult::OK; }
        else              { injected_error = robot::hal::CanResult::TIMEOUT; }
        return recv_result;
    }

    // 使基类的 vector 便利重载可见，解除 C++ 名称隐藏
    using robot::hal::ICanBus::set_filters;

    bool set_filters(const robot::hal::CanFilter* filters, size_t count) override {
        last_set_filters.assign(filters, filters + count);
        return true;
    }
    bool clear_filter() override { clear_filter_called = true; return true; }

    robot::hal::CanResult get_last_error() const override { return injected_error; }
    bool is_bus_off() const override { return injected_bus_off; }
    bool recover() override {
        recover_called = true;
        if (recover_result) { opened = true; injected_bus_off = false; injected_error = robot::hal::CanResult::OK; }
        return recover_result;
    }
};
