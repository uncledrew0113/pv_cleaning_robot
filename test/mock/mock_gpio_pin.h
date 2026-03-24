#pragma once
#include "pv_cleaning_robot/hal/i_gpio_pin.h"
#include <functional>

/// @brief IGpioPin 手工 mock，用于单元测试，不依赖真实硬件
struct MockGpioPin : robot::hal::IGpioPin {
    // ── 可配置的返回值 ──────────────────────────────────────
    bool open_result{true};
    bool read_result{false};
    bool write_result{true};

    // ── 调用记录 ───────────────────────────────────────────
    bool opened{false};
    bool monitoring{false};
    robot::hal::GpioConfig  last_open_config;
    robot::hal::GpioEdge    registered_edge{robot::hal::GpioEdge::BOTH};
    std::function<void()>   registered_cb;
    bool write_value_called{false};
    bool last_written_value{false};

    bool open(const robot::hal::GpioConfig& config) override {
        last_open_config = config;
        opened = open_result;
        return open_result;
    }
    void close() override { opened = false; monitoring = false; }
    bool is_open() const override { return opened; }

    bool read_value() override { return read_result; }

    bool write_value(bool high) override {
        write_value_called = true;
        last_written_value = high;
        return write_result;
    }

    void set_edge_callback(robot::hal::GpioEdge edge, std::function<void()> cb) override {
        registered_edge = edge;
        registered_cb   = std::move(cb);
    }
    void start_monitoring() override { monitoring = true; }
    void stop_monitoring()  override { monitoring = false; }

    /// 辅助函数：模拟一次边沿触发
    void simulate_edge() { if (registered_cb) registered_cb(); }
};
