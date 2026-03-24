#include <cstring>
#include <gpiod.h>
#include <pthread.h>
#include <sched.h>
#include <spdlog/spdlog.h>

#include "pv_cleaning_robot/driver/libgpiod_pin.h"

namespace robot::driver {

namespace {
/// gpiod_line_event_wait 轮询超时。
/// 实际 GPIO 事件延迟由硬件中断决定，与此超时无关。
/// 此值仅影响 stop_monitoring() 的响应速度：最大 1s 可停止。
/// 设为 1000ms：10 个 GPIO 线程每秒唯一唤醒 10 次（原 200 次），降低 CPU 占用 20倍。
constexpr long kWaitTimeoutNs = 1000L * 1000L * 1000L;  // 1 s
}  // namespace

LibGpiodPin::LibGpiodPin(std::string chip_name, unsigned int line_num, std::string consumer)
    : chip_name_(std::move(chip_name))
    , line_num_(line_num)
    , consumer_(std::move(consumer))
    , last_event_time_(std::chrono::steady_clock::now()) {}

LibGpiodPin::~LibGpiodPin() {
    close();  // close 现在安全地包含了 stop_monitoring
}

bool LibGpiodPin::request_line_as_input() {
    struct gpiod_line_request_config cfg {};
    cfg.consumer = consumer_.c_str();
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;

    if (config_.bias == hal::GpioBias::PULL_UP) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (config_.bias == hal::GpioBias::PULL_DOWN) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
        spdlog::error("[LibGpiodPin] input line request failed for {}/{}: {}",
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        return false;
    }
    return true;
}

bool LibGpiodPin::open(const hal::GpioConfig& config) {
    // 允许以新配置重新打开：先安全释放旧资源，再重新申请
    if (is_open())
        close();

    config_ = config;

    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
        spdlog::error("[LibGpiodPin] gpiod_chip_open_by_name({}) failed", chip_name_);
        return false;
    }

    line_ = gpiod_chip_get_line(chip_, line_num_);
    if (!line_) {
        spdlog::error("[LibGpiodPin] gpiod_chip_get_line({}) failed", line_num_);
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        return false;
    }

    bool success = false;
    if (config_.direction == hal::GpioDirection::OUTPUT) {
        success = (gpiod_line_request_output(line_, consumer_.c_str(), 0) == 0);
        if (!success) {
            spdlog::error("[LibGpiodPin] output line request failed for {}/{}: {}",
                          chip_name_,
                          line_num_,
                          std::strerror(errno));
        }
    } else {
        success = request_line_as_input();
    }

    if (!success) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        return false;
    }

    spdlog::info("[LibGpiodPin] opened {}/{} as {} (Bias: {}, Debounce: {}ms)",
                 chip_name_,
                 line_num_,
                 config_.direction == hal::GpioDirection::OUTPUT ? "OUTPUT" : "INPUT",
                 static_cast<int>(config_.bias),
                 config_.debounce_ms);
    return true;
}

void LibGpiodPin::close() {
    // 修复 Bug：关闭前必须强制停止监控线程，防止指针悬挂导致段错误
    stop_monitoring();

    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        spdlog::debug("[LibGpiodPin] closed: {}/{}", chip_name_, line_num_);
    }
}

bool LibGpiodPin::is_open() const {
    return chip_ != nullptr && line_ != nullptr;
}

bool LibGpiodPin::read_value() {
    if (!is_open())
        return false;
    return gpiod_line_get_value(line_) == 1;
}

bool LibGpiodPin::write_value(bool high) {
    if (!is_open() || config_.direction != hal::GpioDirection::OUTPUT) {
        spdlog::warn(
            "[LibGpiodPin] write_value called on non-output pin {}/{}", chip_name_, line_num_);
        return false;
    }
    if (gpiod_line_set_value(line_, high ? 1 : 0) != 0) {
        spdlog::error("[LibGpiodPin] write_value({}) failed for {}/{}: {}",
                      high,
                      chip_name_,
                      line_num_,
                      std::strerror(errno));
        return false;
    }
    return true;
}

void LibGpiodPin::set_edge_callback(hal::GpioEdge edge, std::function<void()> cb) {
    std::lock_guard<PiMutex> lock(cb_mutex_);
    edge_ = edge;
    callback_ = std::move(cb);
}

void LibGpiodPin::start_monitoring() {
    bool expected = false;
    if (!is_open() || !running_.compare_exchange_strong(expected, true))
        return;

    if (config_.direction == hal::GpioDirection::OUTPUT) {
        spdlog::warn(
            "[LibGpiodPin] start_monitoring() on OUTPUT pin {}/{}, ignored", chip_name_, line_num_);
        running_.store(false);
        return;
    }

    gpiod_line_release(line_);

    // 快照 edge_：此后 set_edge_callback() 的修改不影响本次监听配置
    hal::GpioEdge snapshot_edge;
    {
        std::lock_guard<PiMutex> lock(cb_mutex_);
        snapshot_edge = edge_;
    }

    // 修复 Bug：不使用快捷宏，手动构造配置，确保开启中断时仍然保留上下拉(Bias)状态
    struct gpiod_line_request_config cfg {};
    cfg.consumer = consumer_.c_str();

    switch (snapshot_edge) {
        case hal::GpioEdge::RISING:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE;
            break;
        case hal::GpioEdge::FALLING:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
            break;
        case hal::GpioEdge::BOTH:
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
            break;
    }

    if (config_.bias == hal::GpioBias::PULL_UP) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (config_.bias == hal::GpioBias::PULL_DOWN) {
        cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
        spdlog::error("[LibGpiodPin] request edge events failed for {}/{}: {}",
                      chip_name_,
                      line_num_,
                      std::strerror(errno));

        // 修复 Bug：状态回滚，防止引脚彻底“变砖”
        request_line_as_input();
        running_.store(false);
        return;
    }

    last_event_time_ = std::chrono::steady_clock::now();
    monitor_thread_ = std::thread(&LibGpiodPin::monitor_loop, this);
    spdlog::debug("[LibGpiodPin] monitoring started: {}/{}", chip_name_, line_num_);
}

void LibGpiodPin::stop_monitoring() {
    bool was_running = running_.exchange(false);
    if (!was_running)
        return;

    // 要求：stop_monitoring() 严禁在回调内部调用（无论直接还是间接）。
    // detach() 会导致 UAF：对象销毁后游离线程仍常转读成员变量。
    // 正确做法：通过 EventBus 将停止请求转发到主线程处理。
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }

    if (is_open()) {
        gpiod_line_release(line_);
        request_line_as_input();
    }

    spdlog::debug("[LibGpiodPin] monitoring stopped: {}/{}", chip_name_, line_num_);
}

void LibGpiodPin::monitor_loop() {
    // ── 修复竞态调度 ──
    // 线程启动首行由自身完成 RT 提权，确保后续所有代码和硬件响应均在 RT 级别
    // 注：需确保 hal::GpioConfig 中已添加 int rt_priority = 0; 字段
    if (config_.rt_priority > 0) {
        sched_param sp{};
        sp.sched_priority = config_.rt_priority;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            spdlog::warn("[LibGpiodPin] RT priority elevation failed for {}/{}: {}",
                         chip_name_,
                         line_num_,
                         strerror(rc));
        } else {
            spdlog::info("[LibGpiodPin] {}/{} thread elevated to RT SCHED_FIFO priority {}",
                         chip_name_,
                         line_num_,
                         config_.rt_priority);
        }
    }

    struct timespec ts {};
    ts.tv_sec = kWaitTimeoutNs / (1000L * 1000L * 1000L);
    ts.tv_nsec = kWaitTimeoutNs % (1000L * 1000L * 1000L);

    while (running_.load(std::memory_order_relaxed)) {
        int ret = gpiod_line_event_wait(line_, &ts);
        if (ret <= 0)
            continue;

        gpiod_line_event event{};
        if (gpiod_line_event_read(line_, &event) != 0)
            continue;

        // 软件消抖：last_event_time_ 仅在本线程中读写，无需加锁。
        // 注：steady_clock::now() 在 Linux 上经由 vDSO 映射，不是系统调用，成本可忽略。
        if (config_.debounce_ms > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_event_time_)
                    .count();
            if (elapsed_ms < config_.debounce_ms) {
                continue;
            }
            last_event_time_ = now;
        }

        // 持 PI 锁直接调用回调：消除 std::function 拷贝堆分配。
        // RT 约束：回调必须轻量且不得在回调内调用 stop_monitoring()/close()。
        {
            std::lock_guard<PiMutex> lock(cb_mutex_);
            if (callback_) {
                callback_();
            }
        }
    }
}

}  // namespace robot::driver
