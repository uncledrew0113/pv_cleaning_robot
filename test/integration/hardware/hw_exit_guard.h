#pragma once

#include <atomic>
#include <csignal>

namespace hw {

struct IGracefulShutdown {
    virtual ~IGracefulShutdown() = default;
    virtual void shutdown() = 0;
};

class HwExitGuard {
   public:
    static HwExitGuard& instance();

    void install();
    void set_active(IGracefulShutdown* active);
    void clear_active(IGracefulShutdown* active);
    bool exit_requested() const;
    void request_exit();

   private:
    volatile std::sig_atomic_t exit_requested_{0};
    std::atomic<IGracefulShutdown*> active_{nullptr};
    using SignalHandler = void (*)(int);
    SignalHandler prev_sigint_{SIG_DFL};
    SignalHandler prev_sigterm_{SIG_DFL};
    bool installed_{false};
};

}  // namespace hw
