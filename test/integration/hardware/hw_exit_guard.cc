#include "integration/hardware/hw_exit_guard.h"

#include <csignal>

namespace hw {

namespace {

void hw_signal_handler(int) {
    HwExitGuard::instance().request_exit();
}

}  // namespace

HwExitGuard& HwExitGuard::instance() {
    static HwExitGuard guard;
    return guard;
}

void HwExitGuard::install() {
    if (installed_) {
        exit_requested_ = 0;
        return;
    }
    prev_sigint_ = std::signal(SIGINT, hw_signal_handler);
    prev_sigterm_ = std::signal(SIGTERM, hw_signal_handler);
    exit_requested_ = 0;
    installed_ = true;
}

void HwExitGuard::set_active(IGracefulShutdown* active) {
    active_.store(active);
}

void HwExitGuard::clear_active(IGracefulShutdown* active) {
    IGracefulShutdown* expected = active;
    if (!active_.compare_exchange_strong(expected, nullptr)) {
        return;
    }
    if (installed_) {
        std::signal(SIGINT, prev_sigint_);
        std::signal(SIGTERM, prev_sigterm_);
        installed_ = false;
    }
    exit_requested_ = 0;
}

bool HwExitGuard::exit_requested() const {
    return exit_requested_ != 0;
}

void HwExitGuard::request_exit() {
    exit_requested_ = 1;
}

}  // namespace hw
