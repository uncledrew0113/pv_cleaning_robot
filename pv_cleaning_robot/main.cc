/**
 * @file main.cc
 * @brief PV 清扫机器人进程入口。
 *
 * main 只处理进程级职责：安装退出信号、创建 RobotApplication、调用 run()。
 * 具体硬件、服务、线程和状态机编排全部收敛到 RobotApplication，避免入口函数承载业务逻辑。
 */
#include <atomic>
#include <csignal>

#include "pv_cleaning_robot/app/robot_application.h"

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int /*sig*/) {
    g_running.store(false);
}

void install_signal_handlers() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}

}  // namespace

int main() {
    install_signal_handlers();

    robot::app::RobotApplication app;
    return app.run(g_running);
}
