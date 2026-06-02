#include <catch2/catch.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <ctime>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "pv_cleaning_robot/service/heading_corrector.h"

using robot::service::HeadingCorrector;
using robot::domain::Endpoint;

namespace {

struct LocalUdsServer {
    std::string path;
    int server_fd{-1};
    int client_fd{-1};
    std::thread accept_thread;

    explicit LocalUdsServer(std::string socket_path) : path(std::move(socket_path)) {
        server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        REQUIRE(server_fd >= 0);

        unlink(path.c_str());

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
        REQUIRE(bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
        REQUIRE(listen(server_fd, 1) == 0);

        accept_thread = std::thread([this]() {
            client_fd = accept(server_fd, nullptr, nullptr);
        });
    }

    ~LocalUdsServer() {
        if (client_fd >= 0) {
            close(client_fd);
        }
        if (server_fd >= 0) {
            close(server_fd);
        }
        if (accept_thread.joinable()) {
            accept_thread.join();
        }
        unlink(path.c_str());
    }

    void send_line(const std::string& line) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (client_fd < 0 && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        REQUIRE(client_fd >= 0);

        std::string payload = line + "\n";
        const char* data = payload.data();
        size_t remaining = payload.size();
        while (remaining > 0) {
            const ssize_t sent = send(client_fd, data, remaining, MSG_NOSIGNAL);
            REQUIRE(sent > 0);
            data += sent;
            remaining -= static_cast<size_t>(sent);
        }
    }
};

HeadingCorrector::Params test_params(const std::string& uds_path) {
    HeadingCorrector::Params p;
    p.uds_path = uds_path;
    p.reconnect_interval_ms = 20;
    p.result_timeout_ms = 300;
    p.min_confidence = 0.5f;
    p.deadband_yaw_deg = 0.01f;
    p.kp = 10.0f;
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.integral_limit = 1.0f;
    p.max_output = 30.0f;
    p.min_effective_output = 0.0f;
    p.yaw_alpha = 1.0f;
    p.output_sign = 1.0f;
    return p;
}

HeadingCorrector::Input make_input_with_speed(Endpoint target,
                                              float speed_rpm,
                                              Endpoint primary_dock = Endpoint::B) {
    HeadingCorrector::Input input;
    input.dt_s = 0.02f;
    input.has_base_command = true;
    input.travel_direction = robot::domain::travel_direction_to(target);
    input.primary_dock = primary_dock;
    const float dir = target == Endpoint::A ? 1.0f : -1.0f;
    input.base_command = {speed_rpm * dir, speed_rpm * dir, -speed_rpm * dir, -speed_rpm * dir};
    return input;
}

HeadingCorrector::Input make_input(Endpoint target, Endpoint primary_dock = Endpoint::B) {
    return make_input_with_speed(target, 100.0f, primary_dock);
}

std::string unique_socket_path() {
    return "/tmp/pv_heading_test_" + std::to_string(::getpid()) + "_" +
           std::to_string(::time(nullptr)) + ".sock";
}

}  // namespace

TEST_CASE("HeadingCorrector: disabled controller outputs zero", "[service][heading_pid]") {
    HeadingCorrector ctrl;
    auto input = make_input(Endpoint::A);

    const auto out = ctrl.compute(input);
    REQUIRE(out.correction_rpm == Approx(0.0f));
    REQUIRE_FALSE(out.has_speed_command);
}

TEST_CASE("HeadingCorrector: primary dock B toward A yaw>0 speeds bottom only",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto out = ctrl.compute(make_input(Endpoint::A));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(-1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(-101.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(-101.0f));
}

TEST_CASE("HeadingCorrector: primary dock B toward A yaw<0 slows bottom only",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":-0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto out = ctrl.compute(make_input(Endpoint::A));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(-99.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(-99.0f));
}

TEST_CASE("HeadingCorrector: primary dock B toward B yaw>0 slows bottom only",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto out = ctrl.compute(make_input(Endpoint::B));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(-1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(99.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(99.0f));
}

TEST_CASE("HeadingCorrector: primary dock B toward B yaw<0 speeds bottom only",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":-0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto out = ctrl.compute(make_input(Endpoint::B));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(101.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(101.0f));
}

TEST_CASE("HeadingCorrector: correction direction depends on target, not primary dock",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto out = ctrl.compute(make_input(Endpoint::B, Endpoint::A));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(-1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(-100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(99.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(99.0f));

    out = ctrl.compute(make_input(Endpoint::A, Endpoint::A));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(-1.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.rt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(-101.0f));
    REQUIRE(out.speed_command.rb_rpm == Approx(-101.0f));
}

TEST_CASE("HeadingCorrector: bottom correction never reverses wheel direction",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    HeadingCorrector ctrl(test_params(server.path));
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":-10.0,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto toward_a = ctrl.compute(make_input_with_speed(Endpoint::A, 5.0f));
    REQUIRE(toward_a.has_speed_command);
    CHECK(toward_a.speed_command.lt_rpm == Approx(5.0f));
    CHECK(toward_a.speed_command.rt_rpm == Approx(5.0f));
    CHECK(toward_a.speed_command.lb_rpm == Approx(0.0f));
    CHECK(toward_a.speed_command.rb_rpm == Approx(0.0f));
}

TEST_CASE("HeadingCorrector: stale or invalid samples fall back to base command",
          "[service][heading_pid]") {
    LocalUdsServer server(unique_socket_path());
    auto params = test_params(server.path);
    params.result_timeout_ms = 50;
    HeadingCorrector ctrl(params);
    ctrl.enable(true);

    server.send_line(R"({"valid":true,"yaw_deg":0.10,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    auto out = ctrl.compute(make_input(Endpoint::A));
    REQUIRE(out.has_speed_command);
    REQUIRE(out.correction_rpm == Approx(0.0f));
    REQUIRE(out.speed_command.lt_rpm == Approx(100.0f));
    REQUIRE(out.speed_command.lb_rpm == Approx(-100.0f));

    server.send_line(R"({"valid":false,"yaw_deg":0.0,"confidence":0.82})");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    out = ctrl.compute(make_input(Endpoint::A));
    REQUIRE(out.correction_rpm == Approx(0.0f));
}
