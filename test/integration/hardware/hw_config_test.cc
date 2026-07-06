/**
 * @file hw_config_test.cc
 * @brief 真实硬件测试配置加载测试。
 *
 * 本文件验证硬件测试配置文件解析和默认值回退，不直接操作硬件。
 */
#include <catch2/catch.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "integration/hardware/hw_config.h"

namespace {

class ScopedEnv {
   public:
    explicit ScopedEnv(const char* key) : key_(key) {
        const char* current = std::getenv(key_);
        if (current != nullptr) {
            old_value_ = current;
        }
    }

    ~ScopedEnv() {
        if (old_value_.empty()) {
            unsetenv(key_);
        } else {
            setenv(key_, old_value_.c_str(), 1);
        }
    }

   private:
    const char* key_;
    std::string old_value_;
};

}  // namespace

TEST_CASE("correction_compare 未配置的 PID 参数继承 pid 配置",
          "[hw_config][correction_compare]") {
    const auto path = std::filesystem::temp_directory_path() /
                      "pv_cleaning_robot_hw_config_inherit_pid.json";
    {
        std::ofstream out(path, std::ios::trunc);
        REQUIRE(out.is_open());
        out << R"({
  "pid": {
    "kp": 1.25,
    "ki": 0.23,
    "kd": 0.11,
    "integral_limit": 2.5,
    "max_output": 7.0,
    "min_effective_output": 1.7
  },
  "correction_compare": {
    "kp": 5.0,
    "max_output": 10.0
  }
})";
    }

    ScopedEnv env("HW_TEST_CONFIG");
    setenv("HW_TEST_CONFIG", path.string().c_str(), 1);

    const auto params = hw::load_hw_test_config();

    CHECK(params.correction_compare.kp == Approx(5.0f));
    CHECK(params.correction_compare.max_output == Approx(10.0f));
    CHECK(params.correction_compare.ki == Approx(params.pid.ki));
    CHECK(params.correction_compare.kd == Approx(params.pid.kd));
    CHECK(params.correction_compare.integral_limit == Approx(params.pid.integral_limit));
    CHECK(params.correction_compare.min_effective_output ==
          Approx(params.pid.min_effective_output));

    std::filesystem::remove(path);
}
