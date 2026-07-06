/**
 * @file hw_test_main.cc
 * @brief 真实硬件集成测试 Catch2 入口。
 *
 * hw_tests 与 unit_tests 分离构建，运行前必须确认已连接真实硬件并具备安全测试环境。
 */
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
