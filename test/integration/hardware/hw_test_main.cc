/*
 * 硬件集成测试专用 Catch2 入口（需真实硬件，不在 CI 中编译）
 * 与 unit_tests 分离编译为 hw_tests 可执行文件。
 */
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
