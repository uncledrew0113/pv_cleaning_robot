/*
 * 全局 Catch2 入口（仅含 main 定义）
 * 每个测试可执行文件只能有一个 CATCH_CONFIG_MAIN 翻译单元，
 * 因此将其单独提取到此文件，所有测试文件 #include <catch2/catch.hpp> 即可。
 */
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
