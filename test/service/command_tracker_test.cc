#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/command_tracker.h"

using robot::service::CommandTracker;

TEST_CASE("CommandTracker: generates monotonic command ids across accepted and rejected commands",
          "[service][command_tracker]") {
    CommandTracker tracker;

    const auto id = tracker.accept("start", "rpc-001");
    CHECK(id == "cmd-1");
    tracker.mark_running(id);

    // 当前 MVP 不再暴露命令完成态查询，但拒绝命令仍应消耗一个本地序号，
    // 让日志中的 cmd-N 与 RPC 请求处理顺序保持一致。
    tracker.reject("stop", "rpc-002", "not_running");

    const auto next_id = tracker.accept("fault_reset", "rpc-003");
    CHECK(next_id == "cmd-3");
}
