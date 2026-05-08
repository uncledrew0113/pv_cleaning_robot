#include <catch2/catch.hpp>

#include "pv_cleaning_robot/service/command_tracker.h"

using robot::service::CommandPhase;
using robot::service::CommandTracker;

TEST_CASE("CommandTracker: moves active command into last completed on success",
          "[service][command_tracker]") {
    CommandTracker tracker;

    const auto id = tracker.accept("start", "rpc-001");
    const auto active = tracker.active();
    REQUIRE(active.has_value());
    CHECK(active->id == id);
    CHECK(active->name == "start");
    CHECK(active->request_id == "rpc-001");
    CHECK(active->phase == CommandPhase::Accepted);
    CHECK(active->finished_at_ms == 0);

    tracker.mark_running(id);
    const auto running = tracker.active();
    REQUIRE(running.has_value());
    CHECK(running->phase == CommandPhase::Running);

    tracker.finish_success(id, "completed");
    CHECK_FALSE(tracker.active().has_value());

    const auto last = tracker.last_completed();
    REQUIRE(last.has_value());
    CHECK(last->id == id);
    CHECK(last->name == "start");
    CHECK(last->phase == CommandPhase::Succeeded);
    CHECK(last->reason == "completed");
    CHECK(last->finished_at_ms >= last->accepted_at_ms);
}

TEST_CASE("CommandTracker: failed active command becomes last completed failure",
          "[service][command_tracker]") {
    CommandTracker tracker;

    const auto id = tracker.accept("reset", "rpc-002");
    tracker.mark_running(id);
    tracker.finish_failure(id, "self_check_failed");

    CHECK_FALSE(tracker.active().has_value());

    const auto last = tracker.last_completed();
    REQUIRE(last.has_value());
    CHECK(last->id == id);
    CHECK(last->phase == CommandPhase::Failed);
    CHECK(last->reason == "self_check_failed");
}

TEST_CASE("CommandTracker: rejected command is recorded without active command",
          "[service][command_tracker]") {
    CommandTracker tracker;

    tracker.reject("start", "rpc-003", "robot_not_at_parking_side");

    CHECK_FALSE(tracker.active().has_value());

    const auto last = tracker.last_completed();
    REQUIRE(last.has_value());
    CHECK(last->name == "start");
    CHECK(last->request_id == "rpc-003");
    CHECK(last->phase == CommandPhase::Rejected);
    CHECK(last->reason == "robot_not_at_parking_side");
    CHECK(last->finished_at_ms == last->accepted_at_ms);
}
