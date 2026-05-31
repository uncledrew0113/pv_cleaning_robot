#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>
#include <rapidjson/document.h>

#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

using robot::service::ConfigService;
using robot::service::ParkingSide;
using robot::service::SchedulerService;
namespace fs = std::filesystem;

namespace {

rapidjson::Document parse_json(const char* text)
{
    rapidjson::Document doc;
    doc.Parse(text);
    REQUIRE_FALSE(doc.HasParseError());
    return doc;
}

struct Fixture {
    tb_test_support::TempSplitConfigPaths paths{
        tb_test_support::make_temp_split_config_paths("test_runtime_config_patch")};
    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    Fixture() {
        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "min_battery_soc": 30.0,
    "charge_stop_soc": 95.0,
    "parking_side": "left"
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})",
                                            R"({})");
        REQUIRE(cfg.load());
        cfg.apply_active_runtime_schedules(scheduler);
    }

    ~Fixture() {
        tb_test_support::cleanup_split_config_paths(paths);
    }
};

}  // namespace

TEST_CASE("ConfigService runtime patch: invalid speed rejects whole update", "[service][tb_config]") {
    Fixture f;
    const auto before = f.cfg.active_runtime_config();

    auto attrs = parse_json(R"({"clean_speed_rpm":30,"brush_rpm":0})");

    const auto result = f.cfg.apply_runtime_patch(attrs);
    CHECK_FALSE(result.accepted);
    CHECK(f.cfg.active_runtime_config() == before);
    CHECK_FALSE(f.cfg.pending_runtime_config().has_value());
}

TEST_CASE("ConfigService runtime patch: schedule applies immediately, passes stay pending",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"schedules":[{"hour":7,"minute":30}],"passes":2.0})");

    const auto result = f.cfg.apply_runtime_patch(attrs);
    REQUIRE(result.accepted);
    f.cfg.apply_active_runtime_schedules(f.scheduler);

    const auto active = f.cfg.active_runtime_config();
    const robot::service::RuntimeScheduleEntry expected_schedule{7, 30};
    REQUIRE(active.schedules.size() == 1);
    CHECK(active.schedules[0] == expected_schedule);
    CHECK(active.passes == Approx(1.0));

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(2.0));
    CHECK(pending->schedules[0] == expected_schedule);

    const auto windows = f.scheduler.snapshot_windows();
    REQUIRE(windows.size() == 1);
    CHECK(windows[0].hour == 7);
    CHECK(windows[0].minute == 30);
}

TEST_CASE("ConfigService runtime patch: battery thresholds stay pending until next task",
          "[service][tb_config]") {
    Fixture f;
    const auto before = f.cfg.active_runtime_config();
    auto attrs = parse_json(R"({"min_battery_soc":40.0,"charge_stop_soc":90.0})");

    const auto result = f.cfg.apply_runtime_patch(attrs);
    REQUIRE(result.accepted);

    const auto active = f.cfg.active_runtime_config();
    CHECK(active.min_battery_soc == Approx(before.min_battery_soc));
    CHECK(active.charge_stop_soc == Approx(before.charge_stop_soc));

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->min_battery_soc == Approx(40.0));
    CHECK(pending->charge_stop_soc == Approx(90.0));
}

TEST_CASE("ConfigService runtime patch: promote_pending_to_active applies next-task config",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"passes":2.0})");
    REQUIRE(f.cfg.apply_runtime_patch(attrs).accepted);
    REQUIRE(f.cfg.has_pending_runtime_config());

    REQUIRE(f.cfg.promote_pending_runtime_to_active());
    CHECK_FALSE(f.cfg.has_pending_runtime_config());
    CHECK(f.cfg.active_runtime_config().passes == Approx(2.0));
    CHECK_FALSE(fs::exists(f.paths.pending_path));
}

TEST_CASE("ConfigService runtime patch: rejects passes=0.5", "[service][tb_config]") {
    Fixture f;
    const auto before = f.cfg.active_runtime_config();

    auto attrs = parse_json(R"({"passes":0.5})");
    const auto result = f.cfg.apply_runtime_patch(attrs);

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "passes must be a positive integer");
    CHECK(f.cfg.active_runtime_config() == before);
    CHECK_FALSE(f.cfg.pending_runtime_config().has_value());
}

TEST_CASE("ConfigService runtime patch: accepts parking_side left and right only",
          "[service][tb_config]") {
    Fixture f;

    auto left_attrs = parse_json(R"({"parking_side":"left","passes":2.0})");
    const auto left_result = f.cfg.apply_runtime_patch(left_attrs);
    REQUIRE(left_result.accepted);
    REQUIRE(f.cfg.pending_runtime_config().has_value());
    CHECK(f.cfg.pending_runtime_config()->parking_side == ParkingSide::Left);

    auto right_attrs = parse_json(R"({"parking_side":"right"})");
    const auto right_result = f.cfg.apply_runtime_patch(right_attrs);
    REQUIRE(right_result.accepted);
    REQUIRE(f.cfg.pending_runtime_config().has_value());
    CHECK(f.cfg.pending_runtime_config()->parking_side == ParkingSide::Right);

    auto bad_attrs = parse_json(R"({"parking_side":"both"})");
    const auto bad_result = f.cfg.apply_runtime_patch(bad_attrs);

    CHECK_FALSE(bad_result.accepted);
    CHECK(bad_result.reason == "parking_side must be left or right");
}
