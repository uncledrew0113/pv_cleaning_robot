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
using robot::service::ThingsBoardConfigManager;
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
        tb_test_support::make_temp_split_config_paths("test_tb_config_manager")};
    ConfigService cfg{paths.runtime_path.string(), paths.fixed_path.string()};
    SchedulerService scheduler;
    std::unique_ptr<ThingsBoardConfigManager> manager;

    Fixture() {
        tb_test_support::write_split_config(paths,
                                            R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "return_brush_rpm": 1000,
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
        scheduler.clear_windows();
        scheduler.add_window({8, 0});
        manager = std::make_unique<ThingsBoardConfigManager>(cfg, scheduler);
    }

    ~Fixture() {
        tb_test_support::cleanup_split_config_paths(paths);
    }
};

}  // namespace

TEST_CASE("ThingsBoardConfigManager: invalid speed rejects whole update", "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager->active_config();

    auto attrs = parse_json(R"({"clean_speed_rpm":30,"brush_rpm":0})");

    const auto result = f.manager->apply_shared_attributes(attrs);
    CHECK_FALSE(result.accepted);
    CHECK(f.manager->active_config() == before);
    CHECK_FALSE(f.manager->pending_config().has_value());
}

TEST_CASE("ThingsBoardConfigManager: schedule applies immediately, passes stay pending",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"schedules":[{"hour":7,"minute":30}],"passes":2.0})");

    const auto result = f.manager->apply_shared_attributes(attrs);
    REQUIRE(result.accepted);

    const auto active = f.manager->active_config();
    const robot::service::TbScheduleEntry expected_schedule{7, 30};
    REQUIRE(active.schedules.size() == 1);
    CHECK(active.schedules[0] == expected_schedule);
    CHECK(active.passes == Approx(1.0));

    const auto pending = f.manager->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->passes == Approx(2.0));
    CHECK(pending->schedules[0] == expected_schedule);

    const auto windows = f.scheduler.snapshot_windows();
    REQUIRE(windows.size() == 1);
    CHECK(windows[0].hour == 7);
    CHECK(windows[0].minute == 30);
}

TEST_CASE("ThingsBoardConfigManager: battery thresholds stay pending until next task",
          "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager->active_config();
    auto attrs =
        parse_json(R"({"start_battery_soc":40.0,"charge_start_soc":20.0,"charge_stop_soc":90.0})");

    const auto result = f.manager->apply_shared_attributes(attrs);
    REQUIRE(result.accepted);

    const auto active = f.manager->active_config();
    CHECK(active.start_battery_soc == Approx(before.start_battery_soc));
    CHECK(active.charge_start_soc == Approx(before.charge_start_soc));
    CHECK(active.charge_stop_soc == Approx(before.charge_stop_soc));

    const auto pending = f.manager->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->start_battery_soc == Approx(40.0));
    CHECK(pending->charge_start_soc == Approx(20.0));
    CHECK(pending->charge_stop_soc == Approx(90.0));
}

TEST_CASE("ThingsBoardConfigManager: return_brush_rpm stays pending until next task",
          "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager->active_config();
    auto attrs = parse_json(R"({"return_brush_rpm":900})");

    const auto result = f.manager->apply_shared_attributes(attrs);
    REQUIRE(result.accepted);

    const auto active = f.manager->active_config();
    CHECK(active.return_brush_rpm == before.return_brush_rpm);

    const auto pending = f.manager->pending_config();
    REQUIRE(pending.has_value());
    CHECK(pending->return_brush_rpm == 900);
}

TEST_CASE("ThingsBoardConfigManager: promote_pending_to_active applies next-task config",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"passes":2.0})");
    REQUIRE(f.manager->apply_shared_attributes(attrs).accepted);
    REQUIRE(f.manager->has_pending_config());

    REQUIRE(f.manager->promote_pending_to_active());
    CHECK_FALSE(f.manager->has_pending_config());
    CHECK(f.manager->active_config().passes == Approx(2.0));
    CHECK_FALSE(fs::exists(f.paths.pending_path));
}

TEST_CASE("ThingsBoardConfigManager: rejects passes=0.5", "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager->active_config();

    auto attrs = parse_json(R"({"passes":0.5})");
    const auto result = f.manager->apply_shared_attributes(attrs);

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "passes must be a positive integer");
    CHECK(f.manager->active_config() == before);
    CHECK_FALSE(f.manager->pending_config().has_value());
}

TEST_CASE("ThingsBoardConfigManager: accepts parking_side left and right only",
          "[service][tb_config]") {
    Fixture f;

    auto left_attrs = parse_json(R"({"parking_side":"left","passes":2.0})");
    const auto left_result = f.manager->apply_shared_attributes(left_attrs);
    REQUIRE(left_result.accepted);
    REQUIRE(f.manager->pending_config().has_value());
    CHECK(f.manager->pending_config()->parking_side == ParkingSide::Left);

    auto right_attrs = parse_json(R"({"parking_side":"right"})");
    const auto right_result = f.manager->apply_shared_attributes(right_attrs);
    REQUIRE(right_result.accepted);
    REQUIRE(f.manager->pending_config().has_value());
    CHECK(f.manager->pending_config()->parking_side == ParkingSide::Right);

    auto bad_attrs = parse_json(R"({"parking_side":"both"})");
    const auto bad_result = f.manager->apply_shared_attributes(bad_attrs);

    CHECK_FALSE(bad_result.accepted);
    CHECK(bad_result.reason == "parking_side must be left or right");
}
