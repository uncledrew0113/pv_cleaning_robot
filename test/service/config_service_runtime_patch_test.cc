#include <catch2/catch.hpp>

#include <filesystem>
#include <memory>
#include <rapidjson/document.h>

#include "integration/thingsboard_test_support.h"
#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_control_plane.h"

using robot::service::ConfigService;
using robot::service::Endpoint;
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
    "repeat_count": 1,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000,
    "min_battery_soc": 30.0,
    "charge_stop_soc": 95.0,
    "primary_dock": "A"
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

TEST_CASE("ConfigService runtime patch: schedule applies immediately, repeat_count stays pending",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"schedules":[{"hour":7,"minute":30}],"repeat_count":2})");

    const auto result = f.cfg.apply_runtime_patch(attrs);
    REQUIRE(result.accepted);
    f.cfg.apply_active_runtime_schedules(f.scheduler);

    const auto active = f.cfg.active_runtime_config();
    const robot::service::RuntimeScheduleEntry expected_schedule{7, 30};
    REQUIRE(active.schedules.size() == 1);
    CHECK(active.schedules[0] == expected_schedule);
    CHECK(active.repeat_count == 1u);

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->repeat_count == 2u);
    CHECK(pending->schedules[0] == expected_schedule);

    const auto windows = f.scheduler.snapshot_windows();
    REQUIRE(windows.size() == 1);
    CHECK(windows[0].hour == 7);
    CHECK(windows[0].minute == 30);
}

TEST_CASE("ConfigService runtime patch: repeat_count stays pending", "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"repeat_count":2})");

    const auto result = f.cfg.apply_runtime_patch(attrs);

    REQUIRE(result.accepted);
    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->repeat_count == 2u);
}

TEST_CASE("ConfigService applies schedules immediately and other runtime fields pending",
          "[service][tb_config]") {
    Fixture f;
    auto attrs = parse_json(R"({"schedules":[{"hour":8,"minute":30}],"clean_speed_rpm":80})");

    const auto result = f.cfg.apply_runtime_patch(attrs, &f.scheduler);

    REQUIRE(result.accepted);
    const auto active = f.cfg.active_runtime_config();
    REQUIRE(active.schedules.size() == 1);
    CHECK(active.schedules[0] == robot::service::RuntimeScheduleEntry{8, 30});
    CHECK(active.clean_speed_rpm == Approx(300.0));

    const auto pending = f.cfg.pending_runtime_config();
    REQUIRE(pending.has_value());
    CHECK(pending->clean_speed_rpm == Approx(80.0));
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
    auto attrs = parse_json(R"({"repeat_count":2})");
    REQUIRE(f.cfg.apply_runtime_patch(attrs).accepted);
    REQUIRE(f.cfg.has_pending_runtime_config());

    REQUIRE(f.cfg.promote_pending_runtime_to_active());
    CHECK_FALSE(f.cfg.has_pending_runtime_config());
    CHECK(f.cfg.active_runtime_config().repeat_count == 2u);
    CHECK_FALSE(fs::exists(f.paths.pending_path));
}

TEST_CASE("ConfigService runtime patch: rejects repeat_count=0.5", "[service][tb_config]") {
    Fixture f;
    const auto before = f.cfg.active_runtime_config();

    auto attrs = parse_json(R"({"repeat_count":0.5})");
    const auto result = f.cfg.apply_runtime_patch(attrs);

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "repeat_count must be a positive integer");
    CHECK(f.cfg.active_runtime_config() == before);
    CHECK_FALSE(f.cfg.pending_runtime_config().has_value());
}

TEST_CASE("ConfigService runtime patch: accepts primary_dock A and B only",
          "[service][tb_config]") {
    Fixture f;

    auto a_attrs = parse_json(R"({"primary_dock":"A","repeat_count":2})");
    const auto a_result = f.cfg.apply_runtime_patch(a_attrs);
    REQUIRE(a_result.accepted);
    REQUIRE(f.cfg.pending_runtime_config().has_value());
    CHECK(f.cfg.pending_runtime_config()->primary_dock == Endpoint::A);

    auto b_attrs = parse_json(R"({"primary_dock":"B"})");
    const auto b_result = f.cfg.apply_runtime_patch(b_attrs);
    REQUIRE(b_result.accepted);
    REQUIRE(f.cfg.pending_runtime_config().has_value());
    CHECK(f.cfg.pending_runtime_config()->primary_dock == Endpoint::B);

    auto bad_attrs = parse_json(R"({"primary_dock":"C"})");
    const auto bad_result = f.cfg.apply_runtime_patch(bad_attrs);

    CHECK_FALSE(bad_result.accepted);
    CHECK(bad_result.reason == "primary_dock must be A or B");
}
