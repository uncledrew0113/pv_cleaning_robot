#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <memory>

#include "pv_cleaning_robot/service/config_service.h"
#include "pv_cleaning_robot/service/scheduler_service.h"
#include "pv_cleaning_robot/service/thingsboard_config_manager.h"

using robot::service::ConfigService;
using robot::service::SchedulerService;
using robot::service::ThingsBoardConfigManager;
namespace fs = std::filesystem;

namespace {

struct Fixture {
    std::string path{"/tmp/test_tb_config_manager.json"};
    std::string pending_path{"/tmp/test_tb_config_manager.pending.json"};
    std::string backup_path{"/tmp/test_tb_config_manager.backup.json"};
    ConfigService cfg{path};
    SchedulerService scheduler;
    std::unique_ptr<ThingsBoardConfigManager> manager;

    Fixture() {
        std::ofstream f(path);
        f << R"({
  "robot": {
    "passes": 1.0,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 280.0,
    "brush_rpm": 1000
  },
  "scheduler": {
    "windows": [
      { "hour": 8, "minute": 0 }
    ]
  }
})";
        f.close();
        REQUIRE(cfg.load());
        scheduler.clear_windows();
        scheduler.add_window({8, 0});
        manager = std::make_unique<ThingsBoardConfigManager>(cfg, scheduler);
    }

    ~Fixture() {
        fs::remove(path);
        fs::remove(pending_path);
        fs::remove(backup_path);
    }
};

}  // namespace

TEST_CASE("ThingsBoardConfigManager: invalid speed rejects whole update", "[service][tb_config]") {
    Fixture f;
    const auto before = f.manager->active_config();

    const nlohmann::json attrs{
        {"clean_speed_rpm", 30},
        {"brush_rpm", 0}
    };

    const auto result = f.manager->apply_shared_attributes(attrs);
    CHECK_FALSE(result.accepted);
    CHECK(f.manager->active_config() == before);
    CHECK_FALSE(f.manager->pending_config().has_value());
}

TEST_CASE("ThingsBoardConfigManager: schedule applies immediately, passes stay pending",
          "[service][tb_config]") {
    Fixture f;
    const nlohmann::json attrs{
        {"schedules", {{{"hour", 7}, {"minute", 30}}}},
        {"passes", 2.0}
    };

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

TEST_CASE("ThingsBoardConfigManager: promote_pending_to_active applies next-task config",
          "[service][tb_config]") {
    Fixture f;
    REQUIRE(f.manager->apply_shared_attributes(nlohmann::json{{"passes", 2.5}}).accepted);
    REQUIRE(f.manager->has_pending_config());

    REQUIRE(f.manager->promote_pending_to_active());
    CHECK_FALSE(f.manager->has_pending_config());
    CHECK(f.manager->active_config().passes == Approx(2.5));
    CHECK_FALSE(fs::exists(f.pending_path));
}
