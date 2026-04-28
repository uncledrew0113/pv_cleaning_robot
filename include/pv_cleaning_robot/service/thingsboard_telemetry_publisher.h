#pragma once

#include <array>
#include <memory>
#include <string>

#include "pv_cleaning_robot/service/cloud_service.h"
#include "pv_cleaning_robot/service/command_tracker.h"
#include "pv_cleaning_robot/service/config_service.h"

namespace robot::app {
class RobotSupervisor;
}

namespace robot::service {

class ThingsBoardTelemetryPublisher {
public:
    ThingsBoardTelemetryPublisher(ConfigService& config,
                                  std::shared_ptr<CloudService> cloud,
                                  std::shared_ptr<app::RobotSupervisor> supervisor);

    void publish_backup_fallback_event() const;
    void publish_startup_attributes() const;
    void publish_status_event(const char* event_name, bool accepted, const char* reason) const;
    void publish_command_event(const char* event_name, const CommandSnapshot& snapshot) const;
    void publish_business_telemetry() const;

private:
    static constexpr size_t kBusinessPayloadBufferBytes = 4096;
    static constexpr size_t kEventPayloadBufferBytes = 1024;

    ConfigService& config_;
    std::shared_ptr<CloudService> cloud_;
    std::shared_ptr<app::RobotSupervisor> supervisor_;
    mutable std::array<char, kBusinessPayloadBufferBytes> business_payload_buf_{};
    mutable std::string business_payload_cache_;
    mutable std::array<char, kEventPayloadBufferBytes> event_payload_buf_{};
    mutable std::string event_payload_cache_;
};

}  // namespace robot::service
