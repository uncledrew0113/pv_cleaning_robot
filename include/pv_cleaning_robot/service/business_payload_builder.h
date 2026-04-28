#pragma once

#include <cstddef>

#include "pv_cleaning_robot/service/business_telemetry_snapshot.h"

namespace robot::service {

class BusinessPayloadBuilder {
public:
    static size_t build(const BusinessTelemetrySnapshot& view, char* out, size_t cap) noexcept;
};

}  // namespace robot::service
